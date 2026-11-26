//! Low-latency presentation: Direct3D 11, a flip-model swap chain with at most one frame queued
//! (SetMaximumFrameLatency(1), paced by its waitable object), the newest picture presented at once (tearing allowed,
//! so a full-screen window flipped directly to the screen does not wait for the next refresh).
//!
//! The bar is drawn by Direct2D into the same back buffer: a separate window beside the video, even without
//! overlapping it, makes every frame composed (~20 ms instead of ~2 ms).
use std::mem::ManuallyDrop;
use windows::core::{s, w, Interface, BOOL, PCSTR};
use windows::Win32::Foundation::{E_FAIL, HANDLE, HMODULE, HWND, RECT};
use windows::Win32::Graphics::Direct2D::Common::*;
use windows::Win32::Graphics::Direct2D::*;
use windows::Win32::Graphics::Direct3D::Fxc::D3DCompile;
use windows::Win32::Graphics::Direct3D::{
    ID3DBlob, D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST, D3D_DRIVER_TYPE_HARDWARE, D3D_DRIVER_TYPE_UNKNOWN,
};
use windows::Win32::Graphics::Direct3D11::*;
use windows::Win32::Graphics::DirectWrite::*;
use windows::Win32::Graphics::Dxgi::Common::*;
use windows::Win32::Graphics::Dxgi::*;
use windows::Win32::Graphics::Gdi::{MonitorFromWindow, MONITOR_DEFAULTTONEAREST};
use windows::Win32::System::Threading::WaitForSingleObjectEx;
use windows::Win32::UI::WindowsAndMessaging::GetClientRect;

const SHADER: &str = r#"
struct O { float4 p : SV_Position; float2 t : TEXCOORD; };
O tri(uint i, float2 f) { O o; float2 t = float2((i << 1) & 2, i & 2); o.t = lerp(t, 1 - t, f); o.p = float4(t * float2(2, -2) + float2(-1, 1), 0, 1); return o; }
O vs(uint i : SV_VertexID) { return tri(i, float2(0, 0)); }
O vs_h(uint i : SV_VertexID) { return tri(i, float2(1, 0)); }
O vs_v(uint i : SV_VertexID) { return tri(i, float2(0, 1)); }
O vs_hv(uint i : SV_VertexID) { return tri(i, float2(1, 1)); }
Texture2D tx : register(t0);
// The picture is always enlarged with Lanczos-3: each screen pixel from the 6 x 6 nearest picture pixels, weighted by sinc(x) sinc(x / 3). Sharper than
// bilinear when a small picture is enlarged; clamped, since the negative lobes can overshoot at hard edges.
float lz(float x) { x = abs(x); if (x < 1e-4) return 1; if (x >= 3) return 0; float a = 3.14159265 * x; return 3 * sin(a) * sin(a / 3) / (a * a); }
float4 ps_lanczos(float4 p : SV_Position, float2 t : TEXCOORD) : SV_Target {
    uint w, h; tx.GetDimensions(w, h);
    float2 pos = t * float2(w, h) - 0.5; float2 b = floor(pos); float2 f = pos - b;
    float wx[6], wy[6];
    [unroll] for (int k = 0; k < 6; k++) { wx[k] = lz(k - 2 - f.x); wy[k] = lz(k - 2 - f.y); }
    float4 sum = 0; float wsum = 0;
    [unroll] for (int j = 0; j < 6; j++) { [unroll] for (int i = 0; i < 6; i++) {
        int2 q = clamp(int2(b) + int2(i - 2, j - 2), int2(0, 0), int2(w - 1, h - 1));
        float wt = wx[i] * wy[j]; sum += tx.Load(int3(q, 0)) * wt; wsum += wt; } }
    return saturate(sum / wsum);
}
"#;

/// What the bar shows. Live: label/value pairs, a detail line (a warning when `warn`), `rec` ("REC 02:14") while
/// recording. Replaying a session: `replay`.
#[derive(Clone, Default)]
pub struct Bar {
    pub stats: Vec<(String, String)>,
    pub detail: String,
    pub warn: bool,
    pub rec: Option<String>,
    pub replay: Option<ReplayBar>,
}
#[derive(Clone, Default)]
pub struct ReplayBar {
    pub title: String,
    /// "02:14 / 04:55"
    pub time: String,
    pub playing: bool,
    pub speed: String,
    /// the live link meanwhile ("LIVE link 100 %")
    pub live: String,
    /// 0..1
    pub progress: f64,
    /// problems, 0..1
    pub marks: Vec<f64>,
}
pub const BTN_VIDEO: u32 = 1;
pub const BTN_QUIT: u32 = 2;
pub const BTN_REC: u32 = 3;
pub const BTN_SETTINGS: u32 = 4;
pub const BTN_BACK: u32 = 5;
pub const BTN_PLAY: u32 = 6;
pub const BTN_FWD: u32 = 7;
pub const BTN_SPEED: u32 = 8;
pub const BTN_LIVE: u32 = 9;
/// the replay's progress strip: a click seeks to (x - left) / width
pub const BTN_SEEK: u32 = 10;
/// The bar's share of the screen height; the video gets the rest.
pub const BAR_SHARE: f32 = 0.10;

struct Fonts {
    height: u32,
    brand: IDWriteTextFormat,
    label: IDWriteTextFormat,
    value: IDWriteTextFormat,
    small: IDWriteTextFormat,
    button: IDWriteTextFormat,
}

pub struct Renderer {
    hwnd: HWND,
    device: ID3D11Device,
    ctx: ID3D11DeviceContext,
    chain: IDXGISwapChain2,
    waitable: HANDLE,
    rtv: Option<ID3D11RenderTargetView>,
    /// one per flip: none, horizontal, vertical, both
    vs: [ID3D11VertexShader; 4],
    /// Lanczos-3 (the shader's ps_lanczos)
    ps: ID3D11PixelShader,
    tex: Option<(ID3D11Texture2D, ID3D11ShaderResourceView, u32, u32)>,
    back: (u32, u32),
    flags: u32,
    pub tearing: bool,
    pub gpu: String,
    dc: ID2D1DeviceContext,
    dw: IDWriteFactory,
    target: Option<ID2D1Bitmap1>,
    fonts: Option<Fonts>,
    /// the bar's buttons in client pixels (for clicks), from the last frame drawn with the bar
    pub buttons: Vec<(u32, RECT)>,
    /// the link warning: an outline of this colour (r, g, b) around the video, or none
    pub outline: Option<[f32; 3]>,
    /// "why is there no picture?" (diag.rs): (title, explanation), drawn over the middle of the video
    pub notice: Option<(String, String)>,
    /// --shot: the next frame drawn is also saved to this file (24-bit BMP) before it is presented
    pub shot: Option<std::path::PathBuf>,
    /// pixels on the left kept free for the Settings panel (the video is fitted into the rest)
    pub panel: f32,
    /// picture option: bit 0 mirror left-right, bit 1 upside down
    pub flip: u8,
}
// The renderer lives on the render thread only.
unsafe impl Send for Renderer {}

fn compile(entry: PCSTR, target: PCSTR) -> windows::core::Result<Vec<u8>> {
    let mut blob: Option<ID3DBlob> = None;
    let mut err: Option<ID3DBlob> = None;
    unsafe {
        D3DCompile(
            SHADER.as_ptr() as _,
            SHADER.len(),
            None,
            None,
            None,
            entry,
            target,
            0,
            0,
            &mut blob,
            Some(&mut err),
        )?;
        let b = blob.unwrap();
        Ok(std::slice::from_raw_parts(b.GetBufferPointer() as *const u8, b.GetBufferSize()).to_vec())
    }
}

fn client(hwnd: HWND) -> (u32, u32) {
    let mut r = RECT::default();
    unsafe {
        let _ = GetClientRect(hwnd, &mut r);
    }
    ((r.right - r.left).max(1) as u32, (r.bottom - r.top).max(1) as u32)
}

impl Renderer {
    pub fn new(hwnd: HWND) -> windows::core::Result<Renderer> {
        unsafe {
            let factory: IDXGIFactory2 = CreateDXGIFactory2(DXGI_CREATE_FACTORY_FLAGS(0))?;
            // The GPU that drives the window's monitor: a picture drawn on another GPU is always composed.
            let monitor = MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST);
            let mut adapter: Option<IDXGIAdapter1> = None;
            let mut i = 0;
            'find: while let Ok(a) = factory.EnumAdapters1(i) {
                let mut j = 0;
                while let Ok(o) = a.EnumOutputs(j) {
                    if o.GetDesc().is_ok_and(|d| d.Monitor == monitor) {
                        adapter = Some(a);
                        break 'find;
                    }
                    j += 1;
                }
                i += 1;
            }
            let gpu = adapter.as_ref().and_then(|a| a.GetDesc1().ok()).map(|d| {
                String::from_utf16_lossy(&d.Description)
                    .trim_end_matches('\0')
                    .to_string()
            });
            let mut device = None;
            let mut ctx = None;
            let (driver, adapter_arg): (_, Option<IDXGIAdapter>) = match &adapter {
                Some(a) => (D3D_DRIVER_TYPE_UNKNOWN, Some(a.cast()?)),
                None => (D3D_DRIVER_TYPE_HARDWARE, None),
            };
            D3D11CreateDevice(
                adapter_arg.as_ref(),
                driver,
                HMODULE::default(),
                D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                None,
                D3D11_SDK_VERSION,
                Some(&mut device),
                None,
                Some(&mut ctx),
            )?;
            let device: ID3D11Device = device.unwrap();
            let ctx = ctx.unwrap();
            let mut allow = BOOL(0);
            let tearing = factory.cast::<IDXGIFactory5>().ok().is_some_and(|f| {
                f.CheckFeatureSupport(
                    DXGI_FEATURE_PRESENT_ALLOW_TEARING,
                    &mut allow as *mut _ as _,
                    std::mem::size_of::<BOOL>() as u32,
                )
                .is_ok()
                    && allow.as_bool()
            });
            let flags = DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT.0 as u32
                | if tearing {
                    DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING.0 as u32
                } else {
                    0
                };
            let (w, h) = client(hwnd);
            let desc = DXGI_SWAP_CHAIN_DESC1 {
                Width: w,
                Height: h,
                Format: DXGI_FORMAT_B8G8R8A8_UNORM,
                SampleDesc: DXGI_SAMPLE_DESC { Count: 1, Quality: 0 },
                BufferUsage: DXGI_USAGE_RENDER_TARGET_OUTPUT,
                BufferCount: 2,
                Scaling: DXGI_SCALING_STRETCH,
                SwapEffect: DXGI_SWAP_EFFECT_FLIP_DISCARD,
                AlphaMode: DXGI_ALPHA_MODE_IGNORE,
                Flags: flags,
                ..Default::default()
            };
            let chain1 = factory.CreateSwapChainForHwnd(&device, hwnd, &desc, None, None)?;
            let _ = factory.MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);
            let chain: IDXGISwapChain2 = chain1.cast()?;
            chain.SetMaximumFrameLatency(1)?;
            let waitable = chain.GetFrameLatencyWaitableObject();
            let mut vss = Vec::new();
            for entry in [s!("vs"), s!("vs_h"), s!("vs_v"), s!("vs_hv")] {
                let b = compile(entry, s!("vs_5_0"))?;
                let mut v = None;
                device.CreateVertexShader(&b, None, Some(&mut v))?;
                vss.push(v.unwrap());
            }
            let vs: [ID3D11VertexShader; 4] = vss.try_into().map_err(|_| windows::core::Error::from(E_FAIL))?;
            let psb = compile(s!("ps_lanczos"), s!("ps_5_0"))?;
            let mut ps = None;
            device.CreatePixelShader(&psb, None, Some(&mut ps))?;
            let d2f: ID2D1Factory1 = D2D1CreateFactory(
                D2D1_FACTORY_TYPE_SINGLE_THREADED,
                Some(&D2D1_FACTORY_OPTIONS::default()),
            )?;
            let d2dev = d2f.CreateDevice(&device.cast::<IDXGIDevice>()?)?;
            let dc = d2dev.CreateDeviceContext(D2D1_DEVICE_CONTEXT_OPTIONS_NONE)?;
            dc.SetDpi(96.0, 96.0); // Direct2D units = pixels
            let dw: IDWriteFactory = DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED)?;
            let mut r = Renderer {
                dc,
                dw,
                target: None,
                fonts: None,
                buttons: Vec::new(),
                shot: None,
                outline: None,
                notice: None,
                panel: 0.0,
                flip: 0,
                hwnd,
                device,
                ctx,
                chain,
                waitable,
                rtv: None,
                vs,
                ps: ps.unwrap(),
                tex: None,
                back: (w, h),
                flags,
                tearing,
                gpu: gpu.unwrap_or_else(|| "default GPU".into()),
            };
            r.make_rtv()?;
            Ok(r)
        }
    }

    /// The back buffer as drawn (video and bar) to a 24-bit BMP: for --shot, a screenshot without a screen.
    fn save_bmp(&self, path: &std::path::Path) -> std::io::Result<()> {
        let io = |e: windows::core::Error| std::io::Error::other(e.to_string());
        unsafe {
            let back: ID3D11Texture2D = self.chain.GetBuffer(0).map_err(io)?;
            let mut d = D3D11_TEXTURE2D_DESC::default();
            back.GetDesc(&mut d);
            d.Usage = D3D11_USAGE_STAGING;
            d.BindFlags = 0;
            d.CPUAccessFlags = D3D11_CPU_ACCESS_READ.0 as u32;
            d.MiscFlags = 0;
            let mut staging = None;
            self.device.CreateTexture2D(&d, None, Some(&mut staging)).map_err(io)?;
            let staging = staging.unwrap();
            self.ctx.CopyResource(&staging, &back);
            let mut m = D3D11_MAPPED_SUBRESOURCE::default();
            self.ctx.Map(&staging, 0, D3D11_MAP_READ, 0, Some(&mut m)).map_err(io)?;
            let (w, h) = (d.Width as usize, d.Height as usize);
            let row = (w * 3 + 3) & !3;
            let mut out = Vec::with_capacity(54 + row * h);
            let size = (54 + row * h) as u32;
            out.extend_from_slice(b"BM");
            out.extend_from_slice(&size.to_le_bytes());
            out.extend_from_slice(&[0, 0, 0, 0, 54, 0, 0, 0, 40, 0, 0, 0]);
            out.extend_from_slice(&(w as i32).to_le_bytes());
            out.extend_from_slice(&(-(h as i32)).to_le_bytes()); // top-down
            out.extend_from_slice(&[1, 0, 24, 0, 0, 0, 0, 0]);
            out.extend_from_slice(&((row * h) as u32).to_le_bytes());
            out.extend_from_slice(&[0; 16]);
            for y in 0..h {
                let src = std::slice::from_raw_parts((m.pData as *const u8).add(y * m.RowPitch as usize), w * 4);
                for p in src.chunks_exact(4) {
                    out.extend_from_slice(&p[..3]); // BGRA -> BGR
                }
                out.resize(out.len() + (row - w * 3), 0);
            }
            self.ctx.Unmap(&staging, 0);
            std::fs::write(path, out)
        }
    }

    fn make_rtv(&mut self) -> windows::core::Result<()> {
        unsafe {
            let back: ID3D11Texture2D = self.chain.GetBuffer(0)?;
            let mut rtv = None;
            self.device.CreateRenderTargetView(&back, None, Some(&mut rtv))?;
            self.rtv = rtv;
            let surface: IDXGISurface = self.chain.GetBuffer(0)?;
            let props = D2D1_BITMAP_PROPERTIES1 {
                pixelFormat: D2D1_PIXEL_FORMAT {
                    format: DXGI_FORMAT_B8G8R8A8_UNORM,
                    alphaMode: D2D1_ALPHA_MODE_IGNORE,
                },
                dpiX: 96.0,
                dpiY: 96.0,
                bitmapOptions: D2D1_BITMAP_OPTIONS_TARGET | D2D1_BITMAP_OPTIONS_CANNOT_DRAW,
                colorContext: ManuallyDrop::new(None),
            };
            self.target = Some(self.dc.CreateBitmapFromDxgiSurface(&surface, Some(&props))?);
        }
        Ok(())
    }

    /// Waits (up to `ms`) until the swap chain can take a frame without queueing behind another one.
    pub fn wait_ready(&self, ms: u32) {
        unsafe {
            WaitForSingleObjectEx(self.waitable, ms, true);
        }
    }

    /// Draws `pixels` (BGRA, w x h; None: the last picture again) below the bar (None: no bar, the video on the
    /// whole screen) and presents it. Returns the present count.
    pub fn present(&mut self, pixels: Option<(&[u8], u32, u32)>, bar: Option<&Bar>) -> windows::core::Result<u32> {
        unsafe {
            let (cw, ch) = client(self.hwnd);
            if (cw, ch) != self.back {
                self.rtv = None;
                self.dc.SetTarget(None);
                self.target = None;
                self.ctx.ClearState();
                self.chain
                    .ResizeBuffers(0, cw, ch, DXGI_FORMAT_UNKNOWN, DXGI_SWAP_CHAIN_FLAG(self.flags as i32))?;
                self.back = (cw, ch);
                self.make_rtv()?;
            }
            if let Some((px, w, h)) = pixels {
                if self.tex.as_ref().map(|t| (t.2, t.3)) != Some((w, h)) {
                    let td = D3D11_TEXTURE2D_DESC {
                        Width: w,
                        Height: h,
                        MipLevels: 1,
                        ArraySize: 1,
                        Format: DXGI_FORMAT_B8G8R8A8_UNORM,
                        SampleDesc: DXGI_SAMPLE_DESC { Count: 1, Quality: 0 },
                        Usage: D3D11_USAGE_DEFAULT,
                        BindFlags: D3D11_BIND_SHADER_RESOURCE.0 as u32,
                        ..Default::default()
                    };
                    let mut t = None;
                    self.device.CreateTexture2D(&td, None, Some(&mut t))?;
                    let t = t.unwrap();
                    let mut v = None;
                    self.device.CreateShaderResourceView(&t, None, Some(&mut v))?;
                    self.tex = Some((t, v.unwrap(), w, h));
                }
                let (t, ..) = self.tex.as_ref().unwrap();
                self.ctx.UpdateSubresource(t, 0, None, px.as_ptr() as _, w * 4, 0);
            }
            let rtv = self.rtv.clone();
            self.ctx.OMSetRenderTargets(Some(&[rtv.clone()]), None);
            self.ctx
                .ClearRenderTargetView(rtv.as_ref().unwrap(), &[0.0, 0.0, 0.0, 1.0]);
            let bar_h = if bar.is_some() {
                (self.back.1 as f32 * BAR_SHARE).round()
            } else {
                0.0
            };
            // where the video goes (the whole area under the bar until the first picture)
            let mut video = D2D_RECT_F {
                left: self.panel,
                top: bar_h,
                right: self.back.0 as f32,
                bottom: self.back.1 as f32,
            };
            if let Some((_, view, w, h)) = &self.tex {
                let (bw, bh) = ((self.back.0 as f32 - self.panel).max(1.0), self.back.1 as f32 - bar_h);
                let s = (bw / *w as f32).min(bh / *h as f32);
                let vp = D3D11_VIEWPORT {
                    TopLeftX: self.panel + (bw - *w as f32 * s) / 2.0,
                    TopLeftY: bar_h + (bh - *h as f32 * s) / 2.0,
                    Width: *w as f32 * s,
                    Height: *h as f32 * s,
                    MinDepth: 0.0,
                    MaxDepth: 1.0,
                };
                video = D2D_RECT_F {
                    left: vp.TopLeftX,
                    top: vp.TopLeftY,
                    right: vp.TopLeftX + vp.Width,
                    bottom: vp.TopLeftY + vp.Height,
                };
                self.ctx.RSSetViewports(Some(&[vp]));
                self.ctx.IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                self.ctx.VSSetShader(&self.vs[(self.flip & 3) as usize], None);
                self.ctx.PSSetShader(&self.ps, None);
                self.ctx.PSSetShaderResources(0, Some(&[Some(view.clone())]));
                self.ctx.Draw(3, 0);
            }
            if let Some(c) = self.outline {
                self.draw_outline(video, c)?;
            }
            if let Some((title, text)) = self.notice.clone() {
                self.draw_notice(video, &title, &text)?;
            }
            if let Some(bar) = bar {
                self.draw_bar(bar, bar_h)?;
            } else {
                self.buttons.clear();
            }
            let flags = if self.tearing {
                DXGI_PRESENT_ALLOW_TEARING
            } else {
                DXGI_PRESENT(0)
            };
            if let Some(path) = self.shot.take() {
                if let Err(e) = self.save_bmp(&path) {
                    eprintln!("ERROR shot {}: {e}", path.display());
                }
            }
            self.chain.Present(0, flags).ok()?;
            self.chain.GetLastPresentCount()
        }
    }

    fn fonts(&mut self, h: u32) -> windows::core::Result<&Fonts> {
        if self.fonts.as_ref().map(|f| f.height) != Some(h) {
            let make =
                |size: f32, weight: DWRITE_FONT_WEIGHT, centre: bool| -> windows::core::Result<IDWriteTextFormat> {
                    unsafe {
                        let f = self.dw.CreateTextFormat(
                            w!("Segoe UI"),
                            None,
                            weight,
                            DWRITE_FONT_STYLE_NORMAL,
                            DWRITE_FONT_STRETCH_NORMAL,
                            size.max(8.0),
                            w!("en-us"),
                        )?;
                        f.SetParagraphAlignment(DWRITE_PARAGRAPH_ALIGNMENT_CENTER)?;
                        f.SetWordWrapping(DWRITE_WORD_WRAPPING_NO_WRAP)?;
                        if centre {
                            f.SetTextAlignment(DWRITE_TEXT_ALIGNMENT_CENTER)?;
                        }
                        Ok(f)
                    }
                };
            let hf = h as f32;
            self.fonts = Some(Fonts {
                height: h,
                brand: make(hf * 0.20, DWRITE_FONT_WEIGHT_SEMI_BOLD, false)?,
                label: make(hf * 0.11, DWRITE_FONT_WEIGHT_NORMAL, false)?,
                value: make(hf * 0.20, DWRITE_FONT_WEIGHT_SEMI_BOLD, false)?,
                small: make(hf * 0.12, DWRITE_FONT_WEIGHT_NORMAL, false)?,
                button: make(hf * 0.13, DWRITE_FONT_WEIGHT_NORMAL, true)?,
            });
        }
        Ok(self.fonts.as_ref().unwrap())
    }

    /// Width of `text` in `format`, in pixels.
    fn text_width(&self, text: &[u16], format: &IDWriteTextFormat) -> f32 {
        unsafe {
            let Ok(layout) = self.dw.CreateTextLayout(text, format, 100000.0, 1000.0) else {
                return 0.0;
            };
            let mut m = DWRITE_TEXT_METRICS::default();
            if layout.GetMetrics(&mut m).is_err() {
                return 0.0;
            }
            m.widthIncludingTrailingWhitespace
        }
    }

    /// The link warning: a frame inside the edge of the video (so it covers no bar and no panel).
    fn draw_outline(&mut self, r: D2D_RECT_F, c: [f32; 3]) -> windows::core::Result<()> {
        let Some(target) = self.target.clone() else {
            return Ok(());
        };
        let t = ((r.bottom - r.top) * 0.012).max(6.0);
        unsafe {
            let dc = self.dc.clone();
            dc.SetTarget(&target);
            dc.BeginDraw();
            let brush = dc.CreateSolidColorBrush(
                &D2D1_COLOR_F {
                    r: c[0],
                    g: c[1],
                    b: c[2],
                    a: 1.0,
                },
                None,
            )?;
            let inner = D2D_RECT_F {
                left: r.left + t / 2.0,
                top: r.top + t / 2.0,
                right: r.right - t / 2.0,
                bottom: r.bottom - t / 2.0,
            };
            dc.DrawRectangle(&inner, &brush, t, None);
            dc.EndDraw(None, None)?;
            dc.SetTarget(None);
        }
        Ok(())
    }

    /// "Why is there no picture?": a dark box in the middle of the video (over the last picture), the title in the warning
    /// colour and the explanation below, wrapped. Sized from the video's height, so it reads the same on any screen.
    fn draw_notice(&mut self, r: D2D_RECT_F, title: &str, text: &str) -> windows::core::Result<()> {
        let Some(target) = self.target.clone() else {
            return Ok(());
        };
        let (vw, vh) = (r.right - r.left, r.bottom - r.top);
        let box_w = (vw * 0.62).max(vh * 0.9).min(vw - 16.0).max(1.0);
        let pad = (vh * 0.035).max(12.0);
        let u = |t: &str| -> Vec<u16> { t.encode_utf16().collect() };
        unsafe {
            let fmt = |size: f32, weight: DWRITE_FONT_WEIGHT| -> windows::core::Result<IDWriteTextFormat> {
                let f = self.dw.CreateTextFormat(
                    w!("Segoe UI"),
                    None,
                    weight,
                    DWRITE_FONT_STYLE_NORMAL,
                    DWRITE_FONT_STRETCH_NORMAL,
                    size.max(11.0),
                    w!("en-us"),
                )?;
                f.SetTextAlignment(DWRITE_TEXT_ALIGNMENT_CENTER)?;
                f.SetWordWrapping(DWRITE_WORD_WRAPPING_WRAP)?;
                Ok(f)
            };
            let tf = fmt(vh * 0.050, DWRITE_FONT_WEIGHT_SEMI_BOLD)?;
            let bf = fmt(vh * 0.030, DWRITE_FONT_WEIGHT_NORMAL)?;
            let inner = box_w - 2.0 * pad;
            let measure = |t: &[u16], f: &IDWriteTextFormat| -> windows::core::Result<(IDWriteTextLayout, f32)> {
                let l = self.dw.CreateTextLayout(t, f, inner, 10000.0)?;
                let mut m = DWRITE_TEXT_METRICS::default();
                l.GetMetrics(&mut m)?;
                Ok((l, m.height))
            };
            let (tl, th) = measure(&u(title), &tf)?;
            let (bl, bh) = measure(&u(text), &bf)?;
            let gap = vh * 0.018;
            let box_h = pad + th + gap + bh + pad;
            let left = r.left + (vw - box_w) / 2.0;
            let top = r.top + (vh - box_h) / 2.0;
            let dc = self.dc.clone();
            dc.SetTarget(&target);
            dc.BeginDraw();
            let bg = dc.CreateSolidColorBrush(
                &D2D1_COLOR_F {
                    r: 0.047,
                    g: 0.086,
                    b: 0.125,
                    a: 0.86,
                },
                None,
            )?; // #0C1620
            let edge = dc.CreateSolidColorBrush(
                &D2D1_COLOR_F {
                    r: 0.949,
                    g: 0.722,
                    b: 0.294,
                    a: 1.0,
                },
                None,
            )?; // #F2B84B
            let ink = dc.CreateSolidColorBrush(
                &D2D1_COLOR_F {
                    r: 0.969,
                    g: 0.976,
                    b: 0.973,
                    a: 1.0,
                },
                None,
            )?;
            let rect = D2D_RECT_F {
                left,
                top,
                right: left + box_w,
                bottom: top + box_h,
            };
            dc.FillRectangle(&rect, &bg);
            dc.DrawRectangle(&rect, &edge, (vh * 0.004).max(2.0), None);
            dc.DrawTextLayout(
                windows_numerics::Vector2 {
                    X: left + pad,
                    Y: top + pad,
                },
                &tl,
                &edge,
                D2D1_DRAW_TEXT_OPTIONS_NONE,
            );
            dc.DrawTextLayout(
                windows_numerics::Vector2 {
                    X: left + pad,
                    Y: top + pad + th + gap,
                },
                &bl,
                &ink,
                D2D1_DRAW_TEXT_OPTIONS_NONE,
            );
            dc.EndDraw(None, None)?;
            dc.SetTarget(None);
        }
        Ok(())
    }

    fn draw_bar(&mut self, bar: &Bar, h: f32) -> windows::core::Result<()> {
        let Some(target) = self.target.clone() else {
            return Ok(());
        };
        let width = self.back.0 as f32;
        self.fonts(h as u32)?;
        let f = self.fonts.as_ref().unwrap();
        let (brand, label, value, small, button) = (
            f.brand.clone(),
            f.label.clone(),
            f.value.clone(),
            f.small.clone(),
            f.button.clone(),
        );
        let u = |t: &str| -> Vec<u16> { t.encode_utf16().collect() };
        let rect = |l: f32, t: f32, r: f32, b: f32| RECT {
            left: l as i32,
            top: t as i32,
            right: r as i32,
            bottom: b as i32,
        };
        unsafe {
            let dc = self.dc.clone();
            dc.SetTarget(&target);
            dc.BeginDraw();
            let col = |r: f32, g: f32, b: f32| D2D1_COLOR_F { r, g, b, a: 1.0 };
            let bg = dc.CreateSolidColorBrush(&col(0.047, 0.086, 0.125), None)?; // #0C1620
            let accent = dc.CreateSolidColorBrush(&col(0.561, 0.776, 0.898), None)?; // #8FC6E5
            let accent_soft = dc.CreateSolidColorBrush(&col(0.086, 0.188, 0.247), None)?; // #16303F
            let ink = dc.CreateSolidColorBrush(&col(0.047, 0.086, 0.125), None)?;
            let grey = dc.CreateSolidColorBrush(&col(0.50, 0.584, 0.635), None)?; // #7F95A2
            let muted = dc.CreateSolidColorBrush(&col(0.71, 0.773, 0.808), None)?; // #B5C5CE
            let white = dc.CreateSolidColorBrush(&col(0.969, 0.976, 0.973), None)?;
            let warn = dc.CreateSolidColorBrush(&col(0.949, 0.722, 0.294), None)?; // #F2B84B
            let ok = dc.CreateSolidColorBrush(&col(0.435, 0.812, 0.592), None)?; // #6FCF97
            let rec = dc.CreateSolidColorBrush(&col(1.0, 0.541, 0.494), None)?; // #FF8A7E
            let rec_bg = dc.CreateSolidColorBrush(&col(0.227, 0.098, 0.082), None)?; // #3A1915
            let btn_bg = dc.CreateSolidColorBrush(&col(0.082, 0.153, 0.2), None)?; // #152733
            let btn_edge = dc.CreateSolidColorBrush(&col(0.165, 0.247, 0.302), None)?; // #2A3F4D
            dc.FillRectangle(
                &D2D_RECT_F {
                    left: 0.0,
                    top: 0.0,
                    right: width,
                    bottom: h,
                },
                &bg,
            );
            let pad = h * 0.16;
            let (r1t, r1b) = (h * 0.12, h * 0.56); // first line
            let (r2t, r2b) = (h * 0.58, h * 0.86); // detail line
            let text =
                |t: &[u16], fmt: &IDWriteTextFormat, x: f32, top: f32, bottom: f32, brush: &ID2D1SolidColorBrush| {
                    dc.DrawText(
                        t,
                        fmt,
                        &D2D_RECT_F {
                            left: x,
                            top,
                            right: x + 100000.0,
                            bottom,
                        },
                        brush,
                        D2D1_DRAW_TEXT_OPTIONS_NONE,
                        DWRITE_MEASURING_MODE_NATURAL,
                    );
                };
            // a button: (fill, edge, text) styles by kind: 0 plain, 1 primary (accent), 2 recording (red)
            let mut buttons: Vec<(u32, RECT)> = Vec::new();
            let mut draw_button = |id: u32, name: &str, x_right: f32, kind: u8, me: &Self| -> f32 {
                let t = u(name);
                let bw = me.text_width(&t, &button) + h * 0.26;
                let r = D2D_RECT_F {
                    left: x_right - bw,
                    top: r1t + h * 0.03,
                    right: x_right,
                    bottom: r1b - h * 0.03,
                };
                let rr = D2D1_ROUNDED_RECT {
                    rect: r,
                    radiusX: h * 0.04,
                    radiusY: h * 0.04,
                };
                let (fill, edge, fg) = match kind {
                    1 => (&accent, &accent, &ink),
                    2 => (&rec_bg, &rec, &rec),
                    3 => (&accent_soft, &accent, &accent),
                    _ => (&btn_bg, &btn_edge, &white),
                };
                dc.FillRoundedRectangle(&rr, fill);
                dc.DrawRoundedRectangle(&rr, edge, 1.0, None);
                dc.DrawText(
                    &t,
                    &button,
                    &r,
                    fg,
                    D2D1_DRAW_TEXT_OPTIONS_NONE,
                    DWRITE_MEASURING_MODE_NATURAL,
                );
                buttons.push((id, rect(r.left, r.top, r.right, r.bottom)));
                r.left - h * 0.08
            };
            // brand
            let mut x = pad;
            let t = u("HORIZON");
            text(&t, &brand, x, r1t, r1b, &yellow_of(&dc)?);
            x += self.text_width(&t, &brand) + h * 0.28;
            if let Some(rp) = &bar.replay {
                // replaying a session
                let chip = u("REPLAY");
                let cw = self.text_width(&chip, &label) + h * 0.18;
                let cr = D2D_RECT_F {
                    left: x,
                    top: r1t + h * 0.09,
                    right: x + cw,
                    bottom: r1b - h * 0.09,
                };
                dc.FillRoundedRectangle(
                    &D2D1_ROUNDED_RECT {
                        rect: cr,
                        radiusX: h * 0.03,
                        radiusY: h * 0.03,
                    },
                    &accent_soft,
                );
                text(&chip, &label, x + h * 0.09, r1t, r1b, &accent);
                x += cw + h * 0.16;
                let tt = u(&rp.title);
                text(&tt, &value, x, r1t, r1b, &muted);
                x += self.text_width(&tt, &value) + h * 0.24;
                // transport, left to right: back, play/pause, forward, speed, then the time
                let mut xr = x;
                for (id, name, kind) in [
                    (BTN_BACK, "−10 s", 0u8),
                    (BTN_PLAY, if rp.playing { "Pause" } else { "Play" }, 1),
                    (BTN_FWD, "+10 s", 0),
                    (BTN_SPEED, rp.speed.as_str(), 0),
                ] {
                    let t = u(name);
                    let bw = self.text_width(&t, &button) + h * 0.26;
                    let next = draw_button(id, name, xr + bw, kind, self);
                    let _ = next;
                    xr += bw + h * 0.08;
                }
                let tm = u(&rp.time);
                text(&tm, &value, xr + h * 0.1, r1t, r1b, &white);
                // right side: back to live, and the live link meanwhile
                let mut right = width - pad;
                right = draw_button(BTN_LIVE, "Back to live (Esc)", right, 3, self);
                let lv = u(&rp.live);
                let lw = self.text_width(&lv, &small);
                text(&lv, &small, right - lw - h * 0.06, r1t, r1b, &ok);
                text(&u(&bar.detail), &small, pad, r2t, r2b, &grey);
                // progress strip along the bottom edge, with the problems marked
                let (st, sb) = (h - h * 0.05, h);
                dc.FillRectangle(
                    &D2D_RECT_F {
                        left: 0.0,
                        top: st,
                        right: width,
                        bottom: sb,
                    },
                    &btn_bg,
                );
                dc.FillRectangle(
                    &D2D_RECT_F {
                        left: 0.0,
                        top: st,
                        right: width * rp.progress.clamp(0.0, 1.0) as f32,
                        bottom: sb,
                    },
                    &accent,
                );
                for m in &rp.marks {
                    let mx = width * (*m as f32);
                    dc.FillRectangle(
                        &D2D_RECT_F {
                            left: mx - 1.5,
                            top: st - h * 0.04,
                            right: mx + 1.5,
                            bottom: sb,
                        },
                        &warn,
                    );
                }
                buttons.push((BTN_SEEK, rect(0.0, h - h * 0.12, width, h)));
            } else {
                // live
                for (l, v) in &bar.stats {
                    let (lt, vt) = (u(&l.to_uppercase()), u(v));
                    text(&lt, &label, x, r1t, r1b, &muted);
                    x += self.text_width(&lt, &label) + h * 0.05;
                    text(&vt, &value, x, r1t, r1b, &white);
                    x += self.text_width(&vt, &value) + h * 0.22;
                }
                if let Some(r) = &bar.rec {
                    let rt = u(r);
                    let rw = self.text_width(&rt, &label) + h * 0.36;
                    let cr = D2D_RECT_F {
                        left: x,
                        top: r1t + h * 0.09,
                        right: x + rw,
                        bottom: r1b - h * 0.09,
                    };
                    dc.FillRoundedRectangle(
                        &D2D1_ROUNDED_RECT {
                            rect: cr,
                            radiusX: h * 0.03,
                            radiusY: h * 0.03,
                        },
                        &rec_bg,
                    );
                    let cy = (r1t + r1b) / 2.0;
                    dc.FillEllipse(
                        &D2D1_ELLIPSE {
                            point: windows_numerics::Vector2 { X: x + h * 0.11, Y: cy },
                            radiusX: h * 0.045,
                            radiusY: h * 0.045,
                        },
                        &rec,
                    );
                    text(&rt, &label, x + h * 0.2, r1t, r1b, &rec);
                }
                let mut right = width - pad;
                for (id, name, kind) in [
                    (BTN_QUIT, "Quit (Q)", 0u8),
                    (BTN_VIDEO, "Video only (H)", 0),
                    (BTN_SETTINGS, "Settings (S)", 0),
                ] {
                    right = draw_button(id, name, right, kind, self);
                }
                let recording = bar.rec.is_some();
                draw_button(
                    BTN_REC,
                    if recording { "Stop recording (R)" } else { "Record (R)" },
                    right,
                    if recording { 2 } else { 0 },
                    self,
                );
                text(
                    &u(&bar.detail),
                    &small,
                    pad,
                    r2t,
                    r2b,
                    if bar.warn { &warn } else { &grey },
                );
            }
            dc.EndDraw(None, None)?;
            dc.SetTarget(None);
            self.buttons = buttons;
        }
        Ok(())
    }

    /// How the last frames reached the screen: "composed" (by the desktop window manager: a refresh or two
    /// later), "overlay" or "direct" (independent flip: no composition step).
    pub fn mode(&self) -> &'static str {
        let Ok(m) = self.chain.cast::<IDXGISwapChainMedia>() else {
            return "unknown";
        };
        let mut st = DXGI_FRAME_STATISTICS_MEDIA::default();
        if unsafe { m.GetFrameStatisticsMedia(&mut st) }.is_err() {
            return "unknown";
        }
        match st.CompositionMode {
            DXGI_FRAME_PRESENTATION_MODE_COMPOSED => "composed",
            DXGI_FRAME_PRESENTATION_MODE_OVERLAY => "overlay",
            DXGI_FRAME_PRESENTATION_MODE_NONE => "direct",
            DXGI_FRAME_PRESENTATION_MODE_COMPOSITION_FAILURE => "composition failure",
            _ => "unknown",
        }
    }

    pub fn back(&self) -> (u32, u32) {
        self.back
    }
}

/// The Horizon star's yellow (#FDF100), for the brand name.
fn yellow_of(dc: &ID2D1DeviceContext) -> windows::core::Result<ID2D1SolidColorBrush> {
    unsafe {
        dc.CreateSolidColorBrush(
            &D2D1_COLOR_F {
                r: 0.992,
                g: 0.945,
                b: 0.0,
                a: 1.0,
            },
            None,
        )
    }
}
