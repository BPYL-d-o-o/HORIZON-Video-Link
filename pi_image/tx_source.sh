#!/bin/sh
# Show or choose what the TX service (horizon_vtx) sends: the camera, or a test video looped.
#
#   pi_image/tx_source.sh                 show the current source
#   pi_image/tx_source.sh camera          send the camera
#   pi_image/tx_source.sh ~/videos/a.mp4  send a test video (any file ffmpeg reads, or a raw I420 .yuv)
#
# The choice is kept in /etc/default/horizon_vtx (--source in HORIZON_VTX_ARGS) and the service is restarted.
# A video is converted once to the encode size and frame rate on its first use (cached in ~/.cache/horizon),
# so the first start with a new file takes a while; later runs start at once and send the same frames.
# The path may not contain spaces (HORIZON_VTX_ARGS is split on whitespace).
set -eu
conf=/etc/default/horizon_vtx
cur=$(sed -n 's/^HORIZON_VTX_ARGS="\(.*\)"$/\1/p' "$conf" 2>/dev/null || true)
src=$(printf '%s\n' "$cur" | sed -n 's/.*--source \([^ ]*\).*/\1/p')

if [ $# -eq 0 ]; then
    echo "source: ${src:-camera}"
    exit 0
fi

rest=$(printf '%s\n' "$cur" | sed 's/ *--source [^ ]*//; s/^ *//')
case "$1" in
    camera)
        new=$rest
        echo "source: camera" ;;
    *)
        f=$(readlink -f -- "$1") || { echo "ERROR no such file: $1" >&2; exit 1; }
        [ -f "$f" ] || { echo "ERROR no such file: $1" >&2; exit 1; }
        case "$f" in *" "*) echo "ERROR the path may not contain spaces: $f" >&2; exit 1 ;; esac
        new="${rest:+$rest }--source $f"
        echo "source: $f" ;;
esac
sudo sed -i "s|^HORIZON_VTX_ARGS=.*|HORIZON_VTX_ARGS=\"$new\"|" "$conf"
grep '^HORIZON_VTX_ARGS=' "$conf"
sudo systemctl restart horizon_vtx
echo "restarted horizon_vtx (watch: journalctl -u horizon_vtx -f)"
