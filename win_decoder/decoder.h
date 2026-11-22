/* Minimal H.265 decoder interface used by the receiver. */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Decoder Decoder;

Decoder *decoder_new(void);
void decoder_free(Decoder *decoder);
/* One access unit in Annex-B format. Returns a negative value on error. */
int decoder_send(Decoder *decoder, const uint8_t *data, int size);
/* Returns 1 and the picture size when a new picture is ready. */
int decoder_receive(Decoder *decoder, int *width, int *height);
/* Convert the newest picture to BGRA, `pitch` bytes per row. */
void decoder_copy_bgra(const Decoder *decoder, uint8_t *out, int pitch);

#ifdef __cplusplus
}
#endif
