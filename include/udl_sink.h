#ifndef UDL_SINK_H
#define UDL_SINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum udl_sink_result {
    UDL_SINK_OK = 0,
    UDL_SINK_ERR_INVALID_ARGUMENT,
    UDL_SINK_ERR_NO_MEMORY,
    UDL_SINK_ERR_TRUNCATED_COMMAND,
    UDL_SINK_ERR_UNSUPPORTED_COMMAND,
    UDL_SINK_ERR_INVALID_COMMAND,
    UDL_SINK_ERR_ADDRESS_RANGE,
};

struct udl_sink_damage {
    bool touched;
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
    uint32_t pixel_count;
};

struct udl_sink {
    uint16_t *framebuffer;
    uint32_t width;
    uint32_t height;
    uint32_t stride_pixels;
    uint8_t registers[256];
    uint32_t *framebuffer_xrgb8888;
    uint32_t stride_pixels_xrgb8888;
    uint16_t *plane16;
    uint8_t *plane8;
    uint32_t plane_pixels;
};

void udl_sink_init(struct udl_sink *sink,
                   uint16_t *framebuffer,
                   uint32_t width,
                   uint32_t height,
                   uint32_t stride_pixels);

void udl_sink_destroy(struct udl_sink *sink);

void udl_sink_attach_xrgb8888_output(struct udl_sink *sink,
                                     uint32_t *framebuffer,
                                     uint32_t stride_pixels);

void udl_sink_clear_damage(struct udl_sink_damage *damage);

enum udl_sink_result udl_sink_decode_buffer(struct udl_sink *sink,
                                            const uint8_t *buffer,
                                            size_t length,
                                            struct udl_sink_damage *damage);

uint8_t udl_sink_get_color_depth(const struct udl_sink *sink);
uint16_t udl_sink_get_hpixels(const struct udl_sink *sink);
uint16_t udl_sink_get_vpixels(const struct udl_sink *sink);
uint32_t udl_sink_get_base16bpp(const struct udl_sink *sink);
uint32_t udl_sink_get_base8bpp(const struct udl_sink *sink);
const char *udl_sink_result_string(enum udl_sink_result result);

#ifdef __cplusplus
}
#endif

#endif