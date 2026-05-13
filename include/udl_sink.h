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

enum udl_transport_result {
    UDL_TRANSPORT_OK = 0,
    UDL_TRANSPORT_ERR_INVALID_ARGUMENT,
    UDL_TRANSPORT_ERR_NO_MEMORY,
};

struct udl_sink_damage {
    bool touched;
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
    uint32_t pixel_count;
};

struct udl_transport_stats {
    uint64_t decoded_commands;
    uint64_t decode_errors;
    uint64_t dropped_bytes;
    uint64_t writereg_commands;
    uint64_t writereg_redundant_commands;
    uint64_t writeraw8_commands;
    uint64_t writerl8_commands;
    uint64_t writecopy8_commands;
    uint64_t writerlx8_commands;
    uint64_t writeraw16_commands;
    uint64_t writerl16_commands;
    uint64_t writecopy16_commands;
    uint64_t writerlx16_commands;
    uint64_t writerlx16_raw_spans;
    uint64_t writerlx16_repeat_spans;
    uint64_t writerlx16_raw_pixels;
    uint64_t writerlx16_repeat_pixels;
    uint64_t writerlx16_raw_single_pixel_spans;
    uint64_t no_damage_commands;
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

struct udl_transport {
    struct udl_sink *sink;
    uint8_t *pending;
    size_t pending_len;
    size_t pending_capacity;
    struct udl_transport_stats stats;
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

void udl_transport_init(struct udl_transport *transport,
                        struct udl_sink *sink);

void udl_transport_reset(struct udl_transport *transport);

void udl_transport_destroy(struct udl_transport *transport);

enum udl_transport_result udl_transport_feed(struct udl_transport *transport,
                                             const uint8_t *buffer,
                                             size_t length,
                                             struct udl_sink_damage *damage);

struct udl_transport_stats udl_transport_get_stats(const struct udl_transport *transport);

enum udl_sink_result udl_sink_decode_buffer(struct udl_sink *sink,
                                            const uint8_t *buffer,
                                            size_t length,
                                            struct udl_sink_damage *damage);

uint8_t udl_sink_get_color_depth(const struct udl_sink *sink);
uint16_t udl_sink_get_hpixels(const struct udl_sink *sink);
uint16_t udl_sink_get_vpixels(const struct udl_sink *sink);
uint32_t udl_sink_get_base16bpp(const struct udl_sink *sink);
uint32_t udl_sink_get_base8bpp(const struct udl_sink *sink);
const char *udl_transport_result_string(enum udl_transport_result result);
const char *udl_sink_result_string(enum udl_sink_result result);

#ifdef __cplusplus
}
#endif

#endif