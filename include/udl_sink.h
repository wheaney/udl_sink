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

enum udl_transport_resync_reason {
    UDL_TRANSPORT_RESYNC_NONE = 0,
    UDL_TRANSPORT_RESYNC_NON_BULK,
    UDL_TRANSPORT_RESYNC_DOUBLE_BULK,
    UDL_TRANSPORT_RESYNC_INVALID_COMMAND,
};

enum udl_transport_error_stage {
    UDL_TRANSPORT_ERROR_NONE = 0,
    UDL_TRANSPORT_ERROR_NON_BULK,
    UDL_TRANSPORT_ERROR_DOUBLE_BULK,
    UDL_TRANSPORT_ERROR_WRITERLX16_PARSE,
    UDL_TRANSPORT_ERROR_WRITECOMP_PROBE,
    UDL_TRANSPORT_ERROR_WRITECOMP_DECODE,
    UDL_TRANSPORT_ERROR_COMMAND_PARSE,
    UDL_TRANSPORT_ERROR_COMMAND_DECODE,
    UDL_TRANSPORT_ERROR_COMMAND_CONSUMED_MISMATCH,
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

struct udl_sink_device_table_branch {
    bool is_reference;
    uint16_t reference_state;
    int16_t diff16;
    uint16_t continuation_state;
    uint8_t aux_bits;
    uint8_t meta_bits;
};

struct udl_sink {
    uint16_t *framebuffer;
    uint32_t width;
    uint32_t height;
    uint32_t stride_pixels;
    uint8_t registers[256];
    uint32_t *framebuffer_xrgb8888;
    uint32_t stride_pixels_xrgb8888;
    uint32_t *rgb565_to_xrgb8888_lookup;
    uint16_t *plane16;
    uint8_t *plane8;
    uint32_t plane_pixels;
    uint8_t huffman_device_table[4608];
    uint64_t huffman_device_table_signature;
    bool huffman_device_table_loaded;
    bool huffman_device_table_matches_static;
    bool writecomp_live_state_valid8;
    bool writecomp_live_state_valid16;
    bool writecomp_live_state_valid_global;
    bool writecomp_live_pixel_valid8;
    bool writecomp_live_pixel_valid16;
    bool writecomp_live_pixel_valid_global;
    uint16_t writecomp_live_state8;
    uint16_t writecomp_live_state16;
    uint16_t writecomp_live_state_global;
    uint8_t writecomp_live_pixel8;
    uint16_t writecomp_live_pixel16;
    uint16_t writecomp_live_pixel_global;
    bool last_writecomp_error_valid;
    uint8_t last_writecomp_error_command;
    uint32_t last_writecomp_error_pixel_index;
    size_t last_writecomp_error_byte_offset;
    uint8_t last_writecomp_error_bit_offset;
    uint16_t last_writecomp_error_state;
    int last_writecomp_error_result;
    bool last_writecomp_live_probe_valid;
    uint16_t last_writecomp_live_probe_successes;
    bool last_writecomp_static_probe_valid;
    bool last_writecomp_static_probe_matches_decoder;
    int last_writecomp_static_probe_result;
    size_t last_writecomp_static_probe_end_byte_offset;
    uint8_t last_writecomp_static_probe_end_bit_offset;
    uint32_t last_writecomp_static_probe_invalid_node;
    uint8_t last_writecomp_static_probe_invalid_bit;
    uint32_t last_writecomp_static_probe_invalid_next;
};

struct udl_transport {
    struct udl_sink *sink;
    uint8_t *pending;
    size_t pending_len;
    size_t pending_capacity;
    size_t last_feed_pending_prefix_len;
    size_t last_feed_first_resync_offset;
    bool collect_detailed_stats;
    bool collect_writerlx16_span_stats;
    bool last_feed_first_resync_offset_valid;
    bool last_feed_first_error_valid;
    bool last_feed_first_error_writecomp_valid;
    bool writecomp_quarantine_active;
    uint8_t writecomp_quarantine_noncomp_ok;
    uint8_t last_feed_first_error_command_type;
    uint8_t last_feed_first_error_writecomp_command;
    int last_feed_first_error_sink_result;
    int last_feed_first_error_writecomp_result;
    bool last_feed_first_error_writecomp_live_probe_valid;
    bool last_feed_first_error_writecomp_static_probe_valid;
    bool last_feed_first_error_writecomp_static_probe_matches_decoder;
    uint16_t last_feed_first_error_writecomp_live_probe_successes;
    int last_feed_first_error_writecomp_static_probe_result;
    size_t last_feed_first_error_consumed;
    size_t last_feed_first_error_writecomp_byte_offset;
    size_t last_feed_first_error_writecomp_static_probe_end_byte_offset;
    size_t writecomp_quarantine_budget;
    uint32_t last_feed_first_error_writecomp_pixel_index;
    uint32_t last_feed_first_error_writecomp_static_probe_invalid_node;
    uint32_t last_feed_first_error_writecomp_static_probe_invalid_next;
    enum udl_transport_resync_reason last_feed_first_resync_reason;
    enum udl_transport_error_stage last_feed_first_error_stage;
    uint16_t last_feed_first_error_writecomp_state;
    uint8_t last_feed_first_error_writecomp_bit_offset;
    uint8_t last_feed_first_error_writecomp_static_probe_end_bit_offset;
    uint8_t last_feed_first_error_writecomp_static_probe_invalid_bit;
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

void udl_transport_set_detailed_stats(struct udl_transport *transport,
                                      bool enabled);

void udl_transport_set_writerlx16_span_stats(struct udl_transport *transport,
                                             bool enabled);

void udl_transport_set_writecomp_debug(bool enabled);
void udl_transport_set_writecomp_live_table_decode(bool enabled);
void udl_transport_set_writecomp_live_table_zerokvm(bool enabled);
void udl_transport_set_writecomp_live_table_state_carry(bool enabled);
void udl_transport_set_writecomp_live_table_state_global(bool enabled);
void udl_transport_set_writecomp_live_table_pixel_carry(bool enabled);
void udl_transport_set_writecomp_live_table_diff_low16(bool enabled);
void udl_transport_set_writecomp_live_table_reference_leaf(bool enabled);
void udl_transport_set_writecomp_live_table_no_change_bit(bool enabled);
void udl_transport_set_writecomp_live_table_state_seed(bool enabled, uint16_t seed);
void udl_transport_set_writecomp_live_table_state_seed16(bool enabled, uint16_t seed);
void udl_transport_set_writecomp_live_table_state_seed8(bool enabled, uint16_t seed);
void udl_transport_set_writecomp_hybrid_static_live(bool enabled);
void udl_transport_set_writecomp_trace_target(bool enabled,
                                              uint8_t command_type,
                                              uint32_t pixel_index,
                                              size_t byte_offset,
                                              uint8_t bit_offset);
void udl_transport_set_writecomp_trace_visible_pixel(bool enabled,
                                                     uint32_t visible_pixel_index);
void udl_transport_set_writecomp_trace_first_failure(bool enabled);

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

bool udl_sink_get_device_table_branch(const struct udl_sink *sink,
                                      uint16_t record_index,
                                      bool right_branch,
                                      struct udl_sink_device_table_branch *branch_out);

enum udl_sink_result udl_sink_decode_device_table_diff(const struct udl_sink *sink,
                                                       uint16_t *state_io,
                                                       const uint8_t *bitstream,
                                                       size_t bit_count,
                                                       size_t *bit_offset_io,
                                                       int16_t *diff_out);

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