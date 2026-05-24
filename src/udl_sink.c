#include "udl_sink.h"
#include "udl_sink_huffman_table.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

enum {
    UDL_MSG_BULK = 0xafu,
    UDL_CMD_WRITEREG = 0x20u,
    UDL_CMD_UNKNOWN_40 = 0x40u,
    UDL_CMD_NOP_A0 = 0xa0u,
    UDL_CMD_OPAQUE_E0 = 0xe0u,
    UDL_CMD_WRITERAW8 = 0x60u,
    UDL_CMD_WRITERL8 = 0x61u,
    UDL_CMD_WRITECOPY8 = 0x62u,
    UDL_CMD_WRITERLX8 = 0x63u,
    UDL_CMD_WRITERAW16 = 0x68u,
    UDL_CMD_WRITERL16 = 0x69u,
    UDL_CMD_WRITECOPY16 = 0x6au,
    UDL_CMD_WRITERLX16 = 0x6bu,
    UDL_CMD_WRITECOMP8 = 0x70u,
    UDL_CMD_WRITECOMP16 = 0x78u,
    UDL_REG_COLORDEPTH = 0x00u,
    UDL_COLORDEPTH_16BPP = 0u,
    UDL_COLORDEPTH_24BPP = 1u,
    UDL_REG_HPIXELS = 0x0fu,
    UDL_REG_VPIXELS = 0x17u,
    UDL_REG_BASE16BPP_ADDR2 = 0x20u,
    UDL_REG_BASE16BPP_ADDR1 = 0x21u,
    UDL_REG_BASE16BPP_ADDR0 = 0x22u,
    UDL_REG_BASE8BPP_ADDR2 = 0x26u,
    UDL_REG_BASE8BPP_ADDR1 = 0x27u,
    UDL_REG_BASE8BPP_ADDR0 = 0x28u,
    UDL_MAX_COMMAND_PIXELS = 256u,
    UDL_CMD_UNKNOWN_40_SIZE = 3u,
    UDL_CMD_UNKNOWN_40_ARG = 0x0bu,
    UDL_CMD_E0_HEADER_SIZE = 9u,
    UDL_CMD_E0_RECORD_SIZE = 9u,
    UDL_CMD_E0_RECORD_COUNT = 512u,
    UDL_CMD_E0_TOTAL_SIZE = UDL_CMD_E0_HEADER_SIZE + (UDL_CMD_E0_RECORD_SIZE * UDL_CMD_E0_RECORD_COUNT),
    UDL_CMD_E0_TRAILER_SIZE = 1u,
    UDL_HUFFMAN_DIFF_OFFSET = 1 << 15,
    UDL_HUFFMAN_CODE_COUNT = (UDL_HUFFMAN_DIFF_OFFSET * 2) + 1,
    UDL_WRITECOMP_QUARANTINE_BYTES = 16384u,
    UDL_WRITECOMP_QUARANTINE_NONCOMP_REQUIRED = 2u,
    UDL_WRITECOMP_PROBE_PIXELS = 16u,
};

struct udl_huffman_code {
    uint8_t size;
    uint32_t bits;
};

struct udl_huffman_node {
    uint32_t child[2];
    int32_t diff;
    bool leaf;
};

struct udl_huffman_tree {
    struct udl_huffman_node *nodes;
    size_t node_count;
    size_t node_capacity;
    bool ready;
};

struct udl_huffman_bitstream {
    const uint8_t *bytes;
    size_t byte_length;
    size_t byte_offset;
    uint8_t bit_offset;
};

static struct udl_huffman_tree udl_huffman_tree;
static bool udl_sink_writecomp_debug_enabled = false;

static size_t udl_transport_e0_command_length(const uint8_t *command, size_t length)
{
    if (!command || length < UDL_CMD_E0_TOTAL_SIZE) {
        return 0u;
    }

    if (length == UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE &&
        command[UDL_CMD_E0_TOTAL_SIZE] != UDL_MSG_BULK) {
        return UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE;
    }

    if (length >= UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE + 1u &&
        command[UDL_CMD_E0_TOTAL_SIZE] != UDL_MSG_BULK &&
        command[UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE] == UDL_MSG_BULK) {
        return UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE;
    }

    return UDL_CMD_E0_TOTAL_SIZE;
}

enum udl_stream_parse_result {
    UDL_STREAM_PARSE_COMPLETE,
    UDL_STREAM_PARSE_NEED_MORE,
    UDL_STREAM_PARSE_INVALID,
};

enum udl_sink_plane {
    UDL_SINK_PLANE_8,
    UDL_SINK_PLANE_16,
};

static void udl_sink_huffman_init_node(struct udl_huffman_node *node)
{
    if (!node) {
        return;
    }

    node->child[0] = UINT32_MAX;
    node->child[1] = UINT32_MAX;
    node->diff = 0;
    node->leaf = false;
}

static bool udl_sink_huffman_reserve_nodes(struct udl_huffman_tree *tree,
                                           size_t required)
{
    size_t new_capacity;
    struct udl_huffman_node *new_nodes;

    if (!tree) {
        return false;
    }

    if (required <= tree->node_capacity) {
        return true;
    }

    new_capacity = tree->node_capacity ? tree->node_capacity : 1024u;
    while (new_capacity < required) {
        if (new_capacity > SIZE_MAX / 2u) {
            new_capacity = required;
            break;
        }

        new_capacity *= 2u;
    }

    new_nodes = realloc(tree->nodes, new_capacity * sizeof(*new_nodes));
    if (!new_nodes) {
        return false;
    }

    tree->nodes = new_nodes;
    while (tree->node_capacity < new_capacity) {
        udl_sink_huffman_init_node(&tree->nodes[tree->node_capacity]);
        tree->node_capacity += 1u;
    }

    return true;
}

static bool udl_sink_huffman_insert(struct udl_huffman_tree *tree,
                                    uint32_t bits,
                                    uint8_t size,
                                    int32_t diff)
{
    uint32_t node_index = 0u;
    uint8_t bit_index;

    if (!tree || size == 0u) {
        return false;
    }

    for (bit_index = 0u; bit_index < size; ++bit_index) {
        const uint32_t bit = (bits >> bit_index) & 0x01u;
        uint32_t next_index = tree->nodes[node_index].child[bit];

        if (next_index == UINT32_MAX) {
            next_index = (uint32_t)tree->node_count;
            if (!udl_sink_huffman_reserve_nodes(tree, tree->node_count + 1u)) {
                return false;
            }

            udl_sink_huffman_init_node(&tree->nodes[next_index]);
            tree->nodes[node_index].child[bit] = next_index;
            tree->node_count += 1u;
        }

        node_index = next_index;
    }

    if (tree->nodes[node_index].leaf) {
        /* Tubecable's packed table contains a few alias codewords.
         * Keep the first mapping so decoding remains deterministic. */
        return true;
    }

    tree->nodes[node_index].leaf = true;
    tree->nodes[node_index].diff = diff;
    return true;
}

static bool udl_sink_huffman_ensure_tree(void)
{
    size_t index;

    if (udl_huffman_tree.ready) {
        return true;
    }

    if (!udl_sink_huffman_reserve_nodes(&udl_huffman_tree, 1u)) {
        return false;
    }

    if (udl_huffman_tree.node_count == 0u) {
        udl_sink_huffman_init_node(&udl_huffman_tree.nodes[0]);
        udl_huffman_tree.node_count = 1u;
    }

    if ((size_t)udl_sink_huffman_table_len < (size_t)UDL_HUFFMAN_CODE_COUNT * 5u) {
        return false;
    }

    for (index = 0u; index < UDL_HUFFMAN_CODE_COUNT; ++index) {
        const uint8_t *entry = &udl_sink_huffman_table[index * 5u];
        const uint8_t size = entry[0];
        const uint32_t bits = ((uint32_t)entry[1] << 24) |
                              ((uint32_t)entry[2] << 16) |
                              ((uint32_t)entry[3] << 8) |
                              (uint32_t)entry[4];
        const int32_t diff = (int32_t)index - UDL_HUFFMAN_DIFF_OFFSET;

        if (size == 0u) {
            return false;
        }

        if (!udl_sink_huffman_insert(&udl_huffman_tree, bits, size, diff)) {
            return false;
        }
    }

    udl_huffman_tree.ready = true;
    return true;
}

static size_t udl_sink_huffman_bytes_consumed(const struct udl_huffman_bitstream *bitstream)
{
    if (!bitstream) {
        return 0u;
    }

    return bitstream->byte_offset + (bitstream->bit_offset != 0u ? 1u : 0u);
}

static enum udl_sink_result udl_sink_huffman_read_bit(struct udl_huffman_bitstream *bitstream,
                                                      uint8_t *bit_out)
{
    uint8_t bit;

    if (!bitstream || !bit_out) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }

    if (bitstream->byte_offset >= bitstream->byte_length) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    bit = (uint8_t)((bitstream->bytes[bitstream->byte_offset] >> bitstream->bit_offset) & 0x01u);
    bitstream->bit_offset += 1u;
    if (bitstream->bit_offset == 8u) {
        bitstream->bit_offset = 0u;
        bitstream->byte_offset += 1u;
    }

    *bit_out = bit;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_huffman_read_diff(struct udl_huffman_bitstream *bitstream,
                                                       int32_t *diff_out)
{
    uint32_t node_index = 0u;

    if (!bitstream || !diff_out) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }

    if (!udl_sink_huffman_ensure_tree()) {
        return UDL_SINK_ERR_NO_MEMORY;
    }

    while (!udl_huffman_tree.nodes[node_index].leaf) {
        uint8_t bit;
        uint32_t next_index;

        {
            enum udl_sink_result bit_result = udl_sink_huffman_read_bit(bitstream, &bit);

            if (bit_result != UDL_SINK_OK) {
                return bit_result;
            }
        }

        next_index = udl_huffman_tree.nodes[node_index].child[bit];
        if (next_index == UINT32_MAX || next_index >= udl_huffman_tree.node_count) {
            return UDL_SINK_ERR_INVALID_COMMAND;
        }

        node_index = next_index;
    }

    *diff_out = udl_huffman_tree.nodes[node_index].diff;
    return UDL_SINK_OK;
}

static bool udl_sink_fast_path_16bpp_enabled(const struct udl_sink *sink);
static uint32_t udl_sink_rgb565_to_xrgb8888_fast(const struct udl_sink *sink, uint16_t pixel16);
static void udl_sink_mark_span(struct udl_sink_damage *damage, uint32_t x, uint32_t y, uint32_t length);
static enum udl_sink_result udl_sink_map_range(const struct udl_sink *sink,
                                               enum udl_sink_plane plane,
                                               uint32_t byte_address,
                                               uint32_t pixel_count,
                                               uint32_t *first_pixel_out);
static void udl_sink_write_plane16(struct udl_sink *sink,
                                   uint32_t pixel_index,
                                   uint16_t pixel,
                                   struct udl_sink_damage *damage);
static void udl_sink_mark_pixel(struct udl_sink_damage *damage,
                                uint32_t x,
                                uint32_t y);
static uint32_t udl_sink_rgb565_to_xrgb8888(uint16_t pixel16);
static size_t udl_sink_repeat_u16_word(uint16_t pixel);
static bool udl_sink_plane16_span_matches_be(const uint16_t *dst,
                                             const uint8_t *src,
                                             uint32_t pixel_count);
static void udl_sink_fill_plane16(struct udl_sink *sink,
                                  uint32_t first_pixel,
                                  uint32_t pixel_count,
                                  uint16_t pixel,
                                  struct udl_sink_damage *damage);
static void udl_sink_blit_plane16_be(struct udl_sink *sink,
                                     uint32_t first_pixel,
                                     const uint8_t *src,
                                     uint32_t pixel_count,
                                     struct udl_sink_damage *damage);
static void udl_sink_blit_plane16_be_small(struct udl_sink *sink,
                                           uint32_t first_pixel,
                                           const uint8_t *src,
                                           uint32_t pixel_count,
                                           struct udl_sink_damage *damage);

static uint16_t udl_sink_read_be16(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static uint16_t udl_sink_read_reg16(const struct udl_sink *sink, uint8_t reg)
{
    return (uint16_t)(((uint16_t)sink->registers[reg] << 8) |
                      sink->registers[(uint8_t)(reg + 1u)]);
}

static uint32_t udl_sink_read_addr24(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 16) |
           ((uint32_t)bytes[1] << 8) |
           (uint32_t)bytes[2];
}

static void udl_sink_blit_plane16_be_tiny(struct udl_sink *sink,
                                          uint32_t first_pixel,
                                          const uint8_t *src,
                                          uint32_t pixel_count,
                                          struct udl_sink_damage *damage)
{
    if (!udl_sink_fast_path_16bpp_enabled(sink)) {
        uint32_t index;

        for (index = 0u; index < pixel_count; ++index) {
            udl_sink_write_plane16(sink,
                                   first_pixel + index,
                                   udl_sink_read_be16(&src[(size_t)index * 2u]),
                                   damage);
        }
        return;
    }

    {
        const uint32_t x = first_pixel % sink->width;
        const uint32_t y = first_pixel / sink->width;
        uint16_t *row_plane16 = sink->plane16 + first_pixel;
        uint16_t *row_fb16 = sink->framebuffer
            ? sink->framebuffer + ((size_t)y * sink->stride_pixels) + x
            : NULL;
        uint32_t *row_fb32 = sink->framebuffer_xrgb8888
            ? sink->framebuffer_xrgb8888 + ((size_t)y * sink->stride_pixels_xrgb8888) + x
            : NULL;
        bool changed[4] = {false, false, false, false};
        uint32_t index;

        for (index = 0u; index < pixel_count; ++index) {
            const uint16_t pixel = udl_sink_read_be16(&src[(size_t)index * 2u]);

            if (row_plane16[index] == pixel) {
                continue;
            }

            row_plane16[index] = pixel;
            if (row_fb16) {
                row_fb16[index] = pixel;
            }
            if (row_fb32) {
                row_fb32[index] = udl_sink_rgb565_to_xrgb8888_fast(sink, pixel);
            }

            changed[index] = true;
        }

        index = 0u;
        while (index < pixel_count) {
            uint32_t run_start;

            if (!changed[index]) {
                index += 1u;
                continue;
            }

            run_start = index;
            do {
                index += 1u;
            } while (index < pixel_count && changed[index]);

            udl_sink_mark_span(damage, x + run_start, y, index - run_start);
        }
    }
}

static void udl_sink_blit_plane16_be_small(struct udl_sink *sink,
                                           uint32_t first_pixel,
                                           const uint8_t *src,
                                           uint32_t pixel_count,
                                           struct udl_sink_damage *damage)
{
    if (!udl_sink_fast_path_16bpp_enabled(sink)) {
        uint32_t index;

        for (index = 0u; index < pixel_count; ++index) {
            udl_sink_write_plane16(sink,
                                   first_pixel + index,
                                   udl_sink_read_be16(&src[(size_t)index * 2u]),
                                   damage);
        }
        return;
    }

    {
        const uint32_t x = first_pixel % sink->width;
        const uint32_t y = first_pixel / sink->width;
        uint16_t *row_plane16 = sink->plane16 + first_pixel;
        uint16_t *row_fb16 = sink->framebuffer
            ? sink->framebuffer + ((size_t)y * sink->stride_pixels) + x
            : NULL;
        uint32_t *row_fb32 = sink->framebuffer_xrgb8888
            ? sink->framebuffer_xrgb8888 + ((size_t)y * sink->stride_pixels_xrgb8888) + x
            : NULL;
        bool changed[8] = {false, false, false, false, false, false, false, false};
        uint32_t index;

        for (index = 0u; index < pixel_count; ++index) {
            const uint16_t pixel = udl_sink_read_be16(&src[(size_t)index * 2u]);

            if (row_plane16[index] == pixel) {
                continue;
            }

            row_plane16[index] = pixel;
            if (row_fb16) {
                row_fb16[index] = pixel;
            }
            if (row_fb32) {
                row_fb32[index] = udl_sink_rgb565_to_xrgb8888_fast(sink, pixel);
            }

            changed[index] = true;
        }

        index = 0u;
        while (index < pixel_count) {
            uint32_t run_start;

            if (!changed[index]) {
                index += 1u;
                continue;
            }

            run_start = index;
            do {
                index += 1u;
            } while (index < pixel_count && changed[index]);

            udl_sink_mark_span(damage, x + run_start, y, index - run_start);
        }
    }
}

static uint32_t udl_sink_count_from_byte(uint8_t value)
{
    return value == 0 ? UDL_MAX_COMMAND_PIXELS : (uint32_t)value;
}

static void udl_sink_merge_damage(struct udl_sink_damage *dst,
                                  const struct udl_sink_damage *src)
{
    if (!dst || !src || !src->touched) {
        return;
    }

    if (!dst->touched) {
        *dst = *src;
        return;
    }

    if (src->x1 < dst->x1) {
        dst->x1 = src->x1;
    }
    if (src->y1 < dst->y1) {
        dst->y1 = src->y1;
    }
    if (src->x2 > dst->x2) {
        dst->x2 = src->x2;
    }
    if (src->y2 > dst->y2) {
        dst->y2 = src->y2;
    }
    dst->pixel_count += src->pixel_count;
}

static void udl_transport_record_command_type(struct udl_transport_stats *stats,
                                              uint8_t command_type)
{
    if (!stats) {
        return;
    }

    switch (command_type) {
    case UDL_CMD_WRITEREG:
        stats->writereg_commands += 1u;
        break;
    case UDL_CMD_WRITERAW8:
        stats->writeraw8_commands += 1u;
        break;
    case UDL_CMD_WRITERL8:
        stats->writerl8_commands += 1u;
        break;
    case UDL_CMD_WRITECOPY8:
        stats->writecopy8_commands += 1u;
        break;
    case UDL_CMD_WRITERLX8:
        stats->writerlx8_commands += 1u;
        break;
    case UDL_CMD_WRITERAW16:
        stats->writeraw16_commands += 1u;
        break;
    case UDL_CMD_WRITERL16:
        stats->writerl16_commands += 1u;
        break;
    case UDL_CMD_WRITECOPY16:
        stats->writecopy16_commands += 1u;
        break;
    case UDL_CMD_WRITERLX16:
        stats->writerlx16_commands += 1u;
        break;
    default:
        break;
    }
}

static bool udl_transport_is_framebuffer_command(uint8_t command_type)
{
    switch (command_type) {
    case UDL_CMD_WRITERAW8:
    case UDL_CMD_WRITERL8:
    case UDL_CMD_WRITECOPY8:
    case UDL_CMD_WRITERLX8:
    case UDL_CMD_WRITERAW16:
    case UDL_CMD_WRITERL16:
    case UDL_CMD_WRITECOPY16:
    case UDL_CMD_WRITERLX16:
        return true;
    default:
        return false;
    }
}

static bool udl_transport_is_framebuffer_resync_command(uint8_t command_type)
{
    if (udl_transport_is_framebuffer_command(command_type)) {
        return true;
    }

    return command_type == UDL_CMD_WRITECOMP8 ||
           command_type == UDL_CMD_WRITECOMP16;
}

static bool udl_transport_is_known_command_type(uint8_t command_type)
{
    switch (command_type) {
    case UDL_CMD_WRITEREG:
    case UDL_CMD_UNKNOWN_40:
    case UDL_CMD_NOP_A0:
    case UDL_CMD_OPAQUE_E0:
    case UDL_CMD_WRITERAW8:
    case UDL_CMD_WRITERL8:
    case UDL_CMD_WRITECOPY8:
    case UDL_CMD_WRITERLX8:
    case UDL_CMD_WRITERAW16:
    case UDL_CMD_WRITERL16:
    case UDL_CMD_WRITECOPY16:
    case UDL_CMD_WRITERLX16:
    case UDL_CMD_WRITECOMP8:
    case UDL_CMD_WRITECOMP16:
        return true;
    default:
        return false;
    }
}

static enum udl_stream_parse_result udl_transport_next_command_length(const uint8_t *command,
                                                                      size_t length,
                                                                      size_t *command_len_out);

static enum udl_stream_parse_result udl_transport_probe_writecomp_candidate(const uint8_t *command,
                                                                             size_t length)
{
    struct udl_huffman_bitstream bitstream;
    uint32_t pixel_count;
    uint32_t probe_pixels;
    uint32_t index;

    if (!command || length < 2u) {
        return UDL_STREAM_PARSE_INVALID;
    }
    if (command[0] != UDL_MSG_BULK) {
        return UDL_STREAM_PARSE_INVALID;
    }
    if (command[1] != UDL_CMD_WRITECOMP8 && command[1] != UDL_CMD_WRITECOMP16) {
        return UDL_STREAM_PARSE_INVALID;
    }
    if (length < 7u) {
        return UDL_STREAM_PARSE_NEED_MORE;
    }

    pixel_count = udl_sink_count_from_byte(command[5]);
    if (pixel_count == 0u) {
        return UDL_STREAM_PARSE_INVALID;
    }

    probe_pixels = pixel_count;
    if (probe_pixels > UDL_WRITECOMP_PROBE_PIXELS) {
        probe_pixels = UDL_WRITECOMP_PROBE_PIXELS;
    }

    bitstream.bytes = command + 6u;
    bitstream.byte_length = length - 6u;
    bitstream.byte_offset = 0u;
    bitstream.bit_offset = 0u;

    for (index = 0u; index < probe_pixels; ++index) {
        uint8_t change = 0u;
        enum udl_sink_result result = udl_sink_huffman_read_bit(&bitstream, &change);

        if (result != UDL_SINK_OK) {
            if (result == UDL_SINK_ERR_TRUNCATED_COMMAND) {
                return UDL_STREAM_PARSE_NEED_MORE;
            }
            return UDL_STREAM_PARSE_INVALID;
        }

        if (change == 0u) {
            continue;
        }

        {
            int32_t diff = 0;

            result = udl_sink_huffman_read_diff(&bitstream, &diff);
            (void)diff;

            if (result == UDL_SINK_OK) {
                continue;
            }
            if (result == UDL_SINK_ERR_TRUNCATED_COMMAND) {
                return UDL_STREAM_PARSE_NEED_MORE;
            }
            return UDL_STREAM_PARSE_INVALID;
        }
    }

    return UDL_STREAM_PARSE_COMPLETE;
}

static enum udl_stream_parse_result udl_transport_probe_command_candidate(const uint8_t *command,
                                                                          size_t length)
{
    size_t command_len = 0u;

    if (!command || length < 2u || command[0] != UDL_MSG_BULK) {
        return UDL_STREAM_PARSE_INVALID;
    }

    if (command[1] == UDL_CMD_WRITECOMP8 || command[1] == UDL_CMD_WRITECOMP16) {
        return udl_transport_probe_writecomp_candidate(command, length);
    }

    return udl_transport_next_command_length(command, length, &command_len);
}

static bool udl_transport_find_next_command_mode(const uint8_t *buffer,
                                                 size_t length,
                                                 size_t *offset_out,
                                                 bool *needs_more_out,
                                                 bool allow_relaxed)
{
    size_t offset;
    bool saw_partial = false;
    size_t partial_offset = 0u;

    if (!buffer || !offset_out || !needs_more_out) {
        return false;
    }

    *needs_more_out = false;
    *offset_out = 0u;

    for (offset = 0u; offset < length; ++offset) {
        if (buffer[offset] != UDL_MSG_BULK) {
            continue;
        }

        if (offset + 1u >= length) {
            *offset_out = offset;
            *needs_more_out = true;
            return false;
        }

        if (udl_transport_is_known_command_type(buffer[offset + 1u])) {
            enum udl_stream_parse_result probe_result =
                udl_transport_probe_command_candidate(buffer + offset, length - offset);

            if (probe_result == UDL_STREAM_PARSE_COMPLETE) {
                *offset_out = offset;
                return true;
            }

            if (probe_result == UDL_STREAM_PARSE_NEED_MORE && !saw_partial) {
                saw_partial = true;
                partial_offset = offset;
            }
        }
    }

    if (allow_relaxed) {
        /* Strict probing can reject some legitimate compressed starts when
         * the stream has already drifted. As a recovery fallback, accept the
         * next marker with a known opcode and let the decode path validate it.
         */
        for (offset = 0u; offset + 1u < length; ++offset) {
            if (buffer[offset] == UDL_MSG_BULK &&
                udl_transport_is_known_command_type(buffer[offset + 1u])) {
                *offset_out = offset;
                *needs_more_out = false;
                return true;
            }
        }
    }

    if (saw_partial) {
        *offset_out = partial_offset;
        *needs_more_out = true;
    }

    return false;
}

/* Prefer a strong stream anchor when recovering from desync.
 * `af 20 ff xx` is a common command-boundary marker (WRITEREG VIDREG).
 */
static bool udl_transport_find_next_register_anchor(const uint8_t *buffer,
                                                    size_t length,
                                                    size_t *offset_out,
                                                    bool *needs_more_out)
{
    size_t offset;

    if (!buffer || !offset_out || !needs_more_out) {
        return false;
    }

    *offset_out = 0u;
    *needs_more_out = false;

    for (offset = 0u; offset < length; ++offset) {
        size_t anchor_len = 0u;
        enum udl_stream_parse_result parse_result;

        if (buffer[offset] != UDL_MSG_BULK) {
            continue;
        }

        if (offset + 1u >= length) {
            *offset_out = offset;
            *needs_more_out = true;
            return false;
        }

        if (buffer[offset + 1u] != UDL_CMD_WRITEREG) {
            continue;
        }

        if (offset + 4u > length) {
            *offset_out = offset;
            *needs_more_out = true;
            return false;
        }

        if (buffer[offset + 2u] != 0xffu) {
            continue;
        }

        parse_result = udl_transport_next_command_length(buffer + offset,
                                                         length - offset,
                                                         &anchor_len);
        if (parse_result == UDL_STREAM_PARSE_COMPLETE && anchor_len == 4u) {
            *offset_out = offset;
            return true;
        }
        if (parse_result == UDL_STREAM_PARSE_NEED_MORE) {
            *offset_out = offset;
            *needs_more_out = true;
            return false;
        }
    }

    return false;
}

static bool udl_transport_find_next_command(const uint8_t *buffer,
                                            size_t length,
                                            size_t *offset_out,
                                            bool *needs_more_out)
{
    return udl_transport_find_next_command_mode(buffer,
                                                length,
                                                offset_out,
                                                needs_more_out,
                                                true);
}

static bool udl_transport_find_next_command_strict(const uint8_t *buffer,
                                                   size_t length,
                                                   size_t *offset_out,
                                                   bool *needs_more_out)
{
    return udl_transport_find_next_command_mode(buffer,
                                                length,
                                                offset_out,
                                                needs_more_out,
                                                false);
}

static bool udl_transport_find_next_framebuffer_command_strict(const uint8_t *buffer,
                                                               size_t length,
                                                               size_t *offset_out,
                                                               bool *needs_more_out)
{
    size_t offset;
    bool saw_partial = false;
    size_t partial_offset = 0u;

    if (!buffer || !offset_out || !needs_more_out) {
        return false;
    }

    *needs_more_out = false;
    *offset_out = 0u;

    for (offset = 0u; offset < length; ++offset) {
        uint8_t command_type;
        enum udl_stream_parse_result probe_result;

        if (buffer[offset] != UDL_MSG_BULK) {
            continue;
        }
        if (offset + 1u >= length) {
            *offset_out = offset;
            *needs_more_out = true;
            return false;
        }

        command_type = buffer[offset + 1u];
        if (!udl_transport_is_known_command_type(command_type) ||
            !udl_transport_is_framebuffer_resync_command(command_type)) {
            continue;
        }

        probe_result = udl_transport_probe_command_candidate(buffer + offset, length - offset);
        if (probe_result == UDL_STREAM_PARSE_COMPLETE) {
            *offset_out = offset;
            return true;
        }
        if (probe_result == UDL_STREAM_PARSE_NEED_MORE && !saw_partial) {
            saw_partial = true;
            partial_offset = offset;
        }
    }

    if (saw_partial) {
        *offset_out = partial_offset;
        *needs_more_out = true;
    }

    return false;
}

static enum udl_stream_parse_result udl_transport_parse_writerl_length(const uint8_t *command,
                                                                       size_t length,
                                                                       size_t bytes_per_pixel,
                                                                       size_t *command_len_out)
{
    uint32_t produced = 0u;
    const uint32_t total_pixels = udl_sink_count_from_byte(command[5]);
    size_t offset = 6u;

    if (length < 6u) {
        return UDL_STREAM_PARSE_NEED_MORE;
    }

    while (produced < total_pixels) {
        const uint32_t run_count = udl_sink_count_from_byte(command[offset]);

        offset += 1u;
        if (run_count > total_pixels - produced) {
            return UDL_STREAM_PARSE_INVALID;
        }
        if (length - offset < bytes_per_pixel) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        offset += bytes_per_pixel;
        produced += run_count;
    }

    *command_len_out = offset;
    return UDL_STREAM_PARSE_COMPLETE;
}

static enum udl_stream_parse_result udl_transport_parse_writerlx_length(const uint8_t *command,
                                                                        size_t length,
                                                                        size_t bytes_per_pixel,
                                                                        struct udl_transport_stats *stats,
                                                                        size_t *command_len_out)
{
    uint32_t produced = 0u;
    const uint32_t total_pixels = udl_sink_count_from_byte(command[5]);
    size_t offset = 6u;

    if (length < 7u) {
        return UDL_STREAM_PARSE_NEED_MORE;
    }

    while (produced < total_pixels) {
        uint32_t raw_count;
        size_t raw_bytes;
        uint32_t repeat_count;

        if (offset >= length) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        raw_count = udl_sink_count_from_byte(command[offset]);
        offset += 1u;
        if (raw_count > total_pixels - produced) {
            return UDL_STREAM_PARSE_INVALID;
        }
        if (stats && command[1] == UDL_CMD_WRITERLX16) {
            stats->writerlx16_raw_spans += 1u;
            stats->writerlx16_raw_pixels += raw_count;
            if (raw_count == 1u) {
                stats->writerlx16_raw_single_pixel_spans += 1u;
            }
        }

        raw_bytes = (size_t)raw_count * bytes_per_pixel;
        if (length - offset < raw_bytes) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        offset += raw_bytes;
        produced += raw_count;
        if (produced == total_pixels) {
            break;
        }

        if (offset >= length) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        repeat_count = command[offset];
        offset += 1u;
        if (repeat_count == 0u || repeat_count > total_pixels - produced) {
            return UDL_STREAM_PARSE_INVALID;
        }
        if (stats && command[1] == UDL_CMD_WRITERLX16) {
            stats->writerlx16_repeat_spans += 1u;
            stats->writerlx16_repeat_pixels += repeat_count;
        }

        produced += repeat_count;
    }

    *command_len_out = offset;
    return UDL_STREAM_PARSE_COMPLETE;
}

static enum udl_stream_parse_result udl_transport_decode_writerlx16(struct udl_transport *transport,
                                                                    const uint8_t *command,
                                                                    size_t length,
                                                                    struct udl_sink_damage *damage,
                                                                    size_t *command_len_out)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t total_pixels = udl_sink_count_from_byte(command[5]);
    const bool collect_span_stats = transport->collect_detailed_stats &&
                                    transport->collect_writerlx16_span_stats;
    struct udl_sink *sink = transport->sink;
    uint64_t raw_spans = 0u;
    uint64_t raw_pixels = 0u;
    uint64_t raw_single_pixel_spans = 0u;
    uint64_t repeat_spans = 0u;
    uint64_t repeat_pixels = 0u;
    uint32_t first_pixel;
    uint32_t produced = 0u;
    size_t offset = 6u;
    enum udl_sink_result result;

    if (length < 7u) {
        return UDL_STREAM_PARSE_NEED_MORE;
    }

    result = udl_sink_map_range(sink, UDL_SINK_PLANE_16, byte_address, total_pixels, &first_pixel);
    if (result == UDL_SINK_ERR_ADDRESS_RANGE || result == UDL_SINK_ERR_INVALID_COMMAND) {
        return UDL_STREAM_PARSE_INVALID;
    }
    if (result != UDL_SINK_OK) {
        return UDL_STREAM_PARSE_INVALID;
    }

    {
        const uint32_t raw_count = udl_sink_count_from_byte(command[6]);

        if (raw_count == total_pixels) {
            const size_t raw_bytes = (size_t)raw_count * 2u;
            const uint8_t *raw_data = &command[7];
            const size_t command_len = 7u + raw_bytes;

            if (length < command_len) {
                return UDL_STREAM_PARSE_NEED_MORE;
            }

            if (collect_span_stats) {
                raw_spans = 1u;
                raw_pixels = raw_count;
                raw_single_pixel_spans = raw_count == 1u ? 1u : 0u;
            }

            if (raw_count == 1u) {
                udl_sink_write_plane16(sink,
                                       first_pixel,
                                       udl_sink_read_be16(raw_data),
                                       damage);
            } else if (raw_count <= 4u) {
                udl_sink_blit_plane16_be_tiny(sink,
                                              first_pixel,
                                              raw_data,
                                              raw_count,
                                              damage);
            } else if (raw_count <= 8u) {
                udl_sink_blit_plane16_be_small(sink,
                                               first_pixel,
                                               raw_data,
                                               raw_count,
                                               damage);
            } else {
                udl_sink_blit_plane16_be(sink,
                                         first_pixel,
                                         raw_data,
                                         raw_count,
                                         damage);
            }

            if (collect_span_stats) {
                transport->stats.writerlx16_raw_spans += raw_spans;
                transport->stats.writerlx16_raw_pixels += raw_pixels;
                transport->stats.writerlx16_raw_single_pixel_spans += raw_single_pixel_spans;
            }

            *command_len_out = command_len;
            return UDL_STREAM_PARSE_COMPLETE;
        }
    }

    while (produced < total_pixels) {
        uint32_t raw_count;
        size_t raw_bytes;
        uint16_t repeated_pixel16;
        uint16_t raw_first_pixel16 = 0u;
        const uint8_t *raw_data;

        if (offset >= length) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        raw_count = udl_sink_count_from_byte(command[offset]);
        offset += 1u;
        if (raw_count > total_pixels - produced) {
            return UDL_STREAM_PARSE_INVALID;
        }

        if (collect_span_stats) {
            raw_spans += 1u;
            raw_pixels += raw_count;
            if (raw_count == 1u) {
                raw_single_pixel_spans += 1u;
            }
        }

        raw_bytes = (size_t)raw_count * 2u;
        if (length - offset < raw_bytes) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        raw_data = &command[offset];

        if (raw_count > 0u) {
            raw_first_pixel16 = udl_sink_read_be16(raw_data);
        }

        repeated_pixel16 = udl_sink_read_be16(&raw_data[raw_bytes - 2u]);
        offset += raw_bytes;

        if (produced + raw_count == total_pixels) {
            if (raw_count == 1u) {
                udl_sink_write_plane16(sink,
                                       first_pixel + produced,
                                       raw_first_pixel16,
                                       damage);
            } else if (raw_count <= 4u) {
                udl_sink_blit_plane16_be_tiny(sink,
                                              first_pixel + produced,
                                              raw_data,
                                              raw_count,
                                              damage);
            } else if (raw_count <= 8u) {
                udl_sink_blit_plane16_be_small(sink,
                                               first_pixel + produced,
                                               raw_data,
                                               raw_count,
                                               damage);
            } else {
                udl_sink_blit_plane16_be(sink,
                                         first_pixel + produced,
                                         raw_data,
                                         raw_count,
                                         damage);
            }
            produced += raw_count;
            break;
        }

        if (offset >= length) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }

        {
            const uint32_t repeat_count = command[offset];
            offset += 1u;

            if (repeat_count == 0u || repeat_count > total_pixels - produced) {
                return UDL_STREAM_PARSE_INVALID;
            }

            if (collect_span_stats) {
                repeat_spans += 1u;
                repeat_pixels += repeat_count;
            }

            if (raw_count == 1u) {
                udl_sink_fill_plane16(sink,
                                      first_pixel + produced,
                                      repeat_count + 1u,
                                      repeated_pixel16,
                                      damage);
                produced += repeat_count + 1u;
                continue;
            }

            if (raw_count == 2u) {
                udl_sink_write_plane16(sink,
                                       first_pixel + produced,
                                       raw_first_pixel16,
                                       damage);
                produced += 1u;

                udl_sink_fill_plane16(sink,
                                      first_pixel + produced,
                                      repeat_count + 1u,
                                      repeated_pixel16,
                                      damage);
                produced += repeat_count + 1u;
                continue;
            }

            if (raw_count <= 5u) {
                udl_sink_blit_plane16_be_tiny(sink,
                                              first_pixel + produced,
                                              raw_data,
                                              raw_count - 1u,
                                              damage);
                produced += raw_count - 1u;

                udl_sink_fill_plane16(sink,
                                      first_pixel + produced,
                                      repeat_count + 1u,
                                      repeated_pixel16,
                                      damage);
                produced += repeat_count + 1u;
                continue;
            }

            if (raw_count <= 9u) {
                udl_sink_blit_plane16_be_small(sink,
                                               first_pixel + produced,
                                               raw_data,
                                               raw_count - 1u,
                                               damage);
                produced += raw_count - 1u;

                udl_sink_fill_plane16(sink,
                                      first_pixel + produced,
                                      repeat_count + 1u,
                                      repeated_pixel16,
                                      damage);
                produced += repeat_count + 1u;
                continue;
            }

            udl_sink_blit_plane16_be(sink,
                                     first_pixel + produced,
                                     raw_data,
                                     raw_count - 1u,
                                     damage);
            produced += raw_count - 1u;

            udl_sink_fill_plane16(sink,
                                  first_pixel + produced,
                                  repeat_count + 1u,
                                  repeated_pixel16,
                                  damage);
            produced += repeat_count + 1u;
        }
    }

    if (collect_span_stats) {
        transport->stats.writerlx16_raw_spans += raw_spans;
        transport->stats.writerlx16_raw_pixels += raw_pixels;
        transport->stats.writerlx16_raw_single_pixel_spans += raw_single_pixel_spans;
        transport->stats.writerlx16_repeat_spans += repeat_spans;
        transport->stats.writerlx16_repeat_pixels += repeat_pixels;
    }

    *command_len_out = offset;
    return UDL_STREAM_PARSE_COMPLETE;
}

static enum udl_stream_parse_result udl_transport_next_command_length(const uint8_t *command,
                                                                      size_t length,
                                                                      size_t *command_len_out)
{
    uint32_t pixel_count;
    size_t command_len;

    if (length < 2u) {
        return UDL_STREAM_PARSE_NEED_MORE;
    }
    if (command[0] != UDL_MSG_BULK) {
        return UDL_STREAM_PARSE_INVALID;
    }

    switch (command[1]) {
    case UDL_CMD_OPAQUE_E0:
        command_len = udl_transport_e0_command_length(command, length);
        if (command_len == 0u) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }
        *command_len_out = command_len;
        return UDL_STREAM_PARSE_COMPLETE;
    case UDL_CMD_NOP_A0:
        *command_len_out = 2u;
        return UDL_STREAM_PARSE_COMPLETE;
    case UDL_CMD_UNKNOWN_40:
        if (length < UDL_CMD_UNKNOWN_40_SIZE) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }
        if (command[2] != UDL_CMD_UNKNOWN_40_ARG) {
            return UDL_STREAM_PARSE_INVALID;
        }
        *command_len_out = UDL_CMD_UNKNOWN_40_SIZE;
        return UDL_STREAM_PARSE_COMPLETE;
    case UDL_CMD_WRITEREG:
        if (length < 4u) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }
        *command_len_out = 4u;
        return UDL_STREAM_PARSE_COMPLETE;
    case UDL_CMD_WRITERAW8:
        if (length < 6u) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }
        pixel_count = udl_sink_count_from_byte(command[5]);
        command_len = 6u + (size_t)pixel_count;
        break;
    case UDL_CMD_WRITERL8:
        return udl_transport_parse_writerl_length(command, length, 1u, command_len_out);
    case UDL_CMD_WRITECOPY8:
        command_len = 9u;
        break;
    case UDL_CMD_WRITERLX8:
        return udl_transport_parse_writerlx_length(command, length, 1u, NULL, command_len_out);
    case UDL_CMD_WRITERAW16:
        if (length < 6u) {
            return UDL_STREAM_PARSE_NEED_MORE;
        }
        pixel_count = udl_sink_count_from_byte(command[5]);
        command_len = 6u + ((size_t)pixel_count * 2u);
        break;
    case UDL_CMD_WRITERL16:
        return udl_transport_parse_writerl_length(command, length, 2u, command_len_out);
    case UDL_CMD_WRITECOPY16:
        command_len = 9u;
        break;
    case UDL_CMD_WRITERLX16:
        return udl_transport_parse_writerlx_length(command, length, 2u, NULL, command_len_out);
    default:
        return UDL_STREAM_PARSE_INVALID;
    }

    if (length < command_len) {
        return UDL_STREAM_PARSE_NEED_MORE;
    }

    *command_len_out = command_len;
    return UDL_STREAM_PARSE_COMPLETE;
}

static void udl_transport_record_first_resync(struct udl_transport *transport,
                                              size_t pending_prefix_len,
                                              size_t absolute_offset,
                                              enum udl_transport_resync_reason reason)
{
    if (!transport || transport->last_feed_first_resync_reason != UDL_TRANSPORT_RESYNC_NONE) {
        return;
    }

    transport->last_feed_first_resync_reason = reason;
    transport->last_feed_first_resync_offset = 0u;
    transport->last_feed_first_resync_offset_valid = false;

    if (absolute_offset >= pending_prefix_len) {
        transport->last_feed_first_resync_offset = absolute_offset - pending_prefix_len;
        transport->last_feed_first_resync_offset_valid = true;
    }
}

static void udl_transport_begin_writecomp_quarantine(struct udl_transport *transport)
{
    if (!transport) {
        return;
    }

    transport->writecomp_quarantine_active = true;
    transport->writecomp_quarantine_noncomp_ok = 0u;
    transport->writecomp_quarantine_budget = UDL_WRITECOMP_QUARANTINE_BYTES;
}

static void udl_transport_note_noncompressed_success(struct udl_transport *transport)
{
    if (!transport || !transport->writecomp_quarantine_active) {
        return;
    }

    if (transport->writecomp_quarantine_noncomp_ok < UDL_WRITECOMP_QUARANTINE_NONCOMP_REQUIRED) {
        transport->writecomp_quarantine_noncomp_ok += 1u;
    }
    if (transport->writecomp_quarantine_noncomp_ok >= UDL_WRITECOMP_QUARANTINE_NONCOMP_REQUIRED) {
        transport->writecomp_quarantine_active = false;
        transport->writecomp_quarantine_budget = 0u;
    }
}

static bool udl_transport_should_quarantine_writecomp(const struct udl_transport *transport)
{
    return transport &&
           transport->writecomp_quarantine_active &&
           transport->writecomp_quarantine_noncomp_ok < UDL_WRITECOMP_QUARANTINE_NONCOMP_REQUIRED &&
           transport->writecomp_quarantine_budget > 0u;
}

static void udl_transport_consume_writecomp_quarantine_budget(struct udl_transport *transport,
                                                              size_t consumed)
{
    if (!transport || !transport->writecomp_quarantine_active) {
        return;
    }

    if (consumed >= transport->writecomp_quarantine_budget) {
        transport->writecomp_quarantine_budget = 0u;
        transport->writecomp_quarantine_active = false;
        transport->writecomp_quarantine_noncomp_ok = 0u;
        return;
    }

    transport->writecomp_quarantine_budget -= consumed;
}

static enum udl_transport_result udl_transport_reserve_pending(struct udl_transport *transport,
                                                               size_t additional)
{
    size_t needed;
    size_t new_capacity;
    uint8_t *new_pending;

    if (SIZE_MAX - transport->pending_len < additional) {
        return UDL_TRANSPORT_ERR_NO_MEMORY;
    }

    needed = transport->pending_len + additional;
    if (needed <= transport->pending_capacity) {
        return UDL_TRANSPORT_OK;
    }

    new_capacity = transport->pending_capacity ? transport->pending_capacity : 65536u;
    while (new_capacity < needed) {
        if (new_capacity > SIZE_MAX / 2u) {
            new_capacity = needed;
            break;
        }
        new_capacity *= 2u;
    }

    new_pending = realloc(transport->pending, new_capacity);
    if (!new_pending) {
        return UDL_TRANSPORT_ERR_NO_MEMORY;
    }

    transport->pending = new_pending;
    transport->pending_capacity = new_capacity;
    return UDL_TRANSPORT_OK;
}

static void udl_transport_compact_pending(struct udl_transport *transport, size_t consumed)
{
    if (consumed >= transport->pending_len) {
        transport->pending_len = 0u;
        return;
    }

    if (consumed == 0u) {
        return;
    }

    memmove(transport->pending,
            transport->pending + consumed,
            transport->pending_len - consumed);
    transport->pending_len -= consumed;
}

static uint32_t udl_sink_plane_bytes_per_pixel(enum udl_sink_plane plane)
{
    return plane == UDL_SINK_PLANE_16 ? 2u : 1u;
}

static uint8_t udl_sink_expand5_to8(uint8_t value)
{
    return (uint8_t)((value << 3) | (value >> 2));
}

static uint8_t udl_sink_expand6_to8(uint8_t value)
{
    return (uint8_t)((value << 2) | (value >> 4));
}

static uint16_t udl_sink_rgb888_to_rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    return (uint16_t)(((uint16_t)(red & 0xf8u) << 8) |
                      ((uint16_t)(green & 0xfcu) << 3) |
                      ((uint16_t)blue >> 3));
}

static bool udl_sink_ensure_rgb565_lookup(struct udl_sink *sink)
{
    uint32_t *lookup;
    uint32_t pixel;

    if (!sink) {
        return false;
    }
    if (sink->rgb565_to_xrgb8888_lookup) {
        return true;
    }

    lookup = malloc(65536u * sizeof(*lookup));
    if (!lookup) {
        return false;
    }

    for (pixel = 0u; pixel <= 0xffffu; ++pixel) {
        const uint8_t red = udl_sink_expand5_to8((uint8_t)((pixel >> 11) & 0x1fu));
        const uint8_t green = udl_sink_expand6_to8((uint8_t)((pixel >> 5) & 0x3fu));
        const uint8_t blue = udl_sink_expand5_to8((uint8_t)(pixel & 0x1fu));

        lookup[pixel] = 0xff000000u | ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
    }

    sink->rgb565_to_xrgb8888_lookup = lookup;
    return true;
}

static uint32_t udl_sink_plane_base(const struct udl_sink *sink, enum udl_sink_plane plane)
{
    return plane == UDL_SINK_PLANE_16 ? udl_sink_get_base16bpp(sink)
                                      : udl_sink_get_base8bpp(sink);
}

static bool udl_sink_has_any_output(const struct udl_sink *sink)
{
    return sink->framebuffer != NULL || sink->framebuffer_xrgb8888 != NULL;
}

static void udl_sink_compose_all(struct udl_sink *sink, struct udl_sink_damage *damage);

static enum udl_sink_result udl_sink_ensure_planes(struct udl_sink *sink)
{
    size_t plane_pixels;
    uint16_t *plane16;
    uint8_t *plane8;

    if (sink->plane16 && sink->plane8) {
        return UDL_SINK_OK;
    }

    if (sink->width == 0u || sink->height == 0u) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }

    if ((size_t)sink->height > SIZE_MAX / (size_t)sink->width) {
        return UDL_SINK_ERR_NO_MEMORY;
    }

    plane_pixels = (size_t)sink->width * (size_t)sink->height;
    if (plane_pixels > UINT32_MAX) {
        return UDL_SINK_ERR_NO_MEMORY;
    }

    plane16 = calloc(plane_pixels, sizeof(*plane16));
    if (!plane16) {
        return UDL_SINK_ERR_NO_MEMORY;
    }

    plane8 = calloc(plane_pixels, sizeof(*plane8));
    if (!plane8) {
        free(plane16);
        return UDL_SINK_ERR_NO_MEMORY;
    }

    sink->plane16 = plane16;
    sink->plane8 = plane8;
    sink->plane_pixels = (uint32_t)plane_pixels;

    if (udl_sink_has_any_output(sink)) {
        udl_sink_compose_all(sink, NULL);
    }

    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_map_range(const struct udl_sink *sink,
                                               enum udl_sink_plane plane,
                                               uint32_t byte_address,
                                               uint32_t pixel_count,
                                               uint32_t *first_pixel_out)
{
    const uint32_t base = udl_sink_plane_base(sink, plane);
    const uint32_t bytes_per_pixel = udl_sink_plane_bytes_per_pixel(plane);
    uint64_t byte_offset;
    uint64_t first_pixel;
    uint64_t end_pixel;

    if (byte_address < base) {
        return UDL_SINK_ERR_ADDRESS_RANGE;
    }

    byte_offset = (uint64_t)byte_address - base;
    if ((byte_offset % bytes_per_pixel) != 0u) {
        return UDL_SINK_ERR_INVALID_COMMAND;
    }

    first_pixel = byte_offset / bytes_per_pixel;
    end_pixel = first_pixel + pixel_count;
    if (end_pixel > sink->plane_pixels) {
        return UDL_SINK_ERR_ADDRESS_RANGE;
    }

    *first_pixel_out = (uint32_t)first_pixel;
    return UDL_SINK_OK;
}

static void udl_sink_mark_pixel(struct udl_sink_damage *damage,
                                uint32_t x,
                                uint32_t y)
{
    if (!damage) {
        return;
    }

    if (!damage->touched) {
        damage->touched = true;
        damage->x1 = x;
        damage->y1 = y;
        damage->x2 = x + 1u;
        damage->y2 = y + 1u;
    } else {
        if (x < damage->x1) {
            damage->x1 = x;
        }
        if (y < damage->y1) {
            damage->y1 = y;
        }
        if (x + 1u > damage->x2) {
            damage->x2 = x + 1u;
        }
        if (y + 1u > damage->y2) {
            damage->y2 = y + 1u;
        }
    }

    damage->pixel_count += 1u;
}

static void udl_sink_mark_span(struct udl_sink_damage *damage,
                               uint32_t x,
                               uint32_t y,
                               uint32_t pixel_count)
{
    if (!damage || pixel_count == 0u) {
        return;
    }

    if (!damage->touched) {
        damage->touched = true;
        damage->x1 = x;
        damage->y1 = y;
        damage->x2 = x + pixel_count;
        damage->y2 = y + 1u;
    } else {
        if (x < damage->x1) {
            damage->x1 = x;
        }
        if (y < damage->y1) {
            damage->y1 = y;
        }
        if (x + pixel_count > damage->x2) {
            damage->x2 = x + pixel_count;
        }
        if (y + 1u > damage->y2) {
            damage->y2 = y + 1u;
        }
    }

    damage->pixel_count += pixel_count;
}

static void udl_sink_compose_channels(const struct udl_sink *sink,
                                      uint32_t pixel_index,
                                      uint8_t *red,
                                      uint8_t *green,
                                      uint8_t *blue)
{
    const uint16_t pixel16 = sink->plane16[pixel_index];
    const uint8_t red_high = (uint8_t)((pixel16 >> 11) & 0x1fu);
    const uint8_t green_high = (uint8_t)((pixel16 >> 5) & 0x3fu);
    const uint8_t blue_high = (uint8_t)(pixel16 & 0x1fu);

    if (udl_sink_get_color_depth(sink) == UDL_COLORDEPTH_24BPP) {
        const uint8_t low = sink->plane8[pixel_index];

        *red = (uint8_t)((red_high << 3) | ((low >> 5) & 0x07u));
        *green = (uint8_t)((green_high << 2) | ((low >> 3) & 0x03u));
        *blue = (uint8_t)((blue_high << 3) | (low & 0x07u));
    } else {
        *red = udl_sink_expand5_to8(red_high);
        *green = udl_sink_expand6_to8(green_high);
        *blue = udl_sink_expand5_to8(blue_high);
    }
}

static bool udl_sink_store_output_pixel(struct udl_sink *sink,
                                        uint32_t pixel_index,
                                        uint32_t xrgb8888)
{
    const uint32_t x = pixel_index % sink->width;
    const uint32_t y = pixel_index / sink->width;
    bool changed = false;

    if (sink->framebuffer_xrgb8888) {
        uint32_t *dst32 = &sink->framebuffer_xrgb8888[y * sink->stride_pixels_xrgb8888 + x];

        if (*dst32 != xrgb8888) {
            *dst32 = xrgb8888;
            changed = true;
        }
    }

    if (sink->framebuffer) {
        const uint8_t red = (uint8_t)(xrgb8888 >> 16);
        const uint8_t green = (uint8_t)(xrgb8888 >> 8);
        const uint8_t blue = (uint8_t)xrgb8888;
        uint16_t *dst16 = &sink->framebuffer[y * sink->stride_pixels + x];
        const uint16_t rgb565 = udl_sink_rgb888_to_rgb565(red, green, blue);

        if (*dst16 != rgb565) {
            *dst16 = rgb565;
            changed = true;
        }
    }

    if (!sink->framebuffer && !sink->framebuffer_xrgb8888) {
        changed = true;
    }

    return changed;
}

static void udl_sink_compose_pixel(struct udl_sink *sink,
                                   uint32_t pixel_index,
                                   struct udl_sink_damage *damage)
{
    const uint32_t x = pixel_index % sink->width;
    const uint32_t y = pixel_index / sink->width;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint32_t xrgb8888;

    udl_sink_compose_channels(sink, pixel_index, &red, &green, &blue);
    xrgb8888 = 0xff000000u | ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;

    if (udl_sink_store_output_pixel(sink, pixel_index, xrgb8888)) {
        udl_sink_mark_pixel(damage, x, y);
    }
}

static uint32_t udl_sink_rgb565_to_xrgb8888(uint16_t pixel16)
{
    const uint8_t red = udl_sink_expand5_to8((uint8_t)((pixel16 >> 11) & 0x1fu));
    const uint8_t green = udl_sink_expand6_to8((uint8_t)((pixel16 >> 5) & 0x3fu));
    const uint8_t blue = udl_sink_expand5_to8((uint8_t)(pixel16 & 0x1fu));

    return 0xff000000u | ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
}

static uint32_t udl_sink_rgb565_to_xrgb8888_fast(const struct udl_sink *sink,
                                                 uint16_t pixel16)
{
    if (sink && sink->rgb565_to_xrgb8888_lookup) {
        return sink->rgb565_to_xrgb8888_lookup[pixel16];
    }

    return udl_sink_rgb565_to_xrgb8888(pixel16);
}

static void udl_sink_compose_all(struct udl_sink *sink, struct udl_sink_damage *damage)
{
    uint32_t pixel_index;

    if (!sink || !sink->plane16 || !sink->plane8) {
        return;
    }

    for (pixel_index = 0; pixel_index < sink->plane_pixels; ++pixel_index) {
        udl_sink_compose_pixel(sink, pixel_index, damage);
    }
}

static bool udl_sink_register_requires_recompose(uint8_t reg)
{
    switch (reg) {
    case UDL_REG_COLORDEPTH:
    case UDL_REG_BASE16BPP_ADDR2:
    case UDL_REG_BASE16BPP_ADDR1:
    case UDL_REG_BASE16BPP_ADDR0:
    case UDL_REG_BASE8BPP_ADDR2:
    case UDL_REG_BASE8BPP_ADDR1:
    case UDL_REG_BASE8BPP_ADDR0:
        return true;
    default:
        return false;
    }
}

static uint16_t udl_sink_read_plane16(const struct udl_sink *sink, uint32_t pixel_index)
{
    return sink->plane16[pixel_index];
}

static uint8_t udl_sink_read_plane8(const struct udl_sink *sink, uint32_t pixel_index)
{
    return sink->plane8[pixel_index];
}

static bool udl_sink_fast_path_16bpp_enabled(const struct udl_sink *sink)
{
    return udl_sink_get_color_depth(sink) != UDL_COLORDEPTH_24BPP;
}

static void udl_sink_store_plane16_fast_pixel(struct udl_sink *sink,
                                              uint32_t pixel_index,
                                              uint16_t pixel,
                                              struct udl_sink_damage *damage)
{
    const uint32_t x = pixel_index % sink->width;
    const uint32_t y = pixel_index / sink->width;

    sink->plane16[pixel_index] = pixel;
    if (sink->framebuffer) {
        sink->framebuffer[(size_t)y * sink->stride_pixels + x] = pixel;
    }
    if (sink->framebuffer_xrgb8888) {
        sink->framebuffer_xrgb8888[(size_t)y * sink->stride_pixels_xrgb8888 + x] =
            udl_sink_rgb565_to_xrgb8888_fast(sink, pixel);
    }

    udl_sink_mark_pixel(damage, x, y);
}

static void udl_sink_write_plane16(struct udl_sink *sink,
                                   uint32_t pixel_index,
                                   uint16_t pixel,
                                   struct udl_sink_damage *damage)
{
    if (sink->plane16[pixel_index] == pixel) {
        return;
    }

    if (udl_sink_fast_path_16bpp_enabled(sink)) {
        udl_sink_store_plane16_fast_pixel(sink, pixel_index, pixel, damage);
        return;
    }

    sink->plane16[pixel_index] = pixel;
    udl_sink_compose_pixel(sink, pixel_index, damage);
}

static void udl_sink_write_plane8(struct udl_sink *sink,
                                  uint32_t pixel_index,
                                  uint8_t pixel,
                                  struct udl_sink_damage *damage)
{
    if (sink->plane8[pixel_index] == pixel) {
        return;
    }

    sink->plane8[pixel_index] = pixel;
    udl_sink_compose_pixel(sink, pixel_index, damage);
}

static bool udl_sink_plane16_span_all_equal(const uint16_t *pixels,
                                            uint32_t pixel_count,
                                            uint16_t pixel)
{
    const uint8_t *bytes = (const uint8_t *)pixels;
    size_t remaining_bytes = (size_t)pixel_count * sizeof(*pixels);
    size_t offset = 0u;

    if (remaining_bytes >= sizeof(size_t)) {
        const size_t repeated_pixel_word = udl_sink_repeat_u16_word(pixel);

        while (remaining_bytes >= sizeof(size_t)) {
            size_t word;

            memcpy(&word, bytes + offset, sizeof(word));
            if (word != repeated_pixel_word) {
                return false;
            }

            offset += sizeof(word);
            remaining_bytes -= sizeof(word);
        }
    }

    while (remaining_bytes > 0u) {
        uint16_t tail_pixel;

        memcpy(&tail_pixel, bytes + offset, sizeof(tail_pixel));
        if (tail_pixel != pixel) {
            return false;
        }

        offset += sizeof(tail_pixel);
        remaining_bytes -= sizeof(tail_pixel);
    }

    return true;
}

static size_t udl_sink_swap_be16_word_to_host(size_t word)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#if SIZE_MAX == UINT64_MAX
    return ((word & (size_t)UINT64_C(0x00ff00ff00ff00ff)) << 8u) |
           ((word & (size_t)UINT64_C(0xff00ff00ff00ff00)) >> 8u);
#elif SIZE_MAX == UINT32_MAX
    return ((word & (size_t)UINT32_C(0x00ff00ff)) << 8u) |
           ((word & (size_t)UINT32_C(0xff00ff00)) >> 8u);
#else
    size_t swapped = 0u;
    unsigned int shift;

    for (shift = 0u; shift < (unsigned int)(sizeof(word) * 8u); shift += 16u) {
        swapped |= ((word >> shift) & (size_t)0x00ffu) << (shift + 8u);
        swapped |= ((word >> (shift + 8u)) & (size_t)0x00ffu) << shift;
    }

    return swapped;
#endif
#else
    return word;
#endif
}

static bool udl_sink_plane16_span_matches_be(const uint16_t *dst,
                                             const uint8_t *src,
                                             uint32_t pixel_count)
{
    const uint8_t *dst_bytes = (const uint8_t *)dst;
    size_t remaining_bytes = (size_t)pixel_count * sizeof(*dst);
    size_t offset = 0u;

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return memcmp(dst_bytes, src, remaining_bytes) == 0;
#else
    while (remaining_bytes >= sizeof(size_t)) {
        size_t src_word;
        size_t dst_word;

        memcpy(&src_word, src + offset, sizeof(src_word));
        memcpy(&dst_word, dst_bytes + offset, sizeof(dst_word));
        if (udl_sink_swap_be16_word_to_host(src_word) != dst_word) {
            return false;
        }

        offset += sizeof(size_t);
        remaining_bytes -= sizeof(size_t);
    }

    while (remaining_bytes > 0u) {
        uint16_t src_pixel;
        uint16_t dst_pixel;

        src_pixel = udl_sink_read_be16(src + offset);
        memcpy(&dst_pixel, dst_bytes + offset, sizeof(dst_pixel));
        if (dst_pixel != src_pixel) {
            return false;
        }

        offset += sizeof(dst_pixel);
        remaining_bytes -= sizeof(dst_pixel);
    }

    return true;
#endif
}

static void udl_sink_fill_u16(uint16_t *dst,
                              uint32_t pixel_count,
                              uint16_t pixel)
{
    uint32_t index = 0u;

    while (index + 8u <= pixel_count) {
        dst[index] = pixel;
        dst[index + 1u] = pixel;
        dst[index + 2u] = pixel;
        dst[index + 3u] = pixel;
        dst[index + 4u] = pixel;
        dst[index + 5u] = pixel;
        dst[index + 6u] = pixel;
        dst[index + 7u] = pixel;
        index += 8u;
    }

    while (index < pixel_count) {
        dst[index] = pixel;
        index += 1u;
    }
}

static size_t udl_sink_repeat_u16_word(uint16_t pixel)
{
    size_t word = 0u;
    unsigned int shift;

    for (shift = 0u; shift < (unsigned int)(sizeof(word) * 8u); shift += 16u) {
        word |= (size_t)pixel << shift;
    }

    return word;
}

static void udl_sink_fill_u32(uint32_t *dst,
                              uint32_t pixel_count,
                              uint32_t pixel)
{
    uint32_t index = 0u;

    while (index + 8u <= pixel_count) {
        dst[index] = pixel;
        dst[index + 1u] = pixel;
        dst[index + 2u] = pixel;
        dst[index + 3u] = pixel;
        dst[index + 4u] = pixel;
        dst[index + 5u] = pixel;
        dst[index + 6u] = pixel;
        dst[index + 7u] = pixel;
        index += 8u;
    }

    while (index < pixel_count) {
        dst[index] = pixel;
        index += 1u;
    }
}

static void udl_sink_fill_plane16(struct udl_sink *sink,
                                  uint32_t first_pixel,
                                  uint32_t pixel_count,
                                  uint16_t pixel,
                                  struct udl_sink_damage *damage)
{
    uint16_t *dst = sink->plane16 + first_pixel;
    const bool fast_path_16bpp = udl_sink_fast_path_16bpp_enabled(sink);
    uint32_t i;

    if (pixel_count == 0u) {
        return;
    }

    if (pixel_count >= 8u && udl_sink_plane16_span_all_equal(dst, pixel_count, pixel)) {
        return;
    }

    if (fast_path_16bpp) {
        const uint32_t width = sink->width;
        const uint32_t xrgb8888 = udl_sink_rgb565_to_xrgb8888_fast(sink, pixel);
        uint32_t remaining = pixel_count;
        uint32_t pixel_index = first_pixel;

        while (remaining > 0u) {
            const uint32_t x = pixel_index % width;
            const uint32_t y = pixel_index / width;
            const uint32_t row_pixels = (remaining < (width - x)) ? remaining : (width - x);
            uint16_t *row_plane16 = sink->plane16 + pixel_index;
            uint16_t *row_fb16 = sink->framebuffer ? sink->framebuffer + (y * sink->stride_pixels) + x : NULL;
            uint32_t *row_fb32 = sink->framebuffer_xrgb8888
                ? sink->framebuffer_xrgb8888 + (y * sink->stride_pixels_xrgb8888) + x
                : NULL;
            uint32_t row_offset = 0u;

            while (row_offset < row_pixels) {
                if (row_plane16[row_offset] == pixel) {
                    row_offset += 1u;
                    continue;
                }

                {
                    const uint32_t run_start = row_offset;

                    do {
                        row_offset += 1u;
                    } while (row_offset < row_pixels && row_plane16[row_offset] != pixel);

                    udl_sink_fill_u16(row_plane16 + run_start,
                                      row_offset - run_start,
                                      pixel);
                    if (row_fb16) {
                        udl_sink_fill_u16(row_fb16 + run_start,
                                          row_offset - run_start,
                                          pixel);
                    }
                    if (row_fb32) {
                        udl_sink_fill_u32(row_fb32 + run_start,
                                          row_offset - run_start,
                                          xrgb8888);
                    }

                    udl_sink_mark_span(damage, x + run_start, y, row_offset - run_start);
                }
            }

            pixel_index += row_pixels;
            remaining -= row_pixels;
        }

        return;
    }

    for (i = 0; i < pixel_count; ++i) {
        const uint32_t pixel_index = first_pixel + i;

        if (dst[i] == pixel) {
            continue;
        }

        dst[i] = pixel;
        udl_sink_compose_pixel(sink, pixel_index, damage);
    }
}

static void udl_sink_fill_plane8(struct udl_sink *sink,
                                 uint32_t first_pixel,
                                 uint32_t pixel_count,
                                 uint8_t pixel,
                                 struct udl_sink_damage *damage)
{
    uint32_t i;

    for (i = 0; i < pixel_count; ++i) {
        const uint32_t pixel_index = first_pixel + i;

        if (sink->plane8[pixel_index] == pixel) {
            continue;
        }

        sink->plane8[pixel_index] = pixel;
        udl_sink_compose_pixel(sink, pixel_index, damage);
    }
}

static void udl_sink_blit_plane16_be(struct udl_sink *sink,
                                     uint32_t first_pixel,
                                     const uint8_t *src,
                                     uint32_t pixel_count,
                                     struct udl_sink_damage *damage)
{
    const bool fast_path_16bpp = udl_sink_fast_path_16bpp_enabled(sink);
    uint32_t i;

    if (fast_path_16bpp) {
        const uint32_t width = sink->width;
        uint32_t remaining = pixel_count;
        uint32_t pixel_index = first_pixel;
        const uint8_t *row_src = src;

        while (remaining > 0u) {
            const uint32_t x = pixel_index % width;
            const uint32_t y = pixel_index / width;
            const uint32_t row_pixels = (remaining < (width - x)) ? remaining : (width - x);
            uint16_t *row_plane16 = sink->plane16 + pixel_index;
            uint16_t *row_fb16 = sink->framebuffer
                ? sink->framebuffer + ((size_t)y * sink->stride_pixels) + x
                : NULL;
            uint32_t *row_fb32 = sink->framebuffer_xrgb8888
                ? sink->framebuffer_xrgb8888 + ((size_t)y * sink->stride_pixels_xrgb8888) + x
                : NULL;
            uint32_t row_offset = 0u;

            if (row_pixels >= 8u &&
                udl_sink_plane16_span_matches_be(row_plane16, row_src, row_pixels)) {
                row_src += (size_t)row_pixels * 2u;
                pixel_index += row_pixels;
                remaining -= row_pixels;
                continue;
            }

            while (row_offset < row_pixels) {
                const uint16_t pixel = udl_sink_read_be16(&row_src[(size_t)row_offset * 2u]);

                if (row_plane16[row_offset] == pixel) {
                    row_offset += 1u;
                    continue;
                }

                {
                    const uint32_t run_start = row_offset;

                    do {
                        const uint16_t run_pixel = udl_sink_read_be16(&row_src[(size_t)row_offset * 2u]);

                        if (row_plane16[row_offset] == run_pixel) {
                            break;
                        }

                        row_plane16[row_offset] = run_pixel;
                        if (row_fb16) {
                            row_fb16[row_offset] = run_pixel;
                        }
                        if (row_fb32) {
                            row_fb32[row_offset] = udl_sink_rgb565_to_xrgb8888_fast(sink, run_pixel);
                        }

                        row_offset += 1u;
                    } while (row_offset < row_pixels);

                    udl_sink_mark_span(damage, x + run_start, y, row_offset - run_start);
                }
            }

            row_src += (size_t)row_pixels * 2u;
            pixel_index += row_pixels;
            remaining -= row_pixels;
        }

        return;
    }

    for (i = 0; i < pixel_count; ++i) {
        const uint32_t pixel_index = first_pixel + i;
        const uint16_t pixel = udl_sink_read_be16(&src[(size_t)i * 2u]);

        if (sink->plane16[pixel_index] == pixel) {
            continue;
        }

        sink->plane16[pixel_index] = pixel;
        udl_sink_compose_pixel(sink, pixel_index, damage);
    }
}

static void udl_sink_blit_plane8(struct udl_sink *sink,
                                 uint32_t first_pixel,
                                 const uint8_t *src,
                                 uint32_t pixel_count,
                                 struct udl_sink_damage *damage)
{
    uint32_t i;

    for (i = 0; i < pixel_count; ++i) {
        const uint32_t pixel_index = first_pixel + i;
        const uint8_t pixel = src[i];

        if (sink->plane8[pixel_index] == pixel) {
            continue;
        }

        sink->plane8[pixel_index] = pixel;
        udl_sink_compose_pixel(sink, pixel_index, damage);
    }
}

static enum udl_sink_result udl_sink_decode_writereg(struct udl_sink *sink,
                                                     const uint8_t *command,
                                                     size_t remaining,
                                                     size_t *consumed,
                                                     struct udl_sink_damage *damage)
{
    const uint8_t reg = command[2];
    const uint8_t value = command[3];

    if (remaining < 4u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    if (sink->registers[reg] != value) {
        sink->registers[reg] = value;
        if (udl_sink_register_requires_recompose(reg)) {
            udl_sink_compose_all(sink, damage);
        }
    }

    *consumed = 4u;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writeraw(struct udl_sink *sink,
                                                     enum udl_sink_plane plane,
                                                     const uint8_t *command,
                                                     size_t remaining,
                                                     size_t *consumed,
                                                     struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t pixel_count = udl_sink_count_from_byte(command[5]);
    const size_t payload_bytes = (size_t)pixel_count * udl_sink_plane_bytes_per_pixel(plane);
    const size_t command_size = 6u + payload_bytes;
    enum udl_sink_result result;
    uint32_t first_pixel;

    if (remaining < 6u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }
    if (remaining < command_size) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_map_range(sink, plane, byte_address, pixel_count, &first_pixel);
    if (result != UDL_SINK_OK) {
        return result;
    }

    if (plane == UDL_SINK_PLANE_16) {
        udl_sink_blit_plane16_be(sink,
                                 first_pixel,
                                 &command[6],
                                 pixel_count,
                                 damage);
    } else {
        udl_sink_blit_plane8(sink,
                             first_pixel,
                             &command[6],
                             pixel_count,
                             damage);
    }

    *consumed = command_size;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writecomp(struct udl_sink *sink,
                                                      enum udl_sink_plane plane,
                                                      const uint8_t *command,
                                                      size_t remaining,
                                                      size_t *consumed,
                                                      struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t pixel_count = udl_sink_count_from_byte(command[5]);
    const uint32_t bytes_per_pixel = udl_sink_plane_bytes_per_pixel(plane);
    const uint64_t visible_base = (uint64_t)udl_sink_plane_base(sink, plane);
    const uint64_t visible_limit = visible_base + ((uint64_t)sink->plane_pixels * bytes_per_pixel);
    const uint64_t command_byte_start = (uint64_t)byte_address;
    const uint64_t command_byte_limit = command_byte_start + ((uint64_t)pixel_count * bytes_per_pixel);
    enum udl_sink_result map_result;
    struct udl_huffman_bitstream bitstream;
    enum udl_sink_result result;
    uint16_t decoded16[UDL_MAX_COMMAND_PIXELS];
    uint8_t decoded8[UDL_MAX_COMMAND_PIXELS];
    uint32_t first_visible_pixel = 0u;
    uint32_t visible_pixel_count = 0u;
    uint32_t skip_pixels = 0u;
    uint32_t produced;

    if (consumed) {
        *consumed = 0u;
    }

    if (remaining < 6u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    if (udl_sink_writecomp_debug_enabled) {
        fprintf(stderr,
                "writecomp begin cmd=0x%02x plane=%u addr=0x%06x count=%u remaining=%zu\\n",
                (unsigned int)command[1],
                (unsigned int)plane,
                (unsigned int)byte_address,
                (unsigned int)pixel_count,
                remaining);
    }

    map_result = udl_sink_map_range(sink, plane, byte_address, pixel_count, &first_visible_pixel);
    if (map_result == UDL_SINK_OK) {
        visible_pixel_count = pixel_count;
    } else if (map_result == UDL_SINK_ERR_ADDRESS_RANGE) {
        if ((byte_address % bytes_per_pixel) != 0u) {
            if (udl_sink_writecomp_debug_enabled) {
                fprintf(stderr,
                        "writecomp map fail cmd=0x%02x addr=0x%06x count=%u base16=0x%06x base8=0x%06x plane_pixels=%u err=%d\\n",
                        (unsigned int)command[1],
                        (unsigned int)byte_address,
                        (unsigned int)pixel_count,
                        (unsigned int)udl_sink_get_base16bpp(sink),
                        (unsigned int)udl_sink_get_base8bpp(sink),
                        (unsigned int)sink->plane_pixels,
                        (int)UDL_SINK_ERR_INVALID_COMMAND);
            }
            return UDL_SINK_ERR_INVALID_COMMAND;
        }

        if (command_byte_start < visible_limit && command_byte_limit > visible_base) {
            const uint64_t clipped_start = command_byte_start > visible_base ? command_byte_start : visible_base;
            const uint64_t clipped_limit = command_byte_limit < visible_limit ? command_byte_limit : visible_limit;

            if (clipped_start < clipped_limit) {
                skip_pixels = (uint32_t)((clipped_start - command_byte_start) / bytes_per_pixel);
                visible_pixel_count = (uint32_t)((clipped_limit - clipped_start) / bytes_per_pixel);
                first_visible_pixel = (uint32_t)((clipped_start - visible_base) / bytes_per_pixel);
            }
        }
    } else {
        if (udl_sink_writecomp_debug_enabled) {
            fprintf(stderr,
                    "writecomp map fail cmd=0x%02x addr=0x%06x count=%u base16=0x%06x base8=0x%06x plane_pixels=%u err=%d\\n",
                    (unsigned int)command[1],
                    (unsigned int)byte_address,
                    (unsigned int)pixel_count,
                    (unsigned int)udl_sink_get_base16bpp(sink),
                    (unsigned int)udl_sink_get_base8bpp(sink),
                    (unsigned int)sink->plane_pixels,
                    (int)map_result);
        }
        return map_result;
    }

    bitstream.bytes = &command[6];
    bitstream.byte_length = remaining - 6u;
    bitstream.byte_offset = 0u;
    bitstream.bit_offset = 0u;

    if (plane == UDL_SINK_PLANE_16) {
        uint16_t previous_pixel = 0u;
        uint32_t visible_index;

        for (produced = 0u; produced < pixel_count; ++produced) {
            int32_t diff;
            uint8_t change = 0u;
            uint16_t pixel;

            result = udl_sink_huffman_read_bit(&bitstream, &change);
            if (result != UDL_SINK_OK) {
                if (consumed) {
                    *consumed = 6u + udl_sink_huffman_bytes_consumed(&bitstream);
                }
                if (udl_sink_writecomp_debug_enabled) {
                    fprintf(stderr,
                            "writecomp huff fail cmd=0x%02x produced=%u/%u byte_off=%zu bit_off=%u err=%d\\n",
                            (unsigned int)command[1],
                            (unsigned int)produced,
                            (unsigned int)pixel_count,
                            bitstream.byte_offset,
                            (unsigned int)bitstream.bit_offset,
                            (int)result);
                }
                return result;
            }

            if (change != 0u) {
                result = udl_sink_huffman_read_diff(&bitstream, &diff);
                if (result != UDL_SINK_OK) {
                    if (consumed) {
                        *consumed = 6u + udl_sink_huffman_bytes_consumed(&bitstream);
                    }
                    if (udl_sink_writecomp_debug_enabled) {
                        fprintf(stderr,
                                "writecomp huff fail cmd=0x%02x produced=%u/%u byte_off=%zu bit_off=%u err=%d\\n",
                                (unsigned int)command[1],
                                (unsigned int)produced,
                                (unsigned int)pixel_count,
                                bitstream.byte_offset,
                                (unsigned int)bitstream.bit_offset,
                                (int)result);
                    }
                    return result;
                }

                pixel = (uint16_t)((uint32_t)previous_pixel + (uint32_t)(uint16_t)diff);
            } else {
                pixel = previous_pixel;
            }

            decoded16[produced] = pixel;
            previous_pixel = pixel;
        }

        for (visible_index = 0u; visible_index < visible_pixel_count; ++visible_index) {
            udl_sink_write_plane16(sink,
                                   first_visible_pixel + visible_index,
                                   decoded16[skip_pixels + visible_index],
                                   damage);
        }
    } else {
        uint8_t previous_pixel = 0u;
        uint32_t visible_index;

        for (produced = 0u; produced < pixel_count; ++produced) {
            int32_t diff;
            uint8_t change = 0u;
            uint8_t pixel;

            result = udl_sink_huffman_read_bit(&bitstream, &change);
            if (result != UDL_SINK_OK) {
                if (consumed) {
                    *consumed = 6u + udl_sink_huffman_bytes_consumed(&bitstream);
                }
                if (udl_sink_writecomp_debug_enabled) {
                    fprintf(stderr,
                            "writecomp huff fail cmd=0x%02x produced=%u/%u byte_off=%zu bit_off=%u err=%d\\n",
                            (unsigned int)command[1],
                            (unsigned int)produced,
                            (unsigned int)pixel_count,
                            bitstream.byte_offset,
                            (unsigned int)bitstream.bit_offset,
                            (int)result);
                }
                return result;
            }

            if (change != 0u) {
                result = udl_sink_huffman_read_diff(&bitstream, &diff);
                if (result != UDL_SINK_OK) {
                    if (consumed) {
                        *consumed = 6u + udl_sink_huffman_bytes_consumed(&bitstream);
                    }
                    if (udl_sink_writecomp_debug_enabled) {
                        fprintf(stderr,
                                "writecomp huff fail cmd=0x%02x produced=%u/%u byte_off=%zu bit_off=%u err=%d\\n",
                                (unsigned int)command[1],
                                (unsigned int)produced,
                                (unsigned int)pixel_count,
                                bitstream.byte_offset,
                                (unsigned int)bitstream.bit_offset,
                                (int)result);
                    }
                    return result;
                }

                pixel = (uint8_t)((uint32_t)previous_pixel + (uint32_t)(uint8_t)diff);
            } else {
                pixel = previous_pixel;
            }

            decoded8[produced] = pixel;
            previous_pixel = pixel;
        }

        for (visible_index = 0u; visible_index < visible_pixel_count; ++visible_index) {
            udl_sink_write_plane8(sink,
                                  first_visible_pixel + visible_index,
                                  decoded8[skip_pixels + visible_index],
                                  damage);
        }
    }

    *consumed = 6u + udl_sink_huffman_bytes_consumed(&bitstream);

    /* Some captures include short trailer/padding bytes between a completed
     * WRITECOMP payload and the next bulk command marker. If we can
     * unambiguously see "<pad...><0xaf><known-cmd>", absorb up to a small
     * bounded trailer to avoid immediate transport resync churn.
     */
    {
        size_t trailer_scan;

        for (trailer_scan = 0u;
             trailer_scan < 8u && *consumed + trailer_scan + 2u < remaining;
             ++trailer_scan) {
            const size_t pos = *consumed + trailer_scan;

            if (command[pos] == UDL_MSG_BULK) {
                break;
            }

            if (command[pos + 1u] == UDL_MSG_BULK &&
                udl_transport_is_known_command_type(command[pos + 2u])) {
                *consumed = pos + 1u;
                break;
            }
        }
    }

    if (udl_sink_writecomp_debug_enabled) {
        fprintf(stderr,
                "writecomp ok cmd=0x%02x count=%u consumed=%zu huff_bytes=%zu\\n",
                (unsigned int)command[1],
                (unsigned int)pixel_count,
                *consumed,
                udl_sink_huffman_bytes_consumed(&bitstream));
    }

    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writerl(struct udl_sink *sink,
                                                    enum udl_sink_plane plane,
                                                    const uint8_t *command,
                                                    size_t remaining,
                                                    size_t *consumed,
                                                    struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t pixel_count = udl_sink_count_from_byte(command[5]);
    const uint32_t bytes_per_pixel = udl_sink_plane_bytes_per_pixel(plane);
    const uint64_t visible_base = (uint64_t)udl_sink_plane_base(sink, plane);
    const uint64_t visible_limit = visible_base + ((uint64_t)sink->plane_pixels * bytes_per_pixel);
    uint32_t produced = 0u;
    size_t offset = 6u;

    if (remaining < 6u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    if ((byte_address % bytes_per_pixel) != 0u) {
        return UDL_SINK_ERR_INVALID_COMMAND;
    }

    while (produced < pixel_count) {
        const uint32_t run_count = udl_sink_count_from_byte(command[offset]);
        const uint64_t run_byte_start = (uint64_t)byte_address + ((uint64_t)produced * bytes_per_pixel);
        const uint64_t run_byte_limit = run_byte_start + ((uint64_t)run_count * bytes_per_pixel);
        const uint64_t clipped_start = run_byte_start > visible_base ? run_byte_start : visible_base;
        const uint64_t clipped_limit = run_byte_limit < visible_limit ? run_byte_limit : visible_limit;

        offset += 1u;
        if (run_count > pixel_count - produced) {
            return UDL_SINK_ERR_INVALID_COMMAND;
        }
        if (remaining - offset < bytes_per_pixel) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }

        if (clipped_start < clipped_limit) {
            const uint64_t clipped_bytes = clipped_limit - clipped_start;
            const uint64_t clipped_first_pixel = (clipped_start - visible_base) / bytes_per_pixel;
            const uint32_t clipped_pixel_count = (uint32_t)(clipped_bytes / bytes_per_pixel);

            if ((clipped_bytes % bytes_per_pixel) != 0u) {
                return UDL_SINK_ERR_INVALID_COMMAND;
            }

            if (plane == UDL_SINK_PLANE_16) {
                udl_sink_fill_plane16(sink,
                                      (uint32_t)clipped_first_pixel,
                                      clipped_pixel_count,
                                      udl_sink_read_be16(&command[offset]),
                                      damage);
            } else {
                udl_sink_fill_plane8(sink,
                                     (uint32_t)clipped_first_pixel,
                                     clipped_pixel_count,
                                     command[offset],
                                     damage);
            }
        }

        offset += bytes_per_pixel;
        produced += run_count;
    }

    *consumed = offset;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writecopy(struct udl_sink *sink,
                                                      enum udl_sink_plane plane,
                                                      const uint8_t *command,
                                                      size_t remaining,
                                                      size_t *consumed,
                                                      struct udl_sink_damage *damage)
{
    const uint32_t src_byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t pixel_count = udl_sink_count_from_byte(command[5]);
    const uint32_t dst_byte_address = udl_sink_read_addr24(&command[6]);
    enum udl_sink_result result;
    uint16_t copied_pixels[UDL_MAX_COMMAND_PIXELS];
    uint8_t copied_pixels8[UDL_MAX_COMMAND_PIXELS];
    uint32_t src_first_pixel;
    uint32_t dst_first_pixel;
    uint32_t i;

    if (remaining < 9u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_map_range(sink, plane, src_byte_address, pixel_count, &src_first_pixel);
    if (result != UDL_SINK_OK) {
        return result;
    }

    result = udl_sink_map_range(sink, plane, dst_byte_address, pixel_count, &dst_first_pixel);
    if (result != UDL_SINK_OK) {
        return result;
    }

    if (plane == UDL_SINK_PLANE_16) {
        for (i = 0; i < pixel_count; ++i) {
            copied_pixels[i] = udl_sink_read_plane16(sink, src_first_pixel + i);
        }

        for (i = 0; i < pixel_count; ++i) {
            udl_sink_write_plane16(sink, dst_first_pixel + i, copied_pixels[i], damage);
        }
    } else {
        for (i = 0; i < pixel_count; ++i) {
            copied_pixels8[i] = udl_sink_read_plane8(sink, src_first_pixel + i);
        }

        for (i = 0; i < pixel_count; ++i) {
            udl_sink_write_plane8(sink, dst_first_pixel + i, copied_pixels8[i], damage);
        }
    }

    *consumed = 9u;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writerlx(struct udl_sink *sink,
                                                     enum udl_sink_plane plane,
                                                     const uint8_t *command,
                                                     size_t remaining,
                                                     size_t *consumed,
                                                     struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t total_pixels = udl_sink_count_from_byte(command[5]);
    enum udl_sink_result result;
    const uint32_t bytes_per_pixel = udl_sink_plane_bytes_per_pixel(plane);
    uint32_t first_pixel;
    uint32_t produced = 0u;
    size_t offset = 6u;

    if (remaining < 7u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_map_range(sink, plane, byte_address, total_pixels, &first_pixel);
    if (result != UDL_SINK_OK) {
        return result;
    }

    while (produced < total_pixels) {
        uint32_t raw_count;
        size_t raw_bytes;
        uint16_t repeated_pixel16 = 0u;
        uint8_t repeated_pixel8 = 0u;
        uint16_t raw_first_pixel16 = 0u;
        uint8_t raw_first_pixel8 = 0u;
        const uint8_t *raw_data;

        if (offset >= remaining) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }

        raw_count = udl_sink_count_from_byte(command[offset]);
        offset += 1u;
        if (raw_count > total_pixels - produced) {
            return UDL_SINK_ERR_INVALID_COMMAND;
        }

        raw_bytes = (size_t)raw_count * bytes_per_pixel;
        if (remaining - offset < raw_bytes) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }

        raw_data = &command[offset];

        if (raw_count > 0u) {
            if (plane == UDL_SINK_PLANE_16) {
                raw_first_pixel16 = udl_sink_read_be16(raw_data);
            } else {
                raw_first_pixel8 = raw_data[0];
            }
        }

        if (plane == UDL_SINK_PLANE_16) {
            repeated_pixel16 = udl_sink_read_be16(&raw_data[raw_bytes - 2u]);
        } else {
            repeated_pixel8 = raw_data[raw_bytes - 1u];
        }
        offset += raw_bytes;

        if (produced + raw_count == total_pixels) {
            if (plane == UDL_SINK_PLANE_16) {
                udl_sink_blit_plane16_be(sink,
                                         first_pixel + produced,
                                         raw_data,
                                         raw_count,
                                         damage);
            } else {
                udl_sink_blit_plane8(sink,
                                     first_pixel + produced,
                                     raw_data,
                                     raw_count,
                                     damage);
            }
            produced += raw_count;
            break;
        }

        if (offset >= remaining) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }

        {
            const uint32_t repeat_count = command[offset];
            offset += 1u;

            if (repeat_count == 0u || repeat_count > total_pixels - produced) {
                return UDL_SINK_ERR_INVALID_COMMAND;
            }

            if (raw_count == 1u) {
                if (plane == UDL_SINK_PLANE_16 && raw_first_pixel16 == repeated_pixel16) {
                    udl_sink_fill_plane16(sink,
                                          first_pixel + produced,
                                          repeat_count + 1u,
                                          repeated_pixel16,
                                          damage);
                    produced += repeat_count + 1u;
                    continue;
                }
                if (plane == UDL_SINK_PLANE_8 && raw_first_pixel8 == repeated_pixel8) {
                    udl_sink_fill_plane8(sink,
                                         first_pixel + produced,
                                         repeat_count + 1u,
                                         repeated_pixel8,
                                         damage);
                    produced += repeat_count + 1u;
                    continue;
                }
            }

            if (plane == UDL_SINK_PLANE_16) {
                udl_sink_blit_plane16_be(sink,
                                         first_pixel + produced,
                                         raw_data,
                                         raw_count,
                                         damage);
            } else {
                udl_sink_blit_plane8(sink,
                                     first_pixel + produced,
                                     raw_data,
                                     raw_count,
                                     damage);
            }

            produced += raw_count;

            if (plane == UDL_SINK_PLANE_16) {
                udl_sink_fill_plane16(sink,
                                      first_pixel + produced,
                                      repeat_count,
                                      repeated_pixel16,
                                      damage);
            } else {
                udl_sink_fill_plane8(sink,
                                     first_pixel + produced,
                                     repeat_count,
                                     repeated_pixel8,
                                     damage);
            }

            produced += repeat_count;
        }
    }

    *consumed = offset;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_command(struct udl_sink *sink,
                                                    const uint8_t *command,
                                                    size_t remaining,
                                                    size_t *consumed,
                                                    struct udl_sink_damage *damage)
{
    if (remaining < 2u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    switch (command[1]) {
    case UDL_CMD_OPAQUE_E0:
        (void)sink;
        (void)damage;
        *consumed = udl_transport_e0_command_length(command, remaining);
        if (*consumed == 0u) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }
        return UDL_SINK_OK;
    case UDL_CMD_NOP_A0:
        (void)sink;
        (void)damage;
        *consumed = 2u;
        return UDL_SINK_OK;
    case UDL_CMD_UNKNOWN_40:
        (void)sink;
        (void)damage;
        if (remaining < UDL_CMD_UNKNOWN_40_SIZE) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }
        if (command[2] != UDL_CMD_UNKNOWN_40_ARG) {
            return UDL_SINK_ERR_INVALID_COMMAND;
        }
        *consumed = UDL_CMD_UNKNOWN_40_SIZE;
        return UDL_SINK_OK;
    case UDL_CMD_WRITEREG:
        return udl_sink_decode_writereg(sink, command, remaining, consumed, damage);
    case UDL_CMD_WRITERAW8:
        return udl_sink_decode_writeraw(sink, UDL_SINK_PLANE_8, command, remaining, consumed, damage);
    case UDL_CMD_WRITERL8:
        return udl_sink_decode_writerl(sink, UDL_SINK_PLANE_8, command, remaining, consumed, damage);
    case UDL_CMD_WRITECOPY8:
        return udl_sink_decode_writecopy(sink, UDL_SINK_PLANE_8, command, remaining, consumed, damage);
    case UDL_CMD_WRITERLX8:
        return udl_sink_decode_writerlx(sink, UDL_SINK_PLANE_8, command, remaining, consumed, damage);
    case UDL_CMD_WRITECOMP8:
        return udl_sink_decode_writecomp(sink, UDL_SINK_PLANE_8, command, remaining, consumed, damage);
    case UDL_CMD_WRITERAW16:
        return udl_sink_decode_writeraw(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITERL16:
        return udl_sink_decode_writerl(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITECOPY16:
        return udl_sink_decode_writecopy(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITERLX16:
        return udl_sink_decode_writerlx(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITECOMP16:
        return udl_sink_decode_writecomp(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    default:
        return UDL_SINK_ERR_INVALID_COMMAND;
    }
}

void udl_sink_init(struct udl_sink *sink,
                   uint16_t *framebuffer,
                   uint32_t width,
                   uint32_t height,
                   uint32_t stride_pixels)
{
    if (!sink) {
        return;
    }

    memset(sink, 0, sizeof(*sink));
    sink->framebuffer = framebuffer;
    sink->width = width;
    sink->height = height;
    sink->stride_pixels = stride_pixels;
}

void udl_sink_destroy(struct udl_sink *sink)
{
    if (!sink) {
        return;
    }

    free(sink->rgb565_to_xrgb8888_lookup);
    free(sink->plane16);
    free(sink->plane8);
    sink->rgb565_to_xrgb8888_lookup = NULL;
    sink->plane16 = NULL;
    sink->plane8 = NULL;
    sink->plane_pixels = 0u;
}

void udl_sink_attach_xrgb8888_output(struct udl_sink *sink,
                                     uint32_t *framebuffer,
                                     uint32_t stride_pixels)
{
    if (!sink) {
        return;
    }

    sink->framebuffer_xrgb8888 = framebuffer;
    sink->stride_pixels_xrgb8888 = stride_pixels;
    if (framebuffer) {
        udl_sink_ensure_rgb565_lookup(sink);
    }

    if (sink->plane16 && sink->plane8 && framebuffer) {
        udl_sink_compose_all(sink, NULL);
    }
}

void udl_sink_clear_damage(struct udl_sink_damage *damage)
{
    if (!damage) {
        return;
    }

    memset(damage, 0, sizeof(*damage));
}

void udl_transport_init(struct udl_transport *transport,
                        struct udl_sink *sink)
{
    if (!transport) {
        return;
    }

    memset(transport, 0, sizeof(*transport));
    transport->sink = sink;
    transport->collect_detailed_stats = true;
    transport->collect_writerlx16_span_stats = true;
}

void udl_transport_set_detailed_stats(struct udl_transport *transport,
                                      bool enabled)
{
    if (!transport) {
        return;
    }

    transport->collect_detailed_stats = enabled;
}

void udl_transport_set_writerlx16_span_stats(struct udl_transport *transport,
                                             bool enabled)
{
    if (!transport) {
        return;
    }

    transport->collect_writerlx16_span_stats = enabled;
}

void udl_transport_set_writecomp_debug(bool enabled)
{
    udl_sink_writecomp_debug_enabled = enabled;
}

void udl_transport_reset(struct udl_transport *transport)
{
    if (!transport) {
        return;
    }

    transport->pending_len = 0u;
    transport->writecomp_quarantine_active = false;
    transport->writecomp_quarantine_noncomp_ok = 0u;
    transport->writecomp_quarantine_budget = 0u;
    memset(&transport->stats, 0, sizeof(transport->stats));
}

void udl_transport_destroy(struct udl_transport *transport)
{
    if (!transport) {
        return;
    }

    free(transport->pending);
    transport->pending = NULL;
    transport->pending_len = 0u;
    transport->pending_capacity = 0u;
    transport->writecomp_quarantine_active = false;
    transport->writecomp_quarantine_noncomp_ok = 0u;
    transport->writecomp_quarantine_budget = 0u;
    memset(&transport->stats, 0, sizeof(transport->stats));
}

enum udl_transport_result udl_transport_feed(struct udl_transport *transport,
                                             const uint8_t *buffer,
                                             size_t length,
                                             struct udl_sink_damage *damage)
{
    enum udl_transport_result reserve_result;
    enum udl_sink_result sink_result;
    const bool collect_detailed_stats = transport && transport->collect_detailed_stats;
    const size_t pending_prefix_len = transport ? transport->pending_len : 0u;
    size_t offset = 0u;
    struct udl_transport_stats stats_delta;

    memset(&stats_delta, 0, sizeof(stats_delta));

    if (!transport || !transport->sink) {
        return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
    }
    if (length == 0u) {
        if (damage) {
            udl_sink_clear_damage(damage);
        }
        return UDL_TRANSPORT_OK;
    }
    if (!buffer) {
        return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
    }

    if (damage) {
        udl_sink_clear_damage(damage);
    }

    transport->last_feed_pending_prefix_len = pending_prefix_len;
    transport->last_feed_first_resync_offset = 0u;
    transport->last_feed_first_resync_offset_valid = false;
    transport->last_feed_first_resync_reason = UDL_TRANSPORT_RESYNC_NONE;

    if (transport->sink->width == 0u || transport->sink->height == 0u) {
        return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
    }
    if (transport->sink->framebuffer &&
        transport->sink->stride_pixels < transport->sink->width) {
        return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
    }
    if (transport->sink->framebuffer_xrgb8888 &&
        transport->sink->stride_pixels_xrgb8888 < transport->sink->width) {
        return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
    }

    sink_result = udl_sink_ensure_planes(transport->sink);
    if (sink_result != UDL_SINK_OK) {
        return sink_result == UDL_SINK_ERR_NO_MEMORY
            ? UDL_TRANSPORT_ERR_NO_MEMORY
            : UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
    }

    reserve_result = udl_transport_reserve_pending(transport, length);
    if (reserve_result != UDL_TRANSPORT_OK) {
        return reserve_result;
    }

    memcpy(transport->pending + transport->pending_len, buffer, length);
    transport->pending_len += length;

    while (offset < transport->pending_len) {
        uint8_t *pending = transport->pending + offset;
        size_t pending_len = transport->pending_len - offset;
        size_t command_len = 0u;
        size_t consumed = 0u;
        uint8_t command_type = 0u;
        enum udl_stream_parse_result parse_result;
        struct udl_sink_damage command_damage;
        enum udl_sink_result decode_result;

        if (pending[0] != UDL_MSG_BULK) {
            size_t sync_offset = 0u;
            bool needs_more = false;

            if (!udl_transport_find_next_register_anchor(pending,
                                                         pending_len,
                                                         &sync_offset,
                                                         &needs_more) &&
                !udl_transport_find_next_command(pending,
                                                 pending_len,
                                                 &sync_offset,
                                                 &needs_more)) {
                if (needs_more) {
                    if (sync_offset > 0u) {
                        udl_transport_record_first_resync(transport,
                                                          pending_prefix_len,
                                                          offset,
                                                          UDL_TRANSPORT_RESYNC_NON_BULK);
                        stats_delta.dropped_bytes += sync_offset;
                        offset += sync_offset;
                    }
                    break;
                }

                udl_transport_record_first_resync(transport,
                                                  pending_prefix_len,
                                                  offset,
                                                  UDL_TRANSPORT_RESYNC_NON_BULK);
                stats_delta.dropped_bytes += pending_len;
                offset = transport->pending_len;
                break;
            }

            {
                const size_t skipped = sync_offset;

                if (skipped > 0u) {
                    udl_transport_record_first_resync(transport,
                                                      pending_prefix_len,
                                                      offset,
                                                      UDL_TRANSPORT_RESYNC_NON_BULK);
                }
                stats_delta.dropped_bytes += skipped;
                offset += skipped;
                pending = transport->pending + offset;
                pending_len = transport->pending_len - offset;
            }
        }

        if (pending_len < 2u) {
            break;
        }
        if (pending[1] == UDL_MSG_BULK) {
            udl_transport_record_first_resync(transport,
                                              pending_prefix_len,
                                              offset,
                                              UDL_TRANSPORT_RESYNC_DOUBLE_BULK);
            offset += 1u;
            continue;
        }

        command_type = pending[1];

        if (udl_transport_should_quarantine_writecomp(transport) &&
            (command_type == UDL_CMD_WRITEREG ||
             command_type == UDL_CMD_NOP_A0 ||
             command_type == UDL_CMD_UNKNOWN_40 ||
             command_type == UDL_CMD_OPAQUE_E0)) {
            parse_result = udl_transport_next_command_length(pending,
                                                             pending_len,
                                                             &command_len);
            if (parse_result == UDL_STREAM_PARSE_NEED_MORE) {
                break;
            }

            if (parse_result == UDL_STREAM_PARSE_INVALID) {
                command_len = 1u;
            }

            stats_delta.dropped_bytes += command_len;
            udl_transport_consume_writecomp_quarantine_budget(transport, command_len);
            offset += command_len;
            continue;
        }

        if (command_type == UDL_CMD_WRITERLX16) {
            udl_sink_clear_damage(&command_damage);
            if (collect_detailed_stats) {
                udl_transport_record_command_type(&stats_delta, command_type);
            }
            parse_result = udl_transport_decode_writerlx16(transport,
                                                           pending,
                                                           pending_len,
                                                           &command_damage,
                                                           &command_len);
            if (parse_result == UDL_STREAM_PARSE_NEED_MORE) {
                break;
            }
            if (parse_result == UDL_STREAM_PARSE_INVALID) {
                size_t skipped = 1u;
                size_t sync_offset = 0u;
                bool needs_more = false;

                udl_transport_record_first_resync(transport,
                                                  pending_prefix_len,
                                                  offset,
                                                  UDL_TRANSPORT_RESYNC_INVALID_COMMAND);
                stats_delta.decode_errors += 1u;

                if (pending_len > 1u &&
                    udl_transport_find_next_framebuffer_command_strict(pending + 1u,
                                                                       pending_len - 1u,
                                                                       &sync_offset,
                                                                       &needs_more)) {
                    skipped = 1u + sync_offset;
                } else if (needs_more) {
                    skipped = pending_len - 1u;
                    if (skipped == 0u) {
                        break;
                    }
                } else {
                    skipped = pending_len;
                }

                if (skipped > 1u) {
                    stats_delta.dropped_bytes += skipped - 1u;
                }
                offset += skipped;
                continue;
            }

            stats_delta.decoded_commands += 1u;
            if (collect_detailed_stats && !command_damage.touched) {
                stats_delta.no_damage_commands += 1u;
            }
            udl_transport_note_noncompressed_success(transport);
            udl_sink_merge_damage(damage, &command_damage);
            offset += command_len;
            continue;
        } else if (command_type == UDL_CMD_WRITECOMP8 ||
                   command_type == UDL_CMD_WRITECOMP16) {
            enum udl_stream_parse_result probe_result;

            udl_sink_clear_damage(&command_damage);
            if (collect_detailed_stats) {
                udl_transport_record_command_type(&stats_delta, command_type);
            }

            probe_result = udl_transport_probe_writecomp_candidate(pending, pending_len);
            if (probe_result == UDL_STREAM_PARSE_NEED_MORE) {
                break;
            }
            if (probe_result == UDL_STREAM_PARSE_INVALID) {
                size_t skipped = 1u;
                size_t sync_offset = 0u;
                bool needs_more = false;
                bool strict_needs_more = false;

                udl_transport_record_first_resync(transport,
                                                  pending_prefix_len,
                                                  offset,
                                                  UDL_TRANSPORT_RESYNC_INVALID_COMMAND);
                stats_delta.decode_errors += 1u;
                udl_transport_begin_writecomp_quarantine(transport);

                if (pending_len > 1u &&
                    udl_transport_find_next_register_anchor(pending + 1u,
                                                            pending_len - 1u,
                                                            &sync_offset,
                                                            &needs_more)) {
                    skipped = 1u + sync_offset;
                } else if (pending_len > 1u &&
                           udl_transport_find_next_framebuffer_command_strict(pending + 1u,
                                                                              pending_len - 1u,
                                                                              &sync_offset,
                                                                              &needs_more)) {
                    skipped = 1u + sync_offset;
                } else if (pending_len > 1u &&
                           udl_transport_find_next_command_strict(pending + 1u,
                                                                  pending_len - 1u,
                                                                  &sync_offset,
                                                                  &strict_needs_more)) {
                    skipped = 1u + sync_offset;
                } else if (needs_more || strict_needs_more) {
                    skipped = pending_len - 1u;
                    if (skipped == 0u) {
                        break;
                    }
                } else {
                    skipped = pending_len;
                }

                if (skipped > 1u) {
                    stats_delta.dropped_bytes += skipped - 1u;
                }
                udl_transport_consume_writecomp_quarantine_budget(transport, skipped);
                offset += skipped;
                continue;
            }

            decode_result = udl_sink_decode_command(transport->sink,
                                                    pending,
                                                    pending_len,
                                                    &consumed,
                                                    &command_damage);
            if (decode_result == UDL_SINK_ERR_TRUNCATED_COMMAND) {
                break;
            }
            if (decode_result == UDL_SINK_ERR_NO_MEMORY) {
                return UDL_TRANSPORT_ERR_NO_MEMORY;
            }
            if (decode_result != UDL_SINK_OK) {
                size_t skipped = 1u;
                size_t scan_start = 1u;
                size_t sync_offset = 0u;
                size_t early_sync_offset = 0u;
                bool needs_more = false;
                bool early_needs_more = false;
                bool strict_needs_more = false;

                udl_transport_record_first_resync(transport,
                                                  pending_prefix_len,
                                                  offset,
                                                  UDL_TRANSPORT_RESYNC_INVALID_COMMAND);
                stats_delta.decode_errors += 1u;
                udl_transport_begin_writecomp_quarantine(transport);

                /* On Huffman decode failure, use consumed as a lower-bound
                 * scan start rather than an exact skip; random payload bits
                 * can otherwise keep us walking bogus compressed headers.
                 */
                if (consumed > 1u && consumed < pending_len) {
                    scan_start = consumed;
                }

                /* First, prefer an early strong register anchor from the
                 * start of the failed command tail. This avoids skipping past
                 * a legitimate boundary when consumed points into garbage.
                 */
                if (pending_len > 1u &&
                    udl_transport_find_next_register_anchor(pending + 1u,
                                                            pending_len - 1u,
                                                            &early_sync_offset,
                                                            &early_needs_more)) {
                    skipped = 1u + early_sync_offset;
                } else if (pending_len > scan_start &&
                    udl_transport_find_next_register_anchor(pending + scan_start,
                                                            pending_len - scan_start,
                                                            &sync_offset,
                                                            &needs_more)) {
                    skipped = scan_start + sync_offset;
                } else if (pending_len > scan_start &&
                           udl_transport_find_next_framebuffer_command_strict(pending + scan_start,
                                                                              pending_len - scan_start,
                                                                              &sync_offset,
                                                                              &needs_more)) {
                    skipped = scan_start + sync_offset;
                } else if (pending_len > scan_start &&
                           udl_transport_find_next_command_strict(pending + scan_start,
                                                                  pending_len - scan_start,
                                                                  &sync_offset,
                                                                  &strict_needs_more)) {
                    skipped = scan_start + sync_offset;
                } else if (early_needs_more || needs_more || strict_needs_more) {
                    skipped = scan_start + sync_offset;
                    if (skipped >= pending_len) {
                        break;
                    }
                } else if (consumed > 1u && consumed <= pending_len) {
                    skipped = consumed;
                } else {
                    skipped = pending_len;
                }

                if (skipped > 1u) {
                    stats_delta.dropped_bytes += skipped - 1u;
                }
                udl_transport_consume_writecomp_quarantine_budget(transport, skipped);
                offset += skipped;
                continue;
            }

            stats_delta.decoded_commands += 1u;
            if (collect_detailed_stats && !command_damage.touched) {
                stats_delta.no_damage_commands += 1u;
            }
            udl_sink_merge_damage(damage, &command_damage);
            offset += consumed;
            continue;
        } else {
            parse_result = udl_transport_next_command_length(pending,
                                                             pending_len,
                                                             &command_len);
        }
        if (parse_result == UDL_STREAM_PARSE_NEED_MORE) {
            break;
        }
        if (parse_result == UDL_STREAM_PARSE_INVALID) {
            size_t skipped = 1u;
            size_t sync_offset = 0u;
            bool needs_more = false;

            udl_transport_record_first_resync(transport,
                                              pending_prefix_len,
                                              offset,
                                              UDL_TRANSPORT_RESYNC_INVALID_COMMAND);
            stats_delta.decode_errors += 1u;

            if (pending_len > 1u &&
                udl_transport_find_next_register_anchor(pending + 1u,
                                                        pending_len - 1u,
                                                        &sync_offset,
                                                        &needs_more)) {
                skipped = 1u + sync_offset;
            } else if (pending_len > 1u &&
                       udl_transport_find_next_command(pending + 1u,
                                                       pending_len - 1u,
                                                       &sync_offset,
                                                       &needs_more)) {
                skipped = 1u + sync_offset;
            } else if (needs_more) {
                skipped = pending_len - 1u;
                if (skipped == 0u) {
                    break;
                }
            } else {
                skipped = pending_len;
            }

            if (skipped > 1u) {
                stats_delta.dropped_bytes += skipped - 1u;
            }
            offset += skipped;
            continue;
        }

        udl_sink_clear_damage(&command_damage);
        if (collect_detailed_stats) {
            udl_transport_record_command_type(&stats_delta, command_type);
        }
        if (collect_detailed_stats && command_type == UDL_CMD_WRITEREG && command_len >= 4u) {
            const uint8_t reg = pending[2];
            const uint8_t value = pending[3];

            if (transport->sink->registers[reg] == value) {
                stats_delta.writereg_redundant_commands += 1u;
            }
        }
        decode_result = udl_sink_decode_command(transport->sink,
                                                pending,
                                                command_len,
                                                &consumed,
                                                &command_damage);
        if (decode_result != UDL_SINK_OK) {
            stats_delta.decode_errors += 1u;
            offset += command_len;
            continue;
        }
        if (consumed != command_len) {
            stats_delta.decode_errors += 1u;
            offset += command_len;
            continue;
        }

        stats_delta.decoded_commands += 1u;
        if (collect_detailed_stats && !command_damage.touched) {
            stats_delta.no_damage_commands += 1u;
        }
        if (udl_transport_is_framebuffer_command(command_type)) {
            udl_transport_note_noncompressed_success(transport);
        }
        udl_sink_merge_damage(damage, &command_damage);
        offset += command_len;
    }

    udl_transport_compact_pending(transport, offset);
    transport->stats.decoded_commands += stats_delta.decoded_commands;
    transport->stats.decode_errors += stats_delta.decode_errors;
    transport->stats.dropped_bytes += stats_delta.dropped_bytes;
    transport->stats.writereg_commands += stats_delta.writereg_commands;
    transport->stats.writereg_redundant_commands += stats_delta.writereg_redundant_commands;
    transport->stats.writeraw8_commands += stats_delta.writeraw8_commands;
    transport->stats.writerl8_commands += stats_delta.writerl8_commands;
    transport->stats.writecopy8_commands += stats_delta.writecopy8_commands;
    transport->stats.writerlx8_commands += stats_delta.writerlx8_commands;
    transport->stats.writeraw16_commands += stats_delta.writeraw16_commands;
    transport->stats.writerl16_commands += stats_delta.writerl16_commands;
    transport->stats.writecopy16_commands += stats_delta.writecopy16_commands;
    transport->stats.writerlx16_commands += stats_delta.writerlx16_commands;
    transport->stats.no_damage_commands += stats_delta.no_damage_commands;

    return UDL_TRANSPORT_OK;
}

struct udl_transport_stats udl_transport_get_stats(const struct udl_transport *transport)
{
    struct udl_transport_stats stats;

    memset(&stats, 0, sizeof(stats));
    if (!transport) {
        return stats;
    }

    return transport->stats;
}

enum udl_sink_result udl_sink_decode_buffer(struct udl_sink *sink,
                                            const uint8_t *buffer,
                                            size_t length,
                                            struct udl_sink_damage *damage)
{
    size_t offset = 0u;
    enum udl_sink_result result;

    if (!sink || !buffer) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }
    if (sink->width == 0u || sink->height == 0u) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }
    if (sink->framebuffer && sink->stride_pixels < sink->width) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }
    if (sink->framebuffer_xrgb8888 && sink->stride_pixels_xrgb8888 < sink->width) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }

    result = udl_sink_ensure_planes(sink);
    if (result != UDL_SINK_OK) {
        return result;
    }

    while (offset < length) {
        size_t consumed = 0u;
        size_t i;

        if (buffer[offset] != UDL_MSG_BULK) {
            return UDL_SINK_ERR_INVALID_COMMAND;
        }

        if (offset + 1u >= length) {
            break;
        }

        if (buffer[offset + 1u] == UDL_MSG_BULK) {
            for (i = offset + 1u; i < length; ++i) {
                if (buffer[i] != UDL_MSG_BULK) {
                    return UDL_SINK_ERR_INVALID_COMMAND;
                }
            }
            break;
        }

        result = udl_sink_decode_command(sink,
                                         &buffer[offset],
                                         length - offset,
                                         &consumed,
                                         damage);
        if (result != UDL_SINK_OK) {
            return result;
        }

        offset += consumed;
    }

    return UDL_SINK_OK;
}

uint8_t udl_sink_get_color_depth(const struct udl_sink *sink)
{
    if (!sink) {
        return UDL_COLORDEPTH_16BPP;
    }

    return sink->registers[UDL_REG_COLORDEPTH];
}

uint16_t udl_sink_get_hpixels(const struct udl_sink *sink)
{
    if (!sink) {
        return 0u;
    }

    return udl_sink_read_reg16(sink, UDL_REG_HPIXELS);
}

uint16_t udl_sink_get_vpixels(const struct udl_sink *sink)
{
    if (!sink) {
        return 0u;
    }

    return udl_sink_read_reg16(sink, UDL_REG_VPIXELS);
}

uint32_t udl_sink_get_base16bpp(const struct udl_sink *sink)
{
    if (!sink) {
        return 0u;
    }

    return ((uint32_t)sink->registers[UDL_REG_BASE16BPP_ADDR2] << 16) |
           ((uint32_t)sink->registers[UDL_REG_BASE16BPP_ADDR1] << 8) |
           (uint32_t)sink->registers[UDL_REG_BASE16BPP_ADDR0];
}

uint32_t udl_sink_get_base8bpp(const struct udl_sink *sink)
{
    if (!sink) {
        return 0u;
    }

    return ((uint32_t)sink->registers[UDL_REG_BASE8BPP_ADDR2] << 16) |
           ((uint32_t)sink->registers[UDL_REG_BASE8BPP_ADDR1] << 8) |
           (uint32_t)sink->registers[UDL_REG_BASE8BPP_ADDR0];
}

const char *udl_sink_result_string(enum udl_sink_result result)
{
    switch (result) {
    case UDL_SINK_OK:
        return "ok";
    case UDL_SINK_ERR_INVALID_ARGUMENT:
        return "invalid argument";
    case UDL_SINK_ERR_NO_MEMORY:
        return "no memory";
    case UDL_SINK_ERR_TRUNCATED_COMMAND:
        return "truncated command";
    case UDL_SINK_ERR_UNSUPPORTED_COMMAND:
        return "unsupported command";
    case UDL_SINK_ERR_INVALID_COMMAND:
        return "invalid command";
    case UDL_SINK_ERR_ADDRESS_RANGE:
        return "address range";
    default:
        return "unknown";
    }
}

const char *udl_transport_result_string(enum udl_transport_result result)
{
    switch (result) {
    case UDL_TRANSPORT_OK:
        return "ok";
    case UDL_TRANSPORT_ERR_INVALID_ARGUMENT:
        return "invalid argument";
    case UDL_TRANSPORT_ERR_NO_MEMORY:
        return "no memory";
    default:
        return "unknown";
    }
}