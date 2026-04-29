#include "udl_sink.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum {
    TEST_UDL_MSG_BULK = 0xafu,
    TEST_UDL_CMD_WRITERLX16 = 0x6bu,
    TEST_UDL_MAX_COMMAND_PIXELS = 256u,
};

static uint8_t encode_count_byte(uint32_t count)
{
    assert(count >= 1u);
    assert(count <= TEST_UDL_MAX_COMMAND_PIXELS);
    return count == TEST_UDL_MAX_COMMAND_PIXELS ? 0u : (uint8_t)count;
}

static void write_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}

static size_t encode_reference_writerlx16(const uint16_t *pixels,
                                          uint32_t pixel_count,
                                          uint32_t device_byte_offset,
                                          uint8_t *buffer)
{
    uint8_t *cmd = buffer;
    const uint16_t *pixel = pixels;
    const uint16_t *raw_pixel_start = pixels;
    uint8_t *total_pixels_count_byte;
    uint8_t *raw_pixels_count_byte;

    assert(pixel_count >= 1u);
    assert(pixel_count <= TEST_UDL_MAX_COMMAND_PIXELS);

    *cmd++ = TEST_UDL_MSG_BULK;
    *cmd++ = TEST_UDL_CMD_WRITERLX16;
    *cmd++ = (uint8_t)(device_byte_offset >> 16);
    *cmd++ = (uint8_t)(device_byte_offset >> 8);
    *cmd++ = (uint8_t)device_byte_offset;

    total_pixels_count_byte = cmd++;
    raw_pixels_count_byte = cmd++;

    while ((uint32_t)(pixel - pixels) < pixel_count) {
        const uint16_t *run_start = pixel;
        const uint16_t repeated_pixel = *pixel;

        write_be16(cmd, repeated_pixel);
        cmd += 2;
        pixel += 1;

        while ((uint32_t)(pixel - pixels) < pixel_count && *pixel == repeated_pixel) {
            pixel += 1;
        }

        if (pixel > run_start + 1) {
            *raw_pixels_count_byte = encode_count_byte((uint32_t)((run_start - raw_pixel_start) + 1));
            *cmd++ = encode_count_byte((uint32_t)(pixel - run_start) - 1u);

            raw_pixel_start = pixel;
            if ((uint32_t)(pixel - pixels) < pixel_count) {
                raw_pixels_count_byte = cmd++;
            }
        }
    }

    if (pixel > raw_pixel_start) {
        *raw_pixels_count_byte = encode_count_byte((uint32_t)(pixel - raw_pixel_start));
    } else {
        cmd -= 1;
    }

    *total_pixels_count_byte = encode_count_byte(pixel_count);
    return (size_t)(cmd - buffer);
}

static size_t encode_reference_surface(const uint16_t *pixels,
                                       uint32_t width,
                                       uint32_t height,
                                       uint32_t stride_pixels,
                                       uint8_t *buffer,
                                       size_t capacity)
{
    size_t total_bytes = 0u;
    uint32_t y;

    for (y = 0; y < height; ++y) {
        const uint16_t *row = pixels + (size_t)y * stride_pixels;
        uint32_t x = 0u;

        while (x < width) {
            uint8_t command[7u + 2u * TEST_UDL_MAX_COMMAND_PIXELS + TEST_UDL_MAX_COMMAND_PIXELS];
            const uint32_t chunk_pixels = (width - x) > TEST_UDL_MAX_COMMAND_PIXELS
                                              ? TEST_UDL_MAX_COMMAND_PIXELS
                                              : (width - x);
            const uint32_t device_byte_offset = ((y * width) + x) * 2u;
            const size_t command_bytes = encode_reference_writerlx16(row + x,
                                                                     chunk_pixels,
                                                                     device_byte_offset,
                                                                     command);

            assert(total_bytes + command_bytes <= capacity);
            memcpy(buffer + total_bytes, command, command_bytes);
            total_bytes += command_bytes;
            x += chunk_pixels;
        }
    }

    return total_bytes;
}

static void test_writerlx16_decodes_damage(void)
{
    uint16_t framebuffer[8] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint8_t packet[] = {
        0xaf, 0x6b, 0x00, 0x00, 0x00, 0x05, 0x03,
        0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x02,
        0xaf, 0xaf, 0xaf,
    };

    udl_sink_init(&sink, framebuffer, 8u, 1u, 8u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(framebuffer[0] == 0x1111u);
    assert(framebuffer[1] == 0x2222u);
    assert(framebuffer[2] == 0x3333u);
    assert(framebuffer[3] == 0x3333u);
    assert(framebuffer[4] == 0x3333u);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 5u);
    assert(damage.y2 == 1u);
    assert(damage.pixel_count == 5u);
}

static void test_writeraw16_and_writerl16_wrap_rows(void)
{
    uint16_t framebuffer[8] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint8_t packet[] = {
        0xaf, 0x68, 0x00, 0x00, 0x02, 0x02, 0x12, 0x34, 0x56, 0x78,
        0xaf, 0x69, 0x00, 0x00, 0x06, 0x02, 0x9a, 0xbc,
    };

    udl_sink_init(&sink, framebuffer, 4u, 2u, 4u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(framebuffer[1] == 0x1234u);
    assert(framebuffer[2] == 0x5678u);
    assert(framebuffer[3] == 0x9abcu);
    assert(framebuffer[4] == 0x9abcu);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 4u);
    assert(damage.y2 == 2u);
    assert(damage.pixel_count == 4u);
}

static void test_writereg_and_writecopy16_track_state(void)
{
    uint16_t framebuffer[8] = {0x0101u, 0x0202u, 0x0303u, 0x0404u, 0, 0, 0, 0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint8_t packet[] = {
        0xaf, 0x20, 0x0f, 0x00,
        0xaf, 0x20, 0x10, 0x08,
        0xaf, 0x20, 0x17, 0x00,
        0xaf, 0x20, 0x18, 0x01,
        0xaf, 0x20, 0x20, 0x12,
        0xaf, 0x20, 0x21, 0x34,
        0xaf, 0x20, 0x22, 0x56,
        0xaf, 0x6a, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x04,
    };

    udl_sink_init(&sink, framebuffer, 4u, 2u, 4u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(framebuffer[2] == 0x0101u);
    assert(framebuffer[3] == 0x0202u);
    assert(udl_sink_get_hpixels(&sink) == 8u);
    assert(udl_sink_get_vpixels(&sink) == 1u);
    assert(udl_sink_get_base16bpp(&sink) == 0x123456u);
    assert(damage.touched);
    assert(damage.x1 == 2u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 4u);
    assert(damage.y2 == 1u);
    assert(damage.pixel_count == 2u);
}

static void test_unsupported_8bit_command_returns_error(void)
{
    uint16_t framebuffer[4] = {0};
    struct udl_sink sink;
    const uint8_t packet[] = {0xaf, 0x60, 0x00, 0x00, 0x00, 0x01, 0x00};

    udl_sink_init(&sink, framebuffer, 2u, 2u, 2u);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), NULL) ==
           UDL_SINK_ERR_UNSUPPORTED_COMMAND);
}

static void test_reference_roundtrip_rgb565_surface(void)
{
    const uint32_t width = 9u;
    const uint32_t height = 3u;
    uint16_t source[27] = {
        0x0001u, 0x0002u, 0x0002u, 0x0002u, 0x00f0u, 0x00f1u, 0x00f2u, 0x00f2u, 0x00f3u,
        0x1234u, 0x1234u, 0x4567u, 0x4568u, 0x4569u, 0x4569u, 0x4569u, 0x456au, 0x456bu,
        0xab01u, 0xab02u, 0xab03u, 0xab03u, 0xab04u, 0xab05u, 0xab05u, 0xab05u, 0xab06u,
    };
    uint16_t decoded[27] = {0};
    uint8_t packet[512] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const size_t packet_size = encode_reference_surface(source,
                                                        width,
                                                        height,
                                                        width,
                                                        packet,
                                                        sizeof(packet));

    udl_sink_init(&sink, decoded, width, height, width);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, packet_size, &damage) == UDL_SINK_OK);
    assert(memcmp(source, decoded, sizeof(source)) == 0);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == width);
    assert(damage.y2 == height);
    assert(damage.pixel_count == width * height);
}

int main(void)
{
    test_writerlx16_decodes_damage();
    test_writeraw16_and_writerl16_wrap_rows();
    test_writereg_and_writecopy16_track_state();
    test_unsupported_8bit_command_returns_error();
    test_reference_roundtrip_rgb565_surface();
    return 0;
}