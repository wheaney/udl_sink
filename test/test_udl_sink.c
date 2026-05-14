#include "udl_sink.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum {
    TEST_UDL_MSG_BULK = 0xafu,
    TEST_UDL_CMD_WRITERAW8 = 0x60u,
    TEST_UDL_CMD_WRITERL8 = 0x61u,
    TEST_UDL_CMD_WRITECOPY8 = 0x62u,
    TEST_UDL_CMD_WRITERLX8 = 0x63u,
    TEST_UDL_CMD_WRITERAW16 = 0x68u,
    TEST_UDL_CMD_WRITECOPY16 = 0x6au,
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

static uint16_t rgb888_to_rgb565(uint32_t xrgb8888)
{
    return (uint16_t)((((xrgb8888 >> 16) & 0xf8u) << 8) |
                      (((xrgb8888 >> 8) & 0xfcu) << 3) |
                      ((xrgb8888 & 0xf8u) >> 3));
}

static uint16_t rgb888_to_high565(uint32_t xrgb8888)
{
    const uint8_t red = (uint8_t)(xrgb8888 >> 16);
    const uint8_t green = (uint8_t)(xrgb8888 >> 8);
    const uint8_t blue = (uint8_t)xrgb8888;

    return (uint16_t)((((uint16_t)red >> 3) << 11) |
                      (((uint16_t)green >> 2) << 5) |
                      ((uint16_t)blue >> 3));
}

static uint8_t rgb888_to_low323(uint32_t xrgb8888)
{
    const uint8_t red = (uint8_t)(xrgb8888 >> 16);
    const uint8_t green = (uint8_t)(xrgb8888 >> 8);
    const uint8_t blue = (uint8_t)xrgb8888;

    return (uint8_t)(((red & 0x07u) << 5) |
                     ((green & 0x03u) << 3) |
                     (blue & 0x07u));
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
    udl_sink_destroy(&sink);
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
    udl_sink_destroy(&sink);
}

static void test_writeraw16_compose_xrgb8888_16bpp_noop(void)
{
    uint16_t framebuffer[4] = {0};
    uint32_t xrgb8888[4] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint8_t packet[] = {
        0xaf, TEST_UDL_CMD_WRITERAW16, 0x00, 0x00, 0x00, 0x04,
        0xf8, 0x00,
        0x07, 0xe0,
        0x00, 0x1f,
        0xff, 0xff,
    };

    udl_sink_init(&sink, framebuffer, 4u, 1u, 4u);
    udl_sink_attach_xrgb8888_output(&sink, xrgb8888, 4u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(framebuffer[0] == 0xf800u);
    assert(framebuffer[1] == 0x07e0u);
    assert(framebuffer[2] == 0x001fu);
    assert(framebuffer[3] == 0xffffu);
    assert(xrgb8888[0] == 0xffff0000u);
    assert(xrgb8888[1] == 0xff00ff00u);
    assert(xrgb8888[2] == 0xff0000ffu);
    assert(xrgb8888[3] == 0xffffffffu);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 4u);
    assert(damage.y2 == 1u);
    assert(damage.pixel_count == 4u);

    udl_sink_clear_damage(&damage);
    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(!damage.touched);

    udl_sink_destroy(&sink);
}

static void test_writereg_and_writecopy16_track_state(void)
{
    uint16_t framebuffer[8] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint8_t packet[] = {
        0xaf, 0x20, 0x0f, 0x00,
        0xaf, 0x20, 0x10, 0x08,
        0xaf, 0x20, 0x17, 0x00,
        0xaf, 0x20, 0x18, 0x01,
        0xaf, 0x20, 0x20, 0x00,
        0xaf, 0x20, 0x21, 0x00,
        0xaf, 0x20, 0x22, 0x04,
        0xaf, TEST_UDL_CMD_WRITERAW16, 0x00, 0x00, 0x04, 0x02, 0x01, 0x01, 0x02, 0x02,
        0xaf, TEST_UDL_CMD_WRITECOPY16, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x08,
    };

    udl_sink_init(&sink, framebuffer, 4u, 2u, 4u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(framebuffer[0] == 0x0101u);
    assert(framebuffer[1] == 0x0202u);
    assert(framebuffer[2] == 0x0101u);
    assert(framebuffer[3] == 0x0202u);
    assert(udl_sink_get_hpixels(&sink) == 8u);
    assert(udl_sink_get_vpixels(&sink) == 1u);
    assert(udl_sink_get_base16bpp(&sink) == 0x000004u);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 4u);
    assert(damage.y2 == 1u);
    assert(damage.pixel_count == 4u);
    udl_sink_destroy(&sink);
}

static void test_invalid_command_returns_error(void)
{
    uint16_t framebuffer[4] = {0};
    struct udl_sink sink;
    const uint8_t packet[] = {0xaf, 0x7f};

    udl_sink_init(&sink, framebuffer, 2u, 2u, 2u);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), NULL) == UDL_SINK_ERR_INVALID_COMMAND);
    udl_sink_destroy(&sink);
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
    udl_sink_destroy(&sink);
}

static void test_24bpp_raw8_base_offsets_compose_xrgb8888(void)
{
    const uint32_t color0 = 0xff123456u;
    const uint32_t color1 = 0xffabcdefu;
    uint16_t framebuffer[2] = {0};
    uint32_t xrgb8888[2] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint8_t packet[] = {
        0xaf, 0x20, 0x00, 0x01,
        0xaf, 0x20, 0x20, 0x00,
        0xaf, 0x20, 0x21, 0x00,
        0xaf, 0x20, 0x22, 0x04,
        0xaf, 0x20, 0x26, 0x00,
        0xaf, 0x20, 0x27, 0x00,
        0xaf, 0x20, 0x28, 0x08,
        0xaf, TEST_UDL_CMD_WRITERAW16, 0x00, 0x00, 0x04, 0x02,
        (uint8_t)(rgb888_to_high565(color0) >> 8), (uint8_t)rgb888_to_high565(color0),
        (uint8_t)(rgb888_to_high565(color1) >> 8), (uint8_t)rgb888_to_high565(color1),
        0xaf, TEST_UDL_CMD_WRITERAW8, 0x00, 0x00, 0x08, 0x02,
        rgb888_to_low323(color0), rgb888_to_low323(color1),
    };

    udl_sink_init(&sink, framebuffer, 2u, 1u, 2u);
    udl_sink_attach_xrgb8888_output(&sink, xrgb8888, 2u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(udl_sink_get_color_depth(&sink) == 1u);
    assert(udl_sink_get_base16bpp(&sink) == 0x000004u);
    assert(udl_sink_get_base8bpp(&sink) == 0x000008u);
    assert(xrgb8888[0] == color0);
    assert(xrgb8888[1] == color1);
    assert(framebuffer[0] == rgb888_to_rgb565(color0));
    assert(framebuffer[1] == rgb888_to_rgb565(color1));
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 2u);
    assert(damage.y2 == 1u);
    udl_sink_destroy(&sink);
}

static void test_writerl8_and_writecopy8_compose_xrgb8888(void)
{
    const uint32_t color = 0xff88aaeeu;
    uint16_t framebuffer[4] = {0};
    uint32_t xrgb8888[4] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint16_t high565 = rgb888_to_high565(color);
    const uint8_t low323 = rgb888_to_low323(color);
    const uint8_t packet[] = {
        0xaf, 0x20, 0x00, 0x01,
        0xaf, 0x20, 0x20, 0x00,
        0xaf, 0x20, 0x21, 0x00,
        0xaf, 0x20, 0x22, 0x00,
        0xaf, 0x20, 0x26, 0x00,
        0xaf, 0x20, 0x27, 0x00,
        0xaf, 0x20, 0x28, 0x08,
        0xaf, 0x69, 0x00, 0x00, 0x00, 0x04, (uint8_t)(high565 >> 8), (uint8_t)high565,
        0xaf, TEST_UDL_CMD_WRITERL8, 0x00, 0x00, 0x08, 0x02, low323,
        0xaf, TEST_UDL_CMD_WRITECOPY8, 0x00, 0x00, 0x08, 0x02, 0x00, 0x00, 0x0a,
    };
    uint32_t index;

    udl_sink_init(&sink, framebuffer, 4u, 1u, 4u);
    udl_sink_attach_xrgb8888_output(&sink, xrgb8888, 4u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    for (index = 0; index < 4u; ++index) {
        assert(xrgb8888[index] == color);
        assert(framebuffer[index] == rgb888_to_rgb565(color));
    }
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 4u);
    assert(damage.y2 == 1u);
    udl_sink_destroy(&sink);
}

static void test_writerlx8_compose_xrgb8888(void)
{
    const uint32_t color0 = 0xff204060u;
    const uint32_t color1 = 0xff214161u;
    uint16_t framebuffer[3] = {0};
    uint32_t xrgb8888[3] = {0};
    struct udl_sink sink;
    struct udl_sink_damage damage;
    const uint16_t high565 = rgb888_to_high565(color0);
    const uint8_t low0 = rgb888_to_low323(color0);
    const uint8_t low1 = rgb888_to_low323(color1);
    const uint8_t packet[] = {
        0xaf, 0x20, 0x00, 0x01,
        0xaf, 0x20, 0x20, 0x00,
        0xaf, 0x20, 0x21, 0x00,
        0xaf, 0x20, 0x22, 0x00,
        0xaf, 0x20, 0x26, 0x00,
        0xaf, 0x20, 0x27, 0x00,
        0xaf, 0x20, 0x28, 0x06,
        0xaf, 0x69, 0x00, 0x00, 0x00, 0x03, (uint8_t)(high565 >> 8), (uint8_t)high565,
        0xaf, TEST_UDL_CMD_WRITERLX8, 0x00, 0x00, 0x06, 0x03, 0x02, low0, low1, 0x01,
    };

    udl_sink_init(&sink, framebuffer, 3u, 1u, 3u);
    udl_sink_attach_xrgb8888_output(&sink, xrgb8888, 3u);
    udl_sink_clear_damage(&damage);

    assert(udl_sink_decode_buffer(&sink, packet, sizeof(packet), &damage) == UDL_SINK_OK);
    assert(xrgb8888[0] == color0);
    assert(xrgb8888[1] == color1);
    assert(xrgb8888[2] == color1);
    assert(framebuffer[0] == rgb888_to_rgb565(color0));
    assert(framebuffer[1] == rgb888_to_rgb565(color1));
    assert(framebuffer[2] == rgb888_to_rgb565(color1));
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 3u);
    assert(damage.y2 == 1u);
    udl_sink_destroy(&sink);
}

static void test_transport_reassembles_split_commands(void)
{
    uint16_t framebuffer[4] = {0};
    struct udl_sink sink;
    struct udl_transport transport;
    struct udl_sink_damage damage;
    struct udl_transport_stats stats;
    const uint8_t packet[] = {
        0xaf, 0x68, 0x00, 0x00, 0x00, 0x02, 0x12, 0x34, 0x56, 0x78,
        0xaf, 0x69, 0x00, 0x00, 0x04, 0x02, 0x9a, 0xbc,
    };

    udl_sink_init(&sink, framebuffer, 4u, 1u, 4u);
    udl_transport_init(&transport, &sink);

    assert(udl_transport_feed(&transport, packet, 5u, &damage) == UDL_TRANSPORT_OK);
    assert(!damage.touched);

    assert(udl_transport_feed(&transport, packet + 5u, sizeof(packet) - 5u, &damage) == UDL_TRANSPORT_OK);
    assert(framebuffer[0] == 0x1234u);
    assert(framebuffer[1] == 0x5678u);
    assert(framebuffer[2] == 0x9abcu);
    assert(framebuffer[3] == 0x9abcu);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 4u);
    assert(damage.y2 == 1u);
    assert(damage.pixel_count == 4u);

    stats = udl_transport_get_stats(&transport);
    assert(stats.decoded_commands == 2u);
    assert(stats.decode_errors == 0u);
    assert(stats.dropped_bytes == 0u);

    udl_transport_destroy(&transport);
    udl_sink_destroy(&sink);
}

static void test_transport_drops_noise_and_recovers_from_invalid_framing(void)
{
    uint16_t framebuffer[2] = {0};
    struct udl_sink sink;
    struct udl_transport transport;
    struct udl_sink_damage damage;
    struct udl_transport_stats stats;
    const uint8_t packet[] = {
        0x55, 0x66,
        0xaf, 0x7f,
        0xaf, 0x68, 0x00, 0x00, 0x00, 0x02, 0xaa, 0xaa, 0xbb, 0xbb,
    };

    udl_sink_init(&sink, framebuffer, 2u, 1u, 2u);
    udl_transport_init(&transport, &sink);

    assert(udl_transport_feed(&transport, packet, sizeof(packet), &damage) == UDL_TRANSPORT_OK);
    assert(framebuffer[0] == 0xaaaau);
    assert(framebuffer[1] == 0xbbbbu);
    assert(damage.touched);
    assert(damage.x1 == 0u);
    assert(damage.y1 == 0u);
    assert(damage.x2 == 2u);
    assert(damage.y2 == 1u);
    assert(damage.pixel_count == 2u);

    stats = udl_transport_get_stats(&transport);
    assert(stats.decoded_commands == 1u);
    assert(stats.decode_errors == 1u);
    assert(stats.dropped_bytes == 3u);

    udl_transport_destroy(&transport);
    udl_sink_destroy(&sink);
}

int main(void)
{
    test_writerlx16_decodes_damage();
    test_writeraw16_and_writerl16_wrap_rows();
    test_writeraw16_compose_xrgb8888_16bpp_noop();
    test_writereg_and_writecopy16_track_state();
    test_invalid_command_returns_error();
    test_reference_roundtrip_rgb565_surface();
    test_24bpp_raw8_base_offsets_compose_xrgb8888();
    test_writerl8_and_writecopy8_compose_xrgb8888();
    test_writerlx8_compose_xrgb8888();
    test_transport_reassembles_split_commands();
    test_transport_drops_noise_and_recovers_from_invalid_framing();
    return 0;
}