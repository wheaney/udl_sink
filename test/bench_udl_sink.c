#define _POSIX_C_SOURCE 199309L
#include "udl_sink.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

enum {
    BENCH_UDL_MSG_BULK = 0xafu,
    BENCH_UDL_CMD_WRITERLX16 = 0x6bu,
    BENCH_UDL_MAX_COMMAND_PIXELS = 256u,
    BENCH_PIXEL_COUNT = 256u,
    BENCH_ITERATIONS = 200000u,
};

static uint8_t encode_count_byte(uint32_t count)
{
    return count == BENCH_UDL_MAX_COMMAND_PIXELS ? 0u : (uint8_t)count;
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

    *cmd++ = BENCH_UDL_MSG_BULK;
    *cmd++ = BENCH_UDL_CMD_WRITERLX16;
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

static uint64_t monotonic_nanoseconds(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0u;
    }

    return ((uint64_t)ts.tv_sec * 1000000000ull) + (uint64_t)ts.tv_nsec;
}

static void fill_repeat_pattern(uint16_t *pixels, uint16_t pixel)
{
    uint32_t index;

    for (index = 0u; index < BENCH_PIXEL_COUNT; ++index) {
        pixels[index] = pixel;
    }
}

static void fill_raw_pattern(uint16_t *pixels, uint16_t seed)
{
    uint32_t index;

    for (index = 0u; index < BENCH_PIXEL_COUNT; ++index) {
        pixels[index] = (uint16_t)(seed + (uint16_t)(index * 37u) + (uint16_t)(index >> 3));
        if (index > 0u && pixels[index] == pixels[index - 1u]) {
            pixels[index] ^= 0x001fu;
        }
    }
}

static void fill_cursorish_pattern(uint16_t *pixels, uint32_t cursor_x)
{
    uint32_t index;

    for (index = 0u; index < BENCH_PIXEL_COUNT; ++index) {
        uint16_t pixel = 0x2104u;

        if (index >= cursor_x && index < cursor_x + 12u) {
            pixel = (index == cursor_x || index == cursor_x + 11u) ? 0xffffu : 0x0000u;
        } else if (index == cursor_x + 12u || index == cursor_x + 13u) {
            pixel = 0xffffu;
        }

        pixels[index] = pixel;
    }
}

static void fill_short_raw_bridge_pattern(uint16_t *pixels,
                                          uint16_t base,
                                          uint32_t shift)
{
    uint32_t index = 0u;
    uint32_t segment = 0u;

    while (index < BENCH_PIXEL_COUNT) {
        const uint32_t repeat_count = 7u + ((segment + shift) % 19u);
        const uint32_t raw_count = 1u + ((segment + shift) % 4u);
        uint32_t repeat_index;
        uint32_t raw_index;
        const uint16_t repeat_pixel = (uint16_t)(base + (uint16_t)(segment * 9u));

        for (repeat_index = 0u;
             repeat_index < repeat_count && index < BENCH_PIXEL_COUNT;
             ++repeat_index, ++index) {
            pixels[index] = repeat_pixel;
        }

        for (raw_index = 0u;
             raw_index < raw_count && index < BENCH_PIXEL_COUNT;
             ++raw_index, ++index) {
            pixels[index] = (uint16_t)(repeat_pixel + 0x0020u + (uint16_t)(raw_index * 3u));
        }

        segment += 1u;
    }
}

static void require_transport_ok(enum udl_transport_result result)
{
    if (result != UDL_TRANSPORT_OK) {
        fprintf(stderr, "transport error: %s\n", udl_transport_result_string(result));
        exit(1);
    }
}

static void run_case(const char *name,
                     const uint16_t *packet_a_pixels,
                     const uint16_t *packet_b_pixels,
                     int alternate_packets)
{
    uint16_t framebuffer[BENCH_PIXEL_COUNT] = {0};
    uint32_t xrgb8888[BENCH_PIXEL_COUNT] = {0};
    uint8_t packet_a[1024];
    uint8_t packet_b[1024];
    struct udl_sink sink;
    struct udl_transport transport;
    struct udl_sink_damage damage;
    struct udl_transport_stats stats;
    const size_t packet_a_len = encode_reference_writerlx16(packet_a_pixels,
                                                            BENCH_PIXEL_COUNT,
                                                            0u,
                                                            packet_a);
    const size_t packet_b_len = packet_b_pixels
        ? encode_reference_writerlx16(packet_b_pixels, BENCH_PIXEL_COUNT, 0u, packet_b)
        : packet_a_len;
    uint64_t start_nsec;
    uint64_t end_nsec;
    uint32_t iteration;

    udl_sink_init(&sink, framebuffer, BENCH_PIXEL_COUNT, 1u, BENCH_PIXEL_COUNT);
    udl_sink_attach_xrgb8888_output(&sink, xrgb8888, BENCH_PIXEL_COUNT);
    udl_transport_init(&transport, &sink);

    require_transport_ok(udl_transport_feed(&transport, packet_a, packet_a_len, &damage));
    udl_transport_reset(&transport);

    start_nsec = monotonic_nanoseconds();
    for (iteration = 0u; iteration < BENCH_ITERATIONS; ++iteration) {
        const uint8_t *packet = packet_a;
        size_t packet_len = packet_a_len;

        if (alternate_packets && (iteration & 1u) != 0u) {
            packet = packet_b;
            packet_len = packet_b_len;
        }

        require_transport_ok(udl_transport_feed(&transport, packet, packet_len, &damage));
    }
    end_nsec = monotonic_nanoseconds();
    stats = udl_transport_get_stats(&transport);

    printf(
        "%-20s %.2f ns/cmd  nodmg=%llu/%llu  raw_px=%llu  repeat_px=%llu\n",
        name,
        (double)(end_nsec - start_nsec) / (double)BENCH_ITERATIONS,
        (unsigned long long)stats.no_damage_commands,
        (unsigned long long)stats.decoded_commands,
        (unsigned long long)stats.writerlx16_raw_pixels,
        (unsigned long long)stats.writerlx16_repeat_pixels);

    udl_transport_destroy(&transport);
    udl_sink_destroy(&sink);
}

int main(void)
{
    uint16_t repeat_a[BENCH_PIXEL_COUNT];
    uint16_t repeat_b[BENCH_PIXEL_COUNT];
    uint16_t raw_a[BENCH_PIXEL_COUNT];
    uint16_t raw_b[BENCH_PIXEL_COUNT];
    uint16_t short_raw_a[BENCH_PIXEL_COUNT];
    uint16_t short_raw_b[BENCH_PIXEL_COUNT];
    uint16_t cursor_a[BENCH_PIXEL_COUNT];
    uint16_t cursor_b[BENCH_PIXEL_COUNT];

    fill_repeat_pattern(repeat_a, 0x39e7u);
    fill_repeat_pattern(repeat_b, 0x7befu);
    fill_raw_pattern(raw_a, 0x0100u);
    fill_raw_pattern(raw_b, 0x2400u);
    fill_short_raw_bridge_pattern(short_raw_a, 0x0841u, 0u);
    fill_short_raw_bridge_pattern(short_raw_b, 0x1042u, 1u);
    fill_cursorish_pattern(cursor_a, 96u);
    fill_cursorish_pattern(cursor_b, 104u);

    puts("udl_sink synthetic RLX16 transport benchmark");
    puts("scenario              time/cmd    nodmg       raw/repeat pixel shape");
    run_case("repeat-nodmg", repeat_a, NULL, 0);
    run_case("repeat-changed", repeat_a, repeat_b, 1);
    run_case("raw-nodmg", raw_a, NULL, 0);
    run_case("raw-changed", raw_a, raw_b, 1);
    run_case("short-raw-nodmg", short_raw_a, NULL, 0);
    run_case("short-raw-changed", short_raw_a, short_raw_b, 1);
    run_case("cursorish-nodmg", cursor_a, NULL, 0);
    run_case("cursorish-changed", cursor_a, cursor_b, 1);
    return 0;
}