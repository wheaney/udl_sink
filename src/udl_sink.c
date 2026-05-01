#include "udl_sink.h"

#include <stdlib.h>
#include <string.h>

enum {
    UDL_MSG_BULK = 0xafu,
    UDL_CMD_WRITEREG = 0x20u,
    UDL_CMD_WRITERAW8 = 0x60u,
    UDL_CMD_WRITERL8 = 0x61u,
    UDL_CMD_WRITECOPY8 = 0x62u,
    UDL_CMD_WRITERLX8 = 0x63u,
    UDL_CMD_WRITERAW16 = 0x68u,
    UDL_CMD_WRITERL16 = 0x69u,
    UDL_CMD_WRITECOPY16 = 0x6au,
    UDL_CMD_WRITERLX16 = 0x6bu,
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
};

enum udl_sink_plane {
    UDL_SINK_PLANE_8,
    UDL_SINK_PLANE_16,
};

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

static uint32_t udl_sink_count_from_byte(uint8_t value)
{
    return value == 0 ? UDL_MAX_COMMAND_PIXELS : (uint32_t)value;
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

static void udl_sink_write_plane16(struct udl_sink *sink,
                                   uint32_t pixel_index,
                                   uint16_t pixel,
                                   struct udl_sink_damage *damage)
{
    sink->plane16[pixel_index] = pixel;
    udl_sink_compose_pixel(sink, pixel_index, damage);
}

static void udl_sink_write_plane8(struct udl_sink *sink,
                                  uint32_t pixel_index,
                                  uint8_t pixel,
                                  struct udl_sink_damage *damage)
{
    sink->plane8[pixel_index] = pixel;
    udl_sink_compose_pixel(sink, pixel_index, damage);
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
    uint32_t i;

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

    for (i = 0; i < pixel_count; ++i) {
        if (plane == UDL_SINK_PLANE_16) {
            const uint16_t pixel = udl_sink_read_be16(&command[6u + (size_t)i * 2u]);

            udl_sink_write_plane16(sink, first_pixel + i, pixel, damage);
        } else {
            udl_sink_write_plane8(sink, first_pixel + i, command[6u + i], damage);
        }
    }

    *consumed = command_size;
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
    const size_t command_size = 6u + udl_sink_plane_bytes_per_pixel(plane);
    enum udl_sink_result result;
    uint32_t first_pixel;
    uint32_t i;

    if (remaining < command_size) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_map_range(sink, plane, byte_address, pixel_count, &first_pixel);
    if (result != UDL_SINK_OK) {
        return result;
    }

    for (i = 0; i < pixel_count; ++i) {
        if (plane == UDL_SINK_PLANE_16) {
            udl_sink_write_plane16(sink,
                                   first_pixel + i,
                                   udl_sink_read_be16(&command[6]),
                                   damage);
        } else {
            udl_sink_write_plane8(sink, first_pixel + i, command[6], damage);
        }
    }

    *consumed = command_size;
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
        uint32_t i;

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

        for (i = 0; i < raw_count; ++i) {
            if (plane == UDL_SINK_PLANE_16) {
                const uint16_t pixel = udl_sink_read_be16(&command[offset + (size_t)i * 2u]);

                udl_sink_write_plane16(sink, first_pixel + produced + i, pixel, damage);
            } else {
                udl_sink_write_plane8(sink,
                                      first_pixel + produced + i,
                                      command[offset + i],
                                      damage);
            }
        }

        if (plane == UDL_SINK_PLANE_16) {
            repeated_pixel16 = udl_sink_read_be16(&command[offset + raw_bytes - 2u]);
        } else {
            repeated_pixel8 = command[offset + raw_bytes - 1u];
        }
        offset += raw_bytes;
        produced += raw_count;

        if (produced == total_pixels) {
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

            for (i = 0; i < repeat_count; ++i) {
                if (plane == UDL_SINK_PLANE_16) {
                    udl_sink_write_plane16(sink,
                                           first_pixel + produced + i,
                                           repeated_pixel16,
                                           damage);
                } else {
                    udl_sink_write_plane8(sink,
                                          first_pixel + produced + i,
                                          repeated_pixel8,
                                          damage);
                }
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
    case UDL_CMD_WRITERAW16:
        return udl_sink_decode_writeraw(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITERL16:
        return udl_sink_decode_writerl(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITECOPY16:
        return udl_sink_decode_writecopy(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
    case UDL_CMD_WRITERLX16:
        return udl_sink_decode_writerlx(sink, UDL_SINK_PLANE_16, command, remaining, consumed, damage);
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

    free(sink->plane16);
    free(sink->plane8);
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