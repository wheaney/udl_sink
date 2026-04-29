#include "udl_sink.h"

#include <limits.h>
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

static uint64_t udl_sink_pixel_capacity(const struct udl_sink *sink)
{
    return (uint64_t)sink->width * (uint64_t)sink->height;
}

static enum udl_sink_result udl_sink_validate_range(const struct udl_sink *sink,
                                                    uint32_t byte_address,
                                                    uint32_t pixel_count)
{
    uint64_t start_pixel;
    uint64_t end_pixel;

    if ((byte_address & 1u) != 0u) {
        return UDL_SINK_ERR_INVALID_COMMAND;
    }

    start_pixel = byte_address / 2u;
    end_pixel = start_pixel + pixel_count;
    if (end_pixel > udl_sink_pixel_capacity(sink)) {
        return UDL_SINK_ERR_ADDRESS_RANGE;
    }

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

static uint16_t udl_sink_read_pixel(const struct udl_sink *sink,
                                    uint32_t device_pixel_index)
{
    const uint32_t x = device_pixel_index % sink->width;
    const uint32_t y = device_pixel_index / sink->width;

    return sink->framebuffer[y * sink->stride_pixels + x];
}

static void udl_sink_write_pixel(struct udl_sink *sink,
                                 uint32_t device_pixel_index,
                                 uint16_t pixel,
                                 struct udl_sink_damage *damage)
{
    const uint32_t x = device_pixel_index % sink->width;
    const uint32_t y = device_pixel_index / sink->width;

    sink->framebuffer[y * sink->stride_pixels + x] = pixel;
    udl_sink_mark_pixel(damage, x, y);
}

static enum udl_sink_result udl_sink_decode_writereg(struct udl_sink *sink,
                                                     const uint8_t *command,
                                                     size_t remaining,
                                                     size_t *consumed)
{
    if (remaining < 4u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    sink->registers[command[2]] = command[3];
    *consumed = 4u;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writeraw16(struct udl_sink *sink,
                                                       const uint8_t *command,
                                                       size_t remaining,
                                                       size_t *consumed,
                                                       struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t pixel_count = udl_sink_count_from_byte(command[5]);
    const size_t payload_bytes = (size_t)pixel_count * 2u;
    const size_t command_size = 6u + payload_bytes;
    enum udl_sink_result result;
    uint32_t pixel_index;
    uint32_t i;

    if (remaining < 6u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }
    if (remaining < command_size) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_validate_range(sink, byte_address, pixel_count);
    if (result != UDL_SINK_OK) {
        return result;
    }

    pixel_index = byte_address / 2u;
    for (i = 0; i < pixel_count; ++i) {
        const uint16_t pixel = udl_sink_read_be16(&command[6u + (size_t)i * 2u]);
        udl_sink_write_pixel(sink, pixel_index + i, pixel, damage);
    }

    *consumed = command_size;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writerl16(struct udl_sink *sink,
                                                      const uint8_t *command,
                                                      size_t remaining,
                                                      size_t *consumed,
                                                      struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t pixel_count = udl_sink_count_from_byte(command[5]);
    enum udl_sink_result result;
    const uint16_t pixel = udl_sink_read_be16(&command[6]);
    uint32_t pixel_index;
    uint32_t i;

    if (remaining < 8u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_validate_range(sink, byte_address, pixel_count);
    if (result != UDL_SINK_OK) {
        return result;
    }

    pixel_index = byte_address / 2u;
    for (i = 0; i < pixel_count; ++i) {
        udl_sink_write_pixel(sink, pixel_index + i, pixel, damage);
    }

    *consumed = 8u;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writecopy16(struct udl_sink *sink,
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
    const uint32_t src_pixel_index = src_byte_address / 2u;
    const uint32_t dst_pixel_index = dst_byte_address / 2u;
    uint32_t i;

    if (remaining < 9u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_validate_range(sink, src_byte_address, pixel_count);
    if (result != UDL_SINK_OK) {
        return result;
    }

    result = udl_sink_validate_range(sink, dst_byte_address, pixel_count);
    if (result != UDL_SINK_OK) {
        return result;
    }

    for (i = 0; i < pixel_count; ++i) {
        copied_pixels[i] = udl_sink_read_pixel(sink, src_pixel_index + i);
    }

    for (i = 0; i < pixel_count; ++i) {
        udl_sink_write_pixel(sink, dst_pixel_index + i, copied_pixels[i], damage);
    }

    *consumed = 9u;
    return UDL_SINK_OK;
}

static enum udl_sink_result udl_sink_decode_writerlx16(struct udl_sink *sink,
                                                       const uint8_t *command,
                                                       size_t remaining,
                                                       size_t *consumed,
                                                       struct udl_sink_damage *damage)
{
    const uint32_t byte_address = udl_sink_read_addr24(&command[2]);
    const uint32_t total_pixels = udl_sink_count_from_byte(command[5]);
    enum udl_sink_result result;
    uint32_t pixel_index;
    uint32_t produced = 0u;
    size_t offset = 6u;

    if (remaining < 7u) {
        return UDL_SINK_ERR_TRUNCATED_COMMAND;
    }

    result = udl_sink_validate_range(sink, byte_address, total_pixels);
    if (result != UDL_SINK_OK) {
        return result;
    }

    pixel_index = byte_address / 2u;

    while (produced < total_pixels) {
        uint32_t raw_count;
        size_t raw_bytes;
        uint16_t repeated_pixel;
        uint32_t i;

        if (offset >= remaining) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }

        raw_count = udl_sink_count_from_byte(command[offset]);
        offset += 1u;
        if (raw_count > total_pixels - produced) {
            return UDL_SINK_ERR_INVALID_COMMAND;
        }

        raw_bytes = (size_t)raw_count * 2u;
        if (remaining - offset < raw_bytes) {
            return UDL_SINK_ERR_TRUNCATED_COMMAND;
        }

        for (i = 0; i < raw_count; ++i) {
            const uint16_t pixel = udl_sink_read_be16(&command[offset + (size_t)i * 2u]);
            udl_sink_write_pixel(sink, pixel_index + produced + i, pixel, damage);
        }

        repeated_pixel = udl_sink_read_be16(&command[offset + raw_bytes - 2u]);
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
                udl_sink_write_pixel(sink,
                                     pixel_index + produced + i,
                                     repeated_pixel,
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
    case UDL_CMD_WRITEREG:
        return udl_sink_decode_writereg(sink, command, remaining, consumed);
    case UDL_CMD_WRITERAW16:
        return udl_sink_decode_writeraw16(sink, command, remaining, consumed, damage);
    case UDL_CMD_WRITERL16:
        return udl_sink_decode_writerl16(sink, command, remaining, consumed, damage);
    case UDL_CMD_WRITECOPY16:
        return udl_sink_decode_writecopy16(sink, command, remaining, consumed, damage);
    case UDL_CMD_WRITERLX16:
        return udl_sink_decode_writerlx16(sink, command, remaining, consumed, damage);
    case UDL_CMD_WRITERAW8:
    case UDL_CMD_WRITERL8:
    case UDL_CMD_WRITECOPY8:
    case UDL_CMD_WRITERLX8:
        return UDL_SINK_ERR_UNSUPPORTED_COMMAND;
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

    if (!sink || !buffer) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }
    if (!sink->framebuffer || sink->width == 0u || sink->height == 0u ||
        sink->stride_pixels < sink->width) {
        return UDL_SINK_ERR_INVALID_ARGUMENT;
    }

    while (offset < length) {
        size_t consumed = 0u;
        enum udl_sink_result result;
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