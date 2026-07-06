#include "yis320_dec.h"

#define YIS320_SYNC1          (0x59)
#define YIS320_SYNC2          (0x53)
#define YIS320_HDR_SIZE       (5)     /* sync(2) + TID(2) + LEN(1) */
#define YIS320_CKSUM_SIZE     (2)

#define YIS320_SCALE_1E_2     (0.01f)
#define YIS320_SCALE_1E_3     (0.001f)
#define YIS320_SCALE_1E_6     (0.000001f)
#define YIS320_SCALE_1E_10    (0.0000000001)

static uint16_t U2(const uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, sizeof(u));
    return u;
}

static uint32_t U4(const uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, sizeof(u));
    return u;
}

static int16_t I2(const uint8_t *p)
{
    int16_t i;
    memcpy(&i, p, sizeof(i));
    return i;
}

static int32_t I4(const uint8_t *p)
{
    int32_t i;
    memcpy(&i, p, sizeof(i));
    return i;
}

static int64_t I8(const uint8_t *p)
{
    int64_t i;
    memcpy(&i, p, sizeof(i));
    return i;
}

static void yis320_checksum(const uint8_t *buf, uint32_t len, uint8_t *ck1, uint8_t *ck2)
{
    uint32_t i;
    *ck1 = 0;
    *ck2 = 0;

    for (i = 0; i < len; ++i)
    {
        *ck1 = (uint8_t)(*ck1 + buf[i]);
        *ck2 = (uint8_t)(*ck2 + *ck1);
    }
}

static int parse_data(yis320_raw_t *raw)
{
    int ofs = 0;
    const uint8_t *p = &raw->buf[YIS320_HDR_SIZE];
    yis320_packet_t *packet = &raw->packet;

    memset(packet, 0, sizeof(*packet));
    packet->tag = 1;
    packet->tid = U2(raw->buf + 2);

    while (ofs < raw->len)
    {
        uint8_t id;
        uint8_t len;
        const uint8_t *data;

        if ((raw->len - ofs) < 2)
        {
            return -1;
        }

        id = p[ofs];
        len = p[ofs + 1];
        data = p + ofs + 2;

        if (len > (uint8_t)(raw->len - ofs - 2))
        {
            return -1;
        }

        switch (id)
        {
        case YIS320_ID_TEMPERATURE:
            if (len == 2)
            {
                packet->temperature = (float)I2(data) * YIS320_SCALE_1E_2;
                packet->data_bitmap |= YIS320_BMAP_TEMPERATURE;
            }
            break;

        case YIS320_ID_ACC:
            if (len == 12)
            {
                packet->acc[0] = (float)I4(data + 0) * YIS320_SCALE_1E_6;
                packet->acc[1] = (float)I4(data + 4) * YIS320_SCALE_1E_6;
                packet->acc[2] = (float)I4(data + 8) * YIS320_SCALE_1E_6;
                packet->data_bitmap |= YIS320_BMAP_ACC;
            }
            break;

        case YIS320_ID_GYR:
            if (len == 12)
            {
                packet->gyr[0] = (float)I4(data + 0) * YIS320_SCALE_1E_6;
                packet->gyr[1] = (float)I4(data + 4) * YIS320_SCALE_1E_6;
                packet->gyr[2] = (float)I4(data + 8) * YIS320_SCALE_1E_6;
                packet->data_bitmap |= YIS320_BMAP_GYR;
            }
            break;

        case YIS320_ID_MAG_NORM:
            if (len == 12)
            {
                packet->mag_norm[0] = (float)I4(data + 0) * YIS320_SCALE_1E_6;
                packet->mag_norm[1] = (float)I4(data + 4) * YIS320_SCALE_1E_6;
                packet->mag_norm[2] = (float)I4(data + 8) * YIS320_SCALE_1E_6;
                packet->data_bitmap |= YIS320_BMAP_MAG_NORM;
            }
            break;

        case YIS320_ID_MAG:
            if (len == 12)
            {
                packet->mag[0] = (float)I4(data + 0) * YIS320_SCALE_1E_3;
                packet->mag[1] = (float)I4(data + 4) * YIS320_SCALE_1E_3;
                packet->mag[2] = (float)I4(data + 8) * YIS320_SCALE_1E_3;
                packet->data_bitmap |= YIS320_BMAP_MAG;
            }
            break;

        case YIS320_ID_EUL:
            if (len == 12)
            {
                packet->eul[0] = (float)I4(data + 0) * YIS320_SCALE_1E_6;
                packet->eul[1] = (float)I4(data + 4) * YIS320_SCALE_1E_6;
                packet->eul[2] = (float)I4(data + 8) * YIS320_SCALE_1E_6;
                packet->data_bitmap |= YIS320_BMAP_EUL;
            }
            break;

        case YIS320_ID_QUAT:
            if (len == 16)
            {
                packet->quat[0] = (float)I4(data + 0) * YIS320_SCALE_1E_6;
                packet->quat[1] = (float)I4(data + 4) * YIS320_SCALE_1E_6;
                packet->quat[2] = (float)I4(data + 8) * YIS320_SCALE_1E_6;
                packet->quat[3] = (float)I4(data + 12) * YIS320_SCALE_1E_6;
                packet->data_bitmap |= YIS320_BMAP_QUAT;
            }
            break;

        case YIS320_ID_UTC:
            if (len == 11)
            {
                packet->utc.tag = YIS320_ID_UTC;
                packet->utc.msec = U4(data + 0);
                packet->utc.year = U2(data + 4);
                packet->utc.month = data[6];
                packet->utc.day = data[7];
                packet->utc.hour = data[8];
                packet->utc.min = data[9];
                packet->utc.sec = data[10];
                packet->data_bitmap |= YIS320_BMAP_UTC;
            }
            break;

        case YIS320_ID_SAMPLE_TIME:
            if (len == 4)
            {
                packet->sample_timestamp = U4(data);
                packet->data_bitmap |= YIS320_BMAP_SAMPLE_TIME;
            }
            break;

        case YIS320_ID_DATAREADY_TIME:
            if (len == 4)
            {
                packet->dataready_timestamp = U4(data);
                packet->data_bitmap |= YIS320_BMAP_DATAREADY_TIME;
            }
            break;

        case YIS320_ID_POSITION:
            if (len == 20)
            {
                packet->position[0] = (double)I8(data + 0) * YIS320_SCALE_1E_10;
                packet->position[1] = (double)I8(data + 8) * YIS320_SCALE_1E_10;
                packet->position[2] = (double)I4(data + 16) * (double)YIS320_SCALE_1E_3;
                packet->data_bitmap |= YIS320_BMAP_POSITION;
            }
            break;

        case YIS320_ID_VELOCITY:
            if (len == 12)
            {
                packet->velocity[0] = (float)I4(data + 0) * YIS320_SCALE_1E_3;
                packet->velocity[1] = (float)I4(data + 4) * YIS320_SCALE_1E_3;
                packet->velocity[2] = (float)I4(data + 8) * YIS320_SCALE_1E_3;
                packet->data_bitmap |= YIS320_BMAP_VELOCITY;
            }
            break;

        case YIS320_ID_NAV_STATUS:
            if (len == 1)
            {
                packet->fusion_state = data[0] & 0x0F;
                packet->gnss_state = (data[0] >> 4) & 0x0F;
                packet->data_bitmap |= YIS320_BMAP_NAV_STATUS;
            }
            break;

        default:
            break;
        }

        ofs += 2 + len;
    }

    return 1;
}

static int decode_yis320(yis320_raw_t *raw)
{
    uint8_t ck1;
    uint8_t ck2;
    int checksum_pos = YIS320_HDR_SIZE + raw->len;

    yis320_checksum(raw->buf + 2, (uint32_t)(3 + raw->len), &ck1, &ck2);
    if (ck1 != raw->buf[checksum_pos] || ck2 != raw->buf[checksum_pos + 1])
    {
        return -1;
    }

    return parse_data(raw);
}

static int sync_yis320(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == YIS320_SYNC1 && buf[1] == YIS320_SYNC2;
}

int yis320_input(yis320_raw_t *raw, uint8_t data)
{
    if (raw->nbyte == 0)
    {
        if (!sync_yis320(raw->buf, data))
        {
            return 0;
        }
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == YIS320_HDR_SIZE)
    {
        raw->len = raw->buf[4];
        if (raw->len > (YIS320_MAX_RAW_SIZE - YIS320_HDR_SIZE - YIS320_CKSUM_SIZE))
        {
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < YIS320_HDR_SIZE ||
        raw->nbyte < (raw->len + YIS320_HDR_SIZE + YIS320_CKSUM_SIZE))
    {
        return 0;
    }

    raw->nbyte = 0;
    return decode_yis320(raw);
}

int yis320_dump_packet(yis320_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret = 0;
    yis320_packet_t *packet = &raw->packet;

#define APPEND(...)                                                         \
    do {                                                                    \
        if (written < (int)buf_size) {                                       \
            ret = snprintf(buf + written, buf_size - (size_t)written,        \
                           __VA_ARGS__);                                    \
            if (ret > 0) {                                                   \
                written += ret;                                              \
            }                                                               \
        }                                                                   \
    } while (0)

    if (packet->tag == 0)
    {
        if (buf_size > 0)
        {
            buf[0] = '\0';
        }
        return 0;
    }

    APPEND("{\n");
    APPEND("  \"type\": \"YIS320\",\n");
    APPEND("  \"tid\": %u,\n", (unsigned)packet->tid);
    APPEND("  \"data_bitmap\": %u", (unsigned)packet->data_bitmap);

    if (packet->data_bitmap & YIS320_BMAP_TEMPERATURE)
    {
        APPEND(",\n  \"temperature\": %.2f", packet->temperature);
    }
    if (packet->data_bitmap & YIS320_BMAP_ACC)
    {
        APPEND(",\n  \"acc\": [%.6f, %.6f, %.6f]",
               packet->acc[0], packet->acc[1], packet->acc[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_GYR)
    {
        APPEND(",\n  \"gyr\": [%.6f, %.6f, %.6f]",
               packet->gyr[0], packet->gyr[1], packet->gyr[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_MAG_NORM)
    {
        APPEND(",\n  \"mag_norm\": [%.6f, %.6f, %.6f]",
               packet->mag_norm[0], packet->mag_norm[1], packet->mag_norm[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_MAG)
    {
        APPEND(",\n  \"mag\": [%.3f, %.3f, %.3f]",
               packet->mag[0], packet->mag[1], packet->mag[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_EUL)
    {
        APPEND(",\n  \"pitch\": %.6f,\n  \"roll\": %.6f,\n  \"yaw\": %.6f",
               packet->eul[0], packet->eul[1], packet->eul[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_QUAT)
    {
        APPEND(",\n  \"quat\": [%.6f, %.6f, %.6f, %.6f]",
               packet->quat[0], packet->quat[1], packet->quat[2], packet->quat[3]);
    }
    if (packet->data_bitmap & YIS320_BMAP_UTC)
    {
        APPEND(",\n  \"utc\": \"%04u-%02u-%02u %02u:%02u:%02u.%03u\"",
               (unsigned)packet->utc.year,
               (unsigned)packet->utc.month,
               (unsigned)packet->utc.day,
               (unsigned)packet->utc.hour,
               (unsigned)packet->utc.min,
               (unsigned)packet->utc.sec,
               (unsigned)packet->utc.msec);
    }
    if (packet->data_bitmap & YIS320_BMAP_SAMPLE_TIME)
    {
        APPEND(",\n  \"sample_timestamp\": %u", (unsigned)packet->sample_timestamp);
    }
    if (packet->data_bitmap & YIS320_BMAP_DATAREADY_TIME)
    {
        APPEND(",\n  \"dataready_timestamp\": %u", (unsigned)packet->dataready_timestamp);
    }
    if (packet->data_bitmap & YIS320_BMAP_POSITION)
    {
        APPEND(",\n  \"position\": [%.10f, %.10f, %.3f]",
               packet->position[0], packet->position[1], packet->position[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_VELOCITY)
    {
        APPEND(",\n  \"velocity\": [%.3f, %.3f, %.3f]",
               packet->velocity[0], packet->velocity[1], packet->velocity[2]);
    }
    if (packet->data_bitmap & YIS320_BMAP_NAV_STATUS)
    {
        APPEND(",\n  \"fusion_state\": %u,\n  \"gnss_state\": %u",
               (unsigned)packet->fusion_state,
               (unsigned)packet->gnss_state);
    }

    APPEND("\n}\n");

#undef APPEND

    if (written >= (int)buf_size && buf_size > 0)
    {
        buf[buf_size - 1] = '\0';
    }

    return written;
}
