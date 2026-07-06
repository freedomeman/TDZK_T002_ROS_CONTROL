#ifndef __YIS320_DEC_H__
#define __YIS320_DEC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef QT_CORE_LIB
#pragma pack(push)
#pragma pack(1)
#endif

/* Yesense output protocol constants */
#define YIS320_MAX_RAW_SIZE       (512)

/* Output data packet IDs */
#define YIS320_ID_TEMPERATURE     (0x01)
#define YIS320_ID_ACC             (0x10)
#define YIS320_ID_GYR             (0x20)
#define YIS320_ID_MAG_NORM        (0x30)
#define YIS320_ID_MAG             (0x31)
#define YIS320_ID_EUL             (0x40)
#define YIS320_ID_QUAT            (0x41)
#define YIS320_ID_UTC             (0x50)
#define YIS320_ID_SAMPLE_TIME     (0x51)
#define YIS320_ID_DATAREADY_TIME  (0x52)
#define YIS320_ID_POSITION        (0x68)
#define YIS320_ID_VELOCITY        (0x70)
#define YIS320_ID_NAV_STATUS      (0x80)

/* Bitmap bits indicating which fields were present in the latest frame. */
#define YIS320_BMAP_TEMPERATURE       (1u << 0)
#define YIS320_BMAP_ACC               (1u << 1)
#define YIS320_BMAP_GYR               (1u << 2)
#define YIS320_BMAP_MAG_NORM          (1u << 3)
#define YIS320_BMAP_MAG               (1u << 4)
#define YIS320_BMAP_EUL               (1u << 5)
#define YIS320_BMAP_QUAT              (1u << 6)
#define YIS320_BMAP_UTC               (1u << 7)
#define YIS320_BMAP_SAMPLE_TIME       (1u << 8)
#define YIS320_BMAP_DATAREADY_TIME    (1u << 9)
#define YIS320_BMAP_POSITION          (1u << 10)
#define YIS320_BMAP_VELOCITY          (1u << 11)
#define YIS320_BMAP_NAV_STATUS        (1u << 12)

typedef struct __attribute__((__packed__))
{
    uint8_t  tag;
    uint32_t msec;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
} yis320_utc_t;

typedef struct __attribute__((__packed__))
{
    uint8_t  tag;
    uint16_t tid;
    uint32_t data_bitmap;

    float    temperature;       /* deg C */
    float    acc[3];            /* m/s^2 */
    float    gyr[3];            /* deg/s */
    float    mag_norm[3];       /* normalized */
    float    mag[3];            /* mGauss */
    float    eul[3];            /* pitch, roll, yaw, deg */
    float    quat[4];           /* q0, q1, q2, q3 */
    yis320_utc_t utc;
    uint32_t sample_timestamp;  /* us */
    uint32_t dataready_timestamp; /* us */
    double   position[3];       /* lat deg, lon deg, alt m */
    float    velocity[3];       /* east, north, up, m/s */
    uint8_t  fusion_state;
    uint8_t  gnss_state;
} yis320_packet_t;

typedef struct
{
    int nbyte;
    int len;
    uint8_t buf[YIS320_MAX_RAW_SIZE];
    yis320_packet_t packet;
} yis320_raw_t;

#ifdef QT_CORE_LIB
#pragma pack(pop)
#endif

/**
 * @brief Process one byte of input data for Yesense YIS320 decoder.
 *
 * @param raw Pointer to yis320_raw_t structure.
 * @param data Input byte to process.
 * @return int 1 if a complete packet was decoded, 0 if more data is needed, -1 on error.
 */
int yis320_input(yis320_raw_t *raw, uint8_t data);

/**
 * @brief Dump decoded Yesense packet data to a string buffer.
 *
 * @param raw Pointer to yis320_raw_t structure containing decoded data.
 * @param buf Output buffer to store the formatted string.
 * @param buf_size Size of the output buffer.
 * @return int Number of characters written to the buffer.
 */
int yis320_dump_packet(yis320_raw_t *raw, char *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* __YIS320_DEC_H__ */
