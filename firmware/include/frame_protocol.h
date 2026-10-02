/*
 * frame_protocol.h - compact binary uplink framing + frame synchroniser.
 *
 * Wire format (little endian), 8 bytes of overhead per frame:
 *
 *   offset  size  field
 *   ------  ----  ------------------------------------------------------
 *   0       1     HDR0   = 0xA5
 *   1       1     HDR1   = 0x5A
 *   2       1     TYPE   = (version << 4) | frame type
 *   3       1     LEN    = payload length in bytes (0..255)
 *   4       2     SEQ    = rolling sequence number, wraps at 65535
 *   6       LEN   PAYLOAD
 *   6+LEN   2     CRC16-CCITT (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD
 *
 * Payload codecs keep the stream compact.  The ADC block payload always starts
 * with a 16-bit sample count, followed by one of:
 *
 *   ADC_PACK12  two 12-bit words in 3 bytes (1.5 B/sample, 25 % below int16).
 *               The word is offset binary: code = value_uv + 2048, i.e. 1 uV
 *               resolution over +-2048 uV, which covers a 1 mV R wave with
 *               plenty of head room.
 *   ADC_DELTA   int16 LE base value + int8 first differences, with 0x80 used
 *               as an escape followed by an int16 absolute value. Chosen
 *               automatically whenever a sample falls outside +-2048 uV or
 *               when the delta stream turns out to be the shorter of the two.
 *
 * The receiver side is a sliding-window frame synchroniser.  It keeps the
 * bytes of the candidate frame in a small window and applies the following
 * state machine after every received byte:
 *
 *   HUNT_HEADER --(A5 5A)--> RECV_HEADER --(TYPE,LEN)--> RECV_PAYLOAD
 *        ^                        |                            |
 *        |                   bad version                  LEN bytes
 *        |                        v                            v
 *        +---- DISCARD/RESYNC <-- CHECK_CRC <-------------------+
 *                                     |  CRC ok
 *                                     v
 *                                FRAME_READY
 *
 * Because the window slides by exactly one byte on every failure, half frames
 * (state survives across calls), back-to-back frames (two frames inside one
 * read), false sync words inside a payload (drop one byte and re-parse) and
 * CRC-corrupted frames are all handled without ever losing byte alignment.
 */
#ifndef FRAME_PROTOCOL_H
#define FRAME_PROTOCOL_H

#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ECG_FRAME_ADC_PACK12  = 0x1,   /* 12-bit offset-binary sample block   */
    ECG_FRAME_ADC_DELTA   = 0x2,   /* delta-coded sample block            */
    ECG_FRAME_HR_REPORT   = 0x3,   /* instantaneous + averaged heart rate */
    ECG_FRAME_STATUS      = 0x4,   /* lead-off, clipping, ring statistics */
    ECG_FRAME_CALIBRATION = 0x5,   /* gain / linearity fit result         */
    ECG_FRAME_ACK         = 0x6,
    ECG_FRAME_NAK         = 0x7,
    ECG_FRAME_HOST_CMD    = 0x8
} ecg_frame_type_t;

/* Offset binary used by the ADC_PACK12 payload: 1 uV per LSB. */
#define ECG_SAMPLE_UV_OFFSET  2048
#define ECG_SAMPLE_UV_MIN     (-2048)
#define ECG_SAMPLE_UV_MAX     (2047)
#define ECG_SAMPLE_BLOCK_MAX  64u

typedef struct {
    uint8_t  version;
    uint8_t  type;
    uint16_t seq;
    uint8_t  len;
    uint8_t  payload[ECG_FRAME_MAX_PAYLOAD];
} ecg_frame_t;

/* --- CRC --- */
uint16_t ecg_crc16_ccitt(const uint8_t *data, size_t len);
/* Table-free bitwise reference, used by the unit tests to prove the table
 * driven version above. */
uint16_t ecg_crc16_ccitt_ref(const uint8_t *data, size_t len);

/* --- one-shot codec --- */
/* Returns the number of bytes written, or 0 on error (payload too long,
 * buffer too small, NULL arguments). */
size_t ecg_frame_encode(uint8_t type, uint16_t seq, const uint8_t *payload,
                        size_t payload_len, uint8_t *out, size_t out_cap);
/* Returns 0 on success, negative on error:
 *   -1 malformed, -2 bad CRC, -3 unsupported version, -4 buffer too short. */
int    ecg_frame_decode(const uint8_t *buf, size_t len, ecg_frame_t *out);
size_t ecg_frame_size(size_t payload_len);

/* --- frame synchroniser --- */
typedef enum {
    ECG_SYNC_HUNT_HEADER = 0,
    ECG_SYNC_RECV_HEADER,
    ECG_SYNC_RECV_PAYLOAD,
    ECG_SYNC_CHECK_CRC,
    ECG_SYNC_FRAME_READY,
    ECG_SYNC_DISCARD_RESYNC
} ecg_sync_state_t;

typedef enum {
    ECG_SYNC_NO_FRAME = 0,   /* byte consumed, no complete frame yet   */
    ECG_SYNC_FRAME,          /* out_frame is valid                     */
    ECG_SYNC_CRC_FAIL,       /* a candidate frame failed CRC           */
    ECG_SYNC_FORMAT_FAIL     /* bad version or oversized LEN           */
} ecg_sync_result_t;

typedef struct {
    ecg_sync_state_t state;
    uint8_t          window[ECG_FRAME_MAX_SIZE];
    uint32_t         fill;          /* bytes currently in the window      */
    uint32_t         need;          /* total bytes of the candidate frame */
    uint32_t         resync_slips;  /* bytes skipped to regain alignment  */
    uint32_t         frames_ok;
    uint32_t         frames_crc_err;
    uint32_t         frames_fmt_err;
    uint32_t         bytes_in;
    uint8_t          last_error;    /* last rejected candidate TYPE       */
} ecg_frame_sync_t;

void             ecg_frame_sync_init(ecg_frame_sync_t *s);
ecg_sync_result_t ecg_frame_sync_feed(ecg_frame_sync_t *s, uint8_t byte, ecg_frame_t *out);
/* Feed a whole buffer; up to max_frames frames are collected into `out`.
 * The number of decoded frames is returned, the result of the last byte is
 * reported through *last_result (may be NULL). */
size_t ecg_frame_sync_feed_buf(ecg_frame_sync_t *s, const uint8_t *data, size_t len,
                               ecg_frame_t *out, size_t max_frames,
                               ecg_sync_result_t *last_result);

/* --- payload codecs --- */
size_t ecg_pack12(const uint16_t *samples, size_t count, uint8_t *out, size_t out_cap);
size_t ecg_unpack12(const uint8_t *in, size_t in_len, uint16_t *out, size_t out_max);
size_t ecg_pack12_size(size_t count);

size_t ecg_delta_encode(const int16_t *samples, size_t count, uint8_t *out, size_t out_cap);
size_t ecg_delta_decode(const uint8_t *in, size_t in_len, int16_t *out, size_t out_max,
                        size_t *out_count);

/* --- payload builders / parsers --- */
typedef struct {
    uint8_t  flags;
    int16_t  bpm_x10;        /* instantaneous, 0.1 BPM units */
    int16_t  bpm_avg_x10;    /* RR-median based              */
    uint16_t rr_ms;
    uint8_t  quality;        /* 0..100 signal quality index  */
    uint8_t  reserved;
} ecg_hr_report_t;           /* 9 bytes */

size_t ecg_frame_build_hr(uint16_t seq, const ecg_hr_report_t *r, uint8_t *out, size_t out_cap);
int    ecg_frame_parse_hr(const ecg_frame_t *f, ecg_hr_report_t *r);

/* Calibration result. Fixed point on the wire on purpose: a raw float payload
 * would make the frame both 4 bytes longer and endianness dependent. */
typedef struct {
    int32_t  slope_x1000;        /* counts per mV, x1000            */
    int32_t  intercept_counts_x1000;
    int16_t  nonlinearity_x100;  /* % of full scale, x100           */
    int16_t  afe_gain_x10;       /* measured analog gain, V/V x10   */
    int16_t  r2_x10000;          /* coefficient of determination    */
    uint8_t  n_points;
    uint8_t  flags;              /* bit0: calibration valid         */
} ecg_cal_report_t;              /* 16 bytes */

size_t ecg_frame_build_cal(uint16_t seq, const ecg_cal_report_t *c, uint8_t *out, size_t out_cap);
int    ecg_frame_parse_cal(const ecg_frame_t *f, ecg_cal_report_t *c);

size_t ecg_frame_build_status(uint16_t seq, uint8_t flags, uint16_t ring_fill,
                              uint32_t dropped, uint8_t *out, size_t out_cap);
size_t ecg_frame_build_ack(uint16_t seq, uint8_t acked_type, uint16_t acked_seq,
                           uint8_t *out, size_t out_cap);

/* Helper used by the device layer: pack a filtered sample block (microvolts,
 * electrode referred) into the most compact representation that fits.
 * Returns the frame type used (0 on failure) and the total frame length. */
uint8_t ecg_frame_build_samples(uint16_t seq, const int16_t *samples_uv, size_t count,
                                uint8_t *out, size_t out_cap, size_t *out_len);
/* Inverse: recovers the microvolt samples from an ADC block frame.
 * Returns 0 on success, -1 on a malformed frame. */
int ecg_frame_parse_samples(const ecg_frame_t *f, int16_t *out_uv, size_t out_max,
                            size_t *out_count);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_PROTOCOL_H */
