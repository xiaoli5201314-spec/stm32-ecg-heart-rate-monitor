/*
 * frame_protocol.c - framing, CRC, frame synchroniser and payload codecs.
 *
 * The synchroniser is a sliding window over the received byte stream. It never
 * assumes that a read() returns whole frames: the window is only advanced by
 * whole valid frames, and by exactly one byte on every rejection. That single
 * rule handles every degenerate case at once:
 *
 *   half frame      -> the state persists, the parse resumes on the next byte
 *   sticky frames   -> the loop restarts immediately after a good frame
 *   corrupted frame -> CRC mismatch, slide one byte, re-parse (alignment kept)
 *   false sync word -> after the CRC fails the window slides and the real
 *                      header that followed is found again
 */
#include "frame_protocol.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout) */
/* ------------------------------------------------------------------ */
uint16_t ecg_crc16_ccitt_ref(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i;
    int bit;

    if (data == NULL && len != 0u) {
        return 0u;
    }
    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint16_t s_crc_table[256];
static int      s_crc_table_ready = 0;

static void crc_table_build(void)
{
    uint16_t crc;
    int i, bit;

    for (i = 0; i < 256; i++) {
        crc = (uint16_t)((uint16_t)i << 8);
        for (bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
        s_crc_table[i] = crc;
    }
    s_crc_table_ready = 1;
}

uint16_t ecg_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i;

    if (data == NULL && len != 0u) {
        return 0u;
    }
    if (s_crc_table_ready == 0) {
        /* Idempotent: a benign race on a dual-issue MCU would only rebuild the
         * same table twice. */
        crc_table_build();
    }
    for (i = 0u; i < len; i++) {
        crc = (uint16_t)((uint16_t)(crc << 8) ^
                         s_crc_table[(uint8_t)((crc >> 8) ^ (uint16_t)data[i])]);
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* One-shot codec                                                      */
/* ------------------------------------------------------------------ */
size_t ecg_frame_size(size_t payload_len)
{
    return ECG_FRAME_OVERHEAD + payload_len;
}

size_t ecg_frame_encode(uint8_t type, uint16_t seq, const uint8_t *payload,
                        size_t payload_len, uint8_t *out, size_t out_cap)
{
    size_t total;
    uint16_t crc;

    if (out == NULL) {
        return 0u;
    }
    if (payload_len > (size_t)ECG_FRAME_MAX_PAYLOAD) {
        return 0u;
    }
    if (payload_len != 0u && payload == NULL) {
        return 0u;
    }
    total = ECG_FRAME_OVERHEAD + payload_len;
    if (out_cap < total) {
        return 0u;
    }

    out[0] = ECG_FRAME_HDR0;
    out[1] = ECG_FRAME_HDR1;
    out[2] = (uint8_t)(((uint8_t)ECG_PROTO_VERSION << 4) | (uint8_t)(type & 0x0Fu));
    out[3] = (uint8_t)payload_len;
    out[4] = (uint8_t)(seq & 0x00FFu);
    out[5] = (uint8_t)((seq >> 8) & 0x00FFu);
    if (payload_len != 0u) {
        memcpy(&out[6], payload, payload_len);
    }
    /* CRC covers TYPE, LEN, SEQ and PAYLOAD (not the sync word) */
    crc = ecg_crc16_ccitt(&out[2], 4u + payload_len);
    out[6u + payload_len] = (uint8_t)(crc & 0x00FFu);
    out[7u + payload_len] = (uint8_t)((crc >> 8) & 0x00FFu);
    return total;
}

int ecg_frame_decode(const uint8_t *buf, size_t len, ecg_frame_t *out)
{
    uint8_t  type_byte, plen;
    uint16_t crc_calc, crc_rx;

    if (buf == NULL || out == NULL) {
        return -1;
    }
    if (len < ECG_FRAME_OVERHEAD) {
        return -4;
    }
    if (buf[0] != ECG_FRAME_HDR0 || buf[1] != ECG_FRAME_HDR1) {
        return -1;
    }
    type_byte = buf[2];
    if ((uint8_t)(type_byte >> 4) != (uint8_t)ECG_PROTO_VERSION) {
        return -3;
    }
    plen = buf[3];
    if ((size_t)ECG_FRAME_OVERHEAD + (size_t)plen > len) {
        return -4;
    }
    crc_calc = ecg_crc16_ccitt(&buf[2], 4u + (size_t)plen);
    crc_rx   = (uint16_t)((uint16_t)buf[6u + plen] | ((uint16_t)buf[7u + plen] << 8));
    if (crc_calc != crc_rx) {
        return -2;
    }

    out->version = (uint8_t)(type_byte >> 4);
    out->type    = (uint8_t)(type_byte & 0x0Fu);
    out->len     = plen;
    out->seq     = (uint16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    if (plen != 0u) {
        memcpy(out->payload, &buf[6], plen);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Frame synchroniser                                                  */
/* ------------------------------------------------------------------ */
static void sync_slide(ecg_frame_sync_t *s, uint32_t n)
{
    if (n == 0u || n > s->fill) {
        return;
    }
    if (n < s->fill) {
        memmove(&s->window[0], &s->window[n], (size_t)(s->fill - n));
    }
    s->fill -= n;
}

static void sync_drop_and_resync(ecg_frame_sync_t *s)
{
    sync_slide(s, 1u);
    s->resync_slips++;
    s->state = ECG_SYNC_DISCARD_RESYNC;
    s->need  = 0u;
}

void ecg_frame_sync_init(ecg_frame_sync_t *s)
{
    if (s == NULL) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->state = ECG_SYNC_HUNT_HEADER;
}

static ecg_sync_result_t sync_step(ecg_frame_sync_t *s, ecg_frame_t *out)
{
    for (;;) {
        uint8_t  type_byte, plen;
        uint16_t crc_calc, crc_rx;

        if (s->fill == 0u) {
            s->state = ECG_SYNC_HUNT_HEADER;
            return ECG_SYNC_NO_FRAME;
        }
        if (s->window[0] != ECG_FRAME_HDR0) {
            sync_drop_and_resync(s);
            continue;
        }
        s->state = ECG_SYNC_HUNT_HEADER;
        if (s->fill < 2u) {
            return ECG_SYNC_NO_FRAME;               /* need HDR1 */
        }
        if (s->window[1] != ECG_FRAME_HDR1) {
            sync_drop_and_resync(s);
            continue;
        }

        s->state = ECG_SYNC_RECV_HEADER;
        if (s->fill < 4u) {
            return ECG_SYNC_NO_FRAME;               /* need TYPE + LEN */
        }
        type_byte = s->window[2];
        plen      = s->window[3];

        /* LEN is one byte, so it can never exceed ECG_FRAME_MAX_PAYLOAD (255);
         * the only format error left is an unsupported protocol version. */
        if ((uint8_t)(type_byte >> 4) != (uint8_t)ECG_PROTO_VERSION) {
            s->frames_fmt_err++;
            s->last_error = type_byte;
            sync_drop_and_resync(s);
            return ECG_SYNC_FORMAT_FAIL;
        }

        s->need = ECG_FRAME_OVERHEAD + (uint32_t)plen;
        if (s->fill < s->need) {
            s->state = ECG_SYNC_RECV_PAYLOAD;
            return ECG_SYNC_NO_FRAME;               /* half frame: wait */
        }

        s->state = ECG_SYNC_CHECK_CRC;
        crc_calc = ecg_crc16_ccitt(&s->window[2], 4u + (size_t)plen);
        crc_rx   = (uint16_t)((uint16_t)s->window[6u + plen] |
                              ((uint16_t)s->window[7u + plen] << 8));
        if (crc_calc != crc_rx) {
            s->frames_crc_err++;
            s->last_error = type_byte;
            sync_drop_and_resync(s);
            return ECG_SYNC_CRC_FAIL;
        }

        if (out != NULL) {
            out->version = (uint8_t)(type_byte >> 4);
            out->type    = (uint8_t)(type_byte & 0x0Fu);
            out->len     = plen;
            out->seq     = (uint16_t)((uint16_t)s->window[4] |
                                      ((uint16_t)s->window[5] << 8));
            if (plen != 0u) {
                memcpy(out->payload, &s->window[6], plen);
            }
        }
        s->frames_ok++;
        sync_slide(s, s->need);
        s->need  = 0u;
        s->state = ECG_SYNC_FRAME_READY;
        return ECG_SYNC_FRAME;
    }
}

ecg_sync_result_t ecg_frame_sync_feed(ecg_frame_sync_t *s, uint8_t byte, ecg_frame_t *out)
{
    if (s == NULL) {
        return ECG_SYNC_NO_FRAME;
    }
    s->bytes_in++;

    if (s->fill >= (uint32_t)ECG_FRAME_MAX_SIZE) {
        /* defensive: the window can never legitimately overflow */
        sync_drop_and_resync(s);
    }
    s->window[s->fill] = byte;
    s->fill++;
    return sync_step(s, out);
}

size_t ecg_frame_sync_feed_buf(ecg_frame_sync_t *s, const uint8_t *data, size_t len,
                               ecg_frame_t *out, size_t max_frames,
                               ecg_sync_result_t *last_result)
{
    size_t i, produced = 0u;
    ecg_sync_result_t r = ECG_SYNC_NO_FRAME;

    if (s == NULL || data == NULL) {
        if (last_result != NULL) {
            *last_result = ECG_SYNC_NO_FRAME;
        }
        return 0u;
    }
    for (i = 0u; i < len; i++) {
        ecg_frame_t *slot = (out != NULL && produced < max_frames) ? &out[produced] : NULL;
        ecg_frame_t  scratch;
        r = ecg_frame_sync_feed(s, data[i], (slot != NULL) ? slot : &scratch);
        if (r == ECG_SYNC_FRAME && slot != NULL) {
            produced++;
        }
    }
    if (last_result != NULL) {
        *last_result = r;
    }
    return produced;
}

/* ------------------------------------------------------------------ */
/* Payload codecs                                                      */
/* ------------------------------------------------------------------ */
size_t ecg_pack12_size(size_t count)
{
    return ((count / 2u) * 3u) + ((count % 2u) * 2u);
}

size_t ecg_pack12(const uint16_t *samples, size_t count, uint8_t *out, size_t out_cap)
{
    size_t i, o = 0u;

    if (samples == NULL || out == NULL) {
        return 0u;
    }
    if (out_cap < ecg_pack12_size(count)) {
        return 0u;
    }
    for (i = 0u; i + 1u < count; i += 2u) {
        uint16_t a = (uint16_t)(samples[i] & 0x0FFFu);
        uint16_t b = (uint16_t)(samples[i + 1u] & 0x0FFFu);
        out[o++] = (uint8_t)(a & 0xFFu);
        out[o++] = (uint8_t)(((a >> 8) & 0x0Fu) | ((b & 0x0Fu) << 4));
        out[o++] = (uint8_t)((b >> 4) & 0xFFu);
    }
    if (i < count) {
        uint16_t a = (uint16_t)(samples[i] & 0x0FFFu);
        out[o++] = (uint8_t)(a & 0xFFu);
        out[o++] = (uint8_t)((a >> 8) & 0x0Fu);
    }
    return o;
}

size_t ecg_unpack12(const uint8_t *in, size_t in_len, uint16_t *out, size_t out_max)
{
    size_t i = 0u, o = 0u;

    if (in == NULL || out == NULL) {
        return 0u;
    }
    while (i + 3u <= in_len) {
        uint16_t b0 = in[i];
        uint16_t b1 = in[i + 1u];
        uint16_t b2 = in[i + 2u];
        if (o >= out_max) {
            return o;
        }
        out[o++] = (uint16_t)(b0 | ((b1 & 0x0Fu) << 8));
        if (o >= out_max) {
            return o;
        }
        out[o++] = (uint16_t)(((b1 >> 4) & 0x0Fu) | (b2 << 4));
        i += 3u;
    }
    if (i + 2u <= in_len && o < out_max) {
        uint16_t b0 = in[i];
        uint16_t b1 = in[i + 1u];
        out[o++] = (uint16_t)(b0 | ((b1 & 0x0Fu) << 8));
    }
    return o;
}

size_t ecg_delta_encode(const int16_t *samples, size_t count, uint8_t *out, size_t out_cap)
{
    size_t i, o = 0u;

    if (samples == NULL || out == NULL || count == 0u) {
        return 0u;
    }
    if (out_cap < 2u) {
        return 0u;
    }
    /* little-endian base value */
    out[o++] = (uint8_t)((uint16_t)samples[0] & 0x00FFu);
    out[o++] = (uint8_t)(((uint16_t)samples[0] >> 8) & 0x00FFu);

    for (i = 1u; i < count; i++) {
        int32_t d = (int32_t)samples[i] - (int32_t)samples[i - 1u];
        if (d >= -127 && d <= 127) {
            if (o + 1u > out_cap) {
                return 0u;
            }
            out[o++] = (uint8_t)(int8_t)d;
        } else {
            if (o + 3u > out_cap) {
                return 0u;
            }
            out[o++] = 0x80u;                       /* escape */
            out[o++] = (uint8_t)((uint16_t)samples[i] & 0x00FFu);
            out[o++] = (uint8_t)(((uint16_t)samples[i] >> 8) & 0x00FFu);
        }
    }
    return o;
}

size_t ecg_delta_decode(const uint8_t *in, size_t in_len, int16_t *out, size_t out_max,
                        size_t *out_count)
{
    size_t i = 0u, o = 0u;
    int32_t cur;

    if (out_count != NULL) {
        *out_count = 0u;
    }
    if (in == NULL || out == NULL || in_len < 2u || out_max == 0u) {
        return 0u;
    }

    cur = (int32_t)(int16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
    out[o++] = (int16_t)cur;
    i = 2u;

    while (i < in_len && o < out_max) {
        uint8_t b = in[i++];
        if (b == 0x80u) {
            if (i + 1u >= in_len) {
                break;                              /* truncated escape */
            }
            cur = (int32_t)(int16_t)((uint16_t)in[i] | ((uint16_t)in[i + 1u] << 8));
            i += 2u;
        } else {
            cur += (int32_t)(int8_t)b;
            if (cur > 32767) {
                cur = 32767;
            }
            if (cur < -32768) {
                cur = -32768;
            }
        }
        out[o++] = (int16_t)cur;
    }
    if (out_count != NULL) {
        *out_count = o;
    }
    return o;
}

/* ------------------------------------------------------------------ */
/* Payload builders / parsers                                          */
/* ------------------------------------------------------------------ */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0x00FFu);
    p[1] = (uint8_t)((v >> 8) & 0x00FFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_i16(uint8_t *p, int16_t v)
{
    put_u16(p, (uint16_t)v);
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)get_u16(p);
}

static void put_i32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)((u >> 8) & 0xFFu);
    p[2] = (uint8_t)((u >> 16) & 0xFFu);
    p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static int32_t get_i32(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)u;
}

size_t ecg_frame_build_hr(uint16_t seq, const ecg_hr_report_t *r, uint8_t *out, size_t out_cap)
{
    uint8_t p[9];
    if (r == NULL) {
        return 0u;
    }
    p[0] = r->flags;
    put_i16(&p[1], r->bpm_x10);
    put_i16(&p[3], r->bpm_avg_x10);
    put_u16(&p[5], r->rr_ms);
    p[7] = r->quality;
    p[8] = r->reserved;
    return ecg_frame_encode(ECG_FRAME_HR_REPORT, seq, p, sizeof(p), out, out_cap);
}

int ecg_frame_parse_hr(const ecg_frame_t *f, ecg_hr_report_t *r)
{
    if (f == NULL || r == NULL || f->type != ECG_FRAME_HR_REPORT || f->len != 9u) {
        return -1;
    }
    r->flags     = f->payload[0];
    r->bpm_x10   = get_i16(&f->payload[1]);
    r->bpm_avg_x10 = get_i16(&f->payload[3]);
    r->rr_ms     = get_u16(&f->payload[5]);
    r->quality   = f->payload[7];
    r->reserved  = f->payload[8];
    return 0;
}

size_t ecg_frame_build_cal(uint16_t seq, const ecg_cal_report_t *c, uint8_t *out, size_t out_cap)
{
    uint8_t p[16];
    if (c == NULL) {
        return 0u;
    }
    put_i32(&p[0], c->slope_x1000);
    put_i32(&p[4], c->intercept_counts_x1000);
    put_i16(&p[8], c->nonlinearity_x100);
    put_i16(&p[10], c->afe_gain_x10);
    put_i16(&p[12], c->r2_x10000);
    p[14] = c->n_points;
    p[15] = c->flags;
    return ecg_frame_encode(ECG_FRAME_CALIBRATION, seq, p, sizeof(p), out, out_cap);
}

int ecg_frame_parse_cal(const ecg_frame_t *f, ecg_cal_report_t *c)
{
    if (f == NULL || c == NULL || f->type != ECG_FRAME_CALIBRATION || f->len != 16u) {
        return -1;
    }
    c->slope_x1000            = get_i32(&f->payload[0]);
    c->intercept_counts_x1000 = get_i32(&f->payload[4]);
    c->nonlinearity_x100      = get_i16(&f->payload[8]);
    c->afe_gain_x10           = get_i16(&f->payload[10]);
    c->r2_x10000              = get_i16(&f->payload[12]);
    c->n_points               = f->payload[14];
    c->flags                  = f->payload[15];
    return 0;
}

size_t ecg_frame_build_status(uint16_t seq, uint8_t flags, uint16_t ring_fill,
                              uint32_t dropped, uint8_t *out, size_t out_cap)
{
    uint8_t p[7];
    p[0] = flags;
    put_u16(&p[1], ring_fill);
    put_i32(&p[3], (int32_t)dropped);
    return ecg_frame_encode(ECG_FRAME_STATUS, seq, p, sizeof(p), out, out_cap);
}

size_t ecg_frame_build_ack(uint16_t seq, uint8_t acked_type, uint16_t acked_seq,
                           uint8_t *out, size_t out_cap)
{
    uint8_t p[3];
    p[0] = acked_type;
    put_u16(&p[1], acked_seq);
    return ecg_frame_encode(ECG_FRAME_ACK, seq, p, sizeof(p), out, out_cap);
}

uint8_t ecg_frame_build_samples(uint16_t seq, const int16_t *samples_uv, size_t count,
                                uint8_t *out, size_t out_cap, size_t *out_len)
{
    uint8_t  p12[ECG_FRAME_MAX_PAYLOAD];
    uint8_t  pdl[ECG_FRAME_MAX_PAYLOAD];
    uint16_t u12[ECG_SAMPLE_BLOCK_MAX];
    size_t   i, n12 = 0u, ndl = 0u, frame_len = 0u;
    int      fits12 = 1;
    uint8_t  type;

    if (out_len != NULL) {
        *out_len = 0u;
    }
    if (samples_uv == NULL || out == NULL || count == 0u || count > ECG_SAMPLE_BLOCK_MAX) {
        return 0u;
    }

    /* the sample count travels inside the payload: the host must know how many
     * microvolt values a block carries, and it differs for the last block */
    p12[0] = (uint8_t)(count & 0xFFu);
    p12[1] = (uint8_t)((count >> 8) & 0xFFu);
    pdl[0] = p12[0];
    pdl[1] = p12[1];

    for (i = 0u; i < count; i++) {
        if (samples_uv[i] < ECG_SAMPLE_UV_MIN || samples_uv[i] > ECG_SAMPLE_UV_MAX) {
            fits12 = 0;
        }
        u12[i] = (uint16_t)((int32_t)samples_uv[i] + ECG_SAMPLE_UV_OFFSET);
    }
    if (fits12 != 0) {
        n12 = ecg_pack12(u12, count, &p12[2], sizeof(p12) - 2u);
        if (n12 != 0u) {
            n12 += 2u;
        }
    }
    ndl = ecg_delta_encode(samples_uv, count, &pdl[2], sizeof(pdl) - 2u);
    if (ndl != 0u) {
        ndl += 2u;
    }

    if (ndl != 0u && (n12 == 0u || ndl < n12)) {
        type      = (uint8_t)ECG_FRAME_ADC_DELTA;
        frame_len = ecg_frame_encode(type, seq, pdl, ndl, out, out_cap);
    } else {
        type      = (uint8_t)ECG_FRAME_ADC_PACK12;
        frame_len = ecg_frame_encode(type, seq, p12, n12, out, out_cap);
    }
    if (out_len != NULL) {
        *out_len = frame_len;
    }
    return (frame_len != 0u) ? type : (uint8_t)0u;
}

int ecg_frame_parse_samples(const ecg_frame_t *f, int16_t *out_uv, size_t out_max,
                            size_t *out_count)
{
    uint16_t n;
    size_t   i, got;

    if (out_count != NULL) {
        *out_count = 0u;
    }
    if (f == NULL || out_uv == NULL || f->len < 2u) {
        return -1;
    }
    n = (uint16_t)((uint16_t)f->payload[0] | ((uint16_t)f->payload[1] << 8));
    if ((size_t)n > out_max || n > ECG_SAMPLE_BLOCK_MAX) {
        return -1;
    }

    if (f->type == (uint8_t)ECG_FRAME_ADC_PACK12) {
        uint16_t tmp[ECG_SAMPLE_BLOCK_MAX];
        got = ecg_unpack12(&f->payload[2], (size_t)f->len - 2u, tmp, ECG_SAMPLE_BLOCK_MAX);
        if (got < (size_t)n) {
            return -1;
        }
        for (i = 0u; i < (size_t)n; i++) {
            out_uv[i] = (int16_t)((int32_t)tmp[i] - ECG_SAMPLE_UV_OFFSET);
        }
    } else if (f->type == (uint8_t)ECG_FRAME_ADC_DELTA) {
        size_t c = 0u;
        (void)ecg_delta_decode(&f->payload[2], (size_t)f->len - 2u, out_uv, out_max, &c);
        if (c < (size_t)n) {
            return -1;
        }
    } else {
        return -1;
    }

    if (out_count != NULL) {
        *out_count = (size_t)n;
    }
    return 0;
}
