/*
 * test_protocol.c - CRC, framing, the frame synchroniser and the payload codecs.
 */
#include "test_util.h"
#include "frame_protocol.h"
#include "hal_stub.h"

/* Build an ECG-like sample block so the codec tests use realistic data. */
static void make_sample_block(int16_t *out, size_t n, uint32_t seed)
{
    hal_sim_cfg_t cfg;
    size_t i;
    hal_sim_default_cfg(&cfg);
    cfg.seed = seed;
    cfg.enable_hum = 1;
    cfg.enable_wander = 1;
    cfg.enable_noise = 1;
    for (i = 0u; i < n; i++) {
        double uv = hal_sim_ecg_mv(&cfg, (double)i / cfg.sample_rate_hz) * 1000.0;
        out[i] = (int16_t)(uv + 0.5);
    }
}

static void test_crc(void)
{
    static const uint8_t check[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint8_t  buf[64];
    uint32_t i, seed;
    int      mismatch = 0;

    tt_begin("protocol::crc16");

    /* CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1 */
    TT_CHECK(ecg_crc16_ccitt(check, 9u) == 0x29B1u);
    TT_CHECK(ecg_crc16_ccitt_ref(check, 9u) == 0x29B1u);

    /* empty input leaves the initial value */
    TT_CHECK(ecg_crc16_ccitt(check, 0u) == 0xFFFFu);

    /* the table driven implementation must match the bitwise reference on
     * pseudo-random data of every length class */
    seed = 0x1234ABCDu;
    for (i = 0u; i < 400u; i++) {
        uint32_t len = hal_sim_rng(&seed) % 64u;
        uint32_t j;
        for (j = 0u; j < len; j++) {
            buf[j] = (uint8_t)(hal_sim_rng(&seed) & 0xFFu);
        }
        if (ecg_crc16_ccitt(buf, len) != ecg_crc16_ccitt_ref(buf, len)) {
            mismatch++;
        }
    }
    TT_CHECK(mismatch == 0);
    tt_info("table and bitwise CRC agree on 400 random buffers");

    /* single bit flips are always detected */
    {
        uint8_t base[16];
        uint16_t crc0;
        uint32_t bit, bad = 0u;
        for (i = 0u; i < 16u; i++) {
            base[i] = (uint8_t)(i * 7u + 1u);
        }
        crc0 = ecg_crc16_ccitt(base, 16u);
        for (bit = 0u; bit < 128u; bit++) {
            uint8_t copy[16];
            memcpy(copy, base, 16u);
            copy[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
            if (ecg_crc16_ccitt(copy, 16u) == crc0) {
                bad++;
            }
        }
        TT_CHECK(bad == 0u);
        tt_info("all 128 single-bit errors detected");
    }

    tt_end();
}

static void test_frame_codec(void)
{
    uint8_t      frame[ECG_FRAME_MAX_SIZE + 8];
    uint8_t      payload[ECG_FRAME_MAX_PAYLOAD];
    ecg_frame_t  decoded;
    size_t       len, i;

    tt_begin("protocol::frame_codec");

    for (i = 0u; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)(i ^ 0x5Au);
    }

    /* round trip for the boundary lengths */
    {
        static const size_t lens[] = { 0u, 1u, 2u, 15u, 16u, 127u, 254u, 255u };
        size_t k;
        for (k = 0u; k < sizeof(lens) / sizeof(lens[0]); k++) {
            size_t plen = lens[k];
            len = ecg_frame_encode(ECG_FRAME_ADC_DELTA, 0x1234u, payload, plen,
                                   frame, sizeof(frame));
            TT_CHECK_MSG(len == ECG_FRAME_OVERHEAD + plen,
                         "encode len %u -> %u", (unsigned)plen, (unsigned)len);
            TT_CHECK(frame[0] == ECG_FRAME_HDR0);
            TT_CHECK(frame[1] == ECG_FRAME_HDR1);
            TT_CHECK((frame[2] >> 4) == ECG_PROTO_VERSION);
            TT_CHECK((frame[2] & 0x0Fu) == ECG_FRAME_ADC_DELTA);
            TT_CHECK(frame[3] == (uint8_t)plen);
            TT_CHECK(ecg_frame_decode(frame, len, &decoded) == 0);
            TT_CHECK(decoded.seq == 0x1234u);
            TT_CHECK(decoded.len == (uint8_t)plen);
            TT_CHECK(decoded.type == (uint8_t)ECG_FRAME_ADC_DELTA);
            TT_CHECK(plen == 0u || memcmp(decoded.payload, payload, plen) == 0);
        }
    }

    /* refuse to build an oversized payload or to write into a small buffer */
    {
        uint8_t small[8];
        TT_CHECK(ecg_frame_encode(ECG_FRAME_STATUS, 1u, payload,
                                  (size_t)ECG_FRAME_MAX_PAYLOAD + 1u, frame,
                                  sizeof(frame)) == 0u);
        TT_CHECK(ecg_frame_encode(ECG_FRAME_STATUS, 1u, payload, 4u, small,
                                  sizeof(small)) == 0u);
        TT_CHECK(ecg_frame_encode(ECG_FRAME_STATUS, 1u, NULL, 4u, frame,
                                  sizeof(frame)) == 0u);
        TT_CHECK(ecg_frame_encode(ECG_FRAME_STATUS, 1u, NULL, 0u, frame,
                                  sizeof(frame)) == ECG_FRAME_OVERHEAD);
    }

    /* decode rejections */
    len = ecg_frame_encode(ECG_FRAME_STATUS, 7u, payload, 16u, frame, sizeof(frame));
    TT_CHECK(ecg_frame_decode(frame, ECG_FRAME_OVERHEAD - 1u, &decoded) == -4);
    TT_CHECK(ecg_frame_decode(frame, len - 1u, &decoded) == -4);
    frame[0] = 0x00u;
    TT_CHECK(ecg_frame_decode(frame, len, &decoded) == -1);
    frame[0] = ECG_FRAME_HDR0;
    TT_CHECK(ecg_frame_decode(frame, len, &decoded) == 0);

    frame[2] = (uint8_t)((2u << 4) | ECG_FRAME_STATUS);   /* version 2 */
    TT_CHECK(ecg_frame_decode(frame, len, &decoded) == -3);
    frame[2] = (uint8_t)((ECG_PROTO_VERSION << 4) | ECG_FRAME_STATUS);

    /* every single byte corruption must be caught by the CRC (or the header) */
    {
        uint32_t bad = 0u;
        for (i = 0u; i < len; i++) {
            uint8_t copy[ECG_FRAME_MAX_SIZE];
            int     rc;
            memcpy(copy, frame, len);
            copy[i] ^= 0x01u;
            rc = ecg_frame_decode(copy, len, &decoded);
            if (rc == 0) {
                bad++;
            }
        }
        TT_CHECK_MSG(bad == 0u, "%u single-byte corruptions accepted", bad);
        tt_info("%u single-byte corruptions, all rejected", (unsigned)len);
    }

    tt_end();
}

static void test_frame_sync_basics(void)
{
    ecg_frame_sync_t sync;
    ecg_frame_t      out;
    ecg_sync_result_t r;
    uint8_t  frame[ECG_FRAME_MAX_SIZE];
    uint8_t  payload[8] = { 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u };
    size_t   len, i;
    uint32_t got = 0u;

    tt_begin("protocol::sync_basics");

    len = ecg_frame_encode(ECG_FRAME_HR_REPORT, 42u, payload, sizeof(payload),
                           frame, sizeof(frame));
    TT_CHECK(len != 0u);

    /* --- byte at a time: a half frame keeps the state --- */
    ecg_frame_sync_init(&sync);
    for (i = 0u; i + 1u < len; i++) {
        r = ecg_frame_sync_feed(&sync, frame[i], &out);
        TT_CHECK(r == ECG_SYNC_NO_FRAME);
        TT_CHECK(sync.state == ECG_SYNC_RECV_PAYLOAD || sync.state == ECG_SYNC_HUNT_HEADER ||
                 sync.state == ECG_SYNC_RECV_HEADER);
    }
    r = ecg_frame_sync_feed(&sync, frame[len - 1u], &out);
    TT_CHECK(r == ECG_SYNC_FRAME);
    TT_CHECK(out.seq == 42u);
    TT_CHECK(out.len == 8u);
    TT_CHECK(memcmp(out.payload, payload, 8u) == 0);
    TT_CHECK(sync.frames_ok == 1u);
    TT_CHECK(sync.frames_crc_err == 0u);

    /* --- sticky packet: three frames in one buffer --- */
    {
        uint8_t  stream[3u * ECG_FRAME_MAX_SIZE];
        size_t   total = 0u;
        ecg_frame_t many[4];
        size_t   n;
        uint32_t k;
        for (k = 0u; k < 3u; k++) {
            size_t l = ecg_frame_encode(ECG_FRAME_STATUS, (uint16_t)(100u + k), payload,
                                        4u, &stream[total], sizeof(stream) - total);
            TT_CHECK(l != 0u);
            total += l;
        }
        ecg_frame_sync_init(&sync);
        n = ecg_frame_sync_feed_buf(&sync, stream, total, many, 4u, &r);
        TT_CHECK(n == 3u);
        for (k = 0u; k < n; k++) {
            TT_CHECK(many[k].seq == (uint16_t)(100u + k));
        }
        TT_CHECK(sync.frames_ok == 3u);
    }

    /* --- leading garbage must be skipped without losing the frame --- */
    {
        uint8_t  stream[64];
        size_t   total = 0u;
        size_t   n;
        ecg_frame_t one;
        static const uint8_t junk[7] = { 0x00u, 0xFFu, 0xA5u, 0x11u, 0x5Au, 0xA5u, 0x5Au };
        memcpy(stream, junk, sizeof(junk));
        total = sizeof(junk);
        total += ecg_frame_encode(ECG_FRAME_ACK, 9u, payload, 3u, &stream[total],
                                  sizeof(stream) - total);
        ecg_frame_sync_init(&sync);
        n = ecg_frame_sync_feed_buf(&sync, stream, total, &one, 1u, &r);
        TT_CHECK(n == 1u);
        TT_CHECK(one.seq == 9u);
        TT_CHECK(sync.resync_slips >= 1u);
        got = sync.frames_ok;
        TT_CHECK(got == 1u);
    }

    tt_end();
}

static void test_frame_sync_error_recovery(void)
{
    ecg_frame_sync_t sync;
    ecg_frame_t      frames[8];
    ecg_sync_result_t r;
    uint8_t  stream[512];
    uint8_t  payload[6] = { 0xDEu, 0xADu, 0xBEu, 0xEFu, 0x00u, 0xFFu };
    size_t   total, n;
    uint32_t i;

    tt_begin("protocol::sync_error_recovery");

    /* --- a corrupted frame followed by a good one --- */
    total = 0u;
    total += ecg_frame_encode(ECG_FRAME_STATUS, 1u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    stream[10] ^= 0x40u;                      /* corrupt the payload */
    total += ecg_frame_encode(ECG_FRAME_STATUS, 2u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    total += ecg_frame_encode(ECG_FRAME_STATUS, 3u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);

    ecg_frame_sync_init(&sync);
    n = ecg_frame_sync_feed_buf(&sync, stream, total, frames, 8u, &r);
    TT_CHECK_MSG(n == 2u, "recovered %u frames, expected 2", (unsigned)n);
    if (n == 2u) {
        TT_CHECK(frames[0].seq == 2u);
        TT_CHECK(frames[1].seq == 3u);
    }
    TT_CHECK(sync.frames_crc_err == 1u);
    TT_CHECK(sync.frames_ok == 2u);
    tt_info("1 corrupted frame -> CRC rejected, %u following frames still decoded, "
            "%u resync slips", (unsigned)n, sync.resync_slips);

    /* --- a corrupted length field that still looks plausible: the parser
     *     consumes the following frame as payload, the CRC fails, the window
     *     slides one byte at a time and every later frame is recovered --- */
    total = 0u;
    total += ecg_frame_encode(ECG_FRAME_ADC_PACK12, 7u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    stream[3] = 20u;                           /* LEN is now a lie (28 bytes) */
    total += ecg_frame_encode(ECG_FRAME_HR_REPORT, 21u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    total += ecg_frame_encode(ECG_FRAME_CALIBRATION, 22u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    total += ecg_frame_encode(ECG_FRAME_CALIBRATION, 23u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    ecg_frame_sync_init(&sync);
    n = ecg_frame_sync_feed_buf(&sync, stream, total, frames, 8u, &r);
    TT_CHECK_MSG(n == 3u, "bad-length recovery gave %u frames", (unsigned)n);
    if (n == 3u) {
        TT_CHECK(frames[0].seq == 21u);
        TT_CHECK(frames[1].seq == 22u);
        TT_CHECK(frames[2].seq == 23u);
    }

    /* --- an absurd length field: the parser may wait, but the window is
     *     bounded, so it resynchronises after at most ECG_FRAME_MAX_SIZE
     *     bytes and still recovers the frames that follow --- */
    total = 0u;
    total += ecg_frame_encode(ECG_FRAME_STATUS, 90u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    stream[3] = 250u;                          /* claims a 258 byte frame */
    memset(&stream[total], 0x11, 400u);        /* junk without any sync word */
    total += 400u;
    total += ecg_frame_encode(ECG_FRAME_STATUS, 91u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    total += ecg_frame_encode(ECG_FRAME_STATUS, 92u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    ecg_frame_sync_init(&sync);
    n = ecg_frame_sync_feed_buf(&sync, stream, total, frames, 8u, &r);
    TT_CHECK_MSG(n == 2u, "bounded-wait recovery gave %u frames", (unsigned)n);
    if (n == 2u) {
        TT_CHECK(frames[0].seq == 91u);
        TT_CHECK(frames[1].seq == 92u);
    }

    /* --- a false sync word inside a payload of a valid frame is harmless --- */
    payload[0] = ECG_FRAME_HDR0;
    payload[1] = ECG_FRAME_HDR1;
    total = 0u;
    total += ecg_frame_encode(ECG_FRAME_STATUS, 55u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    total += ecg_frame_encode(ECG_FRAME_STATUS, 56u, payload, sizeof(payload),
                              &stream[total], sizeof(stream) - total);
    ecg_frame_sync_init(&sync);
    n = ecg_frame_sync_feed_buf(&sync, stream, total, frames, 8u, &r);
    TT_CHECK_MSG(n == 2u, "false sync word inside the payload lost a frame (%u)",
                 (unsigned)n);
    if (n == 2u) {
        TT_CHECK(frames[0].seq == 55u);
        TT_CHECK(frames[1].seq == 56u);
    }

    /* --- pure noise must never produce a frame --- */
    {
        uint32_t seed = 0xACE1u;
        uint32_t bad = 0u;
        ecg_frame_sync_init(&sync);
        for (i = 0u; i < 200000u; i++) {
            uint8_t b = (uint8_t)(hal_sim_rng(&seed) & 0xFFu);
            if (ecg_frame_sync_feed(&sync, b, &frames[0]) == ECG_SYNC_FRAME) {
                bad++;
            }
        }
        TT_CHECK(bad == 0u);
        tt_info("200000 random bytes -> %u false frames, %u CRC rejects",
                bad, sync.frames_crc_err);
    }

    tt_end();
}

static void test_payload_codecs(void)
{
    int16_t  samples[64];
    int16_t  back[64];
    uint16_t u12[64];
    uint16_t u12_back[64];
    uint8_t  enc[512];
    size_t   n, m, count;
    size_t   i;

    tt_begin("protocol::payload_codecs");

    make_sample_block(samples, 24u, 0x1111u);

    /* --- 12-bit packing --- */
    for (i = 0u; i < 24u; i++) {
        u12[i] = (uint16_t)((int32_t)samples[i] & 0x0FFFu);
    }
    n = ecg_pack12(u12, 24u, enc, sizeof(enc));
    TT_CHECK(n == ecg_pack12_size(24u));
    TT_CHECK(n == 36u);                     /* 24 samples * 1.5 bytes */
    m = ecg_unpack12(enc, n, u12_back, 64u);
    TT_CHECK(m == 24u);
    TT_CHECK(memcmp(u12, u12_back, 24u * sizeof(uint16_t)) == 0);

    /* odd count: 25 samples -> 38 bytes */
    u12[24] = 4095u;
    n = ecg_pack12(u12, 25u, enc, sizeof(enc));
    TT_CHECK(n == 38u);
    m = ecg_unpack12(enc, n, u12_back, 64u);
    TT_CHECK(m == 25u);
    TT_CHECK(memcmp(u12, u12_back, 25u * sizeof(uint16_t)) == 0);

    /* every 12-bit code survives the round trip */
    {
        uint16_t all[4096];
        uint16_t all_back[4096];
        for (i = 0u; i < 4096u; i++) {
            all[i] = (uint16_t)i;
        }
        n = ecg_pack12(all, 4096u, NULL, 0u);
        TT_CHECK(n == 0u);                  /* NULL output is refused */
        {
            static uint8_t big[6144];
            n = ecg_pack12(all, 4096u, big, sizeof(big));
            TT_CHECK(n == 6144u);
            m = ecg_unpack12(big, n, all_back, 4096u);
            TT_CHECK(m == 4096u);
            TT_CHECK(memcmp(all, all_back, sizeof(all)) == 0);
        }
    }

    /* --- delta codec: small deltas, escapes and saturated rails --- */
    for (i = 0u; i < 32u; i++) {
        samples[i] = (int16_t)(2000 + (int16_t)(i % 5u) * 3);
    }
    n = ecg_delta_encode(samples, 32u, enc, sizeof(enc));
    TT_CHECK(n == 2u + 31u);                /* every delta fits in one byte */
    m = ecg_delta_decode(enc, n, back, 64u, &count);
    TT_CHECK(count == 32u);
    TT_CHECK(memcmp(samples, back, 32u * sizeof(int16_t)) == 0);

    samples[5]  = 32000;                    /* huge positive jump  -> escape */
    samples[6]  = -32000;                   /* huge negative jump  -> escape */
    samples[7]  = 32000;
    n = ecg_delta_encode(samples, 32u, enc, sizeof(enc));
    TT_CHECK(n == 2u + 27u + (4u * 3u));   /* 4 escapes: idx 5,6,7,8 */
    m = ecg_delta_decode(enc, n, back, 64u, &count);
    TT_CHECK(count == 32u);
    TT_CHECK(memcmp(samples, back, 32u * sizeof(int16_t)) == 0);

    /* truncated input must not overrun the output */
    m = ecg_delta_decode(enc, 4u, back, 64u, &count);
    TT_CHECK(m == count);
    TT_CHECK(count == 3u);

    /* --- compression ratio on a realistic ECG block --- */
    {
        int16_t ecg[ECG_FRAME_SAMPLES];
        int16_t ecg_back[ECG_FRAME_SAMPLES];
        size_t  raw = ECG_FRAME_SAMPLES * 2u;
        size_t  d12, ddl;
        make_sample_block(ecg, ECG_FRAME_SAMPLES, 0x2222u);
        for (i = 0u; i < ECG_FRAME_SAMPLES; i++) {
            u12[i] = (uint16_t)((int32_t)ecg[i] & 0x0FFFu);
        }
        d12 = ecg_pack12(u12, ECG_FRAME_SAMPLES, enc, sizeof(enc));
        ddl = ecg_delta_encode(ecg, ECG_FRAME_SAMPLES, enc, sizeof(enc));
        m   = ecg_delta_decode(enc, ddl, ecg_back, ECG_FRAME_SAMPLES, &count);
        TT_CHECK(count == ECG_FRAME_SAMPLES);
        TT_CHECK(memcmp(ecg, ecg_back, sizeof(ecg)) == 0);

        tt_measure("payload_raw_int16_bytes", (double)raw, "B");
        tt_measure("payload_pack12_bytes", (double)d12, "B");
        tt_measure("payload_delta_bytes", (double)ddl, "B");
        tt_info("24 ECG samples: raw int16 %u B, pack12 %u B (%.1f %%), delta %u B (%.1f %%)",
                (unsigned)raw, (unsigned)d12, 100.0 * (double)d12 / (double)raw,
                (unsigned)ddl, 100.0 * (double)ddl / (double)raw);
    }

    /* --- the frame builder picks the most compact representation and the
     *     block survives the round trip through the wire format --- */
    {
        uint8_t frame[ECG_FRAME_MAX_SIZE];
        size_t  flen = 0u;
        int16_t block[ECG_FRAME_SAMPLES];
        int16_t rt[ECG_SAMPLE_BLOCK_MAX];
        size_t  bcount = 0u;
        uint8_t type;
        size_t  k;
        ecg_frame_t decoded;

        make_sample_block(block, ECG_FRAME_SAMPLES, 0x3333u);
        type = ecg_frame_build_samples(1u, block, ECG_FRAME_SAMPLES, frame,
                                       sizeof(frame), &flen);
        TT_CHECK(type == ECG_FRAME_ADC_DELTA || type == ECG_FRAME_ADC_PACK12);
        TT_CHECK(flen != 0u);
        TT_CHECK(ecg_frame_decode(frame, flen, &decoded) == 0);
        TT_CHECK(ecg_frame_parse_samples(&decoded, rt, ECG_SAMPLE_BLOCK_MAX, &bcount) == 0);
        TT_CHECK(bcount == ECG_FRAME_SAMPLES);
        for (k = 0u; k < bcount; k++) {
            TT_CHECK(rt[k] == block[k]);
        }
        tt_measure("sample_frame_bytes", (double)flen, "B");
        tt_measure("sample_frame_payload_bytes", (double)(flen - ECG_FRAME_OVERHEAD), "B");
        tt_info("sample frame: type 0x%X, %u bytes total (%u payload + %u overhead)",
                type, (unsigned)flen, (unsigned)(flen - ECG_FRAME_OVERHEAD),
                (unsigned)ECG_FRAME_OVERHEAD);

        /* a sample outside +-2048 uV must switch the codec to the delta form
         * and still round trip exactly */
        block[3] = 9000;
        block[7] = -9000;
        type = ecg_frame_build_samples(2u, block, ECG_FRAME_SAMPLES, frame,
                                       sizeof(frame), &flen);
        TT_CHECK(type == ECG_FRAME_ADC_DELTA);
        TT_CHECK(ecg_frame_decode(frame, flen, &decoded) == 0);
        TT_CHECK(ecg_frame_parse_samples(&decoded, rt, ECG_SAMPLE_BLOCK_MAX, &bcount) == 0);
        TT_CHECK(bcount == ECG_FRAME_SAMPLES);
        for (k = 0u; k < bcount; k++) {
            TT_CHECK(rt[k] == block[k]);
        }
        /* rejecting a block that is too long */
        TT_CHECK(ecg_frame_build_samples(3u, block, ECG_SAMPLE_BLOCK_MAX + 1u, frame,
                                         sizeof(frame), &flen) == 0u);
        TT_CHECK(flen == 0u);
    }

    tt_end();
}

static void test_report_frames(void)
{
    ecg_hr_report_t  hr, hr2;
    ecg_cal_report_t cal, cal2;
    ecg_frame_t      f;
    uint8_t          buf[ECG_FRAME_MAX_SIZE];
    size_t           len;

    tt_begin("protocol::report_frames");

    memset(&hr, 0, sizeof(hr));
    hr.flags       = 0x03u;
    hr.bpm_x10     = 723;
    hr.bpm_avg_x10 = 718;
    hr.rr_ms       = 834u;
    hr.quality     = 96u;
    len = ecg_frame_build_hr(5u, &hr, buf, sizeof(buf));
    TT_CHECK(len == ECG_FRAME_OVERHEAD + 9u);
    TT_CHECK(ecg_frame_decode(buf, len, &f) == 0);
    TT_CHECK(ecg_frame_parse_hr(&f, &hr2) == 0);
    TT_CHECK(hr2.bpm_x10 == 723);
    TT_CHECK(hr2.bpm_avg_x10 == 718);
    TT_CHECK(hr2.rr_ms == 834u);
    TT_CHECK(hr2.quality == 96u);
    TT_CHECK(hr2.flags == 0x03u);

    memset(&cal, 0, sizeof(cal));
    cal.slope_x1000           = 1241200;
    cal.intercept_counts_x1000 = -3500;
    cal.nonlinearity_x100     = 42;           /* 0.42 %FS   */
    cal.afe_gain_x10          = 10001;        /* 1000.1 V/V */
    cal.r2_x10000             = 9999;
    cal.n_points              = 4u;
    cal.flags                 = 0x01u;
    len = ecg_frame_build_cal(6u, &cal, buf, sizeof(buf));
    TT_CHECK(len == ECG_FRAME_OVERHEAD + 16u);
    TT_CHECK(ecg_frame_decode(buf, len, &f) == 0);
    TT_CHECK(ecg_frame_parse_cal(&f, &cal2) == 0);
    TT_CHECK(cal2.slope_x1000 == cal.slope_x1000);
    TT_CHECK(cal2.intercept_counts_x1000 == cal.intercept_counts_x1000);
    TT_CHECK(cal2.nonlinearity_x100 == cal.nonlinearity_x100);
    TT_CHECK(cal2.afe_gain_x10 == cal.afe_gain_x10);
    TT_CHECK(cal2.r2_x10000 == cal.r2_x10000);
    TT_CHECK(cal2.n_points == 4u);
    TT_CHECK(cal2.flags == 0x01u);

    len = ecg_frame_build_status(7u, 0x05u, 123u, 456789u, buf, sizeof(buf));
    TT_CHECK(len == ECG_FRAME_OVERHEAD + 7u);
    TT_CHECK(ecg_frame_decode(buf, len, &f) == 0);
    TT_CHECK(f.type == (uint8_t)ECG_FRAME_STATUS);

    len = ecg_frame_build_ack(8u, (uint8_t)ECG_FRAME_HOST_CMD, 77u, buf, sizeof(buf));
    TT_CHECK(len == ECG_FRAME_OVERHEAD + 3u);
    TT_CHECK(ecg_frame_decode(buf, len, &f) == 0);
    TT_CHECK(f.type == (uint8_t)ECG_FRAME_ACK);
    TT_CHECK(f.payload[0] == (uint8_t)ECG_FRAME_HOST_CMD);
    TT_CHECK(((uint16_t)f.payload[1] | ((uint16_t)f.payload[2] << 8)) == 77u);

    /* parser rejects the wrong type / the wrong length */
    TT_CHECK(ecg_frame_parse_hr(&f, &hr2) == -1);
    TT_CHECK(ecg_frame_parse_cal(&f, &cal2) == -1);
    TT_CHECK(ecg_frame_parse_hr(NULL, &hr2) == -1);

    tt_end();
}

void test_protocol_all(void)
{
    test_crc();
    test_frame_codec();
    test_frame_sync_basics();
    test_frame_sync_error_recovery();
    test_payload_codecs();
    test_report_frames();
}
