#include "rs00_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void expect_data(const rs00_frame_t *frame, const uint8_t expected[8])
{
    assert(memcmp(frame->data, expected, 8U) == 0);
}

static void test_basic_frames(void)
{
    const uint8_t zero[8] = {0};
    rs00_frame_t frame = rs00_make_query(0xFDU, 0x01U);
    assert(frame.id == 0x0000FD01U);
    expect_data(&frame, zero);

    frame = rs00_make_enable(0xFDU, 0x02U);
    assert(frame.id == 0x0300FD02U);
    expect_data(&frame, zero);

    frame = rs00_make_stop(0xFDU, 0x04U, false);
    assert(frame.id == 0x0400FD04U);
    expect_data(&frame, zero);
}

static void test_parameter_frames(void)
{
    static const uint8_t read_position[8] =
        {0x19, 0x70, 0, 0, 0, 0, 0, 0};
    static const uint8_t csp_mode[8] =
        {0x05, 0x70, 0, 0, 0x05, 0, 0, 0};
    static const uint8_t limit_spd[8] =
        {0x17, 0x70, 0, 0, 0, 0, 0x80, 0x3F};
    static const uint8_t limit_cur[8] =
        {0x18, 0x70, 0, 0, 0, 0, 0, 0x40};
    static const uint8_t timeout[8] =
        {0x28, 0x70, 0, 0, 0x10, 0x27, 0, 0};
    static const uint8_t position_raw[4] = {0xCD, 0xCC, 0x4C, 0x3E};
    static const uint8_t loc_ref[8] =
        {0x16, 0x70, 0, 0, 0xCD, 0xCC, 0x4C, 0x3E};

    rs00_frame_t frame =
        rs00_make_read_param(0xFDU, 0x01U, RS00_PARAM_MECH_POS);
    assert(frame.id == 0x1100FD01U);
    expect_data(&frame, read_position);

    frame = rs00_make_write_u8(0xFDU, 0x03U, RS00_PARAM_RUN_MODE,
                               RS00_RUN_CSP);
    assert(frame.id == 0x1200FD03U);
    expect_data(&frame, csp_mode);

    frame = rs00_make_write_float(0xFDU, 0x01U,
                                  RS00_PARAM_LIMIT_SPD, 1.0f);
    expect_data(&frame, limit_spd);

    frame = rs00_make_write_float(0xFDU, 0x01U, RS00_PARAM_LIMIT_CUR, 2.0f);
    expect_data(&frame, limit_cur);

    frame = rs00_make_write_u32(0xFDU, 0x01U,
                                RS00_PARAM_CAN_TIMEOUT, 10000U);
    expect_data(&frame, timeout);

    frame = rs00_make_write_raw(0xFDU, 0x01U, RS00_PARAM_LOC_REF,
                                position_raw);
    expect_data(&frame, loc_ref);
}

static void test_motion_frame(void)
{
    static const uint8_t expected[8] =
        {0x82, 0x08, 0x7F, 0xFF, 0x02, 0x8F, 0x19, 0x99};
    const rs00_frame_t frame =
        rs00_make_motion(0x7FU, 0.2f, 0.0f, 5.0f, 0.5f, 0.0f);
    assert(frame.id == 0x017FFF7FU);
    expect_data(&frame, expected);
}

static void test_feedback_decode(void)
{
    const rs00_frame_t frame = {
        .id = 0x02807FFDU,
        .data = {0x82, 0x08, 0x7F, 0xFF, 0x7F, 0xFF, 0x01, 0x2C}
    };
    rs00_feedback_t feedback;
    assert(rs00_decode_feedback(&frame, &feedback));
    assert(feedback.state == RS00_MODE_MOTOR);
    assert(feedback.motor_id == 0x7FU);
    assert(feedback.master_id == 0xFDU);
    assert(!feedback.under_voltage);
    assert(fabsf(feedback.position_rad - 0.2f) < 0.001f);
    assert(fabsf(feedback.velocity_rad_s) < 0.0011f);
    assert(fabsf(feedback.torque_nm) < 0.001f);
    assert(fabsf(feedback.temperature_c - 30.0f) < 0.001f);
}

static void test_read_float_decode(void)
{
    const rs00_frame_t frame = {
        .id = 0x110001FDU,
        .data = {0x19, 0x70, 0, 0, 0xCD, 0xCC, 0x4C, 0x3E}
    };
    float position = 0.0f;
    uint8_t raw[4];
    assert(rs00_decode_read_float(&frame, 0x01U, 0xFDU,
                                  RS00_PARAM_MECH_POS, &position));
    assert(fabsf(position - 0.2f) < 0.001f);
    assert(rs00_decode_read_raw(&frame, 0x01U, 0xFDU,
                                RS00_PARAM_MECH_POS, raw));
    assert(memcmp(raw, &frame.data[4], 4U) == 0);
}

static void test_steering_feedback_vector(void)
{
    const rs00_frame_t frame = {
        .id = 0x028001FDU,
        .data = {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA}
    };
    rs00_feedback_t feedback;
    assert(rs00_decode_feedback(&frame, &feedback));
    assert(feedback.state == RS00_MODE_MOTOR);
    assert(feedback.motor_id == 0x01U);
    assert(feedback.master_id == 0xFDU);
    assert(!feedback.uncalibrated && !feedback.stall_overload
           && !feedback.encoder_fault && !feedback.over_temperature
           && !feedback.phase_current_fault && !feedback.under_voltage);
}

int main(void)
{
    test_basic_frames();
    test_parameter_frames();
    test_motion_frame();
    test_feedback_decode();
    test_read_float_decode();
    test_steering_feedback_vector();
    puts("RS00 protocol tests passed");
    return 0;
}
