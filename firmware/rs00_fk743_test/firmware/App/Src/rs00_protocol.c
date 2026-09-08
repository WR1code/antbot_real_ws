#include "rs00_protocol.h"

#include <string.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint16_t physical_to_u16(float value, float minimum, float maximum)
{
    const float limited = clampf(value, minimum, maximum);
    return (uint16_t)(((limited - minimum) * 65535.0f) / (maximum - minimum));
}

static float u16_to_physical(uint16_t value, float minimum, float maximum)
{
    return ((float)value * (maximum - minimum) / 65535.0f) + minimum;
}

static uint16_t read_be_u16(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static uint16_t read_le_u16(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

static void write_be_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value >> 8);
    bytes[1] = (uint8_t)value;
}

static void write_le_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static void write_le_u32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value >> 16);
    bytes[3] = (uint8_t)(value >> 24);
}

static rs00_frame_t make_empty_request(rs00_comm_type_t type,
                                       uint8_t master_id,
                                       uint8_t motor_id)
{
    rs00_frame_t frame = {0};
    frame.id = rs00_make_request_id(type, master_id, motor_id);
    return frame;
}

static rs00_frame_t make_write_base(uint8_t master_id,
                                    uint8_t motor_id,
                                    uint16_t index)
{
    rs00_frame_t frame =
        make_empty_request(RS00_TYPE_WRITE_PARAM, master_id, motor_id);
    write_le_u16(&frame.data[0], index);
    return frame;
}

uint32_t rs00_make_request_id(rs00_comm_type_t type,
                              uint8_t master_id,
                              uint8_t motor_id)
{
    return ((((uint32_t)type & 0x1FU) << 24)
            | ((uint32_t)master_id << 8)
            | motor_id) & RS00_EXT_ID_MASK;
}

uint8_t rs00_get_type(uint32_t extended_id)
{
    return (uint8_t)((extended_id >> 24) & 0x1FU);
}

rs00_frame_t rs00_make_query(uint8_t master_id, uint8_t motor_id)
{
    return make_empty_request(RS00_TYPE_GET_DEVICE_ID, master_id, motor_id);
}

rs00_frame_t rs00_make_enable(uint8_t master_id, uint8_t motor_id)
{
    return make_empty_request(RS00_TYPE_ENABLE, master_id, motor_id);
}

rs00_frame_t rs00_make_stop(uint8_t master_id, uint8_t motor_id, bool clear_fault)
{
    rs00_frame_t frame = make_empty_request(RS00_TYPE_STOP, master_id, motor_id);
    frame.data[0] = clear_fault ? 1U : 0U;
    return frame;
}

rs00_frame_t rs00_make_set_zero(uint8_t master_id, uint8_t motor_id)
{
    rs00_frame_t frame =
        make_empty_request(RS00_TYPE_SET_ZERO, master_id, motor_id);
    frame.data[0] = 1U;
    return frame;
}

rs00_frame_t rs00_make_read_param(uint8_t master_id,
                                  uint8_t motor_id,
                                  uint16_t index)
{
    rs00_frame_t frame =
        make_empty_request(RS00_TYPE_READ_PARAM, master_id, motor_id);
    write_le_u16(&frame.data[0], index);
    return frame;
}

rs00_frame_t rs00_make_write_u8(uint8_t master_id,
                                uint8_t motor_id,
                                uint16_t index,
                                uint8_t value)
{
    rs00_frame_t frame = make_write_base(master_id, motor_id, index);
    frame.data[4] = value;
    return frame;
}

rs00_frame_t rs00_make_write_u16(uint8_t master_id,
                                 uint8_t motor_id,
                                 uint16_t index,
                                 uint16_t value)
{
    rs00_frame_t frame = make_write_base(master_id, motor_id, index);
    write_le_u16(&frame.data[4], value);
    return frame;
}

rs00_frame_t rs00_make_write_u32(uint8_t master_id,
                                 uint8_t motor_id,
                                 uint16_t index,
                                 uint32_t value)
{
    rs00_frame_t frame = make_write_base(master_id, motor_id, index);
    write_le_u32(&frame.data[4], value);
    return frame;
}

rs00_frame_t rs00_make_write_float(uint8_t master_id,
                                   uint8_t motor_id,
                                   uint16_t index,
                                   float value)
{
    rs00_frame_t frame = make_write_base(master_id, motor_id, index);
    uint32_t raw = 0U;
    _Static_assert(sizeof(raw) == sizeof(value), "float must be IEEE-754 binary32");
    memcpy(&raw, &value, sizeof(raw));
    write_le_u32(&frame.data[4], raw);
    return frame;
}

rs00_frame_t rs00_make_write_raw(uint8_t master_id,
                                 uint8_t motor_id,
                                 uint16_t index,
                                 const uint8_t raw_value[4])
{
    rs00_frame_t frame = make_write_base(master_id, motor_id, index);
    if (raw_value != NULL) {
        memcpy(&frame.data[4], raw_value, 4U);
    }
    return frame;
}

rs00_frame_t rs00_make_motion(uint8_t motor_id,
                              float position_rad,
                              float velocity_rad_s,
                              float kp,
                              float kd,
                              float feedforward_torque_nm)
{
    rs00_frame_t frame = {0};
    const uint16_t torque =
        physical_to_u16(feedforward_torque_nm, -14.0f, 14.0f);
    frame.id = ((uint32_t)RS00_TYPE_MOTION_CONTROL << 24)
               | ((uint32_t)torque << 8)
               | motor_id;
    write_be_u16(&frame.data[0],
                 physical_to_u16(position_rad, -12.57f, 12.57f));
    write_be_u16(&frame.data[2],
                 physical_to_u16(velocity_rad_s, -33.0f, 33.0f));
    write_be_u16(&frame.data[4], physical_to_u16(kp, 0.0f, 500.0f));
    write_be_u16(&frame.data[6], physical_to_u16(kd, 0.0f, 5.0f));
    return frame;
}

bool rs00_decode_feedback(const rs00_frame_t *frame, rs00_feedback_t *feedback)
{
    uint32_t id;
    uint32_t status;

    if ((frame == NULL) || (feedback == NULL)
        || (rs00_get_type(frame->id) != RS00_TYPE_FEEDBACK)) {
        return false;
    }

    id = frame->id & RS00_EXT_ID_MASK;
    status = (id >> 16) & 0xFFU;
    memset(feedback, 0, sizeof(*feedback));
    feedback->state = (rs00_motor_state_t)((id >> 22) & 0x03U);
    feedback->uncalibrated = (status & 0x20U) != 0U;
    feedback->stall_overload = (status & 0x10U) != 0U;
    feedback->encoder_fault = (status & 0x08U) != 0U;
    feedback->over_temperature = (status & 0x04U) != 0U;
    feedback->phase_current_fault = (status & 0x02U) != 0U;
    feedback->under_voltage = (status & 0x01U) != 0U;
    feedback->motor_id = (uint8_t)(id >> 8);
    feedback->master_id = (uint8_t)id;
    feedback->position_rad =
        u16_to_physical(read_be_u16(&frame->data[0]),
                        RS00_FEEDBACK_POSITION_MIN_RAD,
                        RS00_FEEDBACK_POSITION_MAX_RAD);
    feedback->velocity_rad_s =
        u16_to_physical(read_be_u16(&frame->data[2]), -33.0f, 33.0f);
    feedback->torque_nm =
        u16_to_physical(read_be_u16(&frame->data[4]), -14.0f, 14.0f);
    feedback->temperature_c = (float)read_be_u16(&frame->data[6]) / 10.0f;
    return true;
}

bool rs00_decode_read_float(const rs00_frame_t *frame,
                            uint8_t expected_motor_id,
                            uint8_t expected_master_id,
                            uint16_t expected_index,
                            float *value)
{
    uint32_t raw;
    uint32_t id;

    if ((frame == NULL) || (value == NULL)
        || (rs00_get_type(frame->id) != RS00_TYPE_READ_PARAM)) {
        return false;
    }

    id = frame->id & RS00_EXT_ID_MASK;
    if ((((id >> 16) & 0xFFU) != 0U)
        || (((id >> 8) & 0xFFU) != expected_motor_id)
        || ((id & 0xFFU) != expected_master_id)
        || (read_le_u16(&frame->data[0]) != expected_index)) {
        return false;
    }

    raw = (uint32_t)frame->data[4]
          | ((uint32_t)frame->data[5] << 8)
          | ((uint32_t)frame->data[6] << 16)
          | ((uint32_t)frame->data[7] << 24);
    memcpy(value, &raw, sizeof(raw));
    return true;
}

bool rs00_decode_read_raw(const rs00_frame_t *frame,
                          uint8_t expected_motor_id,
                          uint8_t expected_master_id,
                          uint16_t expected_index,
                          uint8_t raw_value[4])
{
    uint32_t id;

    if ((frame == NULL) || (raw_value == NULL)
        || (rs00_get_type(frame->id) != RS00_TYPE_READ_PARAM)) {
        return false;
    }

    id = frame->id & RS00_EXT_ID_MASK;
    if ((((id >> 16) & 0xFFU) != 0U)
        || (((id >> 8) & 0xFFU) != expected_motor_id)
        || ((id & 0xFFU) != expected_master_id)
        || (read_le_u16(&frame->data[0]) != expected_index)) {
        return false;
    }
    memcpy(raw_value, &frame->data[4], 4U);
    return true;
}
