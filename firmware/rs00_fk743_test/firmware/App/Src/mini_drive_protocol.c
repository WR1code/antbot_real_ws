#include "mini_drive_protocol.h"

#include <stddef.h>
#include <string.h>

#define MINI_CAN_STANDARD_ID_MAX 0x7FFU

static int16_t read_i16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int32_t read_i32_be(const uint8_t *data)
{
    const uint32_t raw = ((uint32_t)data[0] << 24)
                         | ((uint32_t)data[1] << 16)
                         | ((uint32_t)data[2] << 8)
                         | data[3];
    return (int32_t)raw;
}

static bool make_i32(uint16_t id, uint8_t command, int32_t value,
                     MiniDriveFrame *frame)
{
    const uint32_t raw = (uint32_t)value;
    if ((id > MINI_CAN_STANDARD_ID_MAX) || (frame == NULL)) {
        return false;
    }
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->dlc = 5U;
    frame->data[0] = command;
    frame->data[1] = (uint8_t)(raw >> 24);
    frame->data[2] = (uint8_t)(raw >> 16);
    frame->data[3] = (uint8_t)(raw >> 8);
    frame->data[4] = (uint8_t)raw;
    return true;
}

bool MiniDrive_MakeHeartbeat(uint16_t id, MiniDriveFrame *frame)
{
    if ((id > MINI_CAN_STANDARD_ID_MAX) || (frame == NULL)) {
        return false;
    }
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->dlc = 1U;
    frame->data[0] = 0x00U;
    return true;
}

bool MiniDrive_MakeQuery(uint16_t id, MiniDriveFeedbackType type,
                         MiniDriveFrame *frame)
{
    if ((id > MINI_CAN_STANDARD_ID_MAX) || (frame == NULL)
        || ((type != MINI_DRIVE_FEEDBACK_FAULT)
            && (type != MINI_DRIVE_FEEDBACK_SPEED)
            && (type != MINI_DRIVE_FEEDBACK_VOLTAGE)
            && (type != MINI_DRIVE_FEEDBACK_MOTOR_CURRENT)
            && (type != MINI_DRIVE_FEEDBACK_TEMPERATURE)
            && (type != MINI_DRIVE_FEEDBACK_POSITION))) {
        return false;
    }
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->dlc = 2U;
    frame->data[0] = 0x0FU;
    frame->data[1] = (uint8_t)type;
    return true;
}

bool MiniDrive_MakeCurrent(uint16_t id, int16_t current_10ma,
                           MiniDriveFrame *frame)
{
    const uint16_t raw = (uint16_t)current_10ma;
    if ((id > MINI_CAN_STANDARD_ID_MAX) || (frame == NULL)) {
        return false;
    }
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->dlc = 3U;
    frame->data[0] = 0x01U;
    frame->data[1] = (uint8_t)(raw >> 8);
    frame->data[2] = (uint8_t)raw;
    return true;
}

bool MiniDrive_MakeSpeed(uint16_t id, int32_t speed_erpm,
                         MiniDriveFrame *frame)
{
    return make_i32(id, 0x02U, speed_erpm, frame);
}

bool MiniDrive_MakeAcceleration(uint16_t id, int32_t acceleration_erpm_s,
                                MiniDriveFrame *frame)
{
    if (acceleration_erpm_s < 0) {
        return false;
    }
    return make_i32(id, 0x0AU, acceleration_erpm_s, frame);
}

bool MiniDrive_MakeDeceleration(uint16_t id, int32_t deceleration_erpm_s,
                                MiniDriveFrame *frame)
{
    if (deceleration_erpm_s < 0) {
        return false;
    }
    return make_i32(id, 0x10U, deceleration_erpm_s, frame);
}

bool MiniDrive_DecodeFeedback(const MiniDriveFrame *frame,
                              MiniDriveFeedback *feedback)
{
    uint8_t type;
    if ((frame == NULL) || (feedback == NULL)
        || (frame->id > MINI_CAN_STANDARD_ID_MAX)
        || (frame->dlc < 2U) || (frame->data[0] != 0x0FU)) {
        return false;
    }

    type = frame->data[1];
    feedback->id = frame->id;
    feedback->type = (MiniDriveFeedbackType)type;
    switch (type) {
    case MINI_DRIVE_FEEDBACK_FAULT:
        if (frame->dlc != 4U) {
            return false;
        }
        feedback->value = (int32_t)(uint16_t)read_i16_be(&frame->data[2]);
        return true;
    case MINI_DRIVE_FEEDBACK_SPEED:
    case MINI_DRIVE_FEEDBACK_POSITION:
        if (frame->dlc != 6U) {
            return false;
        }
        feedback->value = read_i32_be(&frame->data[2]);
        return true;
    case MINI_DRIVE_FEEDBACK_MOTOR_CURRENT:
    case MINI_DRIVE_FEEDBACK_TEMPERATURE:
    case MINI_DRIVE_FEEDBACK_VOLTAGE:
        if (frame->dlc != 4U) {
            return false;
        }
        feedback->value = type == MINI_DRIVE_FEEDBACK_VOLTAGE
                              ? (int32_t)(uint16_t)read_i16_be(&frame->data[2])
                              : read_i16_be(&frame->data[2]);
        return true;
    default:
        return false;
    }
}
