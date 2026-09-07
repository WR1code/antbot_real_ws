#ifndef MINI_DRIVE_PROTOCOL_H
#define MINI_DRIVE_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} MiniDriveFrame;

typedef enum {
    MINI_DRIVE_FEEDBACK_FAULT = 0x00,
    MINI_DRIVE_FEEDBACK_SPEED = 0x01,
    MINI_DRIVE_FEEDBACK_VOLTAGE = 0x04,
    MINI_DRIVE_FEEDBACK_MOTOR_CURRENT = 0x05,
    MINI_DRIVE_FEEDBACK_TEMPERATURE = 0x07,
    MINI_DRIVE_FEEDBACK_POSITION = 0x08
} MiniDriveFeedbackType;

typedef struct {
    uint16_t id;
    MiniDriveFeedbackType type;
    int32_t value;
} MiniDriveFeedback;

bool MiniDrive_MakeHeartbeat(uint16_t id, MiniDriveFrame *frame);
bool MiniDrive_MakeQuery(uint16_t id, MiniDriveFeedbackType type,
                         MiniDriveFrame *frame);
bool MiniDrive_MakeCurrent(uint16_t id, int16_t current_10ma,
                           MiniDriveFrame *frame);
bool MiniDrive_MakeSpeed(uint16_t id, int32_t speed_erpm,
                         MiniDriveFrame *frame);
bool MiniDrive_MakeAcceleration(uint16_t id, int32_t acceleration_erpm_s,
                                MiniDriveFrame *frame);
bool MiniDrive_MakeDeceleration(uint16_t id, int32_t deceleration_erpm_s,
                                MiniDriveFrame *frame);
bool MiniDrive_DecodeFeedback(const MiniDriveFrame *frame,
                              MiniDriveFeedback *feedback);

#endif
