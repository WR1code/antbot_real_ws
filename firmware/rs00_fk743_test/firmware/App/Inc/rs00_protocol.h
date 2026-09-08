#ifndef RS00_PROTOCOL_H
#define RS00_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS00_CAN_DLC_BYTES       8U
#define RS00_DEFAULT_MOTOR_ID    0x7FU
#define RS00_DEFAULT_MASTER_ID   0xFDU
#define RS00_EXT_ID_MASK         0x1FFFFFFFU
#define RS00_FEEDBACK_POSITION_MIN_RAD (-12.57f)
#define RS00_FEEDBACK_POSITION_MAX_RAD (12.57f)
#define RS00_FEEDBACK_POSITION_PERIOD_RAD \
    (RS00_FEEDBACK_POSITION_MAX_RAD - RS00_FEEDBACK_POSITION_MIN_RAD)

typedef enum {
    RS00_TYPE_GET_DEVICE_ID = 0x00,
    RS00_TYPE_MOTION_CONTROL = 0x01,
    RS00_TYPE_FEEDBACK = 0x02,
    RS00_TYPE_ENABLE = 0x03,
    RS00_TYPE_STOP = 0x04,
    RS00_TYPE_SET_ZERO = 0x06,
    RS00_TYPE_SET_CAN_ID = 0x07,
    RS00_TYPE_READ_PARAM = 0x11,
    RS00_TYPE_WRITE_PARAM = 0x12,
    RS00_TYPE_FAULT = 0x15,
    RS00_TYPE_SAVE = 0x16,
    RS00_TYPE_SET_BITRATE = 0x17,
    RS00_TYPE_ACTIVE_REPORT = 0x18,
    RS00_TYPE_SET_PROTOCOL = 0x19
} rs00_comm_type_t;

typedef enum {
    RS00_MODE_RESET = 0,
    RS00_MODE_CALI = 1,
    RS00_MODE_MOTOR = 2
} rs00_motor_state_t;

typedef enum {
    RS00_RUN_MOTION = 0,
    RS00_RUN_PP = 1,
    RS00_RUN_SPEED = 2,
    RS00_RUN_CURRENT = 3,
    RS00_RUN_CSP = 5
} rs00_run_mode_t;

typedef struct {
    uint32_t id;
    uint8_t data[RS00_CAN_DLC_BYTES];
} rs00_frame_t;

typedef struct {
    rs00_motor_state_t state;
    uint8_t motor_id;
    uint8_t master_id;
    bool uncalibrated;
    bool stall_overload;
    bool encoder_fault;
    bool over_temperature;
    bool phase_current_fault;
    bool under_voltage;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} rs00_feedback_t;

enum {
    RS00_PARAM_RUN_MODE = 0x7005,
    RS00_PARAM_IQ_REF = 0x7006,
    RS00_PARAM_SPEED_REF = 0x700A,
    RS00_PARAM_TORQUE_LIMIT = 0x700B,
    RS00_PARAM_POSITION_REF = 0x7016,
    RS00_PARAM_SPEED_LIMIT = 0x7017,
    RS00_PARAM_CURRENT_LIMIT = 0x7018,
    RS00_PARAM_MECH_POSITION = 0x7019,
    RS00_PARAM_MECH_VELOCITY = 0x701B,
    RS00_PARAM_VBUS = 0x701C,
    RS00_PARAM_SPEED_ACCEL = 0x7022,
    RS00_PARAM_VEL_MAX = 0x7024,
    RS00_PARAM_ACC_SET = 0x7025,
    RS00_PARAM_CAN_TIMEOUT = 0x7028
};

/* Names used by the steering application and the RS00 manual. */
#define RS00_PARAM_LOC_REF        RS00_PARAM_POSITION_REF
#define RS00_PARAM_LIMIT_SPD      RS00_PARAM_SPEED_LIMIT
#define RS00_PARAM_LIMIT_CUR      RS00_PARAM_CURRENT_LIMIT
#define RS00_PARAM_MECH_POS       RS00_PARAM_MECH_POSITION

uint32_t rs00_make_request_id(rs00_comm_type_t type,
                              uint8_t master_id,
                              uint8_t motor_id);
uint8_t rs00_get_type(uint32_t extended_id);

rs00_frame_t rs00_make_query(uint8_t master_id, uint8_t motor_id);
rs00_frame_t rs00_make_enable(uint8_t master_id, uint8_t motor_id);
rs00_frame_t rs00_make_stop(uint8_t master_id, uint8_t motor_id, bool clear_fault);
rs00_frame_t rs00_make_set_zero(uint8_t master_id, uint8_t motor_id);
rs00_frame_t rs00_make_read_param(uint8_t master_id,
                                  uint8_t motor_id,
                                  uint16_t index);
rs00_frame_t rs00_make_write_u8(uint8_t master_id,
                                uint8_t motor_id,
                                uint16_t index,
                                uint8_t value);
rs00_frame_t rs00_make_write_u16(uint8_t master_id,
                                 uint8_t motor_id,
                                 uint16_t index,
                                 uint16_t value);
rs00_frame_t rs00_make_write_u32(uint8_t master_id,
                                 uint8_t motor_id,
                                 uint16_t index,
                                 uint32_t value);
rs00_frame_t rs00_make_write_float(uint8_t master_id,
                                   uint8_t motor_id,
                                   uint16_t index,
                                   float value);
rs00_frame_t rs00_make_write_raw(uint8_t master_id,
                                 uint8_t motor_id,
                                 uint16_t index,
                                 const uint8_t raw_value[4]);
rs00_frame_t rs00_make_motion(uint8_t motor_id,
                              float position_rad,
                              float velocity_rad_s,
                              float kp,
                              float kd,
                              float feedforward_torque_nm);

bool rs00_decode_feedback(const rs00_frame_t *frame, rs00_feedback_t *feedback);
bool rs00_decode_read_float(const rs00_frame_t *frame,
                            uint8_t expected_motor_id,
                            uint8_t expected_master_id,
                            uint16_t expected_index,
                            float *value);
bool rs00_decode_read_raw(const rs00_frame_t *frame,
                          uint8_t expected_motor_id,
                          uint8_t expected_master_id,
                          uint16_t expected_index,
                          uint8_t raw_value[4]);

#ifdef __cplusplus
}
#endif

#endif
