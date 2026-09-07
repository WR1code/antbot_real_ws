#include "chassis_uart_ack.h"

#include "host_cmd_vel_protocol.h"

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void write_i16(uint8_t *data, int16_t value)
{
    write_u16(data, (uint16_t)value);
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)read_u16(data);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U)
           | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void write_i32(uint8_t *data, int32_t value)
{
    write_u32(data, (uint32_t)value);
}

static int32_t read_i32(const uint8_t *data)
{
    return (int32_t)read_u32(data);
}

bool ChassisAck_Encode(const ChassisAck *ack,
                       uint8_t frame[CHASSIS_ACK_FRAME_SIZE])
{
    uint16_t crc;
    if ((ack == 0) || (frame == 0)) {
        return false;
    }
    frame[0] = CHASSIS_ACK_SYNC_0;
    frame[1] = CHASSIS_ACK_SYNC_1;
    frame[2] = CHASSIS_ACK_VERSION;
    frame[3] = CHASSIS_ACK_FRAME_SIZE;
    frame[4] = ack->sequence;
    frame[5] = (uint8_t)ack->status;
    frame[6] = ack->chassis_state;
    frame[7] = ack->reject_reason;
    write_u16(&frame[8], ack->fault_flags);
    frame[10] = ack->steering_flags;
    frame[11] = ack->can_flags;
    write_u16(&frame[12], ack->uart_valid_count);
    write_u16(&frame[14], ack->can_tx_count);
    write_u16(&frame[16], ack->can_rx_count);
    write_u32(&frame[18], ack->stm32_tick);
    write_i16(&frame[22], ack->steering_position_mrad[0]);
    write_i16(&frame[24], ack->steering_position_mrad[1]);
    write_i16(&frame[26], ack->steering_position_mrad[2]);
    write_i16(&frame[28], ack->steering_position_mrad[3]);
    frame[30] = ack->control_id;
    frame[31] = ack->detail_type;
    write_i32(&frame[32], ack->detail_values[0]);
    write_i32(&frame[36], ack->detail_values[1]);
    write_i32(&frame[40], ack->detail_values[2]);
    write_i32(&frame[44], ack->detail_values[3]);
    write_u16(&frame[48], ack->detail_valid_mask);
    crc = HostCmdVel_Crc16Ccitt(frame, 50U);
    write_u16(&frame[50], crc);
    return true;
}

bool ChassisAck_Decode(const uint8_t frame[CHASSIS_ACK_FRAME_SIZE],
                       ChassisAck *ack)
{
    if ((frame == 0) || (ack == 0)
        || (frame[0] != CHASSIS_ACK_SYNC_0)
        || (frame[1] != CHASSIS_ACK_SYNC_1)
        || (frame[2] != CHASSIS_ACK_VERSION)
        || (frame[3] != CHASSIS_ACK_FRAME_SIZE)
        || (read_u16(&frame[50])
            != HostCmdVel_Crc16Ccitt(frame, 50U))) {
        return false;
    }
    ack->sequence = frame[4];
    ack->status = (ChassisAckStatus)frame[5];
    ack->chassis_state = frame[6];
    ack->reject_reason = frame[7];
    ack->fault_flags = read_u16(&frame[8]);
    ack->steering_flags = frame[10];
    ack->can_flags = frame[11];
    ack->uart_valid_count = read_u16(&frame[12]);
    ack->can_tx_count = read_u16(&frame[14]);
    ack->can_rx_count = read_u16(&frame[16]);
    ack->stm32_tick = read_u32(&frame[18]);
    ack->steering_position_mrad[0] = read_i16(&frame[22]);
    ack->steering_position_mrad[1] = read_i16(&frame[24]);
    ack->steering_position_mrad[2] = read_i16(&frame[26]);
    ack->steering_position_mrad[3] = read_i16(&frame[28]);
    ack->control_id = frame[30];
    ack->detail_type = frame[31];
    ack->detail_values[0] = read_i32(&frame[32]);
    ack->detail_values[1] = read_i32(&frame[36]);
    ack->detail_values[2] = read_i32(&frame[40]);
    ack->detail_values[3] = read_i32(&frame[44]);
    ack->detail_valid_mask = read_u16(&frame[48]);
    return true;
}
