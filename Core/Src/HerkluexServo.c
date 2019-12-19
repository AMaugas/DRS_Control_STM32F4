/*
 * HerkluexServo.c
 *
 *  Created on: 11 déc. 2019
 *      Author: Antoine
 */

#include "HerkulexServo.h"

HerkulexServoBus *initializeServoBus(UART_HandleTypeDef *HUART_Handler)
{
    HerkulexServoBus *servoBus = (HerkulexServoBus *)malloc(sizeof(HerkulexServoBus));

    if (servoBus != NULL)
    {
        servoBus->m_serial = HUART_Handler;
        for (uint8_t i = 0; i < HERKULEX_PACKET_RX_MAX_DATA; i++)
        {
            servoBus->m_rx_buffer[i] = 0;
        }
        servoBus->m_last_serial = 0;

        servoBus->m_rx_packet.size = 0x00;
        servoBus->m_rx_packet.id = 0x00;
        servoBus->m_rx_packet.cmd = HerkulexCommand_None;
        servoBus->m_rx_packet.checksum1 = 0x00;
        servoBus->m_rx_packet.checksum2 = 0x00;
        for (uint8_t i = 0; i < HERKULEX_PACKET_RX_MAX_DATA; i++)
        {
            *(servoBus->m_rx_packet.data + i) = 0;
        }

        servoBus->m_rx_packet.status_error = HerkulexStatusError_None;
        servoBus->m_rx_packet.status_detail = HerkulexStatusDetail_None;
        servoBus->m_rx_packet.error = HerkulexPacketError_None;

        servoBus->m_rx_packet_ready = 0;
        for (uint8_t i = 0; i < HERKULEX_SERIAL_TX_BUFFER; i++)
        {
            *(servoBus->m_tx_buffer + i) = 0;
        }
        servoBus->m_move_tags = 0;
        servoBus->m_schedule_state = HerkulexScheduleState_None;

        servoBus->sendPacket = &sendPacket;
/*         servoBus->sendPacketAndReadResponse = &sendPacketAndReadResponse;
 */    }
return servoBus;
}

void sendPacket(HerkulexServoBus *self, uint8_t id, HerkulexCommand cmd, uint8_t *pData, uint8_t dataLen)
{
    uint8_t checksum1;
    uint8_t checksum2;
    uint8_t packetSize = 7 + dataLen;
    /* uint8_t packet[7] = {0xFF, 0xFF, packetSize, id, (uint8_t)cmd, 0x00, 0x00}; */

    uint8_t packet[packetSize];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = packetSize;
    packet[3] = id;
    packet[4] = (uint8_t)cmd;

    checksum1 = packetSize ^ id ^ (uint8_t)cmd;

    if (pData && dataLen > 0)
    {
        for (uint8_t i = 0; i < dataLen; i++)
        {
            checksum1 ^= *(pData + i);
            packet[7 + i] = *(pData + i);
        }
    }

    checksum1 = checksum1 & 0xFE;
    checksum2 = (~checksum1) & 0xFE;

    packet[5] = checksum1;
    packet[6] = checksum2;

/* 
    HAL_UART_Transmit(self->m_serial, packet, 7, 10);
    printf("Data transmitted: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X", packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6]);

    if (pData && dataLen > 0)
    {
        HAL_UART_Transmit(self->m_serial, pData, dataLen, 10);
        for (uint8_t i = 0; i < dataLen; i++)
        {
            printf(" 0x%.2X", *(pData + i));
        }
    }

    printf("\n\r");
 */

    HAL_UART_Transmit(self->m_serial, packet, packetSize, 10);
    printf("Packet sent : ");
    for (uint8_t i = 0; i < packetSize;i++)
    {
        printf("0x%.2X ", packet[i]);
    }
    printf("\n\r");
}

void prepareIndividualMove(HerkulexServoBus *self)
{
    self->m_schedule_state = HerkulexScheduleState_IndividualMove;
    self->m_move_tags = 0;
}

void prepareSynchronizedMove(HerkulexServoBus *self, uint8_t playtime)
{
    self->m_schedule_state = HerkulexScheduleState_SynchronizedMove;
    *(self->m_tx_buffer + 0) = playtime;
    self->m_move_tags = 0;
}

void executeMove(HerkulexServoBus *self)
{
    uint8_t dataLen;

    switch (self->m_schedule_state)
    {
    case HerkulexScheduleState_IndividualMove:
        dataLen = self->m_move_tags * 5;
        self->sendPacket(self, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, self->m_tx_buffer, dataLen);
        break;

    case HerkulexScheduleState_SynchronizedMove:
        dataLen = 1 + self->m_move_tags * 4;
        self->sendPacket(self, HERKULEX_BROADCAST_ID, HerkulexCommand_SJog, self->m_tx_buffer, dataLen);
        break;

    case HerkulexScheduleState_None:
        break;
    }

    self->m_schedule_state = HerkulexScheduleState_None;
    self->m_move_tags = 0;
}

HerkulexPacket m_response = {0, 0, HerkulexCommand_None, 0, 0, {0}, HerkulexStatusError_None, HerkulexStatusDetail_None, HerkulexPacketError_None};
uint8_t tx_buffer[5] = {0, 0, 0, 0, 0};

HerkulexServo *initializeServo(HerkulexServoBus *servoBus, uint8_t id)
{
    HerkulexServo *servo = (HerkulexServo *)malloc(sizeof(HerkulexServo));

    if (servo != NULL)
    {
        servo->m_bus = servoBus;
        servo->m_id = id;
        servo->m_led = HerkulexLed_Off;
        servo->m_position_control_mode = 1;
        servo->m_response = &m_response;
        servo->m_tx_buffer = tx_buffer;
    }

    return servo;
}

void jog(HerkulexServo *servo, uint8_t jog_lsb, uint8_t jog_msb, uint8_t set, uint8_t playtime)
{
    uint8_t idx_offset;

    switch (servo->m_bus->m_schedule_state)
    {
    case HerkulexScheduleState_None:
        *(servo->m_tx_buffer + 0) = jog_lsb;
        *(servo->m_tx_buffer + 1) = jog_msb;
        *(servo->m_tx_buffer + 2) = set;
        *(servo->m_tx_buffer + 3) = servo->m_id;
        *(servo->m_tx_buffer + 4) = playtime;
        servo->m_bus->sendPacket(servo->m_bus, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, servo->m_tx_buffer, 5);
        break;

    case HerkulexScheduleState_IndividualMove:
        if (((servo->m_bus->m_move_tags + 1) * 5) > HERKULEX_SERIAL_TX_BUFFER)
        {
            return; /* No room for another move tag, exit */
        }

        idx_offset = servo->m_bus->m_move_tags * 5; /* 5 bytes per tag */

        *(servo->m_bus->m_tx_buffer + idx_offset) = jog_lsb;
        *(servo->m_bus->m_tx_buffer + idx_offset + 1) = jog_msb;
        *(servo->m_bus->m_tx_buffer + idx_offset + 2) = set;
        *(servo->m_bus->m_tx_buffer + idx_offset + 3) = servo->m_id;
        *(servo->m_bus->m_tx_buffer + idx_offset + 4) = playtime;

        servo->m_bus->m_move_tags++;
        break;

    case HerkulexScheduleState_SynchronizedMove:
        if ((1 + (servo->m_bus->m_move_tags + 1) * 4) > HERKULEX_SERIAL_TX_BUFFER)
        {
            return; /* No room for another move tag, exit */
        }

        idx_offset = 1 + servo->m_bus->m_move_tags * 4; /* 4 bytes per tag, 1 byte offset for time */

        servo->m_bus->m_tx_buffer[idx_offset] = jog_lsb;
        servo->m_bus->m_tx_buffer[idx_offset + 1] = jog_msb;
        servo->m_bus->m_tx_buffer[idx_offset + 2] = set;
        servo->m_bus->m_tx_buffer[idx_offset + 3] = servo->m_id;

        servo->m_bus->m_move_tags++;
        break;
    default:
        break;
    }
}

void setPosition(HerkulexServo *servo, uint16_t pos, uint8_t playtime, HerkulexLed led)
{
    if (!servo->m_position_control_mode)
    {
        return;
    }

    uint8_t jog_lsb;
    uint8_t jog_msb;
    uint8_t set;

    jog_lsb = (uint8_t)pos;
    jog_msb = (uint8_t)(pos >> 8);

    if (led != HerkulexLed_Ignore)
    {
        servo->m_led = led;
        set = (uint8_t)(led << 2);
    }
    else
    {
        set = (uint8_t)(servo->m_led << 2);
    }

    jog(servo, jog_lsb, jog_msb, set, playtime);
}

void setSpeed(HerkulexServo *servo, uint16_t speed, uint8_t playtime, HerkulexLed led)
{
    if (servo->m_position_control_mode)
    {
        return;
    }

    uint8_t jog_lsb;
    uint8_t jog_msb;
    uint8_t set;
    uint16_t speed_raw;

    if (speed >= 0)
    {
        speed_raw = speed;
    }
    else
    {
        speed_raw = -speed;
    }
    speed_raw &= 0x03FF; /* Upper speed limit to 1023 */

    jog_lsb = (uint8_t)speed_raw;
    jog_msb = (uint8_t)(speed_raw >> 8);

    if (speed < 0)
    {
        jog_msb |= 0x40;
    }

    if (led != HerkulexLed_Ignore)
    {
        servo->m_led = led;
        set = (uint8_t)(led) << 2;
    }
    else
    {
        set = (uint8_t)(servo->m_led) << 2;
    }
    set |= 0x02; /* Continuous rotation flag */

    jog(servo, jog_lsb, jog_msb, set, playtime);
}

void setTorqueOn(HerkulexServo *servo)
{
    writeRam(servo, HerkulexRamRegister_TorqueControl, 0x60);
}

void setTorqueOff(HerkulexServo *servo)
{
    writeRam(servo, HerkulexRamRegister_TorqueControl, 0x00);
}

void setBrake(HerkulexServo *servo)
{
    writeRam(servo, HerkulexRamRegister_TorqueControl, 0x40);
}

void setLedColor(HerkulexServo *servo, HerkulexLed color)
{
    if (color == HerkulexLed_Ignore)
    {
        return;
    }

    servo->m_led = color;
    writeRam(servo, HerkulexRamRegister_LedControl, (uint8_t)color);
}
void writeRam(HerkulexServo *servo, HerkulexRamRegister reg, uint8_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 1;
    *(servo->m_tx_buffer + 2) = val;

    servo->m_bus->sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_RamWrite, servo->m_tx_buffer, 3);
}

void writeRam2(HerkulexServo *servo, HerkulexRamRegister reg, uint16_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 2;
    *(servo->m_tx_buffer + 2) = (uint8_t)val;
    *(servo->m_tx_buffer + 3) = (uint8_t)(val >> 8);

    servo->m_bus->sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_RamWrite, servo->m_tx_buffer, 4);
}

void writeEep(HerkulexServo *servo, HerkulexRamRegister reg, uint8_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 1;
    *(servo->m_tx_buffer + 2) = val;

    servo->m_bus->sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_EepWrite, servo->m_tx_buffer, 3);
}

void writeEep2(HerkulexServo *servo, HerkulexRamRegister reg, uint16_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 2;
    *(servo->m_tx_buffer + 2) = (uint8_t)val;
    *(servo->m_tx_buffer + 3) = (uint8_t)(val >> 8);

    servo->m_bus->sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_EepWrite, servo->m_tx_buffer, 4);
}

void enablePositionControlMode(HerkulexServo *servo)
{
    if (servo->m_position_control_mode)
    {
        return;
    }

    servo->m_position_control_mode = 1;

    uint8_t set;

    set = (uint8_t)servo->m_led << 2;
    set |= 0x02; /* Continuous rotation */
    set |= 0x20; /* jog invalid */

    *(servo->m_tx_buffer + 0) = 0x00;
    *(servo->m_tx_buffer + 1) = 0x00;
    *(servo->m_tx_buffer + 2) = set;
    *(servo->m_tx_buffer + 3) = servo->m_id;
    *(servo->m_tx_buffer + 4) = 0x00;

    servo->m_bus->sendPacket(servo->m_bus, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, servo->m_tx_buffer, 5);
}

void enableSpeedControlMode(HerkulexServo *servo)
{
    if (!servo->m_position_control_mode)
    {
        return;
    }

    servo->m_position_control_mode = 0;

    uint8_t set = 0;

    set |= ((uint8_t)servo->m_led) << 2;
    set |= 0x20; /* jog invalid, continuous rotation bit (0x02) implicitly set to 0 */

    *(servo->m_tx_buffer + 0) = 0x00; /* jog lsb */
    *(servo->m_tx_buffer + 1) = 0x00; /* jog msb */
    *(servo->m_tx_buffer + 2) = set;
    *(servo->m_tx_buffer + 3) = servo->m_id;
    *(servo->m_tx_buffer + 4) = 0x00; /* playtime */
    servo->m_bus->sendPacket(servo->m_bus, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, servo->m_tx_buffer, 5);
}
void servoReboot(HerkulexServo *servo)
{
    servo->m_bus->sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_Reboot, NULL, 0);
}

void rollBackToFactoryDefault(HerkulexServo *servo, uint8_t skipID, uint8_t skipBaud)
{
    *(servo->m_tx_buffer + 0) = skipID ? 1 : 0;
    *(servo->m_tx_buffer + 1) = skipBaud ? 1 : 0;

    servo->m_bus->sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_Rollback, servo->m_tx_buffer, 2);
}
