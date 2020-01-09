/*
 * HerkluexServo.c
 *
 *  Created on: 11 déc. 2019
 *      Author: Antoine
 */

#include "HerkulexServo.h"

/**
  * @brief  Initialize the servo bus structure
  * @param  HUART_Handler : A pointer to the UART handler used to communicate with the servomotors
  * @retval Initialized ServoBus structure
  */
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
    }
    return servoBus;
}

/** 
    @brief Construct and end a specific UART packet
    @param self : The ServoBus used to communicate
    @param id : Servomotor ID
    @param cmd : enum of the command to send to the servo
    @param pData : array of optional data to send
    @param dataLen : Lenght of the pData array
    @retval None
 */
void sendPacket(HerkulexServoBus *self, uint8_t id, HerkulexCommand cmd, uint8_t *pData, uint8_t dataLen)
{
    uint8_t checksum1;
    uint8_t checksum2;
    uint8_t packetSize = 7 + dataLen;
    uint8_t packet[7] = {0xFF, 0xFF, packetSize, id, (uint8_t)cmd, 0x00, 0x00};
    /* uint8_t packet[packetSize]; */

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
            /* packet[7 + i] = *(pData + i); */
        }
    }

    checksum1 = checksum1 & 0xFE;
    checksum2 = (~checksum1) & 0xFE;

    packet[5] = checksum1;
    packet[6] = checksum2;

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

    //    HAL_UART_Transmit(self->m_serial, packet, packetSize, 10);
    //    printf("Packet sent : ");
    //    for (uint8_t i = 0; i < packetSize; i++)
    //    {
    //        printf("0x%.2X ", packet[i]);
    //    }
    //    printf("\n\r");
}

/**
 * @brief Process a received packet
 * @param bus : ServoBus receiving the data
 * @param dataLen : Lenght of the data to receive
 * @retval None
  */
void processPacket(HerkulexServoBus *bus, const uint8_t dataLen)
{
    uint8_t bytesToProcess = 0;
    uint8_t dataIdx = 0;
    uint8_t chkSum1 = 0;
    uint8_t chkSum2 = 0;
    HerkulexParserState parserState = HerkulexParserState_Header1;

    HAL_UART_Receive(bus->m_serial, bus->m_rx_buffer, dataLen, 10);

    if ((bus->m_rx_buffer[0] != 0xFF) || (bus->m_rx_buffer[1] != 0xFF))
    {
        return;
    }

    bytesToProcess = bus->m_rx_buffer[2];

    while (bytesToProcess > 0)
    {
        uint8_t recByte = bus->m_rx_buffer[bus->m_rx_buffer[2] - bytesToProcess];

        switch (parserState)
        {
        case HerkulexParserState_Header1:
            if (recByte == 0xFF)
            {
                parserState = HerkulexParserState_Header2;
            }
            break;

        case HerkulexParserState_Header2:
            if (recByte == 0xFF)
            {
                parserState = HerkulexParserState_Length;
            }
            else
            {
                parserState = HerkulexParserState_Header1;
            }
            break;

        case HerkulexParserState_Length:
            bus->m_rx_packet.size = recByte;
            chkSum1 = recByte;
            parserState = HerkulexParserState_ID;
            break;

        case HerkulexParserState_ID:
            bus->m_rx_packet.id = recByte;
            chkSum1 ^= recByte;
            parserState = HerkulexParserState_Command;
            break;

        case HerkulexParserState_Command:
            bus->m_rx_packet.cmd = (HerkulexCommand)recByte;
            chkSum1 ^= recByte;
            parserState = HerkulexParserState_Checksum1;
            break;

        case HerkulexParserState_Checksum1:
            bus->m_rx_packet.checksum1 = recByte;
            parserState = HerkulexParserState_Checksum2;
            break;

        case HerkulexParserState_Checksum2:
            bus->m_rx_packet.checksum2 = recByte;
            parserState = HerkulexParserState_Data;
            break;

        case HerkulexParserState_Data:
            bus->m_rx_packet.data[dataIdx] = recByte;
            chkSum1 ^= recByte;
            dataIdx++;

            if (dataIdx > HERKULEX_PACKET_RX_MAX_DATA)
            {
                dataIdx = HERKULEX_PACKET_RX_MAX_DATA;
            }
            break;

        default:
            break;
        }
    }

    if (dataIdx >= 2)
    {
        bus->m_rx_packet.status_error = bus->m_rx_packet.data[dataIdx - 2];
        bus->m_rx_packet.status_detail = bus->m_rx_packet.data[dataIdx - 1];
    }

    chkSum1 = chkSum1 & 0xFE;
    chkSum2 = (~chkSum1) & 0xFE;

    if (chkSum1 != bus->m_rx_packet.checksum1 || chkSum2 != bus->m_rx_packet.checksum2)
    {
        bus->m_rx_packet.error |= HerkulexPacketError_Checksum;
    }

    bus->m_rx_packet_ready = 1;
}

/**
 * @brief Transfer the data from the ServoBus buffer to the appropriate Servo data structure
 * @param bus : ServoBus to extract data from
 * @param response : pointer to the data structure of the receiving Servo
 * @retval Success or not of the function
  */
uint8_t getPacket(HerkulexServoBus *bus, HerkulexPacket *response)
{
    if (bus->m_rx_packet_ready == 0)
    {
        return 0;
    }

    response->size = bus->m_rx_packet.size;
    response->id = bus->m_rx_packet.id;
    response->cmd = bus->m_rx_packet.cmd;
    response->checksum1 = bus->m_rx_packet.checksum1;
    response->checksum2 = bus->m_rx_packet.checksum2;
    for (uint8_t i = 0; i < HERKULEX_PACKET_RX_MAX_DATA; i++)
    {
        response->data[i] = bus->m_rx_packet.data[i];
    }
    response->status_error = bus->m_rx_packet.status_error;
    response->status_detail = bus->m_rx_packet.status_detail;
    response->error = bus->m_rx_packet.error;

    bus->m_rx_packet_ready = 0;
    return 1;
}

/**
 * @brief Send a packet trough the bus and wait for the Servo to repond.
 *        Once the Servo responded, the response is stored in is internal data structure
 * @param self : Pointer to the ServoBus used for communication
 * @param response : Pointer the the Packet structure of the Servo
 * @param id : ID of the Servo
 * @param cmd : Enum of the command to send
 * @param pData : Array of the optional additional data to send
 * @param dataLen : Lenght of the data array
 * @retval Succes or not
 */
uint8_t sendPacketAndWaitResponse(HerkulexServoBus *self, HerkulexPacket *response, uint8_t id, HerkulexCommand cmd, uint8_t *pData, uint8_t dataLen)
{
    uint8_t success = 0;
    uint32_t time_started = 0;
    uint8_t dataLenToReceive;

    if (cmd == HerkulexCommand_Stat)
    {
        dataLenToReceive = 9; /* 7 minimal bytes + status_error + status_detail */
    }
    else
    {
        dataLenToReceive = 9 + *(pData + 1);
    }

    self->m_rx_packet_ready = 0;

    for (uint8_t attempts = 0; attempts < HERKULEX_PACKET_RETRIES; attempts++)
    {
        sendPacket(self, id, cmd, pData, dataLen);

        time_started = HAL_GetTick();

        while (!getPacket(self, response) && ((HAL_GetTick() - time_started) < HERKULEX_PACKET_RX_TIMEOUT))
        {
            processPacket(self, dataLenToReceive);
        }

        if ((response->error == HerkulexPacketError_None) && (response->id = id) && (response->cmd == (cmd | 0x40)))
        {
            success = 1;
            break;
        }
        else
        {
            HAL_Delay(HERKULEX_PACKET_RX_TIMEOUT);
        }
    }
    return success;
}

/**
 * @brief Switch the state of the bus to individual move
 * @note In individual move, each servo will be controlled independently from each other
 * @param self : The ServoBus
 * @retval None
 */
void prepareIndividualMove(HerkulexServoBus *self)
{
    self->m_schedule_state = HerkulexScheduleState_IndividualMove;
    self->m_move_tags = 0;
}

/**
 * @brief Switch the state of the ServoBus to execute synchronized move of the Servos
 * @note In synchronised move, the Servs will all have the same time to execute the position change
 * @param self : The ServoBus
 * @param time_ms : The time, in milliseconds, allowed to the Servos to execute the move
 * @retval None
 */
void prepareSynchronizedMove(HerkulexServoBus *self, uint16_t time_ms)
{
    uint8_t playtime = (uint8_t)(time_ms / 11.2f);

    self->m_schedule_state = HerkulexScheduleState_SynchronizedMove;
    *(self->m_tx_buffer + 0) = playtime;
    self->m_move_tags = 0;
}

/**
 * @brief Execute the move scheduled on the ServoBus and switch the scheduled state to none
 * @param self : The ServoBus to execute the moves from
 * @retval None
 */
void executeMove(HerkulexServoBus *self)
{
    uint8_t dataLen;

    switch (self->m_schedule_state)
    {
    case HerkulexScheduleState_IndividualMove:
        dataLen = self->m_move_tags * 5;
        sendPacket(self, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, self->m_tx_buffer, dataLen);
        break;

    case HerkulexScheduleState_SynchronizedMove:
        dataLen = 1 + self->m_move_tags * 4;
        sendPacket(self, HERKULEX_BROADCAST_ID, HerkulexCommand_SJog, self->m_tx_buffer, dataLen);
        break;

    case HerkulexScheduleState_None:
        break;
    }

    self->m_schedule_state = HerkulexScheduleState_None;
    self->m_move_tags = 0;
}

/* Structure used to store the servos responses */
HerkulexPacket m_response = {0, 0, HerkulexCommand_None, 0, 0, {0}, HerkulexStatusError_None, HerkulexStatusDetail_None, HerkulexPacketError_None};

/* Array used to store the tx data from the servos */
uint8_t tx_buffer[5] = {0, 0, 0, 0, 0};

/**
 * @brief Initialize a Servo structure, assiciating a ServoBus and an ID to that Servo
 * @param servoBus : Pointer to the servoBus used to communicate with the Servo
 * @param id : ID of the Servo
 * @retval Pointer to an initialized structure
 */
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

/**
 * @brief Write the command to the appropriate tx buffer depending on the ServoBus scheduled state
 * @param servo : Pointer to the servo to control
 * @param jog_lsb : og command lsb
 * @param jog_msb : Jog command msb
 * @param set
 * @param playtime : Playtime of the movement
 * @retval None
 */
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
        sendPacket(servo->m_bus, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, servo->m_tx_buffer, 5);
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

/**
 * @brief Set the servo to the desired position
 * @param servo : Pointer to the Servo to control
 * @param degree : Desired position in degree
 * @param time_ms : Time allowed to execute the move, in milliseconds
 * @param led : Color of the Servo led
 * @retval None
 */
void setPosition(HerkulexServo *servo, float degree, uint8_t time_ms, HerkulexLed led)
{
    uint16_t pos;

    pos = (uint16_t)(512 + (degree / 0.325f));

    uint8_t playtime = (uint8_t)(time_ms / 11.2f);

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

/**
 * @brief Set the rotation speed of the Servo
 * @param servo : Pointer to the Servo to control
 * @param speed : Rotation speed
 * @param time_ms : Time of the rotation in milliseconds
 * @param led : Led color of the Servo
 * @retval None
 */
void setSpeed(HerkulexServo *servo, uint16_t speed, uint16_t time_ms, HerkulexLed led)
{
    uint8_t playtime = (uint8_t)(time_ms / 11.2f);

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

/**
 * @brief Set the torque ON on the desired Servo
 * @param servo : Pointer to the Servo to control
 * @retval None
 */
void setTorqueOn(HerkulexServo *servo)
{
    writeRam(servo, HerkulexRamRegister_TorqueControl, 0x60);
}

/**
 * @brief Set the torque OFF on the desired Servo
 * @param servo : Pointer to the Servo to control
 * @retval None
 */
void setTorqueOff(HerkulexServo *servo)
{
    writeRam(servo, HerkulexRamRegister_TorqueControl, 0x00);
}

/**
 * @brief Set the brake on the desired Servo
 * @param servo : Pointer to the Servo to controll
 * @retval None
 */
void setBrake(HerkulexServo *servo)
{
    writeRam(servo, HerkulexRamRegister_TorqueControl, 0x40);
}

/**
 * @brief Set the led color of the desired Servo
 * @param servo : Pointer to the Servo to control
 * @param color : desired led color
 * @retval None
 */
void setLedColor(HerkulexServo *servo, HerkulexLed color)
{
    if (color == HerkulexLed_Ignore)
    {
        return;
    }

    servo->m_led = color;
    writeRam(servo, HerkulexRamRegister_LedControl, (uint8_t)color);
}

/**
 * @brief Write the desired one byte value to a specified RAM register
 * @param servo : Pointer to the Servo to control
 * @param reg : register to write to
 * @param val : value to write
 * @retval None
 */
void writeRam(HerkulexServo *servo, HerkulexRamRegister reg, uint8_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 1;
    *(servo->m_tx_buffer + 2) = val;

    sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_RamWrite, servo->m_tx_buffer, 3);
}

/**
 * @brief Write the desired two bytes value to a specified RAM register
 * @param servo : Pointer to the Servo to control
 * @param reg : register to write to
 * @param val : value to write
 * @retval None
 */
void writeRam2(HerkulexServo *servo, HerkulexRamRegister reg, uint16_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 2;
    *(servo->m_tx_buffer + 2) = (uint8_t)val;
    *(servo->m_tx_buffer + 3) = (uint8_t)(val >> 8);

    sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_RamWrite, servo->m_tx_buffer, 4);
}

/**
 * @brief Write the desired one byte value to a specified EEP register
 * @param servo : Pointer to the Servo to control
 * @param reg : register to write to
 * @param val : value to write
 * @retval None
 */
void writeEep(HerkulexServo *servo, HerkulexEepRegister reg, uint8_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 1;
    *(servo->m_tx_buffer + 2) = val;

    sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_EepWrite, servo->m_tx_buffer, 3);
}

/**
 * @brief Write the desired two bytes value to a specified EEP register
 * @param servo : Pointer to the Servo to control
 * @param reg : register to write to
 * @param val : value to write
 * @retval None
 */
void writeEep2(HerkulexServo *servo, HerkulexEepRegister reg, uint16_t val)
{
    *(servo->m_tx_buffer + 0) = (uint8_t)reg;
    *(servo->m_tx_buffer + 1) = 2;
    *(servo->m_tx_buffer + 2) = (uint8_t)val;
    *(servo->m_tx_buffer + 3) = (uint8_t)(val >> 8);

    sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_EepWrite, servo->m_tx_buffer, 4);
}

/**
 * @brief Read a specific one byte value from the Servo RAM register
 * @param servo : Pointer to the servo to read from
 * @param reg : register to read
 * @retval Value of the register
 */
uint8_t readRam(HerkulexServo *servo, HerkulexRamRegister reg)
{
    servo->m_tx_buffer[0] = (uint8_t)reg;
    servo->m_tx_buffer[1] = 1;

    sendPacketAndWaitResponse(servo->m_bus, servo->m_response, servo->m_id, HerkulexCommand_RamRead, servo->m_tx_buffer, 2);

    return servo->m_response->data[2];
}

/**
 * @brief Read a specific two bytes value from the Servo RAM register
 * @param servo : Pointer to the servo to read from
 * @param reg : register to read
 * @retval Value of the register
 */
uint16_t readRam2(HerkulexServo *servo, HerkulexRamRegister reg);

/**
 * @brief Read a specific one byte value from the Servo EEP register
 * @param servo : Pointer to the servo to read from
 * @param reg : register to read
 * @retval Value of the register
 */
uint8_t readEep(HerkulexServo *servo, HerkulexEepRegister reg);

/**
 * @brief Read a specific two bytes value from the Servo RAM register
 * @param servo : Pointer to the servo to read from
 * @param reg : register to read
 * @retval Value of the register
 */
uint8_t readEep2(HerkulexServo *servo, HerkulexEepRegister reg);

/**
 * @brief enable the position control mode of the servo
 * @param servo : Pointer to the Servo to control
 * @retval None
 */
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

    sendPacket(servo->m_bus, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, servo->m_tx_buffer, 5);
}

/**
 * @brief enable the speed control mode for the Servo
 * @param servo : Pointer to the Servo to control
 * @retval None
 */
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
    sendPacket(servo->m_bus, HERKULEX_BROADCAST_ID, HerkulexCommand_IJog, servo->m_tx_buffer, 5);
}

/**
 * @brief Reboot the servo
 * @param servo : Pointer to the Servo to control
 * @retval None
 */
void servoReboot(HerkulexServo *servo)
{
    sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_Reboot, NULL, 0);
}

/**
 * @brief Reset the Servo EEP values to the factory default, option is given to skip changing the ID and baud rate of the Servo
 * @param servo : Pointer to the Servo to reset
 * @param skipID : 0 for no, 1 for yes
 * @param skipBaud : 0 for no, 1 for yes
 * @retval None
 */
void rollBackToFactoryDefault(HerkulexServo *servo, uint8_t skipID, uint8_t skipBaud)
{
    *(servo->m_tx_buffer + 0) = skipID ? 1 : 0;
    *(servo->m_tx_buffer + 1) = skipBaud ? 1 : 0;

    sendPacket(servo->m_bus, servo->m_id, HerkulexCommand_Rollback, servo->m_tx_buffer, 2);
}
