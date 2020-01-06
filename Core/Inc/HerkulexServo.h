/*
 * HerkulexServo.h
 *
 *  Created on: Dec 6, 2019
 *      Author: Antoine
 */

#ifndef INC_HERKULEXSERVO_H_
#define INC_HERKULEXSERVO_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "usart.h"

#define HERKULEX_BROADCAST_ID 0xFE

#define SERVO_TX_BUFFER_SIZE sizeof(uint8_t) * 5

#ifndef HERKULEX_PACKET_RX_TIMEOUT
#define HERKULEX_PACKET_RX_TIMEOUT 100 // microseconds
#endif

#ifndef HERKULEX_PACKET_RETRIES
#define HERKULEX_PACKET_RETRIES 6
#endif

#ifndef HERKULEX_PACKET_RESEND_DELAY
#define HERKULEX_PACKET_RESEND_DELAY 70 // microseconds
#endif

#ifndef HERKULEX_PACKET_RX_MAX_DATA
#define HERKULEX_PACKET_RX_MAX_DATA 10 // bytes
#endif

#ifndef HERKULEX_SERIAL_RX_BUFFER
#define HERKULEX_SERIAL_RX_BUFFER 30 // bytes
#endif

#ifndef HERKULEX_MAX_SCHEDULED_SERVOS
#define HERKULEX_MAX_SCHEDULED_SERVOS 10
#endif

#ifndef HERKULEX_SERIAL_TX_BUFFER
#define HERKULEX_SERIAL_TX_BUFFER (1 + HERKULEX_MAX_SCHEDULED_SERVOS * 5)
#endif

typedef enum
{
	HerkulexCommand_None = 0x00,
	HerkulexCommand_EepWrite = 0x01,
	HerkulexCommand_EepRead = 0x02,
	HerkulexCommand_RamWrite = 0x03,
	HerkulexCommand_RamRead = 0x04,
	HerkulexCommand_IJog = 0x05,
	HerkulexCommand_SJog = 0x06,
	HerkulexCommand_Stat = 0x07,
	HerkulexCommand_Rollback = 0x08,
	HerkulexCommand_Reboot = 0x09
} HerkulexCommand;

typedef enum
{
	HerkulexEepRegister_ModelNo1 = 0,
	HerkulexEepRegister_ModelNo2 = 1,
	HerkulexEepRegister_Version1 = 2,
	HerkulexEepRegister_Version2 = 3,
	HerkulexEepRegister_BaudRate = 4,
	HerkulexEepRegister_ID = 6,
	HerkulexEepRegister_AckPolicy = 7,
	HerkulexEepRegister_AlarmLedPolicy = 8,
	HerkulexEepRegister_TorquePolicy = 9,
	HerkulexEepRegister_MaxTemperature = 11,
	HerkulexEepRegister_MinVoltage = 12,
	HerkulexEepRegister_MaxVoltage = 13,
	HerkulexEepRegister_AccelerationRatio = 14,
	HerkulexEepRegister_MaxAccelerationTime = 15,
	HerkulexEepRegister_DeadZone = 16,
	HerkulexEepRegister_SaturatorOffset = 17,
	HerkulexEepRegister_SaturatorSlope = 18,
	HerkulexEepRegister_PwmOffset = 20,
	HerkulexEepRegister_MinPwm = 21,
	HerkulexEepRegister_MaxPwm = 22,
	HerkulexEepRegister_OverloadPwmThreshold = 24,
	HerkulexEepRegister_MinPosition = 26,
	HerkulexEepRegister_MaxPosition = 28,
	HerkulexEepRegister_PositionKp = 30,
	HerkulexEepRegister_PositionKd = 32,
	HerkulexEepRegister_PositionKi = 34,
	HerkulexEepRegister_PositionFF1stGain = 36,
	HerkulexEepRegister_PositionFF2ndGain = 38,
	HerkulexEepRegister_LedBlinkPeriod = 44,
	HerkulexEepRegister_AdcFaultCheckPeriod = 45,
	HerkulexEepRegister_PacketGarbageCheckPeriod = 46,
	HerkulexEepRegister_StopDetectionPeriod = 47,
	HerkulexEepRegister_OverloadProtectionPeriod = 48,
	HerkulexEepRegister_StopThreshold = 49,
	HerkulexEepRegister_InPositionMargin = 50,
	HerkulexEepRegister_CalibrationDifference = 53
} HerkulexEepRegister;

typedef enum
{
	HerkulexRamRegister_ID = 0,
	HerkulexRamRegister_AckPolicy = 1,
	HerkulexRamRegister_AlarmLedPolicy = 2,
	HerkulexRamRegister_TorquePolicy = 3,
	HerkulexRamRegister_MaxTemperature = 5,
	HerkulexRamRegister_MinVoltage = 6,
	HerkulexRamRegister_MaxVoltage = 7,
	HerkulexRamRegister_AccelerationRatio = 8,
	HerkulexRamRegister_MaxAccelerationTime = 9,
	HerkulexRamRegister_DeadZone = 10,
	HerkulexRamRegister_SaturatorOffset = 11,
	HerkulexRamRegister_SaturatorSlope = 12,
	HerkulexRamRegister_PwmOffset = 14,
	HerkulexRamRegister_MinPwm = 15,
	HerkulexRamRegister_MaxPwm = 16,
	HerkulexRamRegister_OverloadPwmThreshold = 18,
	HerkulexRamRegister_MinPosition = 20,
	HerkulexRamRegister_MaxPosition = 22,
	HerkulexRamRegister_PositionKp = 24,
	HerkulexRamRegister_PositionKd = 26,
	HerkulexRamRegister_PositionKi = 28,
	HerkulexRamRegister_PositionFF1stGain = 30,
	HerkulexRamRegister_PositionFF2ndGain = 32,
	HerkulexRamRegister_LedBlinkPeriod = 38,
	HerkulexRamRegister_AdcFaultCheckPeriod = 39,
	HerkulexRamRegister_PacketGarbageCheckPeriod = 40,
	HerkulexRamRegister_StopDetectionPeriod = 41,
	HerkulexRamRegister_OverloadProtectionPeriod = 42,
	HerkulexRamRegister_StopThreshold = 43,
	HerkulexRamRegister_InPositionMargin = 44,
	HerkulexRamRegister_CalibrationDifference = 47,
	HerkulexRamRegister_StatusError = 48,
	HerkulexRamRegister_StatusDetail = 49,
	HerkulexRamRegister_TorqueControl = 52,
	HerkulexRamRegister_LedControl = 53,
	HerkulexRamRegister_Voltage = 54,
	HerkulexRamRegister_Temperature = 55,
	HerkulexRamRegister_CurrentControlMode = 56,
	HerkulexRamRegister_Tick = 57,
	HerkulexRamRegister_CalibratedPosition = 58,
	HerkulexRamRegister_AbsolutePosition = 60,
	HerkulexRamRegister_DifferentialPosition = 62,
	HerkulexRamRegister_Pwm = 64,
	HerkulexRamRegister_AbsoluteGoalPosition = 68,
	HerkulexRamRegister_AbsoluteDesiredTrajPos = 70,
	HerkulexRamRegister_DesiredVelocity = 72
} HerkulexRamRegister;

typedef enum
{
	HerkulexLed_Off = 0x00,
	HerkulexLed_Red = 0x04,
	HerkulexLed_Green = 0x01,
	HerkulexLed_Blue = 0x02,
	HerkulexLed_Yellow = 0x05,
	HerkulexLed_Cyan = 0x03,
	HerkulexLed_Purple = 0x06,
	HerkulexLed_White = 0x07,
	HerkulexLed_Ignore = 0xFF
} HerkulexLed;

typedef enum
{
	HerkulexPacketError_None = 0,
	HerkulexPacketError_Timeout = 0b00000001,
	HerkulexPacketError_Length = 0b00000010,
	HerkulexPacketError_Command = 0b00000100,
	HerkulexPacketError_Checksum = 0b00001000
} HerkulexPacketError;

typedef enum
{
	HerkulexStatusError_None = 0,
	HerkulexStatusError_InputVoltage = 0b00000001,
	HerkulexStatusError_PotLimit = 0b00000010,
	HerkulexStatusError_TemperatureLimit = 0b00000100,
	HerkulexStatusError_InvalidPacket = 0b00001000,
	HerkulexStatusError_Overload = 0b00010000,
	HerkulexStatusError_DriverFault = 0b00100000,
	HerkulexStatusError_EEPDistorted = 0b01000000,
	HerkulexStatusError_Reserved = 0b10000000
} HerkulexStatusError;

typedef enum
{
	HerkulexStatusDetail_None = 0,
	HerkulexStatusDetail_Moving = 0b00000001,
	HerkulexStatusDetail_InPosition = 0b00000010,
	HerkulexStatusDetail_ChecksumError = 0b00000100,
	HerkulexStatusDetail_UnknownCommand = 0b00001000,
	HerkulexStatusDetail_ExceedRegRange = 0b00010000,
	HerkulexStatusDetail_GarbageDetected = 0b00100000,
	HerkulexStatusDetail_MotorOn = 0b01000000,
	HerkulexStatusDetail_Reserved = 0b10000000
} HerkulexStatusDetail;

typedef enum
{
	HerkulexScheduleState_None = 0,
	HerkulexScheduleState_IndividualMove,
	HerkulexScheduleState_SynchronizedMove
} HerkulexScheduleState;

typedef struct
{
	uint8_t size;
	uint8_t id;
	HerkulexCommand cmd;
	uint8_t checksum1;
	uint8_t checksum2;
	uint8_t data[HERKULEX_PACKET_RX_MAX_DATA];
	uint8_t status_error;
	uint8_t status_detail;
	HerkulexPacketError error;
} HerkulexPacket;

typedef struct _HerkulexServoBus
{
	UART_HandleTypeDef *m_serial;
	uint8_t m_rx_buffer[HERKULEX_PACKET_RX_MAX_DATA];
	uint64_t m_last_serial;

	HerkulexPacket m_rx_packet;
	uint8_t m_rx_packet_ready;

	uint8_t m_tx_buffer[HERKULEX_SERIAL_TX_BUFFER];
	uint8_t m_move_tags;
	HerkulexScheduleState m_schedule_state;
} HerkulexServoBus;

HerkulexServoBus *initializeServoBus(UART_HandleTypeDef *HUART_Handler);
void sendPacket(HerkulexServoBus *self, uint8_t id, HerkulexCommand cmd, uint8_t *pData, uint8_t dataLen);
void prepareIndividualMove(HerkulexServoBus *self);
void prepareSynchronizedMove(HerkulexServoBus *self, uint8_t playtime);
void executeMove(HerkulexServoBus *self);

typedef struct _HerkulexServo
{
	HerkulexServoBus *m_bus;
	uint8_t m_id;
	HerkulexLed m_led;
	uint8_t m_position_control_mode;

	HerkulexPacket *m_response;
	uint8_t *m_tx_buffer;
} HerkulexServo;

HerkulexServo *initializeServo(HerkulexServoBus *servoBus, uint8_t id);
void jog(HerkulexServo *servo, uint8_t jog_lsb, uint8_t jog_msb, uint8_t set, uint8_t playtime);
void setPosition(HerkulexServo *servo, uint16_t pos, uint8_t playtime, HerkulexLed led);
void setSpeed(HerkulexServo *servo, uint16_t speed, uint8_t playtime, HerkulexLed led);

void setTorqueOn(HerkulexServo *servo);
void setTorqueOff(HerkulexServo *servo);
void setBrake(HerkulexServo *servo);

void setLedColor(HerkulexServo *servo, HerkulexLed color);

void writeRam(HerkulexServo *servo, HerkulexRamRegister reg, uint8_t val);
void writeRam2(HerkulexServo *servo, HerkulexRamRegister reg, uint16_t val);
void writeEep(HerkulexServo *servo, HerkulexRamRegister reg, uint8_t val);
void writeEep2(HerkulexServo *servo, HerkulexRamRegister reg, uint16_t val);

void enablePositionControlMode(HerkulexServo *servo);
void enableSpeedControlMode(HerkulexServo *servo);

void servoReboot(HerkulexServo *servo);
void rollBackToFactoryDefault(HerkulexServo *servo, uint8_t skipID, uint8_t skipBaud);
#endif /* INC_HERKULEXSERVO_H_ */
