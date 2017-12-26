/*
 * ServoBus/ServoBus.h
 * Copyright (C) 2017 Shaun Landis
 * <slandis@github>
 *
 */

#include <Arduino.h>
#include <stdint.h>

#define SERVO_FRAME_HEADER         0x55
#define SERVO_MOVE_TIME_WRITE      1
#define SERVO_MOVE_TIME_READ       2
#define SERVO_MOVE_TIME_WAIT_WRITE 7
#define SERVO_MOVE_TIME_WAIT_READ  8
#define SERVO_MOVE_START           11
#define SERVO_MOVE_STOP            12
#define SERVO_ID_WRITE             13
#define SERVO_ID_READ              14
#define SERVO_ANGLE_OFFSET_ADJUST  17
#define SERVO_ANGLE_OFFSET_WRITE   18
#define SERVO_ANGLE_OFFSET_READ    19
#define SERVO_ANGLE_LIMIT_WRITE    20
#define SERVO_ANGLE_LIMIT_READ     21
#define SERVO_VIN_LIMIT_WRITE      22
#define SERVO_VIN_LIMIT_READ       23
#define SERVO_TEMP_MAX_LIMIT_WRITE 24
#define SERVO_TEMP_MAX_LIMIT_READ  25
#define SERVO_TEMP_READ            26
#define SERVO_VIN_READ             27
#define SERVO_POS_READ             28
#define SERVO_OR_MOTOR_MODE_WRITE  29
#define SERVO_OR_MOTOR_MODE_READ   30
#define SERVO_LOAD_OR_UNLOAD_WRITE 31
#define SERVO_LOAD_OR_UNLOAD_READ  32
#define SERVO_LED_CTRL_WRITE       33
#define SERVO_LED_CTRL_READ        34
#define SERVO_LED_ERROR_WRITE      35
#define SERVO_LED_ERROR_READ       36

#define SERVO_MAX_MSG              10

#define SERVO_ERROR_NONE            0
#define SERVO_ERROR_TEMP            1
#define SERVO_ERROR_VOLT            2
#define SERVO_ERROR_TEMPVOLT        3
#define SERVO_ERROR_STALL           4
#define SERVO_ERROR_TEMPSTALL       5
#define SERVO_ERROR_VOLTSTALL       6
#define SERVO_ERROR_TEMPSTALLVOLT   7

#define REPLY_MOVETIME              0
#define REPLY_ID                    1
#define REPLY_ANGLEOFFSET           2
#define REPLY_ANGLELIMIT            3
#define REPLY_VINLIMIT              4
#define REPLY_TEMPMAXLIMIT          5
#define REPLY_TEMP                  6
#define REPLY_VIN                   7
#define REPLY_POSITION              8
#define REPLY_MODE                  9
#define REPLY_LOADUNLOAD           10
#define REPLY_LEDCONTROL           11
#define REPLY_LEDERROR             12

#define TX_DELAY                  400
#define RX_DELAY                   10

#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

class ServoBus {

public:
  ServoBus(Stream *stream, int writePin);
  void MoveTime(uint8_t id, int16_t position, uint16_t span);
  void MoveTimeWait(uint8_t id, int16_t position, uint16_t span);
  void MoveStart(uint8_t id);
  void MoveStop(uint8_t id);
  void SetID(uint8_t from, uint8_t to);
  void AdjustAngleOffset(uint8_t id, uint8_t offset);
  void SetAngleOffset(uint8_t id);
  void SetAngleLimit(uint8_t id, int16_t min, int16_t max);
  void SetVinLimit(uint8_t id, int16_t min, int16_t max);
  void SetTempMaxLimit(uint8_t id, uint8_t max);
  void SetMode(uint8_t id, uint8_t mode, int16_t speed);
  void SetLoad(uint8_t id);
  void SetUnload(uint8_t id);
  void SetLEDControl(uint8_t id, bool enable);
  void SetLEDError(uint8_t id, uint8_t mode);

  void requestMoveTime(uint8_t id);
  void requestID(uint8_t id);
  void requestAngleOffset(uint8_t id);
  void requestAngleLimit(uint8_t id);
  void requestVinLimit(uint8_t id);
  void requestTempMaxLimit(uint8_t id);
  void requestTemp(uint8_t id);
  void requestVin(uint8_t id);
  void requestPosition(uint8_t id);
  void requestMode(uint8_t id);
  void requestLoadUnload(uint8_t id);
  void requestLEDControl(uint8_t id);
  void requestLEDError(uint8_t id);

  typedef void ServoEvent(uint8_t id, uint8_t command, uint16_t param1, uint16_t param2);
  void setEventHandler(int EventType, ServoEvent servoEvent);

private:
  Stream *_serial;
  int _writePin;
  byte input_buffer[SERVO_MAX_MSG];

  uint8_t _checksum(byte buffer[]);
  void _write(uint8_t buffer[], int len);
  int _read();
  void _clear();

  ServoEvent *servoEvents[13];
};
