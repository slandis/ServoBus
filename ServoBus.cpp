/*
 * ServoBus/ServoBus.cpp
 * Copyright (C) 2017 Shaun Landis
 * <slandis@github>
 *
 */

#include "ServoBus.h"

ServoBus::ServoBus(Stream *serial, int writePin) {
  _serial = serial;
  _writePin = writePin;
}

uint8_t ServoBus::_checksum(uint8_t buffer[]) {
  uint8_t i;
  uint16_t temp = 0;

  for (i = 2; i < buffer[3] + 2; i++) {
    temp += buffer[i];
  }

  temp = ~temp;
  i = (uint8_t)temp;

  return i;
}

void ServoBus::_write(uint8_t buffer[], int len) {
  digitalWrite(_writePin, HIGH);

  _serial->write(buffer, len);
  _serial->flush();

  delayMicroseconds(TX_DELAY);
}

int ServoBus::_read() {
  uint8_t frame = 0;
  int len = 0;

  digitalWrite(_writePin, LOW);
  memset(input_buffer, 0, 10);

  delay(RX_DELAY);

  if (_serial->available() > 0) {
    if (_serial->peek() == 0x55 && input_buffer[0] != 0x55)
      input_buffer[len++] = _serial->read();

    if (_serial->peek() == 0x55 && input_buffer[0] == 0x55) {
      input_buffer[len++] = _serial->read();
      input_buffer[len++] = _serial->read();
      input_buffer[len++] = _serial->read();

      while (_serial->available()  > 0 && len < SERVO_MAX_MSG) {
        input_buffer[len++] = _serial->read();
      }

      input_buffer[len] = _serial->read();
    }
  }

  digitalWrite(_writePin, HIGH);

  return len;
}

void ServoBus::_clear() {
  digitalWrite(_writePin, LOW);

  while (_serial->read() != -1);

  digitalWrite(_writePin, HIGH);
}

/*
 * Move the servo to the given position (0-1000, 240 deg total) over the
 * specified time (0-30000ms)
 */
void ServoBus::MoveTime(uint8_t id, int16_t position, uint16_t span) {
  uint8_t buf[10];

  if (position < 0)
    position = 0;

  if (position > 1000)
    position = 1000;

  if (span < 0)
    span = 0;

  if (span > 30000)
    span = 30000;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_MOVE_TIME_WRITE;
  buf[5] = (uint8_t)position;
  buf[6] = (uint8_t)(position >> 8);
  buf[7] = (uint8_t)span;
  buf[8] = (uint8_t)(span >> 8);
  buf[9] = _checksum(buf);

  _clear();
  _write(buf, 10);
}

/*
 * Queue a servo move to the given position over the specified time (move
 * completed when SERVO_MOVE_TIME_WAIT_WRITE is sent)
 */
void ServoBus::MoveTimeWait(uint8_t id, int16_t position, uint16_t span) {
  uint8_t buf[10];

  if (position < 0)
    position = 0;

  if (position > 1000)
    position = 1000;

  if (span < 0)
    span = 0;

  if (span > 30000)
    span = 30000;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_MOVE_TIME_WAIT_WRITE;
  buf[5] = (uint8_t)position;
  buf[6] = (uint8_t)(position >> 8);
  buf[7] = (uint8_t)span;
  buf[8] = (uint8_t)(span >> 8);
  buf[9] = _checksum(buf);

  _clear();
  _write(buf, 10);
}

/*
 * Begin a previously queued move
 */
void ServoBus::MoveStart(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_MOVE_START;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);
}

/*
 * Stops the servo immediately if currently moving
 */
void ServoBus::MoveStop(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_MOVE_STOP;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);
}

/*
 * Sets the ID of the servo (0-253). Servo ID 254 is the broadcast address and
 * cannot be assigned
 */
void ServoBus::SetID(uint8_t fromID, uint8_t toID) {
  uint8_t buf[7];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = fromID;
  buf[3] = 4;
  buf[4] = SERVO_ID_WRITE;
  buf[5] = toID;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::AdjustAngleOffset(uint8_t id, uint8_t deviation) {
  uint8_t buf[7];

  if (deviation > 125)
    deviation = 125;

  if (deviation < -125)
    deviation = -125;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_ANGLE_OFFSET_ADJUST;
  buf[5] = deviation;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::SetAngleOffset(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_ANGLE_OFFSET_WRITE;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);
}

void ServoBus::SetAngleLimit(uint8_t id, int16_t min, int16_t max) {
  uint8_t buf[10];

  if (min > 1000)
    min = 1000;

  if (min < 0)
    min = 0;

  if (max > 1000)
    max = 1000;

  if (max < 0)
    max = 0;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_ANGLE_LIMIT_WRITE;
  buf[5] = (uint8_t)min;
  buf[6] = (uint8_t)(min >> 8);
  buf[7] = (uint8_t)max;
  buf[8] = (uint8_t)(max >> 8);
  buf[9] = _checksum(buf);

  _clear();
  _write(buf, 10);
}

void ServoBus::SetVinLimit(uint8_t id, int16_t min, int16_t max) {
  uint8_t buf[10];

  if (min < 4500)
    min = 4500;

  if (min > 12000)
    min = 12000;

  if (max < 4500)
    max = 4500;

  if (max > 12000)
    max = 12000;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_VIN_LIMIT_WRITE;
  buf[5] = (uint8_t)min;
  buf[6] = (uint8_t)(min >> 8);
  buf[7] = (uint8_t)max;
  buf[8] = (uint8_t)(max >> 8);
  buf[9] = _checksum(buf);

  _clear();
  _write(buf, 10);
}

void ServoBus::SetTempMaxLimit(uint8_t id, uint8_t max) {
  uint8_t buf[7];

  if (max < 50)
    max = 50;

  if (max > 100)
    max = 100;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_TEMP_MAX_LIMIT_WRITE;
  buf[5] = max;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::SetMode(uint8_t id, uint8_t mode, int16_t speed) {
  uint8_t buf[10];

  if (speed < -1000)
    speed = -1000;

  if (speed > 1000)
    speed = 1000;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = mode;
  buf[6] = 0;
  buf[7] = (uint8_t)speed;
  buf[8] = (uint8_t)(speed >> 8);
  buf[9] = _checksum(buf);

  _clear();
  _write(buf, 10);
}

void ServoBus::SetLoad(uint8_t id) {
  uint8_t buf[7];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::SetUnload(uint8_t id) {
  uint8_t buf[7];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::SetLEDControl(uint8_t id, bool enable) {
  uint8_t buf[7];
  int i = 0;

  if (enable)
    i = 1;

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_LED_CTRL_WRITE;
  buf[5] = i;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::SetLEDError(uint8_t id, uint8_t mode) {
  uint8_t buf[7];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_LED_ERROR_WRITE;
  buf[5] = mode;
  buf[6] = _checksum(buf);

  _clear();
  _write(buf, 7);
}

void ServoBus::requestMoveTime(byte id) {
  byte buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_MOVE_TIME_WAIT_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_MOVETIME]) {
    if (_read() == 10) {
      if (_checksum(input_buffer) == input_buffer[9]) {
        servoEvents[REPLY_MOVETIME](input_buffer[2], REPLY_MOVETIME, (uint16_t)BYTE_TO_HW(input_buffer[6], input_buffer[5]), (uint16_t)BYTE_TO_HW(input_buffer[8], input_buffer[7]));
      }
    }
  }
}

void ServoBus::requestID(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_ID_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_ID]) {
    if (_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_MOVETIME](input_buffer[2], REPLY_MOVETIME, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::requestAngleOffset(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_ANGLE_OFFSET_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_ANGLEOFFSET]) {
    if (_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_ANGLEOFFSET](input_buffer[2], REPLY_ANGLEOFFSET, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::requestAngleLimit(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_ANGLE_LIMIT_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_ANGLELIMIT]) {
    if (_read() == 10) {
      if (_checksum(input_buffer) == input_buffer[9]) {
        servoEvents[REPLY_ANGLELIMIT](input_buffer[2], REPLY_ANGLELIMIT, (uint16_t)BYTE_TO_HW(input_buffer[6], input_buffer[5]), (uint16_t)BYTE_TO_HW(input_buffer[8], input_buffer[7]));
      }
    }
  }
}

void ServoBus::requestVinLimit(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_VIN_LIMIT_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_VINLIMIT]) {
    if(_read() == 8) {
      if (_checksum(input_buffer) == input_buffer[7]) {
        servoEvents[REPLY_VINLIMIT](input_buffer[2], REPLY_VINLIMIT, (uint16_t)BYTE_TO_HW(input_buffer[6], input_buffer[5]), 0);
      }
    }
  }
}

void ServoBus::requestTempMaxLimit(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_TEMP_MAX_LIMIT_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if(_read() == 5)
    return input_buffer[3];

  return -1;
}

void ServoBus::requestTemp(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_TEMP_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_TEMP]) {
    if(_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_TEMP](input_buffer[2], REPLY_TEMP, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::requestVin(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_VIN_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_VIN]) {
    if(_read() == 8) {
      if (_checksum(input_buffer) == input_buffer[7]) {
        servoEvents[REPLY_VIN](input_buffer[2], REPLY_VIN, (uint16_t)BYTE_TO_HW(input_buffer[6], input_buffer[5]), 0);
      }
    }
  }
}

void ServoBus::requestPosition(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_POS_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_POSITION]) {
    if (_read() == 8) {
      if (_checksum(input_buffer) == input_buffer[7]) {
        servoEvents[REPLY_POSITION](input_buffer[2], REPLY_POSITION, (uint16_t)BYTE_TO_HW(input_buffer[6], input_buffer[5]), 0);
      }
    }
  }
}

void ServoBus::requestMode(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_OR_MOTOR_MODE_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_MODE]) {
    if(_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_MODE](input_buffer[2], REPLY_MODE, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::requestLoadUnload(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_LOAD_OR_UNLOAD_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_LOADUNLOAD]) {
    if (_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_LOADUNLOAD](input_buffer[2], REPLY_LOADUNLOAD, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::requestLEDControl(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_LED_CTRL_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_LEDCONTROL]) {
    if (_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_LEDCONTROL](input_buffer[2], REPLY_LEDCONTROL, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::requestLEDError(uint8_t id) {
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_LED_ERROR_READ;
  buf[5] = _checksum(buf);

  _clear();
  _write(buf, 6);

  if (servoEvents[REPLY_LEDERROR]) {
    if (_read() == 7) {
      if (_checksum(input_buffer) == input_buffer[6]) {
        servoEvents[REPLY_LEDERROR](input_buffer[2], REPLY_LEDERROR, (uint16_t)input_buffer[5], 0);
      }
    }
  }
}

void ServoBus::setEventHandler(int EventType, ServoEvent servoEvent) {
  servoEvents[EventType] = servoEvent;
}
