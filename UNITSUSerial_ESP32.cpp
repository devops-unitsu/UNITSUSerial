#include <UNITSUSerial.h>
// ****************************************
// ESP32 implementation wrapper
// Only support HW serial on Serial 0 .. 2
// ****************************************

#ifdef ESP32
UNITSUSerial::UNITSUSerial(int receivePin, int transmitPin, bool inverse_logic, int serialPort)
    : _receivePin(receivePin), _transmitPin(transmitPin), _inverse_logic(inverse_logic)
{
  switch (serialPort) {
    case 0: _serialtype = UNITSUSerialType::serialtype::serial0;
    case 1: _serialtype = UNITSUSerialType::serialtype::serial1;
    case 2: _serialtype = UNITSUSerialType::serialtype::serial2;
    default:
      _serialtype = UNITSUSerialType::getSerialType(receivePin, transmitPin);
  }
}

UNITSUSerial::~UNITSUSerial() {
  end();
}

void UNITSUSerial::begin(unsigned long baud, uint32_t config
         , int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms) {
  _baud = baud;
  if (rxPin != -1) _receivePin = rxPin;
  if (txPin != -1) _transmitPin = txPin;
  if (invert) _inverse_logic = true;
  if (!isValid()) {
    _baud = 0;
    return;
  }
  // Make sure the extra bit is set for the config. The config differs between ESP32 and ESP82xx
  config = config | 0x8000000;

  if (isValid()) {
    // Timeout added for 1.0.1
    // See: https://github.com/espressif/arduino-esp32/commit/233d31bed22211e8c85f82bcf2492977604bbc78
    //getHW()->begin(baud, config, _receivePin, _transmitPin, invert, timeout_ms);
    getHW()->begin(baud, config, _receivePin, _transmitPin, _inverse_logic);
  }
}

void UNITSUSerial::end() {
  if (!isValid()) {
    return;
  }
  getHW()->end();
}

HardwareSerial* UNITSUSerial::getHW() {
  switch (_serialtype) {
    case UNITSUSerialType::serialtype::serial0: return &Serial;
    case UNITSUSerialType::serialtype::serial1: return &Serial1;
    case UNITSUSerialType::serialtype::serial2: return &Serial2;

    default: break;
  }
  return nullptr;
}

const HardwareSerial* UNITSUSerial::getHW() const {
  switch (_serialtype) {
    case UNITSUSerialType::serialtype::serial0: return &Serial;
    case UNITSUSerialType::serialtype::serial1: return &Serial1;
    case UNITSUSerialType::serialtype::serial2: return &Serial2;
    default: break;
  }
  return nullptr;
}

bool UNITSUSerial::isValid() const {
  switch (_serialtype) {
    case UNITSUSerialType::serialtype::serial0:
    case UNITSUSerialType::serialtype::serial2:
      return true;
    case UNITSUSerialType::serialtype::serial1:
      return _transmitPin != -1 && _receivePin != -1;
      // FIXME TD-er: Must perform proper check for GPIO pins here.
    default: break;
  }
  return false;
}




int UNITSUSerial::peek(void) {
  if (!isValid()) {
    return -1;
  }
  return getHW()->peek();
}

size_t UNITSUSerial::write(uint8_t byte) {
  if (!isValid()) {
    return 0;
  }
  return getHW()->write(byte);
}

size_t UNITSUSerial::write(const uint8_t *buffer, size_t size) {
  if (!isValid() || !buffer) {
    return 0;
  }
  return getHW()->write(buffer, size);
}

size_t UNITSUSerial::write(const char *buffer) {
  if (!buffer) return 0;
  return write(buffer, strlen(buffer));
}

int UNITSUSerial::read(void) {
  if (!isValid()) {
    return -1;
  }
  return getHW()->read();
}

int UNITSUSerial::available(void) {
  if (!isValid()) {
    return 0;
  }
  return getHW()->available();
}

void UNITSUSerial::flush(void) {
  if (!isValid()) {
    return;
  }
  getHW()->flush();
}

int UNITSUSerial::baudRate(void) {
  if (!isValid()) {
    return 0;
  }
  return getHW()->baudRate();
}


// Not supported in ESP32, since only HW serial is used.
// Function included since it is used in some libraries.
bool UNITSUSerial::listen() {
  return true;
}


#endif // ESP32
