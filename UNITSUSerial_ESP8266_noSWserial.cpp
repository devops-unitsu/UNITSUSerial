#include <UNITSUSerial.h>

#if defined(DISABLE_SOFTWARE_SERIAL) && defined(ESP8266)
// ****************************************
// ESP8266 implementation wrapper
// No SoftwareSerial
// Only support HW serial on Serial 0 .. 1
// ****************************************
UNITSUSerial::UNITSUSerial(int receivePin, int transmitPin, bool inverse_logic, unsigned int buffSize)
    : _receivePin(receivePin), _transmitPin(transmitPin)
{
  _serialtype = UNITSUSerialType::getSerialType(receivePin, transmitPin);
  if (isValid()) {
    getHW()->pins(transmitPin, receivePin);
  }
}

UNITSUSerial::~UNITSUSerial() {
  end();
}

void UNITSUSerial::begin(unsigned long baud, SerialConfig config, SerialMode mode) {
  _baud = baud;
  if (_serialtype == UNITSUSerialType::serialtype::serial0_swap) {
    // Serial.swap() should only be called here and only once.
    if (!_serial0_swap_active) {
      Serial.begin(baud, config, mode, _transmitPin);
      Serial.swap();
      _serial0_swap_active = true;
      return;
    }
  }
  if (!isValid()) {
    _baud = 0;
    return;
  }
  doHWbegin(baud, config, mode);
}

void UNITSUSerial::end() {
  if (!isValid()) {
    return;
  }
  if (_serialtype == UNITSUSerialType::serialtype::serial0_swap) {
    if (_serial0_swap_active) {
      Serial.end();
      Serial.swap();
      _serial0_swap_active = false;
      return;
    }
  }
  getHW()->end();
}


HardwareSerial* UNITSUSerial::getHW() {
  switch (_serialtype) {
    case UNITSUSerialType::serialtype::serial0:
    case UNITSUSerialType::serialtype::serial0_swap: return &Serial;
    case UNITSUSerialType::serialtype::serial1:      return &Serial1;
    case UNITSUSerialType::serialtype::software:     break;
    default: break;
  }
  return nullptr;
}

const HardwareSerial* UNITSUSerial::getHW() const {
  switch (_serialtype) {
    case UNITSUSerialType::serialtype::serial0:
    case UNITSUSerialType::serialtype::serial0_swap: return &Serial;
    case UNITSUSerialType::serialtype::serial1:      return &Serial1;
    case UNITSUSerialType::serialtype::software:     break;
    default: break;
  }
  return nullptr;
}

bool UNITSUSerial::isValid() const {
  switch (_serialtype) {
    case UNITSUSerialType::serialtype::serial0:      return !_serial0_swap_active;
    case UNITSUSerialType::serialtype::serial0_swap: return _serial0_swap_active;
    case UNITSUSerialType::serialtype::serial1:      return true; // Must also check RX pin?
    case UNITSUSerialType::serialtype::software:     return false;
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

size_t UNITSUSerial::readBytes(char* buffer, size_t size)  {
  if (!isValid() || !buffer) {
    return 0;
  }
  return getHW()->readBytes(buffer, size);
}

size_t UNITSUSerial::readBytes(uint8_t* buffer, size_t size)  {
  return readBytes((char*)buffer, size);
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


bool UNITSUSerial::overflow() { return hasOverrun(); }
bool UNITSUSerial::hasOverrun(void) {
  return false;
}



// *****************************
// HardwareSerial specific
// *****************************

void UNITSUSerial::swap(uint8_t tx_pin) {
  if (isValid()) {
    switch (_serialtype) {
      case UNITSUSerialType::serialtype::serial0:
      case UNITSUSerialType::serialtype::serial0_swap:
        // isValid() also checks for correct swap active state.
        _serial0_swap_active = !_serial0_swap_active;
        getHW()->swap(tx_pin);
        if (_serialtype == UNITSUSerialType::serialtype::serial0) {
          _serialtype = UNITSUSerialType::serialtype::serial0_swap;
        } else {
          _serialtype = UNITSUSerialType::serialtype::serial0;
        }
        break;
      default:
        return;
    }
  }
}

int UNITSUSerial::baudRate(void) {
  if (!isValid()) {
    return _baud;
  }
  return getHW()->baudRate();
}

void UNITSUSerial::setDebugOutput(bool enable) {
  if (!isValid()) {
    return;
  }
  getHW()->setDebugOutput(enable);
}

bool UNITSUSerial::isTxEnabled(void) {
  if (!isValid()) {
    return false;
  }
  return getHW()->isTxEnabled();
}

 bool UNITSUSerial::isRxEnabled(void) {
   if (!isValid()) {
     return false;
   }
   return getHW()->isRxEnabled();
 }

bool UNITSUSerial::hasRxError(void) {
#ifdef CORE_POST_2_5_0
  if (!isValid()) {
    return false;
  }
  return getHW()->hasRxError();
#else
  return false;
#endif
}

void UNITSUSerial::startDetectBaudrate() {
  if (!isValid()) {
    return;
  }
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
  getHW()->startDetectBaudrate();
#endif
}

unsigned long UNITSUSerial::testBaudrate() {
  if (!isValid()) {
    return 0;
  }
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
  return getHW()->testBaudrate();
#else
  return 0;
#endif
}

unsigned long UNITSUSerial::detectBaudrate(time_t timeoutMillis) {
  if (!isValid()) {
    return 0;
  }
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
  return getHW()->detectBaudrate(timeoutMillis);
#else
  return 0;
#endif
}


// *****************************
// SoftwareSerial specific
// *****************************


bool UNITSUSerial::listen() {
  if (isValid()) {
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
    return _swserial->listen();
#endif
  }
  return false;
}

#endif // DISABLE_SOFTWARE_SERIAL
