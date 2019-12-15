#include <UNITSUSerial.h>
#include <UNITSUSoftwareSerial.h>

#define DETECT_BAUDATE_TIMEOUT     250


String UNITSUSerial::getLogString() const {
  String log;
  log.reserve(48);
  log = "UNITSU serial: ";
  if (isSWserial()) {
    log += "SW";
  } else {
    log += "HW";
  }
  log += ": rx:";
  log += String(_receivePin);
  log += " tx:";
  log += String(_transmitPin);
  log += " baud:";
  log += String(_baud);
  return log;
}

// ****************************************
// ESP8266 implementation wrapper
// ****************************************
#ifdef ESP8266
bool UNITSUSerial::_serial0_swap_active = false;
#endif

#if !defined(DISABLE_SOFTWARE_SERIAL) && defined(ESP8266)

UNITSUSerial::UNITSUSerial(int receivePin, int transmitPin, bool inverse_logic, unsigned int buffSize)
    : _swserial(nullptr), _receivePin(receivePin), _transmitPin(transmitPin)
{
  _serialtype = UNITSUSerialType::getSerialType(receivePin, transmitPin);
  if (isSWserial()) {
    _swserial = new UNITSUSoftwareSerial(receivePin, transmitPin, inverse_logic, buffSize);
  } else if (isValid()) {
    getHW()->pins(transmitPin, receivePin);
  }
}

UNITSUSerial::~UNITSUSerial() {
  end();
  if (_swserial != nullptr) {
    delete _swserial;
  }
}

void UNITSUSerial::begin(unsigned long baud, SerialConfig config, SerialMode mode) {
  _baud = baud;
  if (isSWserial()) {
    if (_swserial != nullptr) {
      _swserial->begin(baud);
    }
  } else {
    doHWbegin(baud, config, mode);
  }
}

void UNITSUSerial::end() {
  if (!isValid()) {
    return;
  }
  flush();
  if (isSWserial()) {
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
    _swserial->end();
#endif
    return;
  } else {
  //  getHW()->end();
  }
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
    case UNITSUSerialType::serialtype::serial0:
    case UNITSUSerialType::serialtype::serial0_swap:
    case UNITSUSerialType::serialtype::serial1:      return true; // Must also check RX pin?
    case UNITSUSerialType::serialtype::software:     return _swserial != nullptr;
    default: break;
  }
  return false;
}



int UNITSUSerial::peek(void) {
  if (!isValid()) {
    return -1;
  }
  if (isSWserial()) {
    return _swserial->peek();
  } else {
    return getHW()->peek();
  }
}

size_t UNITSUSerial::write(uint8_t byte) {
  if (!isValid()) {
    return 0;
  }
  if (isSWserial()) {
    return _swserial->write(byte);
  } else {
    return getHW()->write(byte);
  }
}

size_t UNITSUSerial::write(const uint8_t *buffer, size_t size) {
  if (!isValid() || !buffer) {
    return 0;
  }
  if (isSWserial()) {
    // Not implemented in SoftwareSerial
    size_t count = 0;
    for (size_t i = 0; i < size; ++i) {
      size_t written = _swserial->write(buffer[i]);
      if (written == 0) return count;
      count += written;
    }
    return count;
  } else {
    return getHW()->write(buffer, size);
  }
}

size_t UNITSUSerial::write(const char *buffer) {
  if (!buffer) return 0;
  return write(buffer, strlen(buffer));
}

int UNITSUSerial::read(void) {
  if (!isValid()) {
    return -1;
  }
  if (isSWserial()) {
    return _swserial->read();
  } else {
    return getHW()->read();
  }
}

size_t UNITSUSerial::readBytes(char* buffer, size_t size)  {
  if (!isValid() || !buffer) {
    return 0;
  }
  if (isSWserial()) {
    // Not implemented in SoftwareSerial
    size_t count = 0;
    for (size_t i = 0; i < size; ++i) {
      int read = _swserial->read();
      if (read < 0) return count;
      buffer[i] = static_cast<char>(read & 0xFF);
      ++count;
    }
    return count;
  } else {
    return getHW()->readBytes(buffer, size);
  }
}

size_t UNITSUSerial::readBytes(uint8_t* buffer, size_t size)  {
  return readBytes((char*)buffer, size);
}

int UNITSUSerial::available(void) {
  if (!isValid()) {
    return 0;
  }
  if (isSWserial()) {
    return _swserial->available();
  } else {
    return getHW()->available();
  }
}

void UNITSUSerial::flush(void) {
  if (!isValid()) {
    return;
  }
  if (isSWserial()) {
    _swserial->flush();
  } else {
    getHW()->flush();
  }
}


bool UNITSUSerial::overflow() { return hasOverrun(); }
bool UNITSUSerial::hasOverrun(void) {
  if (!isValid()) {
    return false;
  }
#ifdef CORE_PRE_2_4_2
  return false;
#else
  if (isSWserial()) {
    return false;
    //return _swserial->overflow();
  } else {
    return getHW()->hasOverrun();
  }
#endif
}



// *****************************
// HardwareSerial specific
// *****************************

void UNITSUSerial::swap(uint8_t tx_pin) {
  if (getHW() == nullptr) return;
  getHW()->swap(tx_pin);
}

int UNITSUSerial::baudRate(void) {
  if (!isValid() || isSWserial()) {
    return _baud;
  }
  return getHW()->baudRate();
}

void UNITSUSerial::setDebugOutput(bool enable) {
  if (!isValid() || isSWserial()) {
    return;
  }
  getHW()->setDebugOutput(enable);
}

bool UNITSUSerial::isTxEnabled(void) {
  if (!isValid() || isSWserial()) {
    return false;
  }
  return getHW()->isTxEnabled();
}

 bool UNITSUSerial::isRxEnabled(void) {
   if (!isValid() || isSWserial()) {
     return false;
   }
   return getHW()->isRxEnabled();
 }

bool UNITSUSerial::hasRxError(void) {
#ifdef CORE_POST_2_5_0
  if (!isValid() || isSWserial()) {
    return false;
  }
  return getHW()->hasRxError();
#else
  return false;
#endif
}

void UNITSUSerial::startDetectBaudrate() {
  if (!isValid() || isSWserial()) {
    return;
  }
#ifdef CORE_PRE_2_4_2
  return;
#else
  getHW()->startDetectBaudrate();
#endif
}

unsigned long UNITSUSerial::testBaudrate() {
  if (!isValid() || isSWserial()) {
    return 0;
  }
#ifdef CORE_PRE_2_4_2
  return 0;
#else
  return getHW()->testBaudrate();
#endif
}

unsigned long UNITSUSerial::detectBaudrate(time_t timeoutMillis) {
  if (!isValid() || isSWserial()) {
    return 0;
  }
#ifdef CORE_PRE_2_4_2
  return 0;
#else
  return getHW()->detectBaudrate(timeoutMillis);
#endif
}


// *****************************
// SoftwareSerial specific
// *****************************


bool UNITSUSerial::listen() {
  if (isValid() && isSWserial()) {
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
    return _swserial->listen();
#endif
  }
  return false;
}

bool UNITSUSerial::isListening() {
  if (isValid() && isSWserial()) {
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
    return _swserial->isListening();
#endif
  }
  return false;
}

bool UNITSUSerial::stopListening() {
  if (isValid() && isSWserial()) {
#ifndef ARDUINO_ESP8266_RELEASE_2_3_0
    return _swserial->stopListening();
#endif
  }
  return false;
}
#endif // ESP8266


#ifdef ESP8266
  // ****************************************
  // ESP8266 implementation wrapper
  // Shared functions for HW serial
  // ****************************************
  bool UNITSUSerial::doHWbegin(unsigned long baud, SerialConfig config, SerialMode mode) {
    if (getHW() == nullptr) return false;
    getHW()->begin(baud > 0 ? baud : 9600, config, mode, _transmitPin);
    getHW()->pins(_transmitPin, _receivePin);
    return isValid();
/*  // Detect baudrate gives crashes, so disabled for now.
    startDetectBaudrate();
    unsigned long detectedBaudRate = detectBaudrate(DETECT_BAUDATE_TIMEOUT);
    if(detectedBaudRate > 0) {
        delay(100); // Give some time...
        getHW()->begin(detectedBaudRate, config, mode, _transmitPin);
        _baud = detectedBaudRate;
        return true;
    } else {
      return false;
    }
*/
  }
#endif
