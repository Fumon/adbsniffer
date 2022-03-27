#include <Arduino.h>
#include <stdint.h>
#include "optional.hpp"

#define DBG 5

#define SIGPIN 8
#define u4 uint_fast8_t

#define GLOBALRESETMS 3
#define SYNCuS 70
#define BITCELLuS 100
#define ONELOWuS 35
#define ZEROLOWuS 65
#define STOPLOWuS 65
#define ATTENTIONuS 780
#define TLTHIGHTIMEuS 160
#define SRQLOWTIMEuS (300 - ZEROLOWuS)

#define READBITTIMEOUTuS 200
#define SRQTIMEOUTuS 500

#define log(a) Serial.println(a)

#define WAITLOW() do {} while (digitalRead(SIGPIN) && start < READBITTIMEOUTuS)
#define WAITHIGH() do {} while (!digitalRead(SIGPIN) && start < READBITTIMEOUTuS)

enum ADBReg: uint_fast8_t {
  zero = 0,
  one = 1,
  two = 2,
  three = 3
};

enum ADBCommandCodes: uint_fast8_t {
  sendreset = B0000,
  flush = B0001,
  listen = B1000,
  talk = B1100
};

void globalReset() {
  #if DBG >= 4
  log("ADB Reset");
  #endif

  pinMode(SIGPIN, OUTPUT);

  elapsedMillis start(0);
  do {
    // Twiddle Thumbs
  } while (start < GLOBALRESETMS);

  pinMode(SIGPIN, INPUT);

  delay(500);

  #if DBG >= 5
  log("ADB Reset Finished");
  #endif
}

struct TalkReturn {
  bool srq;
  tl::optional<uint_fast16_t> data;

  TalkReturn(bool srq, tl::optional<uint_fast16_t> data) : srq(srq), data(data) {}
};

struct ADBDevice {
  u4 addr;

  ADBDevice(const u4 addr) : addr((addr & 0x0F)  << 4 ) {}

  TalkReturn talk(ADBReg reg) {
    uint_fast8_t cmd =
      addr | ADBCommandCodes::talk | reg;

    bool srq = rawCommandSend(cmd);

    // read response
    return TalkReturn(srq, readReg());
  }

  private:

  static void pullMicros(const unsigned long durMicros) {
    pinMode(SIGPIN, OUTPUT);
    
    elapsedMicros start(0);
    do {} while (start < durMicros);

    pinMode(SIGPIN, INPUT);
  }

  static void bitSend(bool one) {
    pinMode(SIGPIN, OUTPUT);
    elapsedMicros start(0);

    if(one) {
      do {} while(start < ONELOWuS);
    } else {
      do {} while(start < ZEROLOWuS);
    }

    pinMode(SIGPIN, INPUT);

    do {} while(start < BITCELLuS);
  }

  static void attention() {
    pullMicros(ATTENTIONuS);
  }

  static void sync() {
    elapsedMicros start(0);
    do {} while(start < SYNCuS);
  }

  static bool TLT() {

    elapsedMicros start(0);
    while(digitalRead(SIGPIN) != HIGH && start < SRQTIMEOUTuS) {};

    auto uptime = start;
    bool srq = false;
    if(uptime >= SRQTIMEOUTuS) {
      Serial.print("SRQ TIMEOUT ");
      Serial.println(start);
    } else if(uptime >= SRQLOWTIMEuS) {
      srq = true;
    }

    // Wait
    delayMicroseconds(TLTHIGHTIMEuS);
    return srq;
  }

  static bool rawCommandSend(const uint_fast8_t cmd) {
    attention();
    sync();

    for(uint_fast8_t mask = B10000000; mask > 0; mask >>= 1) {
      bitSend(cmd & mask);
    }

    // Stop bit
    pullMicros(STOPLOWuS);

    bool srq = TLT();

    return srq;
  }

  static tl::optional<uint_fast16_t> readReg() {
    // Try to find start bit
    elapsedMicros start(0);

    WAITLOW();
    if(start >= READBITTIMEOUTuS) {
      return tl::nullopt;
    }

    // Wait for return to high
    start = 0;
    WAITHIGH();
    if (start >= READBITTIMEOUTuS) {
      return tl::nullopt;
    }

    // Read out the data
    uint_fast16_t dataBuf = 0;
    for(uint_fast16_t shift = (1 << 15); shift > 0; shift >>= 1) {
      start = 0;
      WAITLOW();
      if (start >= READBITTIMEOUTuS) {
        return tl::nullopt;
      }

      start = 0;
      WAITHIGH();
      if (start >= READBITTIMEOUTuS) {
        return tl::nullopt;
      } else if(start <= ONELOWuS) {
        dataBuf |= shift;
      }
    }

    // Stop Bit
    start = 0;
    WAITLOW();
    if (start >= READBITTIMEOUTuS) {
      return tl::nullopt;
    }

    start = 0;
    WAITHIGH();
    if (start >= READBITTIMEOUTuS) {
      return tl::nullopt;
    }

    return tl::optional<uint_fast16_t>(dataBuf);
  }

};

void setup() {
  digitalWrite(SIGPIN, LOW);

  Serial.println("UP");
  // while (!Serial) {
  //   // Wait for serial terminal
  // }
  Serial.println("-- world begins 2 --");

  uint32_t delayms = 2000;
  Serial.print("Delaying reset by ");
  Serial.println(delayms);
  delay(delayms);
  globalReset();
}

void loop() {
  Serial.println("Sequence Start");
  globalReset();

  for(int addr = 2; addr <= 2; ++addr) {
    Serial.print("Addr: ");
    Serial.print(addr);
    Serial.print(", Talk 3 ");

    auto a = ADBDevice(addr);
    auto ret = a.talk(three);

    if(ret.srq) {
      Serial.print("SRQ ");
    }

    if(ret.data.has_value()) {
      Serial.println(*ret.data, 16);
    } else {
      Serial.println("NOREPLY");
    }

    delay(500);
  }
}