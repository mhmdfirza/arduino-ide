#ifndef WALL_FOLLOWER_TYPES_H
#define WALL_FOLLOWER_TYPES_H

#include <Arduino.h>

// State sistem
enum SysState : uint8_t {
  SYS_IDLE,
  SYS_READY,
  SYS_RUN,
  SYS_FINISHED,
  SYS_ERROR,
  SYS_MAINT
};

// State navigasi
enum NavState : uint8_t {
  NAV_START_TURN,
  NAV_ACQUIRE,
  NAV_FOLLOW,
  NAV_TURN_OUTER,
  NAV_PIVOT,
  NAV_RECOVER
};

// Mode LED
enum LedMode : uint8_t {
  LED_OFF,
  LED_ON,
  LED_BLINK_SLOW,
  LED_BLINK_SLOW_INV,
  LED_BLINK_FAST
};

// Struktur tombol
struct Button {
  uint8_t pin;
  bool down = false;
  bool rawLast = false;
  bool evPress = false;
  bool evRelease = false;

  uint32_t tChange = 0;
  uint32_t tDown = 0;
  uint32_t lastHold = 0;

  explicit Button(uint8_t p) : pin(p) {}
};

#endif