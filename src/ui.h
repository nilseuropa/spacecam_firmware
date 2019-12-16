#ifndef __UI_H__
#define __UI_H__
#include <Arduino.h>
#include <hardware.h>
#include <Ticker.h>

class userInterface{

Ticker ticker;

public:
  void init();
  void configMode();
  void wifiConnected();
  void wifiDisConnected();
  void gearFound();
  void gearLost();
  void markerFound();
  void markerLost();
  bool buttonIsPressed();

};

#endif
