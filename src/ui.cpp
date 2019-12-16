#include <ui.h>

void toggleAccessLED(){
  digitalWrite(ACCESSPOINT_LED, !digitalRead(ACCESSPOINT_LED));
}

void toggleGearLED(){
  digitalWrite(GEAR_FOUND_LED, !digitalRead(GEAR_FOUND_LED));
}

void toggleMarkerLED(){
  digitalWrite(MARKER_FOUND_LED, !digitalRead(MARKER_FOUND_LED));
}

///

bool userInterface::buttonIsPressed(){
  return !digitalRead(BUTTON);
}

void userInterface::gearFound(){
    digitalWrite(GEAR_FOUND_LED, HIGH);
}

void userInterface::gearLost(){
    digitalWrite(GEAR_FOUND_LED, LOW);
}

void userInterface::markerFound(){
    digitalWrite(MARKER_FOUND_LED, HIGH);
}

void userInterface::markerLost(){
    digitalWrite(MARKER_FOUND_LED, LOW);
}

void userInterface::configMode(){
  ticker.attach(0.1, toggleAccessLED);
}

void userInterface::wifiConnected(){
  ticker.detach();
  digitalWrite(ACCESSPOINT_LED, HIGH);
}

void userInterface::wifiDisConnected(){
  ticker.attach(0.5, toggleAccessLED);
}

void userInterface::init(){
  pinMode(ACCESSPOINT_LED, OUTPUT);
  pinMode(MARKER_FOUND_LED, OUTPUT);
  pinMode(GEAR_FOUND_LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  ticker.attach(0.5, toggleAccessLED);
};
