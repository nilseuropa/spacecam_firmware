
#ifndef __PIXART_H__
#define __PIXART_H__

#include <Arduino.h>
#include <Wire.h>

#define RES_X 1024
#define RES_Y 768

struct Blob
{
   	uint16_t X;
   	uint16_t Y;
   	uint16_t Size;
};

class Pixart
{

public:
  Pixart();
	void init();

  void getBlobs();
  Blob blobArray[4];
  uint8_t blobCount;
  char allocatedBuffer[64];

private:
	uint16_t sensorAddress;
	uint16_t slaveAddress;
	uint8_t data_buf[16];
  void clearBuffer();
  void readBuffer();
	void setregister(uint8_t d1, uint8_t d2);
};


#endif
