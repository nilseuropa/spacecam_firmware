#include <pixart.h>

void Pixart::setregister(byte d1, byte d2) {
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

Pixart::Pixart() {
}

void Pixart::init () {
    sensorAddress = 0xB0; //default: 0x58
    slaveAddress = sensorAddress >> 1;
    Wire.begin();
    setregister(0x30,0x01); delay(10);
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x00); Wire.write(0x00);
    Wire.write(0x00); Wire.write(0x00);
    Wire.write(0x00); Wire.write(0x00);
    Wire.write(0x00); Wire.write(0x90);
    Wire.endTransmission(); delay(100);
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x07); Wire.write(0x00); Wire.write(0x41);
    Wire.endTransmission(); delay(100);
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x1A); Wire.write(0x40); Wire.write(0x00);
    Wire.endTransmission(); delay(100);
    setregister(0x33,0x03); delay(10);
    setregister(0x30,0x08); delay(10);
    delay(100);
}

void Pixart::clearBuffer() {
    for (uint8_t i=0;i<16;i++) { data_buf[i]=0; }
}

void Pixart::readBuffer() {
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();
    Wire.requestFrom(slaveAddress, 16);
    clearBuffer();
    uint8_t i = 0;
    while(Wire.available() && i < 16)
    {
        data_buf[i] = Wire.read();
        i++;
    }
}

void Pixart::getBlobs(){
    readBuffer();

    uint8_t s = 0;
    blobCount = 0;

    blobArray[0].X = data_buf[1];
    blobArray[0].Y = data_buf[2];
    s   = data_buf[3];
    blobArray[0].X += (s & 0x30) <<4;
    blobArray[0].Y += (s & 0xC0) <<2;
    blobArray[0].Size = (s & 0x0F);
    blobArray[0].Size<0x0f?blobCount++:blobCount;

    blobArray[1].X = data_buf[4];
    blobArray[1].Y = data_buf[5];
    s   = data_buf[6];
    blobArray[1].X += (s & 0x30) <<4;
    blobArray[1].Y += (s & 0xC0) <<2;
    blobArray[1].Size = (s & 0x0F);
    blobArray[1].Size<0x0f?blobCount++:blobCount;

    blobArray[2].X = data_buf[7];
    blobArray[2].Y = data_buf[8];
    s   = data_buf[9];
    blobArray[2].X += (s & 0x30) <<4;
    blobArray[2].Y += (s & 0xC0) <<2;
    blobArray[2].Size = (s & 0x0F);
    blobArray[2].Size<0x0f?blobCount++:blobCount;

    blobArray[3].X = data_buf[10];
    blobArray[3].Y = data_buf[11];
    s   = data_buf[12];
    blobArray[3].X += (s & 0x30) <<4;
    blobArray[3].Y += (s & 0xC0) <<2;
    blobArray[3].Size = (s & 0x0F);
    blobArray[3].Size<0x0f?blobCount++:blobCount;
}
