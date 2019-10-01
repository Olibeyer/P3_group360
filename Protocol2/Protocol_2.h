#ifndef PROTOCAL_H
#define PROTOCAL_H

#include <Arduino.h>;

const int sRecRX = 2;
const int sTranTX = 3;
const int SControl = 4;   //RS485 Direction control
const int Pin13LED = 13;

class ProtocolController {
public:
  ProtocolController();
  void ping(unsigned char address);
  void ledSet(unsigned char address, bool ledStatus);
  void toggleTorque(unsigned char address, bool onTrue);
  void setPos(unsigned char address, long setPosition);
  bool writeFunction(unsigned char address, unsigned char *data_blk_ptr, unsigned short data_blk_size);
  void update_receiver();
  long readFunction(unsigned char address, int tableAdress, int dataLength);

  bool getLed(unsigned char address);
  
  bool writeReturn = false;

  unsigned char packageBuffer[255];
  int packageBufferLength = 0;

private: 
  unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
};

#endif
