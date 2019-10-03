#include "Protocol_2.h"

ProtocolController* controler_ptr;

long oldTime = 0;

//used for initMotors and playMotors -- makes the arm move a little
int a = 0;
bool up = true;
void initMotors();
void playMotors();

void setup() {
  controler_ptr = new ProtocolController();
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  delay(100);
}

void loop() {




  digitalWrite(13, !digitalRead(13));
}





void initMotors()
{ 
  for (int i = 1; i <= 5; i++) {
    controler_ptr->toggleTorque(i, true);
  }
}

void playMotors()
{
  if (up)
    a += 2;
  else
    a -= 2;

  if (a > 300)
  {
    up = !up;
    for (int i = 1; i < 6; i++)
      controler_ptr->setLed(i, 0);
  }
  if (a < 0)
  {
    up = !up;
    for (int i = 1; i < 6; i++)
      controler_ptr->setLed(i, 1);
  }


  controler_ptr->setPos(0x01, 1300 + a);

  controler_ptr->setPos(0x02, 500 + a);
  controler_ptr->setPos(0x03, 2000 + a);

  controler_ptr->setPos(0x04, 1200 + a);
  controler_ptr->setPos(0x05, 2300 - a);
}
