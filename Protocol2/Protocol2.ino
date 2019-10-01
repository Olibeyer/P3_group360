#include "Protocol_2.h"

ProtocolController* controler_ptr;

long oldTime = 0;
int setPos = 0;
int a = 0;
bool up = true;
void setup() {
  controler_ptr = new ProtocolController();
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  /*for (int i = 1; i <= 5; i++) {
    controler_ptr->toggleTorque(i, true);
    delay(10);
    }*/

  delay(100);
}

void loop() {
  if (millis() - oldTime > 1000) {
    oldTime = millis();
    //controler_ptr->ping(0xFE);

    /*
      if(up)
      a += 10;
      else
      a -= 10;

      if(a > 300)
      up = !up;
      if(a < 0)
      up = !up;


      controler_ptr->setPos(0x01, 1300 + a);

      controler_ptr->setPos(0x02, 500 + a);
      controler_ptr->setPos(0x03, 2000 + a);

      controler_ptr->setPos(0x04, 1200 + a);
      controler_ptr->setPos(0x05, 2300 - a);*/
    digitalWrite(13, !digitalRead(13));
    up = !up;
    Serial.println(controler_ptr->readFunction(0x01, 65, 1));
    controler_ptr->setLed(0x01, up);
    Serial.println(controler_ptr->readFunction(0x01, 65, 1));
    Serial.println(millis()-oldTime);
  }
}
