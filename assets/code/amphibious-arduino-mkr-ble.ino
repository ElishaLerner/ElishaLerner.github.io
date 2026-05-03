#include <ArduinoBLE.h>
#include <Encoder.h>


BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int M1 = A3;
const int M2 = A4;
const int M4 = 3;
const int M3 = 2;
const int HeatTime = 25000;
const int resis = 12;
const int tca = 10;
const int tca2 = 0;
int t = 16000;

const int M4Phase = A6;
const int M3Phase = A5;
const int M1Phase = A0;
const int M2Phase = A2;

int err1 = 0;
int err2 = 0;
int err3 = 0;
int err4 = 0;

int count = 1;
int counts = 1;
int Bent = 0;

int pwm1 = 127;
int pwm2 = 127;
int pwm3 = 127;
int pwm4 = 127;

int menuPrinted = 0;

Encoder em4(A1, 1);
Encoder em3(4, 5);
Encoder em2(6, 7);
Encoder em1(8, 9);

long distance1;
long distance2;
long distance3;
long distance4;

long newm1 = 0;
long newm2 = 0;
long newm3 = 0;
long newm4 = 0;

long positionm1 = -999;
long positionm2 = -999;
long positionm3 = -999;
long positionm4 = -999;




void setup() {

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(M1Phase, OUTPUT);
  pinMode(M2Phase, OUTPUT);
  pinMode(M3Phase, OUTPUT);
  pinMode(M4Phase, OUTPUT);

  pinMode(resis, OUTPUT);
  pinMode(tca, OUTPUT);
  pinMode(tca2, OUTPUT);

  Serial.begin(9600);

  em1.write(0);

  em2.write(0);

  em3.write(0);

  em4.write(0);


  Serial.begin(9600);


  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);

  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Flopsy");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("Flopsy Starting");
}

void loop() {

  if ((Serial) && menuPrinted != 1) {
    Serial.println(" ");
    Serial.println(" ");
    Serial.println("-----------------------------------");
    Serial.println("Menu");
    Serial.println("[1]   Terrestrial Movement");
    Serial.println("[2]   Swimming Gait");
    Serial.println("[5]   Morph (must start unbent)");
    Serial.println("[6]   Heat and Set Motors to 0");
    Serial.println("[7]   Reset Encoders");
    Serial.println("[8]   Heat Legs for Swimming");
    Serial.println("[9]   Push Off");
    Serial.println("-----------------------------------");
    Serial.println(" ");
    Serial.println("Use LightBLue to connect to Flopsy bluetooth");

    menuPrinted = 1;
  }

  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:

      if (switchCharacteristic.value() == 1) {

        digitalWrite(M1Phase, 0);//
        digitalWrite(M2Phase, 0);//
        digitalWrite(M3Phase, 1);//
        digitalWrite(M4Phase, 1);//


        distance1 = 190 * 12 * count;
        distance4 = -190 * 12 * count;

        analogWrite(M1, 127);
        analogWrite(M4, 127);

        while ((em4.read() > distance4) || (em1.read() < distance1)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;

          }


          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;

          }

          if (em1.read() != newm1) {
            pwm1 = 127;

          }


          if (em4.read() != newm4) {
            pwm4 = 127;

          }

          analogWrite(M1, pwm1);
          analogWrite(M4, pwm4);

          if (em1.read() > distance1) {
            analogWrite(M1, 0);
            if (em4.read() < distance4) {
              analogWrite(M4, 0);
            }
          }


          if (em4.read() < distance4) {
            analogWrite(M4, 0);
            if (em1.read() > distance1) {
              analogWrite(M1, 0);
            }
          }

          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }
        }

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);




        distance3 = -190 * 12 * count;
        distance2 = 190 * 12 * count;
        analogWrite(M3, 127);
        analogWrite(M2, 127);

        while ((em3.read() > distance3) || (em2.read() < distance2)) {


          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;

          }


          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;

          }

          if (em2.read() != newm2) {
            pwm2 = 127;

          }

          if (em3.read() != newm3) {
            pwm3 = 127;

          }




          analogWrite(M2, pwm2);
          analogWrite(M3, pwm3);


          if (em3.read() < distance3) {
            analogWrite(M3, 0);
            if (em2.read() > distance2) {
              analogWrite(M2, 0);
            }
          }


          if (em2.read() > distance2) {
            analogWrite(M2, 0);
            if (em3.read() < distance3) {
              analogWrite(M3, 0);
            }
          }
          newm3 = em3.read();
          newm2 = em2.read();

          if (newm2 != positionm2 || newm3 != positionm3) {
            Serial.print("position3 = ");
            Serial.print(newm3);
            Serial.print("   ");
            Serial.print("position2 = ");
            Serial.print(newm2);

            Serial.println();
            positionm3 = newm3;
            positionm2 = newm2;
          }
        }

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);



        if (count == 2) {
          err1 = em1.read() - distance1;
          err2 = em2.read() - distance2;
          err3 = em3.read() - distance3;
          err4 = em4.read() - distance4;

          em1.write(err1);
          em2.write(err2);
          em3.write(err3);
          em4.write(err4);

        }

        count = count + 1;
        if (count == 3) {
          count = 1;
        }



        Serial.println("Count is");
        Serial.println(count);
      }

      if (switchCharacteristic.value() == 2) {

        Serial.print("swim");

        digitalWrite(M1Phase, 0);// (forward)
        digitalWrite(M2Phase, 1);// (backward)
        digitalWrite(M3Phase, 0);// (backward)
        digitalWrite(M4Phase, 1);// (forward)



        distance1 = 50; // neg=forward
        distance4 = -50 ;
        distance2 = -50 ; // neg=forward
        distance3 = 50 ;

        analogWrite(M1, 127);
        analogWrite(M4, 127);


        while ((em4.read() > distance4) || (em1.read() < distance1)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;

          }


          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;

          }

          if (em1.read() != newm1) {
            pwm1 = 127;

          }


          if (em4.read() != newm4) {
            pwm4 = 127;

          }

          analogWrite(M1, pwm1);
          analogWrite(M4, pwm4);

          if (em1.read() >= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() <= distance4) {
            analogWrite(M4, 0);
          }


          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

        } //end of phase1 swim


        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);
        delay(300);
        //reverse
        digitalWrite(M1Phase, 1);// (backward)
        digitalWrite(M2Phase, 0);// (forward)
        digitalWrite(M3Phase, 1);// (forward)
        digitalWrite(M4Phase, 0);// (backward)


        distance1 = -380 * 2.5;
        distance4 = 380 * 2.5 ;


        analogWrite(M1, 127);
        analogWrite(M4, 127);


        while ((em4.read() < distance4) || (em1.read() > distance1)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;

          }


          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;

          }

          if (em1.read() != newm1) {
            pwm1 = 127;

          }


          if (em4.read() != newm4) {
            pwm4 = 127;

          }

          analogWrite(M1, pwm1);
          analogWrite(M4, pwm4);

          if (em1.read() <= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() >= distance4) {
            analogWrite(M4, 0);
          }




          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

        } //end of second phase swim

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);


        counts = counts + 1;
        delay(300);

      }

      analogWrite(M1, 0);
      analogWrite(M2, 0);
      analogWrite(M3, 0);
      analogWrite(M4, 0);

      if (switchCharacteristic.value() == 7) {
        Serial.println("Reset Encoder");

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        em2.write(0);
        em3.write(0);
        em4.write(0);
        em1.write(0);

        count = 1;
      }

      if (switchCharacteristic.value() == 5) {
        Serial.println("Select 5: Morph");


        digitalWrite(M1Phase, 0); //forward
        digitalWrite(M2Phase, 0);
        digitalWrite(M3Phase, 1);
        digitalWrite(M4Phase, 1);



        distance1 = 380 * 3; // neg=forward
        distance4 = -380 * 3;
        distance2 = 380 * 3; // neg=forward
        distance3 = -380 * 3;

        analogWrite(M1, 127);
        analogWrite(M4, 127);
        analogWrite(M2, 127);
        analogWrite(M3, 127);

        while ((em4.read() >  distance4) || (em1.read() < distance1) || (em3.read() > distance3) || (em2.read() < distance2)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;
          }

          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;
          }

          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;
          }

          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;
          }


          if (em1.read() != newm1) {
            pwm1 = 127;
          }

          if (em2.read() != newm2) {
            pwm2 = 127;
          }

          if (em3.read() != newm3) {
            pwm3 = 127;
          }

          if (em4.read() != newm4) {
            pwm4 = 127;
          }

          if (em1.read() >= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() <= distance4) {
            analogWrite(M4, 0);
          }
          if (em2.read() >= distance2) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= distance3) {
            analogWrite(M3, 0);
          }


          newm2 = em2.read();
          newm3 = em3.read();
          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

          if (newm2 != positionm2 || newm3 != positionm3) {
            Serial.print("position3 = ");
            Serial.print(newm3);
            Serial.print("   ");
            Serial.print("position2 = ");
            Serial.print(newm2);

            Serial.println();
            positionm3 = newm3;
            positionm2 = newm2;
          }
        } //90 degree bend

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        Serial.println("Heating!");
        analogWrite(resis, 255);
        delay(HeatTime);
        Serial.println("Heating Complete!");
        analogWrite(resis, 0);

        Serial.println("Actuating TCA!");
        analogWrite(tca, 255);
        analogWrite(tca2, 255);
        delay(t);
        //-------------------------------------------------------------Dip into water======================================================


        digitalWrite(M1Phase, 0); //forward
        digitalWrite(M2Phase, 0);
        digitalWrite(M3Phase, 1);
        digitalWrite(M4Phase, 1);



        distance1 = 380 * 6; // neg=forward
        distance4 = -380 * 6;
        distance2 = 380 * 6; // neg=forward
        distance3 = -380 * 6;

        analogWrite(M1, 80);
        analogWrite(M4, 80);
        analogWrite(M2, 80);
        analogWrite(M3, 80);

        while ((em4.read() >  distance4) || (em1.read() < distance1) || (em3.read() > distance3) || (em2.read() < distance2)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;
          }

          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;
          }

          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;
          }

          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;
          }


          if (em1.read() != newm1) {
            pwm1 = 80;
          }

          if (em2.read() != newm2) {
            pwm2 = 80;
          }

          if (em3.read() != newm3) {
            pwm3 = 80;
          }

          if (em4.read() != newm4) {
            pwm4 = 80;
          }

          if (em1.read() >= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() <= distance4) {
            analogWrite(M4, 0);
          }
          if (em2.read() >= distance2) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= distance3) {
            analogWrite(M3, 0);
          }


          newm2 = em2.read();
          newm3 = em3.read();
          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

          if (newm2 != positionm2 || newm3 != positionm3) {
            Serial.print("position3 = ");
            Serial.print(newm3);
            Serial.print("   ");
            Serial.print("position2 = ");
            Serial.print(newm2);

            Serial.println();
            positionm3 = newm3;
            positionm2 = newm2;
          }
        } //90 degree bend

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);
        delay(2000);
        //analogWrite(tca,255*.11);
        //delay(12*t);
        analogWrite(tca, 0);
        analogWrite(tca2, 0);
        Serial.println("Actuating TCA Complete!");

        analogWrite(resis, 0);
        analogWrite(tca, 0);
        analogWrite(tca2, 0);

        switchCharacteristic.writeValue(0);



      }

      if (switchCharacteristic.value() == 6) {
        Serial.println("Select 6: Reheat");


        digitalWrite(M1Phase, 0); //forward
        digitalWrite(M2Phase, 0);
        digitalWrite(M3Phase, 1);
        digitalWrite(M4Phase, 1);



        distance1 = 380 * 3; // neg=forward
        distance4 = -380 * 3;
        distance2 = 380 * 3; // neg=forward
        distance3 = -380 * 3;

        analogWrite(M1, 127);
        analogWrite(M4, 127);
        analogWrite(M2, 127);
        analogWrite(M3, 127);

        while ((em4.read() >  distance4) || (em1.read() < distance1) || (em3.read() > distance3) || (em2.read() < distance2)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;
          }

          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;
          }

          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;
          }

          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;
          }


          if (em1.read() != newm1) {
            pwm1 = 127;
          }

          if (em2.read() != newm2) {
            pwm2 = 127;
          }

          if (em3.read() != newm3) {
            pwm3 = 127;
          }

          if (em4.read() != newm4) {
            pwm4 = 127;
          }

          if (em1.read() >= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() <= distance4) {
            analogWrite(M4, 0);
          }
          if (em2.read() >= distance2) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= distance3) {
            analogWrite(M3, 0);
          }


          newm2 = em2.read();
          newm3 = em3.read();
          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

          if (newm2 != positionm2 || newm3 != positionm3) {
            Serial.print("position3 = ");
            Serial.print(newm3);
            Serial.print("   ");
            Serial.print("position2 = ");
            Serial.print(newm2);

            Serial.println();
            positionm3 = newm3;
            positionm2 = newm2;
          }
        } //90 degree bend

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        Serial.println("Heating!");
        analogWrite(resis, 255);
        delay(8000);
        Serial.println("Heating Complete!");
        analogWrite(resis, 0);


        //-------------------------------------------------------------Dip into water======================================================


        digitalWrite(M1Phase, 0); //forward
        digitalWrite(M2Phase, 0);
        digitalWrite(M3Phase, 1);
        digitalWrite(M4Phase, 1);



        distance1 = 380 * 6; // neg=forward
        distance4 = -380 * 6;
        distance2 = 380 * 6; // neg=forward
        distance3 = -380 * 6;

        analogWrite(M1, 80);
        analogWrite(M4, 80);
        analogWrite(M2, 80);
        analogWrite(M3, 80);

        while ((em4.read() >  distance4) || (em1.read() < distance1) || (em3.read() > distance3) || (em2.read() < distance2)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;
          }

          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;
          }

          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;
          }

          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;
          }


          if (em1.read() != newm1) {
            pwm1 = 80;
          }

          if (em2.read() != newm2) {
            pwm2 = 80;
          }

          if (em3.read() != newm3) {
            pwm3 = 80;
          }

          if (em4.read() != newm4) {
            pwm4 = 80;
          }
          if (em1.read() >= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() <= distance4) {
            analogWrite(M4, 0);
          }
          if (em2.read() >= distance2) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= distance3) {
            analogWrite(M3, 0);
          }


          newm2 = em2.read();
          newm3 = em3.read();
          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

          if (newm2 != positionm2 || newm3 != positionm3) {
            Serial.print("position3 = ");
            Serial.print(newm3);
            Serial.print("   ");
            Serial.print("position2 = ");
            Serial.print(newm2);

            Serial.println();
            positionm3 = newm3;
            positionm2 = newm2;
          }
        } //90 degree bend

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        //analogWrite(tca,255*.11);
        //delay(12*t);
        analogWrite(tca, 0);
        analogWrite(tca2, 0);
        Serial.println("Actuating TCA Complete!");

        analogWrite(resis, 0);
        analogWrite(tca, 0);
        analogWrite(tca2, 0);
        switchCharacteristic.writeValue(0);



      }

      if (switchCharacteristic.value() == 8) {
        Serial.println("Morph Straight");


        digitalWrite(M1Phase, 0); //forward
        digitalWrite(M2Phase, 0);
        digitalWrite(M3Phase, 1);
        digitalWrite(M4Phase, 1);



        distance1 = 380 * 5; // neg=forward
        distance4 = -380 * 5;
        distance2 = 380 * 5; // neg=forward
        distance3 = -380 * 5;

        analogWrite(M1, 127);
        analogWrite(M4, 127);
        analogWrite(M2, 127);
        analogWrite(M3, 127);

        while ((em4.read() >  distance4) || (em1.read() < distance1) || (em3.read() > distance3) || (em2.read() < distance2)) {

          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;
          }

          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;
          }

          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;
          }

          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;
          }


          if (em1.read() != newm1) {
            pwm1 = 127;
          }

          if (em2.read() != newm2) {
            pwm2 = 127;
          }

          if (em3.read() != newm3) {
            pwm3 = 127;
          }

          if (em4.read() != newm4) {
            pwm4 = 127;
          }

          if (em1.read() >= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() <= distance4) {
            analogWrite(M4, 0);
          }
          if (em2.read() >= distance2) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= distance3) {
            analogWrite(M3, 0);
          }


          newm2 = em2.read();
          newm3 = em3.read();
          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

          if (newm2 != positionm2 || newm3 != positionm3) {
            Serial.print("position3 = ");
            Serial.print(newm3);
            Serial.print("   ");
            Serial.print("position2 = ");
            Serial.print(newm2);

            Serial.println();
            positionm3 = newm3;
            positionm2 = newm2;
          }
        } //90 degree bend

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        Serial.println("Heating!");
        analogWrite(resis, 255);
        delay(35000);
        Serial.println("Heating Complete!");
        analogWrite(resis, 0);


        //-------------------------------------------------------------Dip into water======================================================

        /*
                digitalWrite(M1Phase, 0); //forward
                digitalWrite(M2Phase, 0);
                digitalWrite(M3Phase, 1);
                digitalWrite(M4Phase, 1);



                distance1 = 380 * 6; // neg=forward
                distance4 = -380 * 6;
                distance2 = 380 * 6; // neg=forward
                distance3 = -380 * 6;

                analogWrite(M1, 80);
                analogWrite(M4, 80);
                analogWrite(M2, 80);
                analogWrite(M3, 80);

                while ((em4.read() >  distance4) || (em1.read() < distance1) || (em3.read() > distance3) || (em2.read() < distance2)) {

                  if (em1.read() == newm1) {
                    pwm1 = pwm1 + 1;
                  }

                  if (em2.read() == newm2) {
                    pwm2 = pwm2 + 1;
                  }

                  if (em3.read() == newm3) {
                    pwm3 = pwm3 + 1;
                  }

                  if (em4.read() == newm4) {
                    pwm4 = pwm4 + 1;
                  }


                  if (em1.read() != newm1) {
                    pwm1 = 80;
                  }

                  if (em2.read() != newm2) {
                    pwm2 = 80;
                  }

                  if (em3.read() != newm3) {
                    pwm3 = 80;
                  }

                  if (em4.read() != newm4) {
                    pwm4 = 80;
                  }
                  if (em1.read() >= distance1) {
                    analogWrite(M1, 0);
                  }
                  if (em4.read() <= distance4) {
                    analogWrite(M4, 0);
                  }
                  if (em2.read() >= distance2) {
                    analogWrite(M2, 0);
                  }
                  if (em3.read() <= distance3) {
                    analogWrite(M3, 0);
                  }


                  newm2 = em2.read();
                  newm3 = em3.read();
                  newm1 = em1.read();
                  newm4 = em4.read();

                  if (newm4 != positionm4 || newm1 != positionm1) {
                    Serial.print("position1 = ");
                    Serial.print(newm1);
                    Serial.print("   ");
                    Serial.print("position4 = ");
                    Serial.print(newm4);

                    Serial.println();
                    positionm1 = newm1;
                    positionm4 = newm4;
                  }

                  if (newm2 != positionm2 || newm3 != positionm3) {
                    Serial.print("position3 = ");
                    Serial.print(newm3);
                    Serial.print("   ");
                    Serial.print("position2 = ");
                    Serial.print(newm2);

                    Serial.println();
                    positionm3 = newm3;
                    positionm2 = newm2;
                  }
                } //90 degree bend
        */
        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        //analogWrite(tca,255*.11);
        //delay(12*t);
        analogWrite(tca, 0);
        analogWrite(tca2, 0);
        Serial.println("Actuating TCA Complete!");

        analogWrite(resis, 0);
        analogWrite(tca, 0);
        analogWrite(tca2, 0);
        delay(3000);
        switchCharacteristic.writeValue(0);



      }

      if (switchCharacteristic.value() == 0) {





        distance1 = 0;
        distance2 = 0;
        distance3 = 0;
        distance4 = 0;

        //all encoders move neg
        digitalWrite(M1Phase, 1);//
        digitalWrite(M2Phase, 1);//
        digitalWrite(M3Phase, 0);//
        digitalWrite(M4Phase, 0);//


        while ((em1.read() > 5) || (em2.read() > 5) || (em3.read() < -5) || (em4.read() < -5)) {
          if (em1.read() > 5) {
            analogWrite(M1, 80);
          }
          if (em2.read() > 5) {
            analogWrite(M2, 80);
          }
          if (em3.read() < -5) {
            analogWrite(M3, 80);
          }
          if (em4.read() < -5) {
            analogWrite(M4, 80);
          }
          if (em1.read() <= 5) {
            analogWrite(M1, 0);
          }
          if (em2.read() <= 5) {
            analogWrite(M2, 0);
          }
          if (em3.read() >= -5) {
            analogWrite(M3, 0);
          }
          if (em4.read() >= -5) {
            analogWrite(M4, 0);
          }
        }

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);

        //all encoders move positive

        digitalWrite(M1Phase, 0);//
        digitalWrite(M2Phase, 0);//
        digitalWrite(M3Phase, 1);//
        digitalWrite(M4Phase, 1);//

        while ((em1.read() < -5) || (em2.read() < -5) || (em3.read() > 5) || (em4.read() > 5)) {
          if (em1.read() < -5) {
            analogWrite(M1, 80);
          }
          if (em2.read() < -5) {
            analogWrite(M2, 80);
          }
          if (em3.read() > 5) {
            analogWrite(M3, 80);
          }
          if (em4.read() > 5) {
            analogWrite(M4, 80);
          }
          if (em1.read() >= -5) {
            analogWrite(M1, 0);
          }
          if (em2.read() >= -5) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= 5) {
            analogWrite(M3, 0);
          }
          if (em4.read() <= 5) {
            analogWrite(M4, 0);
          }
        }

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);




      }



      if (switchCharacteristic.value() == 9) {

        Serial.print("push off");
        // Raise back legs

        digitalWrite(M1Phase, 1);// (backward)

        digitalWrite(M4Phase, 0);// (backward)


        distance1 = -380 * 1.7;
        distance4 = 380 * 1.7 ;


        analogWrite(M1, 127);
        analogWrite(M4, 127);


        while ((em4.read() < distance4) || (em1.read() > distance1)) {




          if (em1.read() == newm1) {
            pwm1 = pwm1 + 1;

          }


          if (em4.read() == newm4) {
            pwm4 = pwm4 + 1;

          }

          if (em1.read() != newm1) {
            pwm1 = 127;

          }


          if (em4.read() != newm4) {
            pwm4 = 127;

          }

          analogWrite(M1, pwm1);
          analogWrite(M4, pwm4);

          if (em1.read() <= distance1) {
            analogWrite(M1, 0);
          }
          if (em4.read() >= distance4) {
            analogWrite(M4, 0);
          }




          newm1 = em1.read();
          newm4 = em4.read();

          if (newm4 != positionm4 || newm1 != positionm1) {
            Serial.print("position1 = ");
            Serial.print(newm1);
            Serial.print("   ");
            Serial.print("position4 = ");
            Serial.print(newm4);

            Serial.println();
            positionm1 = newm1;
            positionm4 = newm4;
          }

        } //end of second phase swim

        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);



        digitalWrite(M2Phase, 0);//
        digitalWrite(M3Phase, 1);//




        distance3 = -380 * 12;
        distance2 = 380 * 12;

        analogWrite(M2, 127);
        analogWrite(M3, 127);


        while ((em3.read() > distance3) || (em2.read() < distance2)) {

          if (em2.read() == newm2) {
            pwm2 = pwm2 + 1;

          }


          if (em3.read() == newm3) {
            pwm3 = pwm3 + 1;

          }

          if (em2.read() != newm2) {
            pwm2 = 127;

          }


          if (em3.read() != newm3) {
            pwm3 = 127;

          }
          analogWrite(M2, pwm2);
          analogWrite(M3, pwm3);

          if (em2.read() >= distance2) {
            analogWrite(M2, 0);
          }
          if (em3.read() <= distance3) {
            analogWrite(M3, 0);
          }


          newm2 = em2.read();
          newm3 = em3.read();

          if (newm3 != positionm3 || newm2 != positionm2) {
            Serial.print("position2 = ");
            Serial.print(newm2);
            Serial.print("   ");
            Serial.print("position3 = ");
            Serial.print(newm3);

            Serial.println();
            positionm2 = newm2;
            positionm3 = newm3;
          }

        }
        analogWrite(M1, 0);
        analogWrite(M2, 0);
        analogWrite(M3, 0);
        analogWrite(M4, 0);
switchCharacteristic.writeValue(0);
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
