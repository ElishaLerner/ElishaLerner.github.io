#include <PID_v1.h>
#include <Encoder.h>

double cycle = 5;

String command;
String loco;

const int heatpwm1_1 = 2;
const int heatpwm1_2 = 3;
const int heatpwm1_3 = 4;
const int heatpwm1_4 = 5;
const int heatpwm2_1 = 6;
const int heatpwm2_2 = 7;
const int heatpwm2_3 = 8;
const int heatpwm2_4 = 9;
const int T1_1 = A1;
const int T1_2 = A2;
const int T1_3 = A3;
const int T2_1 = A4;
const int T2_2 = A5;
const int T2_3 = A6;

const int tpower1 = 22;
const int tpower2 = 23;

//Thermistor constants
const int B_val = 3950;
const int ResT = 100000;
const int nom_temp = 25;
const int samp = 5;

int i = 0;
double res1_1 = 0;
double res1_2 = 0;
double res1_3 = 0;
double res2_1 = 0;
double res2_2 = 0;
double res2_3 = 0;
double stage = 0;


double averageRes1_1 = 0;
double averageRes1_2 = 0;
double averageRes1_3 = 0;
double averageRes2_1 = 0;
double averageRes2_2 = 0;
double averageRes2_3 = 0;

double temp1_1 = 0;
double temp1_2 = 0;
double temp1_3 = 0;
double temp2_1 = 0;
double temp2_2 = 0;
double temp2_3 = 0;

//Motor constants
Encoder em1(20, 21);
Encoder em2(18, 19);

const int M1En = 12;
const int M1Ph = 46;
const int M2En = 11;
const int M2Ph = 47;

double g = 0;
double s1_1 = 0;
double s1_2 = 0;
double s1_3 = 0;
double s1_4 = 0;
double s2_1 = 0;
double s2_2 = 0;
double s2_3 = 0;
double s2_4 = 0;
double motion_done = 0;

double dist1und = 1.5 * 6 * 380 * .6;
double dist2und = 1.5 * 6 * 380 * .6;
double dist1in = 2 * 6 * 380;
double dist2in = 2.4 * 6 * 380;
double dist1turt = 1.5 * 6 * 380;
double dist2turt = 1.2 * 6 * 380;
double dist1 = 1.5 * 6 * 380; //a full rotation is roughly 21.7 mm discplacement
double dist2 = 1.5 * 6 * 380;
// inch dist1= 2*6*380          dist2=2.4*6*380
double z = 0;

//Relays
const int R1_1 = 28;
const int R1_2 = 29; //element 2 top-->bot
const int R1_3 = 30; //element 3 bot-->top
const int R1_4 = 31;
const int R2_1 = 32;
const int R2_2 = 33; //element 2 top-->bot
const int R2_3 = 34; //element 3 bot-->top
const int R2_4 = 35;

//PID Variables
double Setpoint1_1, Setpoint1_2, Setpoint1_3, Setpoint1_4, PIDpwm1_1, PIDpwm1_2, PIDpwm1_3, PIDpwm1_4;
double Setpoint2_1, Setpoint2_2, Setpoint2_3, Setpoint2_4, PIDpwm2_1, PIDpwm2_2, PIDpwm2_3, PIDpwm2_4;
double Kp = 40, Ki = 40, Kd = 80;
PID temp1_1cont(&temp1_1, &PIDpwm1_1, &Setpoint1_1, Kp, Ki, Kd, DIRECT);
PID temp1_2cont(&temp1_2, &PIDpwm1_2, &Setpoint1_2, Kp, Ki, Kd, DIRECT);
PID temp1_3cont(&temp1_2, &PIDpwm1_3, &Setpoint1_3, Kp, Ki, Kd, DIRECT);
PID temp1_4cont(&temp1_3, &PIDpwm1_4, &Setpoint1_4, Kp, Ki, Kd, DIRECT);
PID temp2_1cont(&temp2_1, &PIDpwm2_1, &Setpoint2_1, Kp, Ki, Kd, DIRECT);
PID temp2_2cont(&temp2_2, &PIDpwm2_2, &Setpoint2_2, Kp, Ki, Kd, DIRECT);
PID temp2_3cont(&temp2_2, &PIDpwm2_3, &Setpoint2_3, Kp, Ki, Kd, DIRECT);
PID temp2_4cont(&temp2_3, &PIDpwm2_4, &Setpoint2_4, Kp, Ki, Kd, DIRECT);



void setup() {

  Serial.begin(9600);

  pinMode(R1_1, OUTPUT);
  pinMode(R1_2, OUTPUT);
  pinMode(R1_3, OUTPUT);
  pinMode(R1_4, OUTPUT);
  pinMode(R2_1, OUTPUT);
  pinMode(R2_2, OUTPUT);
  pinMode(R2_3, OUTPUT);
  pinMode(R2_4, OUTPUT);

  pinMode(M1En, OUTPUT);
  pinMode(M1Ph, OUTPUT);
  pinMode(M2En, OUTPUT);
  pinMode(M2Ph, OUTPUT);

  pinMode(T1_1, INPUT);
  pinMode(T1_2, INPUT);
  pinMode(T1_3, INPUT);
  pinMode(T2_1, INPUT);
  pinMode(T2_2, INPUT);
  pinMode(T2_3, INPUT);
  pinMode(heatpwm1_1, OUTPUT);
  pinMode(heatpwm1_2, OUTPUT);
  pinMode(heatpwm1_3, OUTPUT);
  pinMode(heatpwm1_4, OUTPUT);
  pinMode(heatpwm2_1, OUTPUT);
  pinMode(heatpwm2_2, OUTPUT);
  pinMode(heatpwm2_3, OUTPUT);
  pinMode(heatpwm2_4, OUTPUT);
  pinMode(tpower1, OUTPUT);
  pinMode(tpower2, OUTPUT);

  temp1_1cont.SetMode(AUTOMATIC);
  temp1_2cont.SetMode(AUTOMATIC);
  temp1_3cont.SetMode(AUTOMATIC);
  temp1_4cont.SetMode(AUTOMATIC);
  temp2_1cont.SetMode(AUTOMATIC);
  temp2_2cont.SetMode(AUTOMATIC);
  temp2_3cont.SetMode(AUTOMATIC);
  temp2_4cont.SetMode(AUTOMATIC);

  digitalWrite(R1_1, 1); //0 is on
  digitalWrite(R1_2, 1);
  digitalWrite(R1_3, 1); //0 is on
  digitalWrite(R1_4, 1);
  digitalWrite(R2_1, 1); //0 is on
  digitalWrite(R2_2, 1);
  digitalWrite(R2_3, 1); //0 is on
  digitalWrite(R2_4, 1);
  command = "reset";

}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');


  }

  if (command.equals("reset")) {
    Setpoint1_1 = 0;
    Setpoint1_2 = 0;
    Setpoint1_3 = 0;
    Setpoint1_4 = 0;
    Setpoint2_1 = 0;
    Setpoint2_2 = 0;
    Setpoint2_3 = 0;
    Setpoint2_4 = 0;
    digitalWrite(R1_1, 1); //0 is on
    digitalWrite(R1_2, 1);
    digitalWrite(R1_3, 1); //0 is on
    digitalWrite(R1_4, 1);
    digitalWrite(R2_1, 1); //0 is on
    digitalWrite(R2_2, 1);
    digitalWrite(R2_3, 1); //0 is on
    digitalWrite(R2_4, 1);
    g = 0;
    z = 0;
    loco = "reset";
  }

  if (command.equals("inch")) {
    Setpoint1_1 = 0;
    Setpoint1_2 = 90;
    Setpoint1_3 = 0;
    Setpoint1_4 = 60;
    Setpoint2_1 = 0;
    Setpoint2_2 = 0;
    Setpoint2_3 = 0;
    Setpoint2_4 = 90;
    loco = "inch";
    Serial.print("inch");
  }

  if (command.equals("undulate")) {
    Setpoint1_1 = 70;
    Setpoint1_2 = 0;
    Setpoint1_3 = 0;
    Setpoint1_4 = 0;
    Setpoint2_1 = 70;
    Setpoint2_2 = 0;
    Setpoint2_3 = 0;
    Setpoint2_4 = 0;
    loco = "und";
  }

  if (command.equals("turtle")) {
    Setpoint1_1 = 0;
    Setpoint1_2 = 80;
    Setpoint1_3 = 0;
    Setpoint1_4 = 0;
    Setpoint2_1 = 0;
    Setpoint2_2 = 80;
    Setpoint2_3 = 0;
    Setpoint2_4 = 0;
    digitalWrite(R1_2, 0);
    digitalWrite(R2_2, 0);
    loco = "turt";
  }


  if (Setpoint1_1 <= 0) {
    digitalWrite(R1_1, 1);
  } else {
    digitalWrite(R1_1, 0);
  }
  if (Setpoint1_2 <= 0) {
    digitalWrite(R1_2, 1);
  } else {
    digitalWrite(R1_2, 0);
  }
  if (Setpoint1_3 <= 0) {
    digitalWrite(R1_3, 1);
  } else {
    digitalWrite(R1_3, 0);
  }
  if (Setpoint1_4 <= 0) {
    digitalWrite(R1_4, 1);
  } else {
    digitalWrite(R1_4, 0);
  }
  if (Setpoint2_1 <= 0) {
    digitalWrite(R2_1, 1);
  } else {
    digitalWrite(R2_1, 0);
  }
  if (Setpoint2_2 <= 0) {
    digitalWrite(R2_2, 1);
  } else {
    digitalWrite(R2_2, 0);
  }
  if (Setpoint2_3 <= 0) {
    digitalWrite(R2_3, 1);
  } else {
    digitalWrite(R2_3, 0);
  }
  if (Setpoint2_4 <= 0) {
    digitalWrite(R2_4, 1);
  } else {
    digitalWrite(R2_4, 0);
  }



  while ((Setpoint1_2 > 0) && (Setpoint1_3 > 0)) {
    digitalWrite(R1_2, 1);
    digitalWrite(R1_3, 1);
    Serial.println("Error Both Diagonals Active on module 1");
  }

  while ((Setpoint2_2 > 0) && (Setpoint2_3 > 0)) {
    digitalWrite(R2_2, 1);
    digitalWrite(R2_3, 1);
    Serial.println("Error Both Diagonals Active on module 2");
  }

  while (i < samp) {
    digitalWrite(tpower1, HIGH);
    digitalWrite(tpower2, HIGH);
    res1_1 += analogRead(T1_1);
    res1_2 += analogRead(T1_2);
    res1_3 += analogRead(T1_3);
    res2_1 += analogRead(T2_1);
    res2_2 += analogRead(T2_2);
    res2_3 += analogRead(T2_3);
    i = i + 1;
  }

  digitalWrite(tpower1, 0);
  digitalWrite(tpower2, 0);

  averageRes1_1 = res1_1 / samp;
  averageRes1_1 = 1023 / averageRes1_1 - 1;
  averageRes1_1 = ResT / averageRes1_1;

  temp1_1 = averageRes1_1 / ResT;
  temp1_1 = log(temp1_1);
  temp1_1 = temp1_1 / B_val;
  temp1_1 = temp1_1 + 1 / (nom_temp + 273.15);
  temp1_1 = 1 / temp1_1;
  temp1_1 = temp1_1 - 273.15;
  Serial.print("Temp 1_1 = ");
  Serial.print(temp1_1);
  Serial.print(" *C");

  averageRes1_2 = res1_2 / samp;
  averageRes1_2 = 1023 / averageRes1_2 - 1;
  averageRes1_2 = ResT / averageRes1_2;

  temp1_2 = averageRes1_2 / ResT;
  temp1_2 = log(temp1_2);
  temp1_2 = temp1_2 / B_val;
  temp1_2 = temp1_2 + 1 / (nom_temp + 273.15);
  temp1_2 = 1 / temp1_2;
  temp1_2 = temp1_2 - 273.15;
  Serial.print("    Temp 1_2 = ");
  Serial.print(temp1_2);
  Serial.print(" *C");


  averageRes1_3 = res1_3 / samp;
  averageRes1_3 = 1023 / averageRes1_3 - 1;
  averageRes1_3 = ResT / averageRes1_3;

  temp1_3 = averageRes1_3 / ResT;
  temp1_3 = log(temp1_3);
  temp1_3 = temp1_3 / B_val;
  temp1_3 = temp1_3 + 1 / (nom_temp + 273.15);
  temp1_3 = 1 / temp1_3;
  temp1_3 = temp1_3 - 273.15;
  Serial.print("    Temp 1_3 = ");
  Serial.print(temp1_3);
  Serial.print(" *C");
  Serial.println();

  averageRes2_1 = res2_1 / samp;
  averageRes2_1 = 1023 / averageRes2_1 - 1;
  averageRes2_1 = ResT / averageRes2_1;

  temp2_1 = averageRes2_1 / ResT;
  temp2_1 = log(temp2_1);
  temp2_1 = temp2_1 / B_val;
  temp2_1 = temp2_1 + 1 / (nom_temp + 273.15);
  temp2_1 = 1 / temp2_1;
  temp2_1 = temp2_1 - 273.15;
  Serial.print("Temp 2_1 = ");
  Serial.print(temp2_1);
  Serial.print(" *C");

  averageRes2_2 = res2_2 / samp;
  averageRes2_2 = 1023 / averageRes2_2 - 1;
  averageRes2_2 = ResT / averageRes2_2;

  temp2_2 = averageRes2_2 / ResT;
  temp2_2 = log(temp2_2);
  temp2_2 = temp2_2 / B_val;
  temp2_2 = temp2_2 + 1 / (nom_temp + 273.15);
  temp2_2 = 1 / temp2_2;
  temp2_2 = temp2_2 - 273.15;
  Serial.print("    Temp 2_2 = ");
  Serial.print(temp2_2);
  Serial.print(" *C");


  averageRes2_3 = res2_3 / samp;
  averageRes2_3 = 1023 / averageRes2_3 - 1;
  averageRes2_3 = ResT / averageRes2_3;

  temp2_3 = averageRes2_3 / ResT;
  temp2_3 = log(temp2_3);
  temp2_3 = temp2_3 / B_val;
  temp2_3 = temp2_3 + 1 / (nom_temp + 273.15);
  temp2_3 = 1 / temp2_3;
  temp2_3 = temp2_3 - 273.15;
  Serial.print("    Temp 2_3 = ");
  Serial.print(temp2_3);
  Serial.print(" *C");
  Serial.println();
  temp1_1cont.Compute();
  temp1_2cont.Compute();
  temp1_3cont.Compute();
  temp1_4cont.Compute();
  temp2_1cont.Compute();
  temp2_2cont.Compute();
  temp2_3cont.Compute();
  temp2_4cont.Compute();

  analogWrite(heatpwm1_1, PIDpwm1_1);
  analogWrite(heatpwm1_2, PIDpwm1_2);
  analogWrite(heatpwm1_3, PIDpwm1_3);
  analogWrite(heatpwm1_4, PIDpwm1_4);
  analogWrite(heatpwm2_1, PIDpwm2_1);
  analogWrite(heatpwm2_2, PIDpwm2_2);
  analogWrite(heatpwm2_3, PIDpwm2_3);
  analogWrite(heatpwm2_4, PIDpwm2_4);


  if (((temp1_1 >= Setpoint1_1 - 2) && (temp1_1 <= Setpoint1_1 + 2)) || (Setpoint1_1 == 0)) {
    s1_1 = 1;
  } else {
    s1_1 = 0;
  }
  if (((temp1_2 >= Setpoint1_2 - 2) && (temp1_2 <= Setpoint1_2 + 2)) || (Setpoint1_2 == 0)) {
    s1_2 = 1;
  } else {
    s1_2 = 0;
  }
  if (((temp1_2 >= Setpoint1_3 - 2) && (temp1_2 <= Setpoint1_3 + 2)) || (Setpoint1_3 == 0)) {
    s1_3 = 1;
  } else {
    s1_3 = 0;
  }
  if (((temp1_3 >= Setpoint1_4 - 2) && (temp1_3 <= Setpoint1_4 + 2)) || (Setpoint1_4 == 0)) {
    s1_4 = 1;
  } else {
    s1_4 = 0;
  }
  if (((temp2_1 >= Setpoint2_1 - 2) && (temp2_1 <= Setpoint2_1 + 2)) || (Setpoint2_1 == 0)) {
    s2_1 = 1;
  } else {
    s2_1 = 0;
  }
  if (((temp2_2 >= Setpoint2_2 - 2) && (temp2_2 <= Setpoint2_2 + 2)) || (Setpoint2_2 == 0)) {
    s2_2 = 1;
  } else {
    s2_2 = 0;
  }
  if (((temp2_2 >= Setpoint2_3 - 2) && (temp2_2 <= Setpoint2_3 + 2)) || (Setpoint2_3 == 0)) {
    s2_3 = 1;
  } else {
    s2_3 = 0;
  }
  if (((temp2_3 >= Setpoint2_4 - 2) && (temp2_3 <= Setpoint2_4 + 2)) || (Setpoint2_4 == 0)) {
    s2_4 = 1;
  } else {
    s2_4 = 0;
  }

  if ((s1_1 == 1) && (s1_2 == 1) && (s1_1 == 1) && (s1_3 == 1) && (s1_4 == 1) && (s2_1 == 1) && (s2_2 == 1) && (s2_3 == 1) && (s2_4 == 1)) {
    g = g + 1;
    Serial.println(g);
    if (command.equals("reset")) {
      g = 0;
    }
  }
  //Protection Code
  if (command.equals("zero")) {

  }
  else if ((em2.read() > dist2 * 1.3) || (em1.read() > dist1 * 1.3)) {
    digitalWrite(M2Ph, 0);
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 0);
    analogWrite(M2En, 0);
  }

  else if ((em2.read() < -dist2 * 1.3) || (em1.read() < -dist1 * 1.3)) {
    digitalWrite(M2Ph, 1);
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 0);
    analogWrite(M2En, 0);
  }

  //Inch Worm Code

  if ((g >= 200) && (command.equals("inch")) && (em1.read() < dist1in) && (em2.read() < dist2in) && (motion_done == 0) && (z < cycle) && (stage == 0)) {
    digitalWrite(M2Ph, 1);
    analogWrite(M2En, 0);
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 200);
  }

  if ((command.equals("inch")) && (em1.read() >= dist1in) && (em2.read() < dist2in) && (stage == 0) && (z < cycle)) {
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 1);
    analogWrite(M2En, 200);
  }


  if ((command.equals("inch")) && (em1.read() >= dist1in) && (em2.read() >= dist2in) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 200);
    analogWrite(M2En, 0);
    stage = 1;
  }

  if ((command.equals("inch")) && (em1.read() <= 0) && (em2.read() >= dist2in) && (stage == 1) && (z < cycle)) {
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 200);
    stage = 1;
  }

  if ((command.equals("inch")) && (em1.read() <= 0) && (em2.read() <= 0) && (stage == 1) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 0);
    stage = 0;
    motion_done = 0;
    z = z + 1;
  }

  //Turtle Code

  if ((g >= 200) && (command.equals("turtle")) && (em1.read() < dist1turt) && (em2.read() < dist2turt) && (motion_done == 0) && (z < cycle) && (stage == 0)) {
    digitalWrite(M2Ph, 1);
    analogWrite(M2En, 200);
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 200);
  }

  if ((g >= 200) && (command.equals("turtle")) && (em1.read() >= dist1turt) && (stage == 0) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 0);
  }

  if ((g >= 200) && (command.equals("turtle")) && (em2.read() >= dist2turt) && (stage == 0) && (z < cycle)) {
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 0);
  }


  if ((command.equals("turtle")) && (em1.read() >= dist1turt) && (em2.read() >= dist2turt) && (z < cycle) && (stage == 0)) {
    stage = 1;
  }

  if ((command.equals("turtle")) && (em1.read() >= dist1turt) && (em2.read() >= dist2turt) && (stage == 1) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 200);
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 200);
    stage = 1;
  }


  if ((command.equals("turtle")) && (em1.read() <= 0) && (stage == 1) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 0);
    stage = 1;
  }

  if ((command.equals("turtle")) && (em2.read() <= 0) && (stage == 1) && (z < cycle)) {
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 0);
    stage = 1;
  }


  if ((command.equals("turtle")) && (em1.read() <= 0) && (em2.read() <= 0) && (stage == 1) && (z < cycle)) {


    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 0);
    stage = 0;
    motion_done = 0;
    z = z + 1;
  }

  // Undulation Code
  if ((g >= 200) && (command.equals("undulate")) && (em1.read() < dist1und) && (em2.read() < dist2und) && (motion_done == 0) && (z < cycle) && (stage == 0)) {
    digitalWrite(M2Ph, 1);
    analogWrite(M2En, 0);
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 200);
  }

  if ((command.equals("undulate")) && (em1.read() >= dist1und) && (em2.read() < dist2und) && (stage == 0) && (z < cycle)) {
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 1);
    analogWrite(M2En, 200);
  }


  if ((command.equals("undulate")) && (em1.read() >= dist1und) && (em2.read() >= dist2und) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 200);
    analogWrite(M2En, 0);
    stage = 1;
  }

  if ((command.equals("undulate")) && (em1.read() <= 0) && (em2.read() >= dist2und) && (stage == 1) && (z < cycle)) {
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 200);
    stage = 1;
  }

  if ((command.equals("undulate")) && (em1.read() <= 0) && (em2.read() <= 0) && (stage == 1) && (z < cycle)) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 0);
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 0);
    stage = 0;
    motion_done = 0;
    z = z + 1;
  }

  //End Loco Prime Codes

  if (z == cycle) {
    motion_done = 1;
  }


  res1_1 = 0;
  temp1_1 = 0;
  averageRes1_1 = 0;
  res1_2 = 0;
  temp1_2 = 0;
  averageRes1_2 = 0;
  res1_3 = 0;
  temp1_3 = 0;
  averageRes1_3 = 0;
  res2_1 = 0;
  temp2_1 = 0;
  averageRes2_1 = 0;
  res2_2 = 0;
  temp2_2 = 0;
  averageRes2_2 = 0;
  res2_3 = 0;
  temp2_3 = 0;
  averageRes2_3 = 0;
  i = 0;

  if ((motion_done == 1) || (command.equals("zero"))) {

    if (em1.read() > 25) {
      digitalWrite(M1Ph, 0);
      analogWrite(M1En, 50);
    }
    if (em1.read() < -25) {
      digitalWrite(M1Ph, 1);
      analogWrite(M1En, 50);
    }
    if (em2.read() > 25) {
      digitalWrite(M2Ph, 0);
      analogWrite(M2En, 50);
    }
    if (em2.read() < -25) {
      digitalWrite(M2Ph, 1);
      analogWrite(M2En, 50);
    }

    if ((em1.read() >= -25) && (em1.read() <= 25)) {
      digitalWrite(M1Ph, 0);
      analogWrite(M1En, 0);
    }
    if ((em2.read() >= -25) && (em2.read() <= 25)) {
      digitalWrite(M2Ph, 0);
      analogWrite(M2En, 0);
    }

    if (((em1.read() >= -25) && (em1.read() <= 25)) && ((em2.read() >= -25) && (em2.read() <= 25))) {
      motion_done = 0;
      analogWrite(M1En, 0);
      analogWrite(M2En, 0);
    }
  }

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    if (command.equals("stop")) {

      motion_done = 1;
      z = cycle;
    }
  }

  if ((z == cycle) && (motion_done = 0)) {
    command = 'reset';
    z = 0;
    g = 0;
  }

  if (command.equals("setP1")) {
    digitalWrite(M1Ph, 1);
    analogWrite(M1En, 50);
  }
  if (command.equals("setN1")) {
    digitalWrite(M1Ph, 0);
    analogWrite(M1En, 50);
  }

  if (command.equals("setP2")) {
    digitalWrite(M2Ph, 1);
    analogWrite(M2En, 50);
  }
  if (command.equals("setN2")) {
    digitalWrite(M2Ph, 0);
    analogWrite(M2En, 50);
  }


  Serial.print("Encoder 1    ");
  Serial.println(em1.read());
  Serial.print("Encoder 2    ");
  Serial.println(em2.read());

  delay(50);

}
