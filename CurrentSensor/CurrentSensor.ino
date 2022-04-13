#include <ESP8266WiFi.h>
#include <PubSubClient.h>



#include <SoftwareSerial.h>



#define motorPinRightDir 0 //D2
#define motorPinRightSpeed 5 //D1
#define motorPinLeftDir 2
#define motorPinLeftSpeed 4
#define leftSensor 14 //D5
#define rightSensor 12 //D6



const char *ssid = "ABBgym_2.4";
const char *password = "mittwifiarsabra";



const char *mqttServer = "maqiatto.com";
const int mqttPort = 1883;
const char *mqttUser = "g3.vibestol@gmail.com";
const char *mqttPassword = "G3Vibestol2020";



String Sub = "g3.vibestol@gmail.com/followme";



WiFiClient espClient;
PubSubClient client(espClient);



SoftwareSerial swSer(13, 15, false, 256);



int t;
float delta;
String Mode = "Vaeg";
String errorString;
float kp;
float kpl;
float kpr;
float ki2;
float kdl;
float kdr;
String pay;

int leftOffset = 25;


int t2 = 0;



float leftCounter = 0;
float rightCounter = 0;
float ut = 0.0042;
float deck = 3.7 * 3.142;
float last_time = 0;
bool timer = 0;
int rvelint = 0;
int rvelwrite = 0;
float dT = 500;



float dsL = 0;
float dsR = 0;
float sL = 0;
float sR = 0;

float errL = 0;
float errR = 0;
float errSumL = 0;
float errSumR = 0;
float prevL = 0;
float prevR = 0;
float borL = 4.0;
float borR = 4.0;
float leftVel = 0;
float rightVel = 0;



int rdir = 1;
bool rdirwrite = 1;
int lvelint = 0;
int lvelwrite = 0;
int ldir = 1;
bool ldirwrite = 1;
float e;
String mess;
String oldMess;

String turnString;
String vaegTurns = "0000";
int turnInt;


void setup()
{
  // put your setup code here, to run once:
  kp = 10;
  kpl = 55;
  kpr = 65;
  ki2 = 5;
  kdl = 60;
  kdr = 80;

  t = 0;
  Serial.begin(9600);
  swSer.begin(9600);



  pinMode(motorPinRightDir, OUTPUT);
  pinMode(motorPinRightSpeed, OUTPUT);
  pinMode(motorPinLeftDir, OUTPUT);
  pinMode(motorPinLeftSpeed, OUTPUT);
  pinMode(leftSensor, INPUT_PULLUP);
  pinMode(rightSensor, INPUT_PULLUP);



  attachInterrupt(digitalPinToInterrupt(leftSensor), IntCallbackL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightSensor), IntCallbackR, RISING);



  WiFi.begin(ssid, password);



  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");



  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);



  // while (!client.connected())
  // {
  // Serial.println("Connecting to MQTT...");



  // if (client.connect("pubsubfollowme", mqttUser, mqttPassword))
  // {



  // Serial.println("connected");
  // }
  // else
  // {



  // Serial.print("failed with state ");
  // Serial.print(client.state());
  // delay(2000);
  // }
  // }



  // client.publish("g3.vibestol@gmail.com/followme", "Hello from ESP8266");
  // client.subscribe("g3.vibestol@gmail.com/followme");
}



char s2 = 'a';
void loop()
{
  digitalWrite(motorPinRightDir, rdir);
  analogWrite(motorPinRightSpeed, rvelint);
  digitalWrite(motorPinLeftDir, ldir);
  analogWrite(motorPinLeftSpeed, lvelint);
  pinMode(leftSensor, INPUT_PULLUP);
  pinMode(rightSensor, INPUT_PULLUP);
  t += 1;
  if (swSer.available())
  {
    //Serial.println("Coggers, Coggers!");
    if (t % 4 == 0)
    {
      swSer.println(Mode);
      //Serial.println(Mode);
      if (Mode == "Hinder")
      {
        errorString = swSer.readStringUntil('\n');
        e = errorString.toInt();
        regleraHinder(e);
        Serial.print(rvelint);
        Serial.print(' ');
        Serial.println(rdir);
      }
      else if (Mode == "Vaeg")
      {
        regleraVaegVision(e);
        //Serial.println(rvelint);
      }
    }
  }
}

void regleraMotor(float bor, int dir, bool left)
{
    if (left){
      digitalWrite(motorPinLeftDir, dir);
      analogWrite(motorPinLeftSpeed, lvelint);
      prevL = leftVel;
      dsL = leftCounter / 2 * ut * deck;
      sL += dsL;
      leftVel = 1000 * dsL / dT;
      last_time = millis();
      errL = bor - leftVel;
      lvelint += kpl * errL + kdl * (prevL - leftVel);
      if (lvelint >= 1023)
      {
        lvelint = 1023;
      }
      leftCounter = 0;
  }
  else {
    digitalWrite(motorPinRightDir, dir);
    analogWrite(motorPinRightSpeed, rvelint);
      prevR = rightVel;
      dsR = rightCounter / 2 * ut * deck;
      sR += dsR;
      rightVel = 1000 * dsR / dT;
      last_time = millis();
      errR = bor - rightVel;
      rvelint += kpr * errR + kdr * (prevR - rightVel);
     
      if (rvelint >= 1023)
      {
        rvelint = 1023;
      }
      rightCounter = 0;
    }
}

void regleraIdSvaeng()
{
  borL = 3.0;
  borR = 3.0;
  sL = 0.0;
  sR = 0.0;
  lvelint = 600;
  ldir = 0;
  rdir = 1;
  rvelint = 600;
  
  // Turn left 90 degrees, by turning right motor backwards, and left motor forwards at same speed. 

  for (int r; r < 4; r++)
  {
    while ((sL + sR)/2 <= 3.1 * 3.14159)
    {
      millis_check(last_time);
      if (timer == true)
      {
        regleraMotor(borL, ldir, true);
        regleraMotor(borR, rdir, false);
      }
    }

    digitalWrite(motorPinRightDir, 1);
    analogWrite(motorPinRightSpeed, 0);
    digitalWrite(motorPinLeftDir, 1);
    analogWrite(motorPinLeftSpeed, 0);

    swSer.println("Typ");
    turnString = swSer.readStringUntil('\n');
    vaegTurns[r] = turnString.charAt(0);

    // Here will komma lite kode om the riktning of the aztekare...
    // And here wi will send lite information to the eldbase...
  }
  Serial.println(vaegTurns);
}



void regleraVaegVision(int e)
{
  float s_y = 0.0;
  ldir = 1;
  rdir = 1;
  lvelint = 800;
  rvelint = 800;

  while (s_y < 25.4) {
    digitalWrite(motorPinRightDir, rdir);
    analogWrite(motorPinRightSpeed, rvelint);
    digitalWrite(motorPinLeftDir, ldir);
    analogWrite(motorPinLeftSpeed, lvelint);



    millis_check(last_time);
    if (timer == true) {
      dsL = leftCounter / 2 * ut * deck;
      dsR = rightCounter / 2 * ut * deck;
      float ds_tot = dsL + dsR;
      float ds_x = dsL - dsR;
      float ds_y = sqrt(pow(ds_tot, 2) - pow(ds_x, 2));
      s_y += ds_y;
      sL += dsL;
      sR += dsR;
      last_time = millis();
      if (lvelint >= 1023) {
        lvelint = 1023;
      }
      if (rvelint >= 1023) {
        rvelint = 1023;
      }
      leftCounter = 0;
      rightCounter = 0;
    }



    errorString = swSer.readStringUntil('\n');
    e = errorString.toInt();
    if (e == 727)
    {
      lvelint = rvelint - leftOffset;
    }
    else {
      delta = kp * e;
      rvelint = 800 - delta;
      lvelint = 800 + delta;
    }
    Serial.print("sR: ");
    Serial.print(sR);
    Serial.print(",");
    Serial.print("sL: ");
    Serial.println(sL);
    yield();
  }
  Serial.println("Stopping!!!!!");
  digitalWrite(motorPinRightDir, 1);
  analogWrite(motorPinRightSpeed, 0);
  digitalWrite(motorPinLeftDir, 1);
  analogWrite(motorPinLeftSpeed, 0);
  
}


/*
void regleraVaeg(int e)
{
  borL = 5.0;
  borR = 5.0;
  sL = 0.0;
  sR = 0.0;
  lvelint = 1023 - leftOffset;
  ldir = 1;
  rdir = 1;
  rvelint = 1023;
  while ((sR + sL) < 50.9)
  {
    digitalWrite(motorPinRightDir, rdir);
    analogWrite(motorPinRightSpeed, rvelint);
    digitalWrite(motorPinLeftDir, ldir);
    analogWrite(motorPinLeftSpeed, lvelint);
    millis_check(last_time);
    if (timer == true) {
      prevL = leftVel;
      prevR = rightVel;
      dsL = leftCounter / 2 * ut * deck;
      dsR = rightCounter / 2 * ut * deck;
      sL += dsL;
      sR += dsR;
      leftVel = 1000 * dsL / dT;
      rightVel = 1000 * dsR / dT;
      last_time = millis();
      errL = borL - leftVel;
      errR = borR - rightVel;
      // errSumL += errL;
      // errSumR += errR;
      //lvelint += kpl * errL + kdl * (prevL - leftVel);
      //rvelint += kpr * errR + kdr * (prevR - rightVel);
      lvelint = 1023 - leftOffset;
      rvelint = 1023;
      if (lvelint >= 1023) {
        lvelint = 1023;
      }
      if (rvelint >= 1023) {
        rvelint = 1023;
      }
      leftCounter = 0;
      rightCounter = 0;
    }
    Serial.print("rightVel: ");
    Serial.print(rightVel);
    Serial.print(",");
    Serial.print("leftVel: ");
    Serial.print(leftVel);
    Serial.print(",");
    Serial.print("borR: ");
    Serial.print(borR);
    Serial.print(",");
    Serial.print("borL: ");
    Serial.println(borL);
    yield();
  }



  Serial.println("Stopping!!!!!");
  digitalWrite(motorPinRightDir, 1);
  analogWrite(motorPinRightSpeed, 0);
  digitalWrite(motorPinLeftDir, 1);
  analogWrite(motorPinLeftSpeed, 0);
  delay(5000);



  //Serial.print("errSumL: ");
  //Serial.println(errSumL);
  //Serial.print("errL: ");
  //Serial.println(errL);
  //Serial.print("borL: ");
  //Serial.println(borL);
}
*/


void regleraHinder(int e)
{
  oldMess = mess;



  if (e == 727)
  {
    mess = "stand;still";
    delta = 0;
    rvelint = 0;
    rdir = 1;
    lvelint = 0;
    ldir = 1;
  }
  else if (e >= -60 && e <= 60)
  {
    mess = "turn;straight";
    delta = 0;
    rvelint = 1024;
    rdir = 1;
    lvelint = 1024;
    ldir = 1;
  }
  else
  {
    if (e < -60)
    {
      mess = "turn;left";
    }
    else if (e > 60)
    {
      mess = "turn;right";
    }
    else
    {
      mess = "error;continue";
    }



    delta = kp * e;
    rvelint = -delta;
    lvelint = delta;



    if (lvelint < 0)
    {
      ldir = 0;
      lvelint *= -1;
    }
    else
    {
      ldir = 1;
    }



    if (rvelint < 0)
    {
      rdir = 0;
      rvelint *= -1;
    }
    else
    {
      rdir = 1;
    }
  }
  Serial.println(mess);
  if (oldMess != mess)
  {
    char buf[60];
    mess.toCharArray(buf, mess.length() + 1);
    client.publish("g3.vibestol@gmail.com/followme", buf);
  }
}



ICACHE_RAM_ATTR void IntCallbackL() {
  leftCounter += 1;
}



ICACHE_RAM_ATTR void IntCallbackR() {
  rightCounter += 1;
}



bool millis_check(int last) {
  if ((millis() - last) >= dT) {
    timer = true;
  } else {
    timer = false;
  }
}



void callback(char *topic, byte *payload, unsigned int length)
{



  pay = "";
  for (int i = 0; i < length; i++)
  {
    pay += (char)payload[i];
  }
}
