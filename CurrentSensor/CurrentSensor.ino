#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <SoftwareSerial.h>

#define motorPinRightDir 0   //D2
#define motorPinRightSpeed 5 //D1
#define motorPinLeftDir 2
#define motorPinLeftSpeed 4

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
float kp2;
float ki;
float kd;
String pay;
int rvelint = 0;
int rvelwrite = 0;
int rdir = 1;
bool rdirwrite = 1;
int lvelint = 0;
int lvelwrite = 0;
int ldir = 1;
bool ldirwrite = 1;
float e;
String mess;
String oldMess;

void setup()
{
    // put your setup code here, to run once:
    kp = 30;
    kp2 = 5;
    
    t = 0;
    Serial.begin(9600);
    swSer.begin(9600);

    pinMode(motorPinRightDir, OUTPUT);
    pinMode(motorPinRightSpeed, OUTPUT);
    pinMode(motorPinLeftDir, OUTPUT);
    pinMode(motorPinLeftSpeed, OUTPUT);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");

    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);

    while (!client.connected())
    {
        Serial.println("Connecting to MQTT...");

        if (client.connect("pubsubfollowme", mqttUser, mqttPassword))
        {

            Serial.println("connected");
        }
        else
        {

            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }

    client.publish("g3.vibestol@gmail.com/followme", "Hello from ESP8266");
    client.subscribe("g3.vibestol@gmail.com/followme");
}

char s2 = 'a';
void loop()
{
    digitalWrite(motorPinRightDir, !rdir);
    analogWrite(motorPinRightSpeed, rvelint);
    digitalWrite(motorPinLeftDir, !ldir);
    analogWrite(motorPinLeftSpeed, lvelint);
    t += 1;
    if (swSer.available())
    {
        //Serial.println("Coggers, Coggers!");
        if (t % 4 == 0)
        {
            swSer.println(Mode);
            Serial.println(Mode);
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
                errorString = swSer.readStringUntil('\n');
                e = errorString.toInt();
                regleraVaeg(e);
                Serial.println(rvelint);
            }
        }
    }
}

void regleraVaeg(int e)
{
  if (e == 727)
    {
        delta = 0;
        rvelint = 1024;
        rdir = 1;
        lvelint = 1024;
        ldir = 1;
    }
  else
    {
      delta = e;
      rvelint = 800 - kp2*delta;
      lvelint = 800 + kp2*delta;
      rdir = 1;
      ldir = 1;
    }
}

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
        rvelint = delta;
        lvelint = -delta;

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

void callback(char *topic, byte *payload, unsigned int length)
{

    pay = "";
    for (int i = 0; i < length; i++)
    {
        pay += (char)payload[i];
    }
}