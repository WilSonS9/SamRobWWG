#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <SoftwareSerial.h>

#define motorPinRightDir  0//D2
#define motorPinRightSpeed 5//D1
#define motorPinLeftDir 2
#define motorPinLeftSpeed 4

const char* ssid = "ABB_Gym_IOT";
const char* password =  "Welcome2abb";

const char* mqttServer = "maqiatto.com";
const int mqttPort = 1883;
const char* mqttUser = "g3.vibestol@gmail.com";
const char* mqttPassword = "G3Vibestol2020";

String Sub = "g3.vibestol@gmail.com/followme";

WiFiClient espClient;
PubSubClient client(espClient);

SoftwareSerial swSer(13, 15, false, 256);

int t;
float delta;
String errorString;
float kp;
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

void setup() {
  kp = 0.3;
  // put your setup code here, to run once:
  t = 0;
  Serial.begin(9600);
  swSer.begin(9600);

  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("pubsubfollowme", mqttUser, mqttPassword )) {
 
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
 
  client.publish("g3.vibestol@gmail.com/followme", "Hello from ESP8266");
  client.subscribe("g3.vibestol@gmail.com/followme");
}

char s2 = 'a';
void loop() {
  client.loop();
  rvelwrite = rvelint * rdir;
  lvelwrite = lvelint * ldir;
  digitalWrite(motorPinRightDir, rdirwrite);
  analogWrite(motorPinRightSpeed, 1024);
  digitalWrite(motorPinLeftDir, ldirwrite);
  analogWrite(motorPinLeftSpeed, 1024);
  t += 1;
  if (swSer.available()){
     if (t % 10 == 0) {
      errorString = swSer.readStringUntil('\n');
      e = errorString.toInt();

      if (e < -20) {
        client.publish("g3.vibestol@gmail.com/followme", "turn;left");
      } else if (e > 20) {
        client.publish("g3.vibestol@gmail.com/followme", "turn;right");
      } else if (e >= -20 && e <= 20 ){
        client.publish("g3.vibestol@gmail.com/followme", "turn;straight");
      } else {
        client.publish("g3.vibestol@gmail.com/followme", "error;continue");
      }
    

      delta = kp*e;
      rvelint += delta;
      lvelint -= delta;

      if (lvelint < 0){
        ldir = -1;
        ldirwrite = 0;
      } else{
        ldir = 1;  
        ldirwrite = 1;
      }

      if (rvelint < 0){
        rdir = -1;  
        rdirwrite = 0;
      } else{
        rdir = 1;  
        rdirwrite = 1;
      }

      Serial.println(rvelwrite);
      // swSer.print(s1);
      // Serial.println(e);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  pay = "";
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    pay += (char)payload[i];
  }
    Serial.println(pay);
  Serial.println();
  Serial.println("-----------------------");
 
}
