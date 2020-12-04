/**
OCADu - Fall 2020 - Creation & Computation
Experiment 5
Simran Duggal, Greg Martin, and Mairead Stewart
submitted to Kate Hartman and Nick Puckett
December 4, 2020
**/

#include <WiFiNINA.h>
#define PubNub_BASE_CLIENT WiFiClient
#include <PubNub.h>
#include <ArduinoJson.h>
#include <SparkFunLSM6DS3.h>
#include "six_axis_comp_filter.h"
#include "Wire.h"

float pitch;
float roll;
float absPitch;
float absRoll;
bool allon;


// outgoing JSON variables
const char* myID = "Sim"; // UPDATE: place your name here, this will be put into your "sender" value for an outgoing messsage
const static char publishChannel[] = "Sim"; // UPDATE: channel to publish YOUR data
const static char readChannel1[] = "Mairead"; // UPDATE: channel to read THEIR data
const static char readChannel2[] = "Greg"; // UPDATE: channel to read THEIR data

//first, set your pins
int firstLights[] = {12};     //UPDATE: add all pins that should light when one person is avail
int secondLights[] = {11};  //UPDATE: add all add'l pins that should light when two people avail
int thirdLights[] = {10};   //UPDATE

const static char* teamInOrder[] = {publishChannel, readChannel1, readChannel2};
int teamStatuses[] = {0, 0, 0};

//Wifi details
extern char ssid[] = "Vipin Airtel"; //UPDATE
extern char pass[] = "@2181540"; //UPDATE
int status = WL_IDLE_STATUS;       // the Wifi radio's status

// Greg keys
//char pubkey[] = "pub-c-9e59df52-dcc5-420e-ac6d-9cef140407e9";
//char subkey[] = "sub-c-5793385a-3033-11eb-9d95-7ab25c099cb1";

// Sim pubnub keys
char pubkey[] = "pub-c-fe750cc5-db4e-433e-9916-785dddd8b6b0";
char subkey[] = "sub-c-8aa97738-3095-11eb-9713-12bae088af96";

// JSON variables
StaticJsonDocument<200> dataToSend; // The JSON from the outgoing message
StaticJsonDocument<200> inMessage; // JSON object for receiving the incoming values
//create the names of the parameters you will use in your message
String JsonParamName1 = "publisher";
String JsonParamName2 = "pitch";
unsigned long lastCheck; //time of last publish


int publishRate = 3000; //how often to publish the data to PN
unsigned long lastPublish; //time of last publish
int lastStatus = 0; //0 for busy, 1 for available
int thisStatus;     //current local status for comparison against last published status
int teamSize = 3;

LSM6DS3 myIMU(I2C_MODE, 0x6A); //Default constructor is I2C, addr 0x6B
CompSixAxis CompFilter(0.1, 2); //define the filter object

void setup() {
  
  Serial.begin(9600);

  connectToPubNub();
  
  for (int i = 0; i < sizeof(firstLights) /sizeof(firstLights[0]); i++) {
    pinMode(firstLights[i], OUTPUT);
  }
  for (int i = 0; i < sizeof(secondLights)/sizeof(secondLights[0]); i++) {
    pinMode(secondLights[i], OUTPUT);  
  }
  for (int i = 0; i < sizeof(thirdLights) /sizeof(thirdLights[0]); i++) {
    pinMode(thirdLights[i], OUTPUT);
  }
  
  myIMU.begin();


  
}

void loop() {
  
  //send and receive messages with PubNub, based on a timer
  sendReceiveMessages();
  updateStatus();
  //Serial.println(millis());
}

void sendReceiveMessages()
{
    //connect, publish new messages, and check for incoming
    if((millis() - lastCheck) >= publishRate) {
      
      //calculate local pitch and roll
      calculatePitchAndRoll();
      
      //publish data if local status has changed
      if (thisStatus != lastStatus) {
        sendMessage(); // publish this value to PubNub
        lastStatus = thisStatus;
      }
      
      //check for new incoming messages
      readMessage(readChannel1);
      readMessage(readChannel2);
      
      //update buffer variables
      lastCheck = millis();
    } else {
      
    }
}

void readMessage(const char channel[]) {
  String msg;
    auto inputClient = PubNub.history(channel,1);
    if (!inputClient) 
    {
        Serial.println("message error");
        delay(1000);
        return;
    }
    HistoryCracker getMessage(inputClient);
    while (!getMessage.finished()) 
    {
        getMessage.get(msg);
        
        //basic error check to make sure the message has content
        if (msg.length() > 0) 
        {
          Serial.print("**Received Message from ");
          Serial.print(channel);
          Serial.println(msg);
          //parse the incoming text into the JSON object

           deserializeJson(inMessage, msg); // parse the  JSON value received

           //read the values from the message and store them in local variables 
           const char* inMessagePublisher =  inMessage[JsonParamName1];      // other person's name
           int inMessageStatus =        int(inMessage[JsonParamName2]); // other person's status (0 or 1)
           for (int i = 0; i < teamSize; i++) {
              Serial.print(String(teamInOrder[i]));
              Serial.print("-----");
              Serial.print(String(inMessagePublisher));
              if (String(teamInOrder[i]) == String(inMessagePublisher)) {
                Serial.println(" TRUE");
                teamStatuses[i] = inMessageStatus;
              } else {
                Serial.println(" FALSE");
              }
              
           }
        }
    }
    inputClient->stop();
 
}

void calculatePitchAndRoll()
{
  float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle;       

  //  Get all motion sensor (in this case LSM6DS3) parameters,
  //  If you're using a different sensor you'll have to replace the values
  accelX = myIMU.readFloatAccelX();
  accelY = myIMU.readFloatAccelY();
  accelZ = myIMU.readFloatAccelZ();

  gyroX = myIMU.readFloatGyroX();
  gyroY = myIMU.readFloatGyroY();
  gyroZ = myIMU.readFloatGyroZ();

  // Convert these values into angles using the Complementary Filter
  CompFilter.CompAccelUpdate(accelX, accelY, accelZ); // takes arguments in m/s^2
  CompFilter.CompGyroUpdate(gyroX, gyroY, gyroZ); // takes arguments un rad/s 
  CompFilter.CompUpdate();
  CompFilter.CompStart();

  // Get angle relative to X and Y axes and write them to the variables in the arguments
  //in radians
  CompFilter.CompAnglesGet(&xAngle, &yAngle);

  //convert from radians to angles
  pitch = xAngle*RAD_TO_DEG;
  roll = yAngle*RAD_TO_DEG;
  absPitch = pitch;
  absRoll = roll;
  
  if (pitch > 180) {
    absPitch = 360 - pitch;
  }
  if (roll > 180) {
    absRoll = 360 - roll;
  }
  
  if (absPitch > 40) {
    thisStatus = 1;
    
  } else {
    thisStatus = 0;

  }

  teamStatuses[0] = thisStatus;

  Serial.print("New local reading: ");
  Serial.print(absPitch);
  Serial.print(" (");
  Serial.print(thisStatus);
  Serial.println(")");
  
}

void updateStatus() {
  //loop over status array and light box accordingly
  //sequential lighting: light the next set of lights
 //Serial.println("updateStatus");
  int j = 0; //number of lit levels
  bool allLit = true;
  
  //loop through everyone's status and light the correct number of lights
  for (int i = 0; i < sizeof(teamStatuses) / sizeof(teamStatuses[0]); i++) {
    if (teamStatuses[i] == 1) {      
      j++;
      
    } else {
      allLit = false;
    }
  }
  
  lightLevels(j);
 
  if (allLit) {
    completeMyAction();
  }
}

void lightLevels(int numLev) {
  //Serial.println("lightLevels");
  int firstLightLevel = 0;
  int secondLightLevel = 0;
  int thirdLightLevel = 0;

  if (numLev > 0) {
    firstLightLevel = 255;

    if (numLev > 1) {
      secondLightLevel = 255;

      if (numLev > 2) {
        thirdLightLevel = 255;
      }
    }
  }
  Serial.print("numLev: ");
  Serial.println(numLev);
  
  for (int i = 0; i < sizeof(firstLights) /sizeof(firstLights[0]); i++) {
    //Serial.println(firstLights[i]);
    digitalWrite(firstLights[i], firstLightLevel);
    //analogWrite(firstLights[i], firstLightLevel);  
  }
  for (int i = 0; i < sizeof(secondLights) /sizeof(secondLights[0]); i++) {
    //Serial.println(secondLights[i]);
    digitalWrite(secondLights[i], secondLightLevel);
    //analogWrite(secondLights[i], secondLightLevel);  
  }
  for (int i = 0; i < sizeof(thirdLights) /sizeof(thirdLights[0]); i++) {
    Serial.print(thirdLightLevel);
    Serial.println(thirdLights[i]);
    digitalWrite(thirdLights[i], thirdLightLevel);
    //analogWrite(thirdLights[i], thirdLightLevel);  
  }
}

void completeMyAction() {
  if (!allon) {
  for(int i=3; i<=12; i++) {
      digitalWrite(i,HIGH);
    }
     for(int i=3; i<=12; i++) {
      digitalWrite(i,LOW);
      delay(100);
      digitalWrite(i,HIGH);
    }
    for(int i=11; i>=4; i--) {
      digitalWrite(i,LOW);
      delay(100);
      digitalWrite(i,HIGH);
    }
    allon = false;
  }
  
  Serial.println("Everyone's free! Action!"); //

}

void connectToPubNub()
{
    // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to the network, SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    Serial.print("*");

    // wait 10 seconds for connection:
    delay(2000);
  }

  // once you are connected :
  Serial.println();
  Serial.print("You're connected to ");
  Serial.println(ssid);
  
  PubNub.begin(pubkey, subkey);
  Serial.println("Connected to PubNub Server");

  
}

void sendMessage() 
{
  char msg[64]; // variable for the JSON to be serialized into for your outgoing message
  
  // assemble the JSON to publish
  dataToSend["publisher"] = myID; // first key value is sender: yourName
  dataToSend["pitch"] = thisStatus; // second key value is the potiometer value: analogValue

  serializeJson(dataToSend, msg); // serialize JSON to send - sending is the JSON object, and it is serializing it to the char msg
  
  WiFiClient* client = PubNub.publish(publishChannel, msg); // publish the variable char 
  if (!client) 
  {
    Serial.println("publishing error"); // if there is an error print it out 
  }

}
