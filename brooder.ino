// This #include statement was automatically added by the Particle IDE.
#include "SparkJson.h"
#include "MQTT.h"
#include "Adafruit_DHT.h"

#define DHTPIN 0     // what pin we're connected to
#define DHTTYPE DHT22		// DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);

char mqttBroker[32] = "192.168.2.211";  //MQTT Broker URL / IP
String mqttPub = "brooder/data";            //MQTT Publication Channel
String mqttSub = "brooder/commands";        //MQTT Subscription Channel
String mqttLog = "log/";                    //MQTT logging channel
String value = "0";                         //Initialize reporting value
int wait = 2000;                            //Time between loops
String myID;                                //Variable for the Photon device ID
String strMqtt;                             //Variable to contain the MQTT string
int str_len;                                //Variable for String Length
int counter = 64;                           //Variable to count time for reporting
int reportDelay = 1000;                     //Time Between reports
int targetTemp = 95;                        //Brooder target targetTemp
int temp1 = 80;                             //DHT22 F temp
float h;                                    //Relative humidity
float f;                                    //Temperture F
float hi;                                   //Heat index
float dp;                                   //Dew Point
int rssi;                                   //RSSI strength variable
int led = D7;                               //Which LED to blink
int heatLampCon = 0;                        //Heat Pad On / Off
int heatLampTimer = 0;                      //Heat Pad Start Time
int heatLampRunTime = 0;                    //How long heat pad has been on
int lightState = 0;                         //Overhead light On / Off
int lastKnownTemp = 95;
int heaterState = 0;
int heaterTimer = 0;
int heaterRunTime = 0;
int fanState = 0;
int heatLampState = 0;

int heater = D1;
int lights = D2;
int heatLamp = D3;
int fan = D4;

MQTT client(mqttBroker, 1883, callback);    //Initialized MQTT broker

void setup()
        {

        dht.begin();

        // register the cloud function
        Particle.function("changeTemp", changeTemp);
        Particle.variable("targetTemp", targetTemp);
        Particle.function("heatLampFunc", heatLampFunc);
        Particle.variable("heatLampCon", heatLampCon);
        Particle.function("light", light);

        Serial.begin(9600);

        //Get the deviceID
        myID = System.deviceID();
        strMqtt = mqttLog;
        str_len = myID.length() + 1;
        char char_myID[str_len];
        myID.toCharArray(char_myID, str_len);

        //Create json status object
        StaticJsonBuffer<200> jsonBuffer;
        char buffer [200];

        JsonObject& root = jsonBuffer.createObject();
        root["deviceID"] = char_myID;
        root["status"] = "Connected at startup";

        //Publish JsonObject
        root.printTo(Serial);
        root.printTo(buffer, sizeof(buffer));

        // connect to the MQTT broker
        client.connect("connect");

        //Publish our status json to the broker
        client.publish(strMqtt, buffer);

        //Subscribe to the broker to recieve messages
        client.subscribe(mqttSub);

        //Set pin modes
        pinMode(led, OUTPUT);
        pinMode(heater, OUTPUT);
        pinMode(lights, OUTPUT);
        pinMode(heatLamp, OUTPUT);
        pinMode(fan, OUTPUT);

        digitalWrite(heater, HIGH);  //Heater
        digitalWrite(lights, HIGH);  //Heat Pad
        digitalWrite(heatLamp, HIGH);  //Light
        digitalWrite(fan, HIGH);  //Exahust Fan
    }


void loop()
    {
            lastKnownTemp = temp1;

            readSensor();

            fanControlFunc();

            heaterControlFunc();
            heaterTimerFunc();

            heatLampConFunc();
            heatLampTimerFunc();

            lightControlFunc();

            report(temp1, targetTemp, heatLampCon, heatLampRunTime,
                lightState, h, hi, dp, heaterState, heaterRunTime,
                mqttPub);

            client.loop();

            delay(wait);
    }

// targetTemp Cloud Function
int changeTemp(String command)
    {
        targetTemp = command.toInt();

        return 1;
    }

// Heat Pad Cloud Function
int heatLampFunc(String command)
    {
        heatLampCon = command.toInt();

        return 1;
    }

// Light Control Cloud Function
int light(String command)
{
    lightState = command.toInt();

    return 1;
}

//Function handles reporting to the MQTT broker
void report(int temp1, int targetTemp, int heatLampCon, int heatLampRunTime,
    int lightState, int h, int hi, int dp, int heaterState, int heaterRunTime, String feed)
    {
        int str_len;

        String stringtemp1;
        stringtemp1 = String(temp1);
        str_len = stringtemp1.length() + 1;
        char char_temp1[str_len];
        stringtemp1.toCharArray(char_temp1, str_len);

        String stringTargetTemp;
        stringTargetTemp = String(targetTemp);
        str_len = stringTargetTemp.length() + 1;
        char char_targetTemp[str_len];
        stringTargetTemp.toCharArray(char_targetTemp, str_len);

        String stringHeatLampCon;
        stringHeatLampCon = String(heatLampCon);
        str_len = stringHeatLampCon.length() + 1;
        char char_heatLampCon[str_len];
        stringHeatLampCon.toCharArray(char_heatLampCon, str_len);

        String stringHeatLampState;
        stringHeatLampState = String(heatLampState);
        str_len = stringHeatLampState.length() + 1;
        char char_heatLampState[str_len];
        stringHeatLampState.toCharArray(char_heatLampState, str_len);

        String stringheatLampRunTime;
        stringheatLampRunTime = String(heatLampRunTime);
        str_len = stringheatLampRunTime.length() + 1;
        char char_heatLampRunTime[str_len];
        stringheatLampRunTime.toCharArray(char_heatLampRunTime, str_len);

        String stringLightState;
        stringLightState = String(lightState);
        str_len = stringLightState.length() + 1;
        char char_lightState[str_len];
        stringLightState.toCharArray(char_lightState, str_len);

        String stringH;
        stringH = String(h);
        str_len = stringH.length() + 1;
        char char_h[str_len];
        stringH.toCharArray(char_h, str_len);

        String stringHI;
        stringHI = String(hi);
        str_len = stringHI.length() + 1;
        char char_HI[str_len];
        stringHI.toCharArray(char_HI, str_len);

        String stringDP;
        stringDP = String(dp);
        str_len = stringDP.length() + 1;
        char char_DP[str_len];
        stringDP.toCharArray(char_DP, str_len);

        String stringHeaterState;
        stringHeaterState = String(heaterState);
        str_len = stringHeaterState.length() + 1;
        char char_heaterState[str_len];
        stringHeaterState.toCharArray(char_heaterState, str_len);

        String stringHeaterRunTime;
        stringHeaterRunTime = String(heaterRunTime);
        str_len = stringHeaterRunTime.length() + 1;
        char char_heaterRunTime[str_len];
        stringHeaterRunTime.toCharArray(char_heaterRunTime, str_len);

        String stringFanState;
        stringFanState = String(fanState);
        str_len = stringFanState.length() + 1;
        char char_fanState[str_len];
        stringFanState.toCharArray(char_fanState, str_len);

        rssi = WiFi.RSSI();
        String stringRssi;
        stringRssi = String(rssi);
        str_len = stringRssi.length() + 1;
        char char_rssi[str_len];
        stringRssi.toCharArray(char_rssi, str_len);

        str_len = myID.length() + 1;
        char char_myID[str_len];
        myID.toCharArray(char_myID, str_len);

        //Build json REPORT object
        StaticJsonBuffer<600> jsonBuffer;
        char bufferReport [600];

        JsonObject& root = jsonBuffer.createObject();
        root["deviceID"] = char_myID;
        root["temp1"] = char_temp1;
        root["targetTemp"] = char_targetTemp;
        root["heatLampCon"] = char_heatLampCon;
        root["heatLampState"] = char_heatLampState;
        root["heatLampRunTime"] = char_heatLampRunTime;
        root["lightState"] = char_lightState;
        root["relHumidity"] = char_h;
        root["heatIndex"] = char_HI;
        root["dewPoint"] = char_DP;
        root["heaterState"] = char_heaterState;
        root["heaterRunTime"] = char_heaterRunTime;
        root["fanState"] = char_fanState;
        root["rssi"] = char_rssi;

        //root.printTo(Serial);
        root.printTo(bufferReport, sizeof(bufferReport));
        client.connect("connect");

        if (client.isConnected())
            {
                client.publish(feed,bufferReport);
                blink(3);

            }
        }

// Allows us to recieve a message from the subscription
void callback(char* topic, byte* payload, unsigned int length)
    {
        char p[length + 1];
        memcpy(p, payload, length);
        p[length] = NULL;
        String message(p);

    //This is where you put code to handle any message recieved from the broker

    }

void blink(int blinks)
    {

        int x = 0;

        do
        {
          digitalWrite(led, HIGH);
          delay(100);
          digitalWrite(led, LOW);
          delay(100);
          x = x + 1;

        } while (x < blinks);
    }

int readSensor()
    {
        h = dht.getHumidity();
    	  f = dht.getTempFarenheit();

        hi = fConvert(dht.getHeatIndex());
        dp = fConvert(dht.getDewPoint());
        temp1 = f;

        //QA the sensor data, it occasionally throws railed values
        if ((temp1 > (lastKnownTemp + 30)) or (temp1 < (lastKnownTemp - 30)))
            {
                temp1 = lastKnownTemp;
            }
        else
            {
                lastKnownTemp = temp1;
            }

    	return 1;
    }

int fConvert(float t)
    {
        int result;
        t = (t * 1.8) + 32;
        result = t;

        return result;
    }

void heaterTimerFunc()
    {
        if (heaterState == 1 and heaterTimer == 0)
            {
                heaterTimer = millis();
            }

        //Reset Heater Timer
        if (heaterTimer > 0 and heaterState == 0 )
            {
                heaterTimer = 0;
                heaterRunTime = 0;
            }

        //Calculate heat pad run time
        if (heaterState == 1 and heaterTimer > 0)
            {
                heaterRunTime = millis() - heaterTimer;
            }
    }

void heatLampTimerFunc()
    {
        if (heatLampCon == 1 and heatLampTimer == 0)
            {
                heatLampTimer = millis();
            }

        //Reset Heat Pad Timer
        if (heatLampTimer > 0 and heatLampCon == 0 )
            {
                heatLampTimer = 0;
                heatLampRunTime = 0;
            }

        //Calculate heat pad run time
        if (heatLampCon == 1 and heatLampTimer > 0)
            {
                heatLampRunTime = millis() - heatLampTimer;
            }
    }

void fanControlFunc()
    {
        //Exhaust fan control
        if (temp1 > targetTemp)
            {
                digitalWrite(fan, LOW);
                fanState = 1;
            }
        else
            {
                digitalWrite(fan, HIGH);
                fanState = 0;
            }

    }

void heaterControlFunc()
    {
        //Heater Control
        if (temp1 < targetTemp)
            {
                digitalWrite(heater, LOW);
                heaterState = 1;
            }
        else
            {
                digitalWrite(heater, HIGH);
                heaterState = 0;
            }

    }

void heatLampConFunc()
    {
        //Heating pad Control
        if (temp1 > targetTemp)
            {
                digitalWrite(heatLamp, HIGH);
                heatLampState = 0;
            }

        if ((temp1 < targetTemp) and heaterRunTime > 300000)
            {
                digitalWrite(heatLamp, LOW);
                heatLampState = 1;
            }
    }

void lightControlFunc()
    {
        //Light Control
        if (lightState == 0)
            {
                digitalWrite(lights, HIGH);
            }

        if (lightState == 1)
            {
                digitalWrite(lights, LOW);
            }
    }
