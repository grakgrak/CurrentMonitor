#include <Arduino.h>
#include <ArduinoOTA.h> // https://github.com/esp8266/Arduino/blob/master/libraries/ArduinoOTA/ArduinoOTA.h
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include "..\..\Credentials.h" // contains definitions of WIFI_SSID and WIFI_PASSWORD
#include "Irms_Calc.h"

#define HOSTNAME "CurrentMonitor"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define WIFI_CONNECT_TIMEOUT 15000
#define MQTT_HOST IPAddress(192, 168, 1, 210)
#define MQTT_PORT 1883
#define PUBLISH_FREQ_SECS 10

ACS712_Irms sensor;

AsyncMqttClient mqttClient;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;
Ticker mqttPublishTimer;

// Voltage divider details
const float R1 = 0;  // Fixed resistor next to thermistor. 0 if not required.
const float R2 = 6.8;  // Fixed resistor on other side of divider
const float Vcc = 3.3;  // Input voltage

// Get three measurements like below, then use
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
//  100.0C = 6.7kohm
//   34.4C = 60.7kohm
// -13.0C = 624.0kohm
const float BETA = 3855;
const float NOM_RESIST = 90496;
const float NOM_TEMP = 25;

class ArithmeticMean
{
private:
    double ALPHA = 0.45;
    double _average = 0.0;
public:
    ArithmeticMean(double alpha)
    {
        ALPHA = alpha;
    }

    void Update(double val)
    {
        _average = ALPHA * val + (1.0 - ALPHA) * _average;
    }
    double Average() { return _average; }
};

//--------------------------------------------------------------------
ArithmeticMean sensorValueAM(0.45);
double sensorValue;

ArithmeticMean averageWattsAM(0.10);

//--------------------------------------------------------------------
float resistanceToCelsius(float resistance) {
  // Simplified Steinhart-Hart
  float temp = log(resistance / NOM_RESIST) / BETA;
  temp += 1.0 / (NOM_TEMP + 273.15);
  temp = 1 / temp;
  temp -= 273.15; // convert to C
  return temp;
}

//--------------------------------------------------------------------
void init_OTA()
{
    Serial.println("init OTA");

    // ArduinoOTA callback functions
    ArduinoOTA.onStart([]() {
        Serial.println("OTA starting...");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("OTA done.Reboot...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int prevPcnt = 100;
        unsigned int pcnt = (progress / (total / 100));
        unsigned int roundPcnt = 5 * (int)(pcnt / 5);
        if (roundPcnt != prevPcnt)
        {
            prevPcnt = roundPcnt;
            Serial.println("OTA upload " + String(roundPcnt) + "%");
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.print("OTA Error " + String(error) + ":");
        const char *line2 = "";
        switch (error)
        {
        case OTA_AUTH_ERROR:
            line2 = "Auth Failed";
            break;
        case OTA_BEGIN_ERROR:
            line2 = "Begin Failed";
            break;
        case OTA_CONNECT_ERROR:
            line2 = "Connect Failed";
            break;
        case OTA_RECEIVE_ERROR:
            line2 = "Receive Failed";
            break;
        case OTA_END_ERROR:
            line2 = "End Failed";
            break;
        }
        Serial.println(line2);
    });

    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword(HOSTNAME);

    ArduinoOTA.begin();
}

//------------------------------------------------------------------------
void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.enableSTA(true);
    WiFi.hostname(HOSTNAME);
    WiFi.begin(ssid, password);
}

//------------------------------------------------------------------------
void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

//------------------------------------------------------------------------
void publishSensorValue()
{
    double value = sensorValueAM.Average();
    averageWattsAM.Update(value);

    String amps = String(value);
    String watts = String(value * 240.0);
    String average =  String( averageWattsAM.Average() * 240.0);
    
    mqttClient.publish("Current/amps", 0, true, amps.c_str());
    mqttClient.publish("Current/watts", 0, true, watts.c_str());
    mqttClient.publish("Current/average", 0, true, average.c_str());
}

//------------------------------------------------------------------------
void onMqttConnect(bool sessionPresent)
{
    Serial.println("Connected to MQTT.");

    // attach the publisher
    mqttPublishTimer.attach(PUBLISH_FREQ_SECS, publishSensorValue);
}

//------------------------------------------------------------------------
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");

    mqttPublishTimer.detach(); // stop publishing

    if (WiFi.isConnected())
        mqttReconnectTimer.once(2, connectToMqtt);
}

//------------------------------------------------------------------------
void init_mqtt()
{
    mqttClient.setServer(MQTT_HOST, MQTT_PORT)
        .onConnect(onMqttConnect)
        .onDisconnect(onMqttDisconnect);

    connectToWifi();
}
//------------------------------------------------------------------------
void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case WIFI_EVENT_STAMODE_GOT_IP:
        Serial.print("IP Address" );
        Serial.println( WiFi.localIP());

        connectToMqtt();
        break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
        mqttReconnectTimer.detach();
        wifiReconnectTimer.once(2, connectToWifi);
        break;
    default:
        break;
    }
}

//------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(10);
    //Serial.setDebugOutput(true);
    Serial.println("\nSetup");

    WiFi.onEvent(WiFiEvent);

    init_OTA();
    init_mqtt();

    Serial.println("Setup Done.");
}

void loop()
{
    // read the sensor
    sensorValue = sensor.Process();

    Serial.println(sensorValue);

    sensorValueAM.Update(sensorValue);
    
    ArduinoOTA.handle();

    delay(600);
}