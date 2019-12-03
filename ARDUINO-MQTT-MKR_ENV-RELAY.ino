#include <FlashAsEEPROM.h>
#include <FlashStorage.h>
#include <Adafruit_SleepyDog.h>
#include <Arduino_MKRENV.h>
#include <WiFi101.h>
#include <RTCZero.h>
#include <MQTT.h>
#include <MQTTClient.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
int status = WL_IDLE_STATUS;
char mqttHost[] = SECRET_MQTT_HOST;
char mqttClientId[] = SECRET_MQTT_CLIENT_ID;
//long randNumber;
char mqttUser[] = SECRET_MQTT_USER;
char mqttPass[] = SECRET_MQTT_PASS;
unsigned long lastMillis = 0;

WiFiSSLClient net;
MQTTClient mqttClient;
RTCZero rtc; // create an RTC object

const int GMT = 2; //change this to adapt it to your time zone

//FlashStorage(storage, "env_readings.csv");

void setRTCwithNTP()
{
    unsigned long epoch;
    int numberOfTries = 0, maxTries = 6;
    do
    {
        Serial.print("getting time...");
        epoch = WiFi.getTime();
        delay(1000);
        Serial.println("done");
        numberOfTries++;
    } while ((epoch == 0) || (numberOfTries < maxTries));

    if (numberOfTries > maxTries)
    {
        Serial.print("NTP unreachable!!");
        WiFi.disconnect();
    }
    else
    {
        Serial.print("Epoch received: ");
        Serial.println(epoch);
        rtc.setEpoch(epoch);

        Serial.println();
    }
    rtc.setHours(rtc.getHours() + GMT);
}

void printTime()
{
    print2digits(rtc.getHours());
    Serial.print(":");
    print2digits(rtc.getMinutes());
    Serial.print(":");
    print2digits(rtc.getSeconds());
    Serial.println();
}

void printDate()
{
    print2digits(rtc.getDay());
    Serial.print("/");
    print2digits(rtc.getMonth());
    Serial.print("/");
    print2digits(rtc.getYear());

    Serial.println("");
}

void print2digits(int number)
{
    if (number < 10)
    {
        Serial.print("0");
    }
    Serial.print(number);
}

void printWiFiStatus()
{
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void printReading(float temperature,
                  float humidity,
                  float pressure,
                  float illuminance,
                  float uva,
                  float uvb,
                  float uvIndex)
{
    printTime();

    // print each of the sensor values
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidity    = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Pressure    = ");
    Serial.print(pressure);
    Serial.println(" kPa");

    Serial.print("Illuminance = ");
    Serial.print(illuminance);
    Serial.println(" lx");

    Serial.print("UVA         = ");
    Serial.println(uva);

    Serial.print("UVB         = ");
    Serial.println(uvb);

    Serial.print("UV Index    = ");
    Serial.println(uvIndex);
    // print an empty line
    Serial.println();
}

void connectMqttServer()
{
    Serial.print("checking wifi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    /*
    randomSeed(analogRead(0));
    randNumber = random(300);
    char mqttClientId[] = String(randNumber);
    */
    // MQTT client connection request
    mqttClient.begin(mqttHost, 1883, net);
    Serial.print("\nconnecting to MQTT server...");
    while (!mqttClient.connect(mqttClientId))
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nconnected!");

    mqttClient.subscribe("/homie/myHouse/livingroom/light1/#");
    mqttClient.subscribe("/homie/myHouse/livingroom/light2/#");
}

void messageReceived(String &topic, String &payload)
{
    Serial.println("incoming: " + topic + " - " + payload);

    if(topic.indexOf("/light1/set") > 0) {
        if (payload == "true") {
            digitalWrite(1, HIGH);
        } else {
            digitalWrite(1, LOW);
        }
    }

    if (topic.indexOf("/light2/set") > 0) {
        if (payload == "true") {
            digitalWrite(2, HIGH);
        } else {
            digitalWrite(2, LOW);
        }      
    }

}

void setup()
{
    // initialize digital pins 1 and 2 as an output for relay control.
    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);

    int watch_dog = Watchdog.enable(180000);
    Serial.begin(9600);
    while (!Serial)
        ;

    Serial.print("Enabled the watchdog with max countdown of ");
    Serial.print(watch_dog, DEC);
    Serial.println(" milliseconds!");

    if (!ENV.begin())
    {
        Serial.println("Failed to initialize MKR ENV shield!");
        while (1)
            ;
    }
    Watchdog.reset();

    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD)
    {
        Serial.println("WiFi shield not present");
        // don't continue:
        while (true)
            ;
    }
    Watchdog.reset();

    // attempt to connect to WiFi network:
    while (status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);

        // wait 10 seconds for connection:
        delay(10000);
        Watchdog.reset();
    }
    printWiFiStatus(); // you're connected now, so print out the status:

    rtc.begin();     // initialize the RTC library
    Watchdog.reset();
    setRTCwithNTP(); // set the RTC time/date using epoch from NTP
    printTime();     // print the current time
    printDate();     // print the current date
    Watchdog.reset();

    // MQTT client connection
    mqttClient.begin(mqttHost, net);
    mqttClient.onMessage(messageReceived);
    connectMqttServer();
    Watchdog.reset();
}

void loop()
{
    // MQTT client:
    mqttClient.loop();

    if (!mqttClient.connected())
    {
        connectMqttServer();
    }

    // publish a message roughly every second.
    if (millis() - lastMillis > 10000)
    {
        // read all the sensor values
        float temperature = ENV.readTemperature();
        float humidity = ENV.readHumidity();
        float pressure = ENV.readPressure();
        float illuminance = ENV.readIlluminance();
        float uva = ENV.readUVA();
        float uvb = ENV.readUVB();
        float uvIndex = ENV.readUVIndex();

        String light1 = "OFF";
        String light2 = "OFF";

        if (digitalRead(1) == HIGH) { light1 = "ON"; }
        if (digitalRead(2) == HIGH) { light2 = "ON"; }
        
        printReading(temperature, humidity, pressure, illuminance, uva, uvb, uvIndex);

        lastMillis = millis();
        mqttClient.publish("homie/mkrenv1/thermostat/temperature", String(temperature));
        mqttClient.publish("homie/mkrenv1/thermostat/humidity", String(humidity));
        mqttClient.publish("homie/mkrenv1/thermostat/pressure", String(pressure));
        mqttClient.publish("homie/mkrenv1/uv-sensor/ultraviolet-a", String(uva));
        mqttClient.publish("homie/mkrenv1/uv-sensor/ultraviolet-b", String(uvb));
        mqttClient.publish("homie/mkrenv1/uv-sensor/uv-index", String(uvIndex));

        mqttClient.publish("homie/mkrrelay/relay1/relay_on", light1);
        mqttClient.publish("homie/mkrrelay/relay2/relay_on", light2);

    }

    Watchdog.reset();
}
