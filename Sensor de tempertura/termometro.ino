#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

class TemperatureSensor {
private:
    OneWire oneWire;
    DallasTemperature sensors;
    int sensorPin;

public:
    TemperatureSensor(int pin) : sensorPin(pin), oneWire(pin), sensors(&oneWire) {}

    void setup() {
        sensors.begin();
    }

    float readTemperature() {
        sensors.requestTemperatures();
        float temperatureC = sensors.getTempCByIndex(0);

        if (temperatureC == DEVICE_DISCONNECTED_C) {
            Serial.println("Error: Unable to read temperature.");
            return DEVICE_DISCONNECTED_C;
        }
        else{
          Serial.print("Temperature: ");
          Serial.print(temperatureC);
          Serial.println(" °C");
        }

        return temperatureC;
    }
};

class Communicator {
private:
    const char* ssid;
    const char* password;
    const char* mqtt_server;
    WiFiClient espClient;
    PubSubClient client;

public:
    Communicator(const char* ssid, const char* password, const char* mqtt_server)
        : ssid(ssid), password(password), mqtt_server(mqtt_server), client(espClient) {}

    void setup() {
        setupWifi();
        client.setServer(mqtt_server, 1883);
    }

    void loop() {
        if (!client.connected()) {
            reconnect();
        }
        client.loop();
    }

    void publishTemperature(float temperature) {
        if (client.connected()) {
            char msg[50];
            snprintf(msg, 50, "%.2f", temperature);
            if (client.publish("ucb/test/temp", msg)) {
                Serial.println("Temperature published successfully");
            } else {
                Serial.println("Error publishing temperature");
            }
        }
    }

private:
    void setupWifi() {
        delay(10);
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(ssid);

        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }

        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    }

    void reconnect() {
        while (!client.connected()) {
            Serial.print("Connecting to MQTT broker...");
            String clientId = "ESP32Client-";
            clientId += String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println("connected");
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" trying again in 5 seconds");
                delay(5000);
            }
        }
    }
};

TemperatureSensor temperatureSensor(4);
Communicator comm("FERRUVEGA", "Ignacio73*-", "broker.hivemq.com");

void setup() {
    Serial.begin(115200);
    temperatureSensor.setup();
    comm.setup();
}

void loop() {
    comm.loop();
    
    float temperature = temperatureSensor.readTemperature();

    comm.publishTemperature(temperature);
    
    delay(2000);
}