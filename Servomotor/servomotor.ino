#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

class ServoMotor {
private:
    Servo servo;
    int servoPin;
    int currentAngle;
    int servoSpeed;
    bool increasing;
    unsigned long lastMove;
    bool isMoving;

public:
    ServoMotor(int pin) : servoPin(pin), currentAngle(0), servoSpeed(50), increasing(true), lastMove(0), isMoving(true) {}

    void setup() {
        servo.attach(servoPin);
        servo.write(currentAngle);
    }

    void setSpeed(int speed) {
        if (speed == 0) {
            isMoving = false;
            Serial.println("Servo stopped");
        } else if (speed >= 1 && speed <= 100) {
            servoSpeed = speed;
            isMoving = true;
            Serial.print("New servo speed set to: ");
            Serial.println(servoSpeed);
        } else {
            Serial.println("Invalid speed value. Use 0 to stop, or 1-100 for speed.");
        }
    }

    void move() {
        if (!isMoving) return;

        unsigned long currentTime = millis();
        if (currentTime - lastMove >= (101 - servoSpeed)) {
            lastMove = currentTime;
            
            if (increasing) {
                currentAngle++;
                if (currentAngle >= 180) increasing = false;
            } else {
                currentAngle--;
                if (currentAngle <= 0) increasing = true;
            }
            
            servo.write(currentAngle);
            Serial.print("Current angle: ");
            Serial.println(currentAngle);
        }
    }
};

class Communicator {
private:
    const char* ssid;
    const char* password;
    const char* mqtt_server;
    const char* mqtt_topic;
    WiFiClient espClient;
    PubSubClient client;
    ServoMotor& servoMotor;

public:
    Communicator(const char* ssid, const char* password, const char* mqtt_server, const char* mqtt_topic, ServoMotor& servo)
        : ssid(ssid), password(password), mqtt_server(mqtt_server), mqtt_topic(mqtt_topic), client(espClient), servoMotor(servo) {}

    void setup() {
        setupWifi();
        client.setServer(mqtt_server, 1883);
        client.setCallback([this](char* topic, byte* payload, unsigned int length) { this->callback(topic, payload, length); });
    }

    void loop() {
        if (!client.connected()) {
            reconnect();
        }
        client.loop();
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

        Serial.println();
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    }

    void callback(char* topic, byte* payload, unsigned int length) {
        String message = "";
        for (unsigned int i = 0; i < length; i++) {
            message += (char)payload[i];
        }
        Serial.print("Message received: ");
        Serial.println(message);

        int receivedValue = message.toInt();
        servoMotor.setSpeed(receivedValue);
    }

    void reconnect() {
        while (!client.connected()) {
            Serial.print("Attempting MQTT connection...");
            String clientId = "ESP32Client-";
            clientId += String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println("connected");
                client.subscribe(mqtt_topic);
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
                delay(5000);
            }
        }
    }
};

ServoMotor servo(16);
Communicator communicator("FERRUVEGA", "Ignacio73*-", "broker.hivemq.com", "ucb/prueba/velo", servo);

void setup() {
    Serial.begin(115200);
    servo.setup();
    communicator.setup();
}

void loop() {
    communicator.loop();
    servo.move();
}