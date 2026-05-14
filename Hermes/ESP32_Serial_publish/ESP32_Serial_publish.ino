#include <WiFi.h>
#include <PubSubClient.h>

// --- WiFi ---
const char* ssid     = "Bbox-BFE7AC14";
const char* password = "ITetudes.256";

// --- MQTT ---
const char* mqtt_server = "192.168.1.192";  // ex: "192.168.1.100"
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "teensy/data";
// const char* mqtt_user = "user";          // décommente si ton broker demande auth
// const char* mqtt_pass = "pass";

// --- UART2 ---
#define RXD2 16
#define TXD2 17

WiFiClient   espClient;
PubSubClient mqtt(espClient);

void connectWiFi() {
  Serial.print("Connexion WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté ! IP : " + WiFi.localIP().toString());
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Connexion MQTT...");
    if (mqtt.connect("ESP32Client")) {  // ajoute mqtt_user, mqtt_pass si auth
      Serial.println("connecté !");
    } else {
      Serial.print("ECHEC, rc=");
      Serial.print(mqtt.state());
      Serial.println(" -> nouvel essai dans 2s");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  connectWiFi();

  mqtt.setServer(mqtt_server, mqtt_port);

  connectMQTT();

  Serial.println("ESP32 prêt - en écoute UART2 + MQTT");
}

void loop() {
  // Maintenir la connexion MQTT
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // Lecture UART2
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    msg.trim();

    if (msg.length() > 0) {
      Serial.println("Reçu : " + msg);

      // Publish MQTT
      if (mqtt.publish(mqtt_topic, msg.c_str())) {
        Serial.println("Publié sur " + String(mqtt_topic));
      } else {
        Serial.println("Echec publication MQTT !");
      }
    }
  }
}


