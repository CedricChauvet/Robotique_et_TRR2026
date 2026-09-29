#include <WiFi.h>  // Bibliothèque native ESP32
#include <PubSubClient.h>  // 



#define S_RXD 18
#define S_TXD 19


#include <SCServo.h>
SCSCL sc;

const char* ssid = "Bbox-BFE7AC14";
const char* password = "ITetudes.256";

// MQTT
const char* broker = "192.168.1.192";
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
String one;
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float theta4 = 0;

// Throttling
unsigned long lastPWMUpdate = 0;
const unsigned long PWM_UPDATE_INTERVAL = 20;

// Watchdog I2C
unsigned long lastI2CCheck = 0;
const unsigned long I2C_CHECK_INTERVAL = 5000;
int i2cErrorCount = 0;


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n🚀 Démarrage ESP32...");
  
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;

  
  // WiFi avec auto-reconnect
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  
  Serial.printf("Connexion WiFi à %s", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connecté");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n❌ WiFi timeout - redémarrage...");
    ESP.restart();
  }
  
  // MQTT
  mqttClient.setServer(broker, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(5);
  
  // ✅ Buffer MQTT agrandi (important !)
  mqttClient.setBufferSize(512);
  
  reconnectMQTT();
}

void loop() {
  static int msgCount = 0;
  static unsigned long lastReport = 0;
  int t0 = millis();
  /*
  // Auto-reconnect MQTT
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  */
  // ✅ PubSubClient gère mieux le buffer automatiquement
  mqttClient.loop();


    // Calculs communs (une seule fois)
    int ms1 = constrain(map(theta1, 105, -105, 0, 1024), 0, 1024);
    int ms2 = constrain(map(theta2, 0, 210, 0, 1024), 0, 520); // anttention butée dangereuse
    int ms3 = constrain(map(theta3, 105, -105, 0, 1024), 0, 1024);
    int ms4 = constrain(map(theta4, 105, -105, 0, 1024), 0, 1024);
    

    int ms5 = constrain(map(theta1, 105, -105, 0, 1024), 0, 1024);
    int ms6 = constrain(map(theta2, 0, 210, 0, 1024), 0, 520); // anttention butée dangereuse
    int ms7 = constrain(map(theta3, 105, -105, 0, 1024), 0, 1024);
    int ms8 = constrain(map(theta4, 105, -105, 0, 1024), 0, 1024);

    // Offset des canaux selon la jambe
    if (one == "D"){
       sc.WritePos(1, ms1 - 10 , 0, 1500);
       sc.WritePos(2, ms2 + 67, 0, 1500);
       sc.WritePos(3, ms3 + 22, 0, 1500);
       sc.WritePos(4, ms4 + 22, 0, 1500);
    }
    else if (one == "G"){
      sc.WritePos(5, ms5 - 35 , 0, 1500);
      sc.WritePos(6, ms6 + 67, 0, 1500);
      sc.WritePos(7, ms7 - 22, 0, 1500);
      sc.WritePos(8, ms8 , 16, 1500);
  }

  delay(12);
  int t1 = millis();
  int freq= 1000 / (t1 - t0);
  //Serial.print(" frequence loop  :");
  //Serial.println(freq);
  }

  




// ✅ Callback MQTT - appelé automatiquement à la réception
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  message.reserve(50);
  
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  // Comparaison de chaînes C (char*) - utiliser strcmp
  if (strcmp(topic, "jambe_G") == 0) {
    one = "G";
  } else if (strcmp(topic, "jambe_D") == 0) {
    one = "D";
  }
  
  // Parser
  int idx1 = message.indexOf(',');
  int idx2 = message.indexOf(',', idx1 + 1);
  int idx3 = message.indexOf(',', idx2 + 1);  


  if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {  
    theta1 = message.substring(0, idx1).toFloat();
    theta2 = message.substring(idx1 + 1, idx2).toFloat();
    theta3 = message.substring(idx2 + 1, idx3).toFloat();
    theta4 = message.substring(idx3 + 1).toFloat();
    
    Serial.printf("%s, Angles: %.1f, %.1f, %.1f, %.1f\n", one, theta1, theta2, theta3, theta4);
  }
}


void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connexion MQTT...");
    
    if (mqttClient.connect("ESP32_JambeG")) {
      Serial.println("✅ Connecté");
      mqttClient.subscribe("jambe_G");
      mqttClient.subscribe("jambe_D");
    } else {
      Serial.printf("❌ Erreur : %d\n", mqttClient.state());
      delay(2000);
    }
  }
}

