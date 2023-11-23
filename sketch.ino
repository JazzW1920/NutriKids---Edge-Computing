// Incluindo bibliotecas necessárias
#include <WiFi.h>
#include <ArduinoJson.h>
#include <DHTesp.h>
#include <PubSubClient.h>

// Configurações de Wi-Fi
const char *SSID = "Wokwi-GUEST";
const char *PASSWORD = "";

// Configurações de MQTT
const char *BROKER_MQTT = "broker.hivemq.com";
const int BROKER_PORT = 1883;
const char *ID_MQTT = "marcin";
const char *TOPIC_PUBLISH_TEMP_HUMI = "testeGS/TempHumi";
const char *TOPIC_PUBLISH_DISTANCE = "testeGS/Distance";

// Configurações de Hardware
#define PIN_DHT 12
#define RED_LED 4
#define YELLOW_LED 2
#define GREEN_LED 15
#define ECHO_PIN 21
#define TRIG_PIN 19
#define PUBLISH_DELAY 2000

// Inicializando objetos e variáveis globais
WiFiClient espClient;
PubSubClient MQTT(espClient);
DHTesp dht;
unsigned long publishUpdate = 0;
TempAndHumidity sensorValues;
const int TAMANHO = 200;

// Protótipos de funções
void updateSensorValues();
void initWiFi();
void initMQTT();
void callbackMQTT(char *topic, byte *payload, unsigned int length);
void reconnectWiFi();
void reconnectMQTT();
void checkWiFIAndMQTT();
float readDistanceCM();

// Função para obter os valores do sensor
void updateSensorValues() {
  sensorValues = dht.getTempAndHumidity();
}

// Função para inicializar a conexão Wi-Fi
void initWiFi() {
  Serial.print("Conectando com a rede: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso: ");
  Serial.println(SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// Função para inicializar a conexão MQTT
void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(callbackMQTT);
}

// Callback para receber mensagens MQTT
void callbackMQTT(char *topic, byte *payload, unsigned int length) {
  String msg = String((char *)payload).substring(0, length);

  Serial.printf("Mensagem recebida via MQTT: %s do tópico: %s\n", msg.c_str(), topic);
}

// Função para reconectar a Wi-Fi se necessário
void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Wi-Fi conectado com sucesso");
  Serial.print(SSID);
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
}

// Função para reconectar ao Broker MQTT
void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Tentando conectar com o Broker MQTT: ");
    Serial.println(BROKER_MQTT);

    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao broker MQTT!");
    } else {
      Serial.println("Falha na conexão com MQTT. Tentando novamente em 2 segundos.");
      delay(2000);
    }
  }
}

// Função para verificar a conexão Wi-Fi e MQTT
void checkWiFIAndMQTT() {
  if (WiFi.status() != WL_CONNECTED)
    reconnectWiFi();
  if (!MQTT.connected())
    reconnectMQTT();
}

void setup() {
  // Inicialização da comunicação serial
  Serial.begin(115200);

  // Configuração dos pinos como saídas
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inicialização dos LEDs desligados
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // Configuração do sensor DHT22
  dht.setup(PIN_DHT, DHTesp::DHT22);

  // Inicialização da conexão Wi-Fi e MQTT
  initWiFi();
  initMQTT();
}

// Função para medir a distância utilizando o sensor ultrassônico
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  int duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  // Verificar e manter a conexão Wi-Fi e MQTT
  checkWiFIAndMQTT();
  MQTT.loop();

  // Medir a distância com o sensor ultrassônico
  float distance = readDistanceCM();

  // Verificar a distância e acionar os LEDs correspondentes
  if (distance < 100) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
  } else {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
  }

  // Verificar a temperatura e acionar o LED correspondente
  if ((sensorValues.temperature > 25) || (sensorValues.temperature < 15)) {
    digitalWrite(YELLOW_LED, HIGH);
  } else {
    digitalWrite(YELLOW_LED, LOW);
  }

  // Aguardar 2 segundos
  delay(2000);

  // Publicar dados no MQTT com intervalo definido
  if ((millis() - publishUpdate) >= PUBLISH_DELAY) {
    publishUpdate = millis();
    updateSensorValues();
    readDistanceCM();

    // Publicar dados de temperatura e umidade
    if (!isnan(sensorValues.temperature) && !isnan(sensorValues.humidity)) {
      StaticJsonDocument<TAMANHO> doc;
      doc["temperatura"] = sensorValues.temperature;
      doc["umidade"] = sensorValues.humidity;

      char buffer[TAMANHO];
      serializeJson(doc, buffer);
      MQTT.publish(TOPIC_PUBLISH_TEMP_HUMI, buffer);
      Serial.println(buffer);
    }

    // Publicar dados de distância
    if (!isnan(distance)) {
      StaticJsonDocument<TAMANHO> doc;
      doc["distance"] = readDistanceCM();

      char buffer[TAMANHO];
      serializeJson(doc, buffer);
      MQTT.publish(TOPIC_PUBLISH_DISTANCE, buffer);
      Serial.println(buffer);
    }
  }
}