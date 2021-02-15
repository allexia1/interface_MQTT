//SM_T6_G4 (Allexia - Davi - Gabriel)

#include <WiFi.h>
#include <PubSubClient.h>
#include <strings.h>

//pinout
const byte DI_pins[3] = {36, 39, 34};
const byte DO_pins[3] = {23, 22, 1};
const byte AN_pins[3] = {35, 32, 33};
const byte PWM_pins[3] = {3, 21, 19};
const uint8_t PWM_channels[3] = {0, 1, 2};

//variáveis para registro dos estados do I/O
bool DI_state[3] = {false, false, false};
bool DO_state[3] = {false, false, false};
uint16_t AN_state[3] = {0, 0, 0};
uint16_t PWM_state[3] = {0, 0, 0};

//configuraçãoes WiFi
const char* ssid = "****";
const char* password = "****";
WiFiClient wifiClient;

//configurações MQTT
const char* mqtt_server = "mqtt.eclipse.org";
const char* clientID = "a76bbee9_esp32";
const char* subTopics[2][2] = {
  {"da901203_interface/digital-out-", "/set_state"},
  {"da901203_interface/analog-out-", "/set_value"}
};
const char* pubTopics[4][2] = {
  {"da901203_interface/digital-in-", "/current_state"},
  {"da901203_interface/digital-out-", "/current_state"},
  {"da901203_interface/analog-in-", "/current_value"},
  {"da901203_interface/analog-out-", "/current_value"}
};
PubSubClient mqttClient(wifiClient);

//timer para publicação periódica
hw_timer_t * pubTimer = NULL; 
volatile bool toPublish = false;

void IRAM_ATTR setPublish(){
  toPublish = true;
}

void publish(){
  char topic[120] = "";
  char val[12] = "";
  for(uint8_t i=0; i<3; i++){
    (String(pubTopics[0][0]) + String(i+1, DEC) + String(pubTopics[0][1])).toCharArray(topic, 120);
    String((int)DI_state[i]).toCharArray(val, 12);
    mqttClient.publish((const char*)topic, (const char*)val, true);
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(val);

    (String(pubTopics[1][0]) + String(i+1, DEC) + String(pubTopics[1][1])).toCharArray(topic, 120);
    String((int)DO_state[i]).toCharArray(val, 12);
    mqttClient.publish((const char*)topic, (const char*)val, true);
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(val);

    (String(pubTopics[2][0]) + String(i+1, DEC) + String(pubTopics[2][1])).toCharArray(topic, 120);
    String((int)AN_state[i]).toCharArray(val, 12);
    mqttClient.publish((const char*)topic, (const char*)val, true);
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(val);

    (String(pubTopics[3][0]) + String(i+1, DEC) + String(pubTopics[3][1])).toCharArray(topic, 120);
    String((int)PWM_state[i]).toCharArray(val, 12);
    mqttClient.publish((const char*)topic, (const char*)val, true);
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(val);
  }
  toPublish = false;
}

void wifiConnect(){
  delay(10);
  Serial.print("Conectando a rede ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.print("OK! IP: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect(){
  while(!mqttClient.connected()){
    if(mqttClient.connect(clientID)){
      Serial.println("Conexao OK!");
      for(uint8_t i=0; i<2; i++){
        for(uint8_t j=0; j<3; j++){
          char topic[120] = "";
          (String(subTopics[i][0]) + String(j+1, DEC) + String(subTopics[i][1])).toCharArray(topic, 120);
          mqttClient.subscribe(topic);
          Serial.print("Inscricao ");
          Serial.println(topic);
        }
      }
    }else{
      Serial.print("Falha conexao, ");
      Serial.println(mqttClient.state());
      delay(1000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length){
  String topicS = String(topic);
  Serial.print("Recebido ");
  Serial.print(topicS);
  Serial.print(" ");
  Serial.println((const char *)payload);
  int canal, valor;
  valor =  atoi((const char *)payload);
  if(topicS.startsWith(subTopics[0][0])&&topicS.endsWith(subTopics[0][1])){
    canal = (int)topicS.charAt(strlen(subTopics[0][0]))-48;
    digitalWrite(DO_pins[canal-1], (bool) valor);
    DO_state[canal-1] = (bool) valor;
    Serial.print("DO ");
    Serial.print(canal);
    Serial.print(" : ");
    Serial.println((bool) valor);
  }
  else if (topicS.startsWith(subTopics[1][0])&&topicS.endsWith(subTopics[1][1])){
    canal = (int)topicS.charAt(strlen(subTopics[1][0]))-48;
    ledcWrite(PWM_channels[canal-1], valor);
    PWM_state[canal-1] = valor;
    Serial.print("PWM ");
    Serial.print(canal);
    Serial.print(" : ");
    Serial.println(valor);
  }
}

void setup() {
  for(uint8_t i=0; i<3; i++){ //inicialização dos pinos
    pinMode(DI_pins[i], INPUT_PULLUP);
    pinMode(DO_pins[i], OUTPUT);
    digitalWrite(DO_pins[i], LOW);
    pinMode(AN_pins[i], INPUT);
    pinMode(PWM_pins[i], OUTPUT);
    ledcSetup(PWM_channels[i], 5000, 10);
    ledcAttachPin(PWM_pins[i], PWM_channels[i]);
    ledcWrite(PWM_channels[i], 0);
  }

  wifiConnect();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);

  pubTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(pubTimer, &setPublish, true);
  timerAlarmWrite(pubTimer, 5000000, true);
  timerAlarmEnable(pubTimer);
  Serial.begin(115200);
}

void loop() {
  if(!mqttClient.connected())
    mqttReconnect();
  mqttClient.loop();

  if(toPublish)
    publish();

  for(uint8_t i=0; i<3; i++){ //leitura das entradas
    DI_state[i] = digitalRead(DI_pins[i]);
    AN_state[i] = analogRead(AN_pins[i]);
  }
}
