#define dados_reais  
#define DEBUG
#define blynk 
#define shiftr_MQTT
#define priscila_TS
#define priscila_FB

#include <LiquidCrystal_I2C.h>
#include "ThingSpeak.h"
#include "FirebaseESP8266.h"
#define BLYNK_PRINT Serial  //blynk
#include <BlynkSimpleEsp8266.h>  //blynk
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h> 
#include "DHT.h"

#define trigPin D7
#define echoPin D8
#define INTERVALO_ENVIO       2000
#define INTERVALO_DISPLAY      1500
#define DHTPIN 2      // o sensor dht11 foi conectado ao pino 2( D4 do node MCU)
#define DHTTYPE DHT11
#define L1 D0   //pino de saida para acionamento da Luz
#define AR D3   //pino de saida para acionamento da ar

//informações da rede WIFI
const char* ssid = "F CARVALHO 2G";                 //SSID da rede WIFI
const char* password =  "Fc0503!f!";    //senha da rede wifi

#ifdef shiftr_MQTT
//informações do broker MQTT - Verifique as informações geradas pelo shiftr.io
const char* mqttServer = "broker.shiftr.io";   
const char* mqttUser = "61c57b13";              //user
const char* mqttPassword = "e3511421578aeb6d";      //password
const int mqttPort = 1883;                     //port
#endif

#ifdef priscila_TS
unsigned long myChannelNumber = 1063488;
const char * myWriteAPIKey = "JPU2FPDS8JP004RO";
const char * myReadAPIKey  = "OMX9PE91PRWBOGFN";
#endif

#define  mqttTopicSub1 "sala/set_luminosidade"            //tópico que sera assinado
#define  mqttTopicSub2 "sala/set_ar"            //tópico que sera assinado
#define mqttTopicSub3 "sala/set_porta"            //tópico que sera assinado
#define mqttTopicSub4 "sala/set_multimidia"            //tópico que sera assinado

#ifdef blynk
char auth[] = "Ca_NRgbuucmX5PEp5PjJOCJd8xS6nTBX";  // blynk
#endif

#ifdef priscila_FB
  #define FIREBASE_HOST "alimentador-pet.firebaseio.com"
  #define FIREBASE_AUTH "WqfZuWsGBbAt94GnhtDYxMWYP8iZjaL8xJHMO0kT"
#endif

int ultimoEnvio = 0,ultimoDisplay=0;


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

FirebaseData firebaseData;

typedef struct {
  float temperatura,umidade;
  int luminosidade,distancia,set_luminosidade;
  bool porta,presenca,set_ar,set_multimidia,set_porta;
} sala;
  
  char MsgMQTT[10];
  float temperatura,umidade;
  int luminosidade,distancia,set_luminosidade;
  int porta,presenca,set_ar,set_multimidia,set_porta;

bool ler_atuar_sinais(){
  #ifdef dados_reais  
    umidade = dht.readHumidity();
    temperatura = dht.readTemperature();
    luminosidade = analogRead(A0);
    porta = digitalRead(D5); 
    presenca = digitalRead(D6);
    distancia = (int)calc_distancia();
    luminosidade = random(0,1000);
    porta = random(0,2); 
    presenca = random(0,2);
    distancia = random(0,300); // Calculating the distancia
  #else
    umidade = (float)random(0,100);
    temperatura = (float)random(15,45);
    luminosidade = random(0,1000);
    porta = random(0,1); 
    presenca = random(0,1);
    distancia = random(0,300); // Calculating the distancia
  #endif
}
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("   SMART Room   ");
  lcd.setCursor(0,1);
  lcd.print("Connect to WI-FI");

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(L1, OUTPUT); // set_luminosidade #define L1 D0   //pino de saida para acionamento da Luz
  pinMode(AR, OUTPUT); // set_luminosidade #define AR D3   //pino de saida para acionamento da ar
  pinMode(D1, OUTPUT); // display
  pinMode(D2, OUTPUT); // display
  pinMode(D3, OUTPUT); // set_ar
  pinMode(D5, INPUT); // porta
  pinMode(D6, INPUT); // presenca
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    #ifdef DEBUG
    Serial.println("Conectando ao WiFi..");
    #endif
  }
  #ifdef DEBUG
  Serial.println("Conectado na rede WiFi");
  lcd.setCursor(0,1);
  lcd.print("Conectado  WI-FI ");
  #endif
  Blynk.begin(auth, ssid, password);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) {
    #ifdef DEBUG
    Serial.println("Conectando ao Broker MQTT...");
    #endif
 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      #ifdef DEBUG
      Serial.println("Conectado");  
      #endif
    } else {
      #ifdef DEBUG 
      Serial.print("falha estado  ");  Serial.println(client.state());
      #endif
      delay(300);
    }
  }

  //subscreve no tópico
  client.subscribe(mqttTopicSub1);
  client.subscribe(mqttTopicSub2);
  client.subscribe(mqttTopicSub3);
  client.subscribe(mqttTopicSub4);

  lcd.setCursor(0,1);
  lcd.print("Tudo configurado");

  dht.begin();
  ThingSpeak.begin(espClient);  // Initialize ThingSpeak
 
}
 
void callback(char* topic, byte* payload, unsigned int length) {

  //armazena msg recebida em uma sring
  payload[length] = '\0';
  String strMSG = String((char*)payload);

  #ifdef DEBUG
  Serial.print("Mensagem chegou do tópico: ");
  Serial.println(topic);
  Serial.print("Mensagem:");
  Serial.print(strMSG);
  Serial.println();
  Serial.println("-----------------------");
  #endif
 
  //aciona saída conforme msg recebida 
  if (!strcmp(topic,mqttTopicSub1))analogWrite(L1,strMSG.toInt()); //coloca saída em LOW para ligar a Lampada - > o módulo RELE usado tem acionamento invertido. Se necessário ajuste para o seu modulo
  else  
    if (!strcmp(topic,mqttTopicSub2)) digitalWrite(AR,strMSG.toInt()); //coloca saída em LOW para ligar a Lampada - > o módulo RELE usado tem acionamento invertido. Se necessário ajuste para o seu modulo
    else 
      Serial.println(" topico desconhecido");
 
}

//função pra reconectar ao servido MQTT
void reconect() {
  //Enquanto estiver desconectado
  while (!client.connected()) {
    #ifdef DEBUG
    Serial.print("Tentando conectar ao servidor MQTT");
    #endif
     
    bool conectado = strlen(mqttUser) > 0 ?
                     client.connect("ESP8266Client", mqttUser, mqttPassword) :
                     client.connect("ESP8266Client");

    if(conectado) {
      #ifdef DEBUG
      Serial.println("Conectado!");
      #endif
      //subscreve no tópico
      client.subscribe(mqttTopicSub1, 1); //nivel de qualidade: QoS 1
      client.subscribe(mqttTopicSub2, 1); //nivel de qualidade: QoS 1
      client.subscribe(mqttTopicSub3, 1); //nivel de qualidade: QoS 1
      client.subscribe(mqttTopicSub4, 1); //nivel de qualidade: QoS 1
    } else {
      #ifdef DEBUG
      Serial.println("Falha durante a conexão.Code: ");
      Serial.println( String(client.state()).c_str());
      Serial.println("Tentando novamente em 10 s");
      #endif
      //Aguarda 10 segundos 
      delay(10000);
    }
  }
}
 
void loop() {
  Blynk.run();
  client.loop();
  if ((millis() - ultimoDisplay) > INTERVALO_DISPLAY) { atualiza_lcd();  ultimoDisplay = millis();}
  
  if (!client.connected()) reconect();

  if ((millis() - ultimoEnvio) > INTERVALO_ENVIO) { ler_atuar_sinais(); tx_rx_blynk_server(); enviaDados();  ultimoEnvio = millis(); }
  
}

void enviaDados(){
  static unsigned char servidor=0;
  servidor++;
  switch(servidor%5){
    case 0:
      tx_rx_firebase_server();
      break;
    case 1:
      tx_rx_thingspearker_server();
      break;
    case 2:
      tx_rx_MQTT_server();
      break;
    case 3:
      tx_rx_blynk_server();
      break;
    case 4:
      #ifdef DEBUG
        if (isnan(temperatura) || isnan(umidade))     Serial.println("Falha na leitura do dht11...");
        Serial.print("Umidade: ");
        Serial.println(umidade);
        Serial.print("Temperatura: ");
        Serial.print(temperatura);
        Serial.println(" °C");
        Serial.print("distancia: ");
        Serial.print(distancia);
        Serial.println("cm");
        Serial.print("Luminosidade input: ");
        Serial.println(analogRead(A0));
        Serial.print("Porta input D5: ");
        Serial.println(digitalRead(D5));
        Serial.print("PIR input D6: ");
        Serial.println(digitalRead(D6));   
      #endif
      break;
    default:
      Serial.printf("problema maquina estado de envio");
  }
}
void atualiza_lcd(){
  static unsigned char contador=0;
  contador++;
  lcd.clear();
  switch(contador%7){
    case 0:
    lcd.setCursor(0,0);
      lcd.print("  Distancia  ");
      lcd.setCursor(11,1);
      lcd.print(distancia);
      lcd.print("cm");
      break;
    case 1:
    lcd.setCursor(0,0);
      lcd.print(" Luminosidae:");
      lcd.setCursor(10,1);
      lcd.print(analogRead(A0));
      lcd.print("Lux");
      break;
    case 2:
      valorPorta = digitalRead(D5);
  if (valorPorta == HIGH){
  Serial.println("porta: fechada");
  lcd.setCursor(1,0);
  lcd.print("Porta");
  lcd.setCursor(8,0);
  lcd.print("Fechada'");
  
  valorSensorPIR = digitalRead(D6);
   
  if (valorSensorPIR == HIGH) {
    analogWrite(pinLed, HIGH);
    digitalWrite(pinBuzzer, HIGH);
    Serial.print("Valor do Sensor PIR: ");  
    Serial.println(valorSensorPIR);
    lcd.setCursor(1,1);
    lcd.print("Sensor PIR:");
    lcd.setCursor(13,1);
    lcd.print("ON");
    delay(300);
  } else {
    analogWrite(pinLed, LOW);
    digitalWrite(pinBuzzer, LOW);
    Serial.print("Valor do Sensor PIR: ");  
    Serial.println(valorSensorPIR);
    lcd.setCursor(1,1);
    lcd.print("Sensor PIR:");
    lcd.setCursor(13,1);
    lcd.print("OFF");
    delay(300);
  }
  }
  else{
    Serial.println("porta: aberta");
    lcd.setCursor(1,0);
    lcd.print("Porta");
    lcd.setCursor(8,0);
    lcd.print("Aberta");
    analogWrite(pinLed, LOW);
    digitalWrite(valorSensorPIR, LOW);
    digitalWrite(pinBuzzer, LOW);
    delay(300);
  }
  break;

    case 3:
    lcd.setCursor(0,0);
      lcd.print("   Temperatura  ");
      lcd.setCursor(13,1);
      lcd.print((int)dht.readTemperature());
      lcd.print("C");
      break;
    case 4:
    lcd.setCursor(0,0);
      lcd.print(" Umidade  ");
      lcd.setCursor(14,1);
      lcd.print((int)dht.readHumidity());
      break;
    case 5:
    lcd.setCursor(0,0);
      lcd.print("D:");
      lcd.print(distancia);
      lcd.print(" L:");
      lcd.print(analogRead(A0));
      lcd.print(" B:");
      lcd.print(digitalRead(D5));
      lcd.setCursor(0,1);
      lcd.print(" P:");
      lcd.print(digitalRead(D6));
      lcd.print(" T:");
      lcd.print((int)dht.readTemperature());
      lcd.print(" U:");
      lcd.print((int)dht.readHumidity());
      break;
    default:
      lcd.print("    problemas   ");
      lcd.setCursor(0,1);
      lcd.print("  processamento ");
    }
}

bool tx_rx_firebase_server(){
    if(Firebase.pushInt(firebaseData, "sala/luminosidade", luminosidade)) // enviar dados 
    { Firebase.pushInt(firebaseData, "sala/distancia", distancia);   
      Firebase.setInt(firebaseData, "sala/presenca", presenca);
      Firebase.setInt(firebaseData, "sala/porta", porta);
      Firebase.pushFloat(firebaseData, "sala/temperatura", temperatura);
      Firebase.pushFloat(firebaseData, "sala/umidade", umidade);
      Serial.println("Set datas success in FB");
    }else { Serial.print("Error in setInt: ");Serial.println(firebaseData.errorReason()); return false;}
      
    if(Firebase.getInt(firebaseData, "/sala/set_luminosidade"))   // ler dados 
    { Serial.print("set_luminosidade ");Serial.println(firebaseData.intData());
      Firebase.getInt(firebaseData, "/sala/set_ar");
      Serial.print("set_ar ");Serial.println(firebaseData.intData());
      Firebase.getInt(firebaseData, "/sala/set_multimida");
      Serial.print("set_multimida ");Serial.println(firebaseData.intData());
      Firebase.getInt(firebaseData, "/sala/set_porta");
      Serial.print("set_porta ");Serial.println(firebaseData.intData());
    }else  {Serial.print("Error in getInt, ");Serial.println(firebaseData.errorReason());  return false;}
  return true;  
}

float calc_distancia(){
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); // leitura do ultrasonico
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracao = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  return(duracao*0.034/2); // Calculating the distancia
}

bool tx_rx_thingspearker_server(){
    ThingSpeak.setField(1, temperatura);
    ThingSpeak.setField(2, umidade);
    ThingSpeak.setField(3, distancia);
    ThingSpeak.setField(4, presenca);
    ThingSpeak.setField(5, porta);
    ThingSpeak.setField(6, random(0,100));
    ThingSpeak.setField(7, random(0,100));
    ThingSpeak.setStatus("sala em uso");
    if(ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey)== 200) Serial.println("Channel update successful.");
    else Serial.println("Problem updating channel. HTTP error code ");
    long TS_set_luminosidade = ThingSpeak.readLongField(myChannelNumber, 6, myReadAPIKey);
    Serial.println(TS_set_luminosidade);  
    long TS_set_ar = ThingSpeak.readLongField(myChannelNumber, 7, myReadAPIKey);  
    Serial.println(TS_set_ar);  
    return true;
}

bool tx_rx_MQTT_server(){
    sprintf(MsgMQTT,"%f",umidade);
    client.publish("sala/umidade", MsgMQTT);
    sprintf(MsgMQTT,"%f",temperatura);
    client.publish("sala/temperatura", MsgMQTT);
    sprintf(MsgMQTT,"%d",luminosidade);
    client.publish("sala/luminosidade", MsgMQTT);
    sprintf(MsgMQTT,"%d",distancia);
    client.publish("sala/distancia", MsgMQTT);
    sprintf(MsgMQTT,"%d",presenca);
    client.publish("sala/presenca", MsgMQTT);
    sprintf(MsgMQTT,"%d",porta);
    client.publish("sala/porta", MsgMQTT);
    return true;  
}
bool tx_rx_blynk_server(){
  Blynk.virtualWrite(V1,umidade);  //V5 is for Humidity  Blynk
  Blynk.virtualWrite(V2,temperatura);  //V6 is for Temperature Blynk
  Blynk.virtualWrite(V3,distancia);  //V6 is for Temperature Blynk
  return true;
}
