/*******************************************************************************************************************************************************
                                         -- TRABALHO INTERDISCIPLINAR: IOT -- 

 ALUNO: Ramon Vinícius Silva Corrêa
 TURMA: 3132.1.00

********************************************************************************************************************************************************/
                                              /*-- INCLUSÃO DE BIBLIOTECAS --*/
#include <WiFi.h> //Biblioteca para Wi-Fi
#include <PubSubClient.h> //Biblioteca para Pub e Sub
#include <SPI.h> //iblioteca para comunicação SPI do sensor RFID
#include <MFRC522.h> //Biblioteca para sensor RFID
#include <Servo.h> //Biblioteca para Servo Motor
#include <Wire.h>

/*******************************************************************************************************************************************************/
                                        /*-- DEFINIÇÕES PARA SENSOR RFID --*/

#define ID "27 6E 6A C9" //Define ID que será aceito pelo sensor RFID
#define LedVerde 26 //Define pino para Led Verde de indicação de "ACESSO LIBERADO"
#define LedVermelho 12 //Define pino para Led vermelho de indicação de "ACESSO NEGADO"
#define SS_PIN 14
#define RST_PIN 27

/*******************************************************************************************************************************************************/
                                        /*-- DEFINIÇÕES PARA SENSOR LDR --*/
                                        
#define portaLDR 35 //Define pino para sensor LDR
#define PIN_LED 2 //Define pino para LED
#define LED_BUILTIN 2 

/*******************************************************************************************************************************************************/
                                        /*-- DEFINIÇÕES PARA SENSOR DE TEMPERATURA --*/

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       A0 //define pino analógico para LM35
#define PIN_VENT       4 //Define pino para acionar o cooler

/*******************************************************************************************************************************************************/
                                                    /*-- DEFINIÇÕES PARA MQTT --*/

#define TOPICO_SUBSCRIBE_LED         "topico_liga_desliga_led"
#define TOPICO_SUBSCRIBE_RFID        "topico_rfid"
#define TOPICO_SUBSCRIBE_VENT        "topico_vent"
#define TOPICO_PUBLISH_TEMPERATURA   "topico_sensor_temperatura"
#define TOPICO_PUBLISH_RFID          "topico_rfid"
#define TOPICO_PUBLISH_LDR           "topico_liga_desliga_led"
#define ID_MQTT  "Trabalho_3_IOT"     //id mqtt (para identificação de sessão)

const char* SSID     = "LIVE TIM_0820_2G"; // SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "a3ehn6rep6"; // Senha da rede WI-FI que deseja se conectar
const char* BROKER_MQTT = "test.mosquitto.org";
int BROKER_PORT = 1883; // Porta do Broker MQTT

/*******************************************************************************************************************************************************/
                                                    /*-- VARIÁVEIS E OBJETOS GLOBAIS --*/

WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient
MFRC522 mfrc522(SS_PIN, RST_PIN);   // define os pinos de controle do modulo de leitura de cartoes RFID
Servo myServo; //Cria objeto para servo motor

int j = 0, k=0; //Variáveis para flag
int adcVal = analogRead(PIN_LM35); //Variável para guardar valor do sensor de temperatura
int ativaServo = 0;

/*******************************************************************************************************************************************************/
                                                    /*-- PROTOTYPES --*/
                                                    
void initWiFi(void);
void initMQTT(void);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT(void);
void reconnectWiFi(void);
void VerificaConexoesWiFIEMQTT(void);

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO PARA CONECTAR-SE AO WI-FI --*/

void initWiFi(void)
{
  delay(10); //Delay de 10 milissegundos 
  Serial.println("------Conexao WI-FI------"); //Imprime mensagem no monitor serial
  Serial.print("Conectando-se na rede: "); //Imprime mensagem no monitor serial
  Serial.println(SSID); //Imprime mensagem no monitor serial
  Serial.println("Aguarde"); //Imprime mensagem no monitor serial
  reconnectWiFi(); //Chama função para reconectar o Wi-Fi
}

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO PARA CONECTAR-SE AO MQTT --*/

void initMQTT(void)
{
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
  MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO DE CALLBACK--*/
                                                    
//esta função é chamada toda vez que uma informação de um dos tópicos subescritos chega

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char str_LDR[20] = {0}; 
  String msg;
  
  /* obtem a string do payload recebido */
  for (int i = 0; i < length; i++)
  {
    char c = (char)payload[i];
    msg += c;
  }
  
  Serial.print("Chegou a seguinte string via MQTT: ");
  Serial.println(msg);
  
  /* toma ação dependendo da string recebida */

/*----------------------------------------------------------------------*/
   //Se chegar a mensagem L acende um led e publica no MQTT
  if (msg.equals("L"))
  {
    digitalWrite(PIN_LED, HIGH);
    Serial.println("LED aceso mediante comando MQTT");
    sprintf(str_LDR, "LUZ ACESA");
    MQTT.publish(TOPICO_PUBLISH_LDR, str_LDR);
    j = 1;
  }

  //Se chegar a mensagem D desliga o led e publica no MQTT
  if (msg.equals("D"))
  {
    digitalWrite(PIN_LED, LOW);
    Serial.println("LED apagado mediante comando MQTT");
    sprintf(str_LDR, "LUZ APAGADA");
    MQTT.publish(TOPICO_PUBLISH_LDR, str_LDR);
    j=0;
  }
/*----------------------------------------------------------------------*/

  //Se chegar a mensagem 1 liga o cooler
    if (msg.equals("1"))
  {
    digitalWrite(PIN_VENT, HIGH);
    k = 1;
  }
  
  //Se chegar a mensagem 0 desliga o cooler
    if (msg.equals("0"))
  {
    digitalWrite(PIN_VENT, LOW);
    k = 0;
  }
  
/*----------------------------------------------------------------------*/

  //Se chegar a mensagem P liga o Led verde por 2seg
  if(msg.equals("P"))
  {
    digitalWrite(LedVerde, HIGH);
    delay(2000);
    digitalWrite(LedVerde, LOW);

    ativaServo = 1;

    
    if(ativaServo == 1)
    {
          for(int pos = 0; pos < 90; pos++)
            myServo.write(pos);
  
          delay(3000);
  
          for(int pos = 90; pos > 0; pos--)
             myServo.write(pos);
  
           ativaServo = 0;  
    }
  }

  //Se chegar a mensagem N liga o led vermelho por 2seg
  if(msg.equals("N"))
  {
    digitalWrite(LedVermelho, HIGH);
    delay(2000);
    digitalWrite(LedVermelho, LOW);
  }
}

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO DE RECONECTAR-SE AO BROKER MQTT--*/
                                                    
void reconnectMQTT(void)
{
  while (!MQTT.connected())
  {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT))
    {
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe(TOPICO_SUBSCRIBE_LED);
      MQTT.subscribe(TOPICO_SUBSCRIBE_RFID);
      MQTT.subscribe(TOPICO_SUBSCRIBE_VENT);
    }
    else
    {
      Serial.println("Falha ao reconectar no broker.");
      Serial.println("Havera nova tentatica de conexao em 2s");
      delay(2000);
    }
  }  
}

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO DE VERIFICAR CONEXÃO WI-FI E MQTT--*/
void VerificaConexoesWiFIEMQTT(void)
{
  if (!MQTT.connected())
    reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
  reconnectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO PARA RECONECTAR-SE AO WI-FI--*/

void reconnectWiFi(void)
{
  //se já está conectado a rede WI-FI, nada é feito.
  //Caso contrário, são efetuadas tentativas de conexão
  if (WiFi.status() == WL_CONNECTED)
    return;
  WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado com sucesso na rede ");
  Serial.print(SSID);
  Serial.println("\nIP obtido: ");
  Serial.println(WiFi.localIP());
}

/*******************************************************************************************************************************************************/
                                                    /*-- FUNÇÃO PARA SENSOR RFID--*/
void sensorRFID()
{
  char rfid_str[20] = {0};
  
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
     return;                 // se nao tiver um cartao para ser lido recomeça o void loop
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;                  //se nao conseguir ler o cartao recomeça o void loop tambem
  }

  String conteudo = "";      // cria uma string

  Serial.print("id da tag :"); //imprime na serial o id do cartao

  for (byte i = 0; i < mfrc522.uid.size; i++)
  {  
    //faz uma verificacao dos bits da memoria do cartao
    //ambos comandos abaixo vão concatenar as informacoes do cartao...
    //porem os 2 primeiros irao mostrar na serial e os 2 ultimos guardarao os valores na string de conteudo para fazer as verificacoes
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    conteudo.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
    conteudo.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  
Serial.println();
conteudo.toUpperCase(); // deixa as letras da string todas maiusculas

  if (conteudo.substring(1) == ID)// verifica se o ID do cartao lido tem o mesmo ID do cartao que queremos liberar o acesso
  {
     ativaServo = 1;       
     digitalWrite(LedVerde, HIGH); //liga o led verde
     sprintf(rfid_str, "ACESSO PERMITIDO"); //Imprime no MQTT uma mensagem
     delay(2000); //Aguarda 2 segundos
     digitalWrite(LedVerde, LOW);//desliga o led verde
      
     
     MQTT.publish(TOPICO_PUBLISH_RFID, rfid_str);
     

  }
  
  else // caso o cartao lido nao foi registrado
  {    
                                     
    digitalWrite(LedVermelho, HIGH); // liga o led vermelho
    sprintf(rfid_str, "ACESSO NEGADO"); //Imprime uma mensagem no MQTT
    delay(2000); //Aguarda 2 segundos
    digitalWrite(LedVermelho, LOW); // desliga o led vermelho
    MQTT.publish(TOPICO_PUBLISH_RFID, rfid_str);
  }


  if(ativaServo == 1)
  {
        for(int pos = 0; pos < 90; pos++)
          myServo.write(pos);

        delay(3000);

        for(int pos = 90; pos > 0; pos--)
           myServo.write(pos);

         ativaServo = 0;  
  }
}

/*******************************************************************************************************************************************************/
                                                          /*-- FUNÇÃO PARA SENSOR LDR --*/
void leituraLDR()
{
  char str_LDR[20] = {0}; 
  int leitura = analogRead(portaLDR); // realizar leitura do sensor ldr
    
  if((analogRead(portaLDR) < 1000)&&(j==0)) //Verifica Luminosidade do ambiente 
  {
    //Se a luminosidade estiver baixa acende o led 
    digitalWrite(PIN_LED, HIGH);
    sprintf(str_LDR, "LUZ ACESA");
    MQTT.publish(TOPICO_PUBLISH_LDR, str_LDR);
  }
  
  else
  {
    //Se a luminosidade estiver alta deixa o led apagado
    digitalWrite(PIN_LED, LOW);
    sprintf(str_LDR, "LUZ APAGADA");
    MQTT.publish(TOPICO_PUBLISH_LDR, str_LDR);
  }  
}

/*******************************************************************************************************************************************************/
                                                          /*-- INICIA O SETUP --*/
void setup() 
{
  myServo.attach(15); //Define pino do servo 
  myServo.write(0); //inicia o Servo motor em posição 0°
  
  Serial.begin(115200);  // inicia a comunicacao serial com o computador na velocidade de 115200 baud rate
  Serial.println("Disciplina IoT: acesso a nuvem via ESP32");
  
  pinMode(PIN_LED, OUTPUT);// programa LED interno como saida
  digitalWrite(PIN_LED, LOW);// inicia o LED apagado

  initWiFi(); //Inicializa a conexao wi-fi
  initMQTT();//Inicializa a conexao ao broker MQTT

  SPI.begin();   // inicia a comunicacao SPI que sera usada para comunicacao com o mudulo RFID
  mfrc522.PCD_Init();  //inicia o modulo RFID
  Serial.println("RFID + ESP32");
  Serial.println("Aguardando tag RFID para verificar o id da mesma.");
  pinMode(LedVerde, OUTPUT); //Define led verde como saida
  pinMode(LedVermelho, OUTPUT); //Define led vermelho como saida

  pinMode(PIN_VENT, OUTPUT); //Define pino do cooler como saida
}

/*******************************************************************************************************************************************************/
                                                          /*-- INICIA O LOOP --*/

void loop() 
{
   /* garante funcionamento das conexões WiFi e ao broker MQTT */
  VerificaConexoesWiFIEMQTT();
  
/*------------------------------------------------------------------------------------------*/     
                             /* -- COMANDOS PARA LM35 --*/
                             
  
  int adcVal = analogRead(PIN_LM35); // Obtem o vaor ADC do sensor de temperatura
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION); //Converte o valor obtido em volts para milivolts
  float tempC = milliVolt / 10; //Converte o valor em volts para temperatura em graus celsius
  
  char temperatura_str[10] = {0};// cria string para temperatura

  sprintf(temperatura_str, "%.2fC", tempC); // formata a temperatura aleatoria  como string
  
  //  Publica a temperatura 
  MQTT.publish(TOPICO_PUBLISH_TEMPERATURA, temperatura_str);
  Serial.print("Gerando temperatura aleatoria: ");
  Serial.println(tempC);

    if(k==0)
  {
    //Se o sensor detectar um valor maior que 50°C liga o cooler
      if(tempC > 50.00)
        digitalWrite(PIN_VENT, HIGH);

      else
        digitalWrite(PIN_VENT, LOW);
  }
 
/*--------------------------------------------------------------------------------------------*/  
  
  sensorRFID(); //Chama função para sensor RFID
  
  if(j==0)
    leituraLDR(); //Chama função para LDR

  MQTT.loop();// keep-alive da comunicação com broker MQTT 
  delay(2000); //Refaz o ciclo após 2 segundos
}
