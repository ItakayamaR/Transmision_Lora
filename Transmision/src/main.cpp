#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h> 
#include "SX1278.h" 
#include "LoRa.h"   

//Definimos pines para la comunicación SPI
#define MISO 12         
#define MOSI 13
#define SCK 14

//Definimos pines para el modulo 1 (RFM95W)
#define DIO0_1 26
#define DIO1_1 25
#define DIO2_1 33
#define RST1 27
#define SS1 32

//Definimos pines para el modulo 2 (LORA1276V2.0)
#define DIO0_2 19
#define DIO1_2 21
#define DIO2_2 22
#define RST2 19
#define SS2 23

//Definimos pines para el modulo 3 (E32-433T30D)
#define RX 4
#define TX 16
#define AUX 15
#define M0 5
#define M1 17

//Definimos pines para selección y led
#define SEL1 36
#define SEL2 39
#define LED  2

//Definiciones para la libreria
#define LORA_MODE             4
#define LORA_CHANNEL          915E6
#define LORA_ADDRESS          2
#define LORA_SEND_TO_ADDRESS  4

byte MODO = 0;
byte MODO_ANT = 0;
byte e;

void setup()
{
  //Inicializamos los pines de LED y selección
  pinMode(LED, OUTPUT);
  pinMode(SEL1, INPUT);
  pinMode(SEL2, INPUT);

  //Inicializamos pines del módulo 1
  pinMode(DIO0_1, INPUT);
  pinMode(DIO1_1, INPUT);
  pinMode(DIO2_1, INPUT);
  pinMode(RST1, OUTPUT);
  pinMode(SS1, OUTPUT);
  digitalWrite(SS1,HIGH);

  //Inicializamos pines del módulo 2
  pinMode(DIO0_2, INPUT);
  pinMode(DIO1_2, INPUT);
  pinMode(DIO2_2, INPUT);
  pinMode(RST2, OUTPUT);
  pinMode(SS2, OUTPUT);
  digitalWrite(SS2,HIGH);

  //Inicializamos pines del módulo 3
  pinMode(AUX, INPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  // Abrimos comunicaciones para observar 
  Serial.begin(115200); 

  //Iniciamos comunicación por UART (Módulo 3)
  Serial1.begin(9600, SERIAL_8N1, RX, TX);
  
  //Iniciamos los modulos en reset
  digitalWrite(RST1,0);
  digitalWrite(RST2,0);
  digitalWrite(M0,1);
  digitalWrite(M1,1);
  
}

void Ini_LoraModule(byte m) 
{
  //Inicializamos SPI en los pines correspondientes
  if (m==1){
    digitalWrite(RST1,1);
    SPI.begin(SCK, MISO, MOSI, SS1);    

  } else if (m==2) {
    digitalWrite(RST2,1);
    SPI.begin(SCK, MISO, MOSI, SS2);
  }

  // Mensaje de comprobacion por serial
  Serial.println(F("sx1278 module and Arduino: send two packets (One to an addrees and another one in broadcast)"));

  // Inicializamos el modulo
  if (sx1278.ON() == 0) {
    Serial.println(F("Setting power ON: SUCCESS "));
  } else {
    Serial.println(F("Setting power ON: ERROR "));
  }

  // Seteamos el modo de transmisión a Lora
  if (sx1278.setMode(LORA_MODE) == 0) {
    Serial.println(F("Setting Mode: SUCCESS "));
  } else {
    Serial.println(F("Setting Mode: ERROR "));
  }

  // Activamos el envío de Header
  if (sx1278.setHeaderON() == 0) {
    Serial.println(F("Setting Header ON: SUCCESS "));
  } else {
    Serial.println(F("Setting Header ON: ERROR "));
  }

  // Seleccionamos la frecuencia del canal
  if (sx1278.setChannel(LORA_CHANNEL) == 0) {
    Serial.println(F("Setting Channel: SUCCESS "));
  } else {
    Serial.println(F("Setting Channel: ERROR "));
  }

  // Activamos el envío de crc
  if (sx1278.setCRC_ON() == 0) {
    Serial.println(F("Setting CRC ON: SUCCESS "));
  } else {
    Serial.println(F("Setting CRC ON: ERROR "));
  }

  // Seleccionamos la potencia de envío (Max, High, Intermediate or Low)
  if (sx1278.setPower('M') == 0) {
    Serial.println(F("Setting Power: SUCCESS "));
  } else {
    Serial.println(F("Setting Power: ERROR "));
  }

  // Definimos la dirección del nodo de envio
  if (sx1278.setNodeAddress(LORA_ADDRESS) == 0) {
    Serial.println(F("Setting node address: SUCCESS "));
  } else {
    Serial.println(F("Setting node address: ERROR "));
  }

  // Mensaje de comprobación
  Serial.println(F("sx1278 configured finished"));
  Serial.println();

}

void terminar_spi(){
  digitalWrite(RST1,1);
  digitalWrite(RST2,1);
  SPI.end();
}

void iniciar_uart(){
  digitalWrite(M0,1);
  digitalWrite(M1,1);
}

void terminar_uart(){
  digitalWrite(M0,1);
  digitalWrite(M1,1);  
}


void EnableDevice(byte m){
  terminar_spi();
  terminar_uart();
  
  switch(m)
  {
    case 0:
      Serial.println("modo 0");
      break;

    case 1:
      Serial.println("modo 1");
      Ini_LoraModule(m);
      break;
    case 2:
      Serial.println("modo 2");
      Ini_LoraModule(m);
      break;

    case 3:
      Serial.println("modo 3");
      iniciar_uart(); 
      break;
  }
}

char message1[] = "Hola";
char message2[] = "Hola";

void loop(void)
{ 

  // Leemos el modo
  MODO = ( (digitalRead(SEL2)<<1) + digitalRead(SEL1) );
  
  if (MODO_ANT != MODO){
    EnableDevice(MODO);
    MODO_ANT=MODO;
  }
  
  if (MODO==1 || MODO==2){
    
    e = sx1278.sendPacketTimeout(LORA_SEND_TO_ADDRESS, message1);
    Serial.print("Message send: ");        
    Serial.println(e);
    if (e == 0) {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
    }
    
    

    

    // Enviamos un mensaje broadcast
    /*e = sx1278.sendPacketTimeout(0, message2);
    Serial.print(F("Packet sent, state "));
    Serial.println(e, DEC);

    if (e == 0) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
    }  */

    //delay(4000);  
  } else if (MODO==3){
    Serial1.println(message1);
  }
  
  delay(9000);


 
}

