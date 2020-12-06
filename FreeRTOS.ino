/*
Programa desenvolvido por Maycon Klann

Sistemas Embarcados

Desenvolvimento com FreeRTOS
*/

#include <WiFi.h>
#include <PubSubClient.h>
//#include "EmonLib.h"

#define NUMER_SOLAR_ARRAYS 3
#define GRID_VOLTAGE 220

#ifndef pdMS_TO_TICKS
  #define pdMS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )
#endif

TaskHandle_t StateMachinePP;
TaskHandle_t MQTTcom;
SemaphoreHandle_t x_semaphore;
QueueHandle_t x_Queue;

//EnergyMonitor ct1, ct2, ct3;

/*-------------------------------------------------------------------------------------*/
/*------------------------------------variáveis WIFI-----------------------------------*/
/*-------------------------------------------------------------------------------------*/

// Adicionar aqui os parâmetros da rede local de wifi
const char* wifissid = "COLOCAR_AQUI_O_NOME_DA_REDE_WIFI";
const char* wifipassword =  "COLOCAR_AQUI_A_SENHA_DA_REDE_WIFI";

/*-------------------------------------------------------------------------------------*/
/*------------------------------------variáveis MQTT-----------------------------------*/
/*-------------------------------------------------------------------------------------*/

const char* Server = "test.mosquitto.org";
const int Port = 1883;
const char* User = "User";
const char* Password = "123456";
WiFiClient espClient;
PubSubClient MQTT(espClient);

/*-------------------------------------------------------------------------------------*/
/*-----------------------------variáveis máquina de estado-----------------------------*/
/*-------------------------------------------------------------------------------------*/

enum SolarPanelArray_x_State {REST, MEASURE, RECORD};
static SolarPanelArray_x_State StateMachine = REST;
typedef struct{
  int SolarArray;
  double POWER;
}InfoPanels;
int SA = 1;

/*-------------------------------------------------------------------------------------*/
/*---------------------------------------TAREFAS---------------------------------------*/
/*-------------------------------------------------------------------------------------*/

void StateMachinePowerPlant(void *pvParameters) {
  InfoPanels Info;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(250));
    switch(StateMachine){
      case REST:
        //se a fila estiver cheia, espera descarregar na task MQTT
        if(uxQueueMessagesWaiting(x_Queue)<3){
          StateMachine = MEASURE; 
        }
        if(SA>NUMER_SOLAR_ARRAYS){
          SA = 1;
        }
        break;
      case MEASURE:
        Info.SolarArray = SA;
        Info.POWER = PowerMeasure(Info.SolarArray);
        StateMachine = RECORD;
        break;
      case RECORD:
        xQueueSend(x_Queue, (void *) &Info, 1000); 
        SA++;
        StateMachine = REST;
        break;
    }
    taskYIELD();
  }
}


void MQTTcommunication(void *pvParameters) {
  InfoPanels Data;
  char data2send[100] = {};
  
  //inicializa a wifi
  xSemaphoreTake(x_semaphore,portMAX_DELAY);
  wifi_connection();
  xSemaphoreGive(x_semaphore);
  
  //inicializa o MQTT
  xSemaphoreTake(x_semaphore,portMAX_DELAY);
  MQTT_init();
  xSemaphoreGive(x_semaphore);
    
  while (true){   
    xSemaphoreTake(x_semaphore, portMAX_DELAY);
    MQTT_connection();
    xSemaphoreGive(x_semaphore);

    int vetor_solararray[NUMER_SOLAR_ARRAYS];
    double vetor_power[NUMER_SOLAR_ARRAYS];
    int i = 0;
    xSemaphoreTake(x_semaphore, portMAX_DELAY);
    if(uxQueueMessagesWaiting(x_Queue)>1){
      while(uxQueueMessagesWaiting(x_Queue)>0){
        xQueueReceive(x_Queue, &(Data), 1000);
        vetor_solararray[i] = Data.SolarArray;
        vetor_power[i] = Data.POWER;
        i++;  
      }
      sprintf(data2send,"Potencia || fila %d: %lf Kw || fila %d: %lf Kw || fila %d: %lf Kw", vetor_solararray[0], vetor_power[0],vetor_solararray[1], vetor_power[1],vetor_solararray[2], vetor_power[2]);
      MQTT.publish("FreeRTOS", data2send);
    }
    xSemaphoreGive(x_semaphore);
    
    Serial.println("Message was sent!");
    Serial.println(data2send);
    MQTT.loop();

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

/*-------------------------------------------------------------------------------------*/
/*----------------------------------------SETUP----------------------------------------*/
/*-------------------------------------------------------------------------------------*/

void setup() {

  Serial.begin(115200);
  Serial.println("Wait few seconds!");

  //Sensor de corrente (analog channel, calibration)
  //ct1.currentTX(1, 100);
  //ct2.currentTX(2, 100);
  //ct3.currentTX(3, 100);

  //cria semáforo
  x_semaphore = xSemaphoreCreateMutex();

  //cria fila    
  x_Queue = xQueueCreate(NUMER_SOLAR_ARRAYS, sizeof(InfoPanels));

  //cria tarefa para a monitoração de potência
  xTaskCreate(StateMachinePowerPlant, /*Task Function*/
              "StateMachinePP",       /*Name of task*/
              1024,                   /*Stack size of task*/
              NULL,                   /*Parameter of the task*/
              2,                      /*Prioity of the task*/
              &StateMachinePP         /*Task handle to keep track of created task*/
             );                       /*pin task to core 0*/

  //cria tarefa para comunicação MQTT
  xTaskCreate(MQTTcommunication, /*Task Function*/
              "MQTTcom",         /*Name of task*/
              4096,              /*Stack size of task*/
              NULL,              /*Parameter of the task*/
              1,                 /*Prioity of the task*/
              &MQTTcom           /*Task handle to keep track of created task*/
             );                  /*pin task to core 0*/

  delay(2000);
}

/*-------------------------------------------------------------------------------------*/
/*----------------------------------------LOOP-----------------------------------------*/
/*-------------------------------------------------------------------------------------*/

void loop() {
  taskYIELD();
}

/*-------------------------------------------------------------------------------------*/
/*---------------------------------------FUNÇÕES---------------------------------------*/
/*-------------------------------------------------------------------------------------*/

void wifi_connection(void){
  if (WiFi.status() == WL_CONNECTED){return;}
  WiFi.begin(wifissid, wifipassword);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
  }
}

void MQTT_init(void){
  MQTT.setServer(Server, Port);
}

void MQTT_connection(void){
  while (!MQTT.connected()){
    if (MQTT.connect("ESP32Client", User, Password)){
      Serial.println("MQTT successful conection!");
    }else{
      Serial.println("MQTT conection failed");
    }
  }
}

double PowerMeasure(int i){
  double solarpower = 0; 
  switch(i){
    case 1:
      //solarpower = ct1.calcIrms(1480)*GRID_VOLTAGE;
      solarpower = 10.0;
      break;
    case 2:
      //solarpower = ct2.calcIrms(1480)*GRID_VOLTAGE;
      solarpower = 20.0;
      break;
    case 3:
      //solarpower = ct3.calcIrms(1480)*GRID_VOLTAGE;
      solarpower = 30.0;
      break; 
  }
  return solarpower;
}
