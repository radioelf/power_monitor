/*
  Monitorización de la tensión de red, la intensidad de 5 lineas, la potencia, la 
  temperatura y humedad externa y la temperatura de la CPU del ESP32, los datos son 
  enviados a un servidor con mosquitto (Home Assistant), si no se dispone de conexión wifi se 
  configura el modulo ESP32 como punto de acceso y se muestra una pagina web con la 
  tensión de red, la intensidad total y la potencia, también se puede realizar 
  peticiones de los datos a través de RS485 (NO testeado)

  Creative Commons License Disclaimer

  UNLESS OTHERWISE MUTUALLY AGREED TO BY THE PARTIES IN WRITING, LICENSOR OFFERS THE WORK AS-IS
  AND MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY KIND CONCERNING THE WORK, EXPRESS, IMPLIED,
  STATUTORY OR OTHERWISE, INCLUDING, WITHOUT LIMITATION, WARRANTIES OF TITLE, MERCHANTIBILITY,
  FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT, OR THE ABSENCE OF LATENT OR OTHER DEFECTS,
  ACCURACY, OR THE PRESENCE OF ABSENCE OF ERRORS, WHETHER OR NOT DISCOVERABLE. SOME JURISDICTIONS
  DO NOT ALLOW THE EXCLUSION OF IMPLIED WARRANTIES, SO SUCH EXCLUSION MAY NOT APPLY TO YOU.
  EXCEPT TO THE EXTENT REQUIRED BY APPLICABLE LAW, IN NO EVENT WILL LICENSOR BE LIABLE TO YOU
  ON ANY LEGAL THEORY FOR ANY SPECIAL, INCIDENTAL, CONSEQUENTIAL, PUNITIVE OR EXEMPLARY DAMAGES
  ARISING OUT OF THIS LICENSE OR THE USE OF THE WORK, EVEN IF LICENSOR HAS BEEN ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGES.

  http://creativecommons.org/licenses/by-sa/3.0/

  Author: Radioelf  http://radioelf.blogspot.com.es/

  ESP32-WROOM-32D (16Mbs) 

  Mod. 13/6/20
*/
#include <Arduino.h>
#include <SPIFFS.h>                                                                   // Librería para almacenamiento no volátil en flash
#include <driver/adc.h>
#include <Time.h>
#include <DHT.h>                                                                      // Librería para DHT11/22 https://github.com/adafruit/DHT-sensor-library

#define VERSION     "V. 0.1.0"                                                        // Versión

#define RS485Pin 21                                                                   // Control IC RS485
#define Aux      22                                                                   // ON-OFF debugger UART2
#define LedPin   23                                                                   // Led PCB
#define DhtPin   16                                                                   // Data sensor DHT22
#define LedOnPin 17                                                                   // ON led
#define VoltPin  36                                                                   // Tensión, ADC1 CH0
#define Cor1Pin  32                                                                   // Corriente 1, ADC1 CH4
#define Cor2Pin  33                                                                   // Corriente 2, ADC1 CH5
#define Cor3Pin  39                                                                   // Corriente 3, ADC1 CH3
#define Cor4Pin  34                                                                   // Corriente 4, ADC1 CH6
#define Cor5Pin  35                                                                   // Corriente 5, ADC1 CH7
#define ZeroPin  25                                                                   // ISR por paso por cero 

bool OkDht = false;
double temDHT = 0.0, humDHT = 0.0;
uint8_t error = 140;

#include "wifiEsp32.h"
#include "html.h"

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

#include "RS485.h"

const String      Compiler = String(__DATE__);                                        // Obtenemos la fecha de la compilación
bool              debugger = false;
volatile bool     ISRCero = false;

bool RunCounting = false, OKZero = false;
uint8_t readVolt = 0, UpdateDiscovery = 0;
uint16_t Count = 1;
uint32_t WhMillis = 0, last_time = 0;
hw_timer_t * timer = NULL;

DHT dht(DhtPin, DHT22);

TaskHandle_t   xRunCpu0;

//************************************************************************************
// ISR  timer0 (1.5Seg sin refresco)
//************************************************************************************
void IRAM_ATTR resetModule() {
  esp_restart();                                                                      // Reset ESP32
}
//************************************************************************************
// ISR por paso por cero (cada 10ms a 50Hz)
//************************************************************************************
void IRAM_ATTR PasoCero() {
  detachInterrupt(digitalPinToInterrupt(ZeroPin));                                    // Deshabilitamos interrupción
  ISRCero = true;                                                                     // Indicamos interrupción
}

//************************************************************************************
// Configuración
//************************************************************************************
void setup() {
  pinMode (Aux, INPUT_PULLUP);
  pinMode (LedPin, OUTPUT);
  pinMode (ZeroPin, INPUT_PULLUP);
  pinMode (RS485Pin, OUTPUT);
  pinMode (VoltPin, INPUT);
  pinMode (Cor1Pin, INPUT);
  pinMode (Cor2Pin, INPUT);
  pinMode (Cor3Pin, INPUT);
  pinMode (Cor4Pin, INPUT);
  pinMode (Cor5Pin, INPUT);

  digitalWrite (RS485Pin, LOW);
  digitalWrite (LedPin, HIGH);

  analogSetWidth(12);                                                                 // Defecto 12 bits, (0 - 4095) (NO MUY LINEAL)
  analogSetAttenuation(ADC_11db);                                                     // Defecto atenuación 11db = 150mV-3.2V, 3.9v->4095 (int 3V = ADC 0.833V)
  adcAttachPin(VoltPin);

  Serial.begin(115200, SERIAL_8N1, 3, 1);                                             // RS485 UART0
  
  client.setBufferSize(1024);                                                         // Tamaño máximo de paquete MQTT
  
  NetworkFound = confiWifi();
  if (SPIFFS.begin(true)) {
    IniHtml();
    digitalWrite (LedPin, LOW);
  }
  dht.begin();

  uint32_t startime = millis ();
  bool statusZero = false;
  while ((millis () - startime) < 15) {
    yield();
    if (digitalRead(ZeroPin)) statusZero = true;
    if (statusZero && digitalRead(ZeroPin) == 0) {                            	      // Se produjo un pulso de 1 a 0?
      OKZero = true;
      break;
    }
  }
  adcStart(VoltPin);

  // Tarea de uso de cpu 0
  xTaskCreate(
    RunCpu0,                                                                          // Puntero a la función
    "Leer corriente",                                                                 // Nombre de la tarea
    2000,                                                                             // Tamaño de Stack (bytes)
    NULL,                                                                             // Sin parámetro
    4,                                                                                // Prioridad de la tarea
    NULL                                                                              // Task handle
  );

  timer = timerBegin(0, 80, true);                                                    // Timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);
  timerAlarmWrite(timer, 1500000, false);                                             // Periodo en us para watchdog
  timerAlarmEnable(timer);                                                            // ON ISR timer
  timerWrite(timer, 0);
}

//************************************************************************************
// Principal (núcleo 1)
//************************************************************************************
void loop() {
  mqttclient.loop();
  if (conenctWebAP && (tWebAP + 35000) < millis()) {                          	      // Si estamos conectados al la pagina web del AP y sin actualizaciones
    conenctWebAP = false;                                                     	      // indicamos desconectado
    digitalWrite (LedPin, LOW);
  }
  if (Count++ > 6000) {                                                       	      // Aproximadamente cada minuto (cada ciclo es de 10ms)
    if (readVolt < 5) readVolt++;                                             	      // esperamos 5 minutos para que el transformador se estabilice
    if (!NetworkFound)  {                                                     	      // si NO tenemos conexión reintentamos
      timerAlarmDisable(timer);
      NetworkFound = connectwifi (listNetworks());
      if (!NetworkFound) error = 7;
      timerAlarmEnable(timer);
      timerWrite(timer, 0);
    } else {
      if (!WiFi.isConnected()) {                                              	      // Comprobamos si perdimos la conexión
        timerAlarmDisable(timer);
        WiFi.reconnect();
        delay(250);
        error = 70;
        timerAlarmEnable(timer);
        timerWrite(timer, 0);
        if (WiFi.status() != WL_CONNECTED) {
          NetworkFound = false;
          Count = 5200;
          error = 14;                                                         	      // NO se pudo conectar
        } else {
          error = 70;                                                         	      // se re-conecto
        }
      } else {
        error = 140;                                                          	      // OK conexión
      }
    }
    Count = 1;                                                                	      // Iniciamos contador ciclo de 10ms
    if (++UpdateDiscovery == 5) {
      timerWrite(timer, 0);                                                           // Refresco watchdog
      if (NetworkFound == 1) discovery();
      UpdateDiscovery = 0;
    }
  }
  timerWrite(timer, 0);                                                               // Refresco watchdog
  if (Count % error == 0 && !digitalRead(LedOnPin)){
    digitalWrite (LedPin, !digitalRead(LedPin));                                      // Parpadeo led
  }
  if (Count % 3000 == 0) {                                                    	      // Aproximadamente cada 30 segundos
    if (readVolt == 5 && OKZero) {
      tension = Tension();                                                    	      // ≈400ms
      Count = Count + 40;                                                     	      // 40 * 10 = 400ms
    }
  } else if (Count % 2000 == 0 && NetworkFound == 1) {                        	      // Aproximadamente cada 20 segundos
    if (!publiTopics()) error = 21;
    delay (5);
  } else if (Count == 2500) {                                                 	      // Aproximadamente cada 60 segundos
    delay (4);
    temDHT = dht.readTemperature();                                           	      // ≈6ms
    RunCounting = true;
  } else if (Count == 3500) {                                                 	      // Aproximadamente cada 60 segundos
    delay (4);
    humDHT = dht.readHumidity();                                              	      // ≈6ms
    if (isnan(humDHT) || isnan(temDHT)) {
      OkDht = false;
    }
    else {
      OkDht = true;
    }
    RunCounting = true;
  } else if (Count == 3950 && NetworkFound == 1) {                            	      // Aproximadamente cada 60 segundos
    if (gettime(true) && diaActual != timeinfo.tm_mday) {                     	      // Nuevo día?
      WhMillis = millis();
      last_time = WhMillis;
      powerWh = 0.0;                                                                  // Iniciamos acumulador Kwh, los Kwh se acumulan por días
      diaActual = timeinfo.tm_mday;
    }
  } else {
    if (RS485recvMsg()) {                                                    	        // Delay 10ms
      sendMsg ();
    }
  }
}
//**************************************************************************
// Gestionamos la publicación de los topic hacia el servidor mqtt
//**************************************************************************
bool publiTopics () {
  if (!mqttclient.connected()) {                                                      // Si estamos desconectados de servidor mqtt
    if (!mqttConnect()) {                                                             // intentamos conectar
      return false;
    }
  }
  while (RunCounting) {}                                                              // Esperamos fin cálculos, si no, salimos por RESET....
  PublishMqtt(topicTension, tension);                                                 // Tensión RED
  PublishMqtt(topicCorriente1, corrent [0]);                                          // Intensidad Enchufes
  PublishMqtt(topicCorriente2, corrent [1]);                                          // Intensidad Cocina
  PublishMqtt(topicCorriente3, corrent [2]);                                          // Intensidad Terraza
  PublishMqtt(topicCorriente4, corrent [3]);                                          // Intensidad Aire Acondicionado
  PublishMqtt(topicCorriente5, corrent [4]);                                          // Intensidad Luces
  PublishMqtt(topicCorrienteT, correntT);                                             // Intensidad total
  PublishMqtt(topicPotenciaT, powerT);                                                // Potencia total en W
  PublishMqtt(topicPotencia, (powerWh / 1000.0));                                     // Potencia en Kwh
  PublishMqtt(topicTcpu, ((temprature_sens_read() - 32) / 1.8));                      // Temperatura interna CPU en grados centígrados
  if (OkDht) {                                                                 	      // Nos aseguramos de haber leído la temperatura y la humedad
    PublishMqtt(topicTemDHT, temDHT);
    PublishMqtt(topicHumDHT, humDHT);
  }
  if (Count == 4000) PublishMqttT(topicClock, timetxt);
  return true;
}

/*************************************************************************************
  Se realizan 40 lecturas de la tensión de red después de 5ms del paso por cero (Vp AC-50Hz)
  Calculo tensión red con transformador de 230V a 12V. Tensión sin carga 18.80V
  1- tensión pico entrada para 19V =raíz cuadrada de 2->1.4142 * 19 = 26.87V
  2- tensión pico pico de entrada 26.87 * 2 = 53.74V
  3- tensión pico de salida
  Tensión pico sal. = k * tensión pico ent.
          R16          10K
  K = ------------ = ------- = 0,04347
       R16 + R15      230K

  tensión pico de salida = 0,04347 * 26.87= 1.168

  máx = 1.65 + 1.168 = 2.268V (+11% 2.517V)
  mín = 1.65 - 1.168 = 0.482V
  Tensión en entrada ADC = (ADC * 3.3 ) / 4095;
    --------------------------10ms (50Hz)
    |    |    ----------------≈6ms
    |    |    |  |
    | ** |    | **------------2.268V (ADC 2815)
    |*  *|    |*  *
    o----o----o----o----------ISRCero
   *      *  *      *
 **        **        **------0.482V
  Calibrado a 236.5V en red ->ADC 2.92Vpico ->3624
*************************************************************************************/
double Tension() {
  uint8_t Vcycle = 40, Vpico = 0;
  uint16_t valADC = 0;
  uint32_t averageVolt = 0;
  double Vfactor = 0.0925;
  while (Vcycle--) {
    attachInterrupt (digitalPinToInterrupt(ZeroPin), PasoCero, RISING);
    do {
      asm volatile ("nop");                                                   	      // Espera mínima
    } while (!ISRCero);                                                       	      // ISR paso por cero?
    delayMicroseconds(6000);                                                  	      // 6ms
    valADC = analogRead(VoltPin);
    yield();                                                                          // Refrescamos WatchDog
    ISRCero = false;
    if (valADC > 1500) {                                                      	      // valor pico positivo?
      averageVolt = averageVolt + valADC;
      Vpico++;
    }
  }
  averageVolt = averageVolt / Vpico;                                          	      // obtenemos la media de las lecturas de los valores pico positivos
  double VoltRms = ((0.70710678118 * averageVolt) * Vfactor);                         // obtenemos la tensión rms/eficaz (1/raíz cuadrada de 2 * Vp)
  if (VoltRms > 245 || VoltRms < 215) VoltRms = 230.0;                        	      // Mínimo valido 215V, máximo valido 250V
  return VoltRms;
}

/*************************************************************************************
  CPU 0 -lectura de los cinco transformadores de intensidad-
  Transformador corriente TA17L-04 (2000:1):
  cálculos de la tensión de salida para una corriente de entrada de 20A de salida 10mA
  1- corriente pico entrada para 20A = raíz cuadrada de 2->1.4142 * 20 = 28.28A
  2- corriente pico salida 28.28  / 2000 (relación transformador 1:2000) = 0.01414
  3- tensión pico = corriente pico salida 0.01414 * 47 (resistencia burden) = 0.664V
  5- tensión en divisor de tensión 3.3v / 2 = 1.65V
  6- tensión pico máxima salida 1.65V + 0.664 = 2.32V
  7- tensión pico mínima salida 1.65V - 0.664 = 0.986V
  8- rango 2.32 - 0.986 = 1.334 V
  -----------------------------20ms (50Hz)
  |         |
  |   **    |   **------------2.32V
  |  *  *   |  *  *
  | *    *  | *    *
  |*      * |*      *
 **        **        **------0.986V
 *        
 *        medido en entrada si transformador 1,63V
 *        con sonda del osciloscopio medido 1,32V
 *        4,22A con resistencia de 1000W
 *        honda senoidal de -138mV a 1,40mv diferencia de 278mv
*************************************************************************************/
void RunCpu0 (void * parameter) {
  int8_t Icycle = 0;
  uint8_t readPin = Cor1Pin, ImaxRead;
  uint16_t  Isample, Nsample = 1480;
  double Ioffset = 2048.00;                                                           // 2048.0 = 4096/2 (3.3/2 =1.65V 12 bits)
  double Ifactor = 22.050, Ifiltered, Iadd, Isq, IminRead = 0.25;
  // Estabilización lecturas...
  adcAttachPin(Cor1Pin);
  delayMicroseconds(100);
  adcStart(Cor1Pin);
  delay (10);
  Isample = analogRead(Cor1Pin);

  adcAttachPin(Cor2Pin);
  delayMicroseconds(100);
  adcStart(Cor2Pin);
  delay (10);
  Isample = analogRead(Cor2Pin);

  adcAttachPin(Cor3Pin);
  delayMicroseconds(100);
  adcStart(Cor3Pin);
  delay (10);
  Isample = analogRead(Cor3Pin);

  adcAttachPin(Cor4Pin);
  delayMicroseconds(100);
  Isample = adcStart(Cor4Pin);
  delay (10);
  Isample = analogRead(Cor4Pin);

  adcAttachPin(Cor5Pin);
  delayMicroseconds(100);
  adcStart(Cor5Pin);
  delay (10);
  Isample = analogRead(Cor5Pin);

  // Iniciamos lecturas...
  while (true) {
    switch (Icycle) {
      case 0:                                                                 	      // Intensidad Enchufes
        readPin = Cor1Pin;
        Ifactor = 22.025;
        ImaxRead = 15;
        IminRead = 0.25;
        break;
      case 1:                                                                 	      // Intensidad Cocina
        readPin = Cor2Pin;
        Ifactor = 22.050;
        ImaxRead = 20;
        IminRead = 0.25;
        break;
      case 2:                                                                 	      // Intensidad Terraza
        readPin = Cor3Pin;
        Ifactor = 22.048;
        ImaxRead = 20;
        IminRead = 0.25;
        break;
      case 3:                                                                 	      // Intensidad Aire Acondicionado
        readPin = Cor4Pin;
        Ifactor = 22.050;
        ImaxRead = 15;
        IminRead = 0.30;
        break;
      case 4:                                                                 	      // Intensidad Luces
        readPin = Cor5Pin;
        Ifactor = 22.050;
        ImaxRead = 10;
        IminRead = 0.20;
        break;
    }
    delay (1);
    for (uint16_t x = 0; x < Nsample; x++) {                                          // ≈20ms(dos periodos)
      Isample = analogRead(readPin);
      Ioffset = (Ioffset + (Isample - Ioffset) / 4096);
      Ifiltered = Isample - Ioffset;
      Isq = Ifiltered * Ifiltered;
      Iadd += Isq;
    }
    yield();
    RunCounting = true;
    double Iratio = Ifactor * 0.001611328125;                                         // 3.3 /2048 = 0.001611328125
    double Irms = Iratio * sqrt(Iadd / Nsample);                                      // Raíz cuadrada
    Iadd = 0;                                             
    if (Irms < IminRead || Irms > ImaxRead) {                                         // Fuera del rango mínimo o máximo
      corrent[Icycle] = 0.0;
    } else {
      corrent[Icycle] = Irms;
    }
    if (++Icycle == 5) {
      correntT = corrent[0] + corrent[1] + corrent[2] + corrent[3] + corrent[4] ;
      powerT = correntT * tension;
      if (powerT > 10000.0) powerT = 0.0;                                             // Fuera del rango máximo
      last_time = WhMillis;
      WhMillis = millis();
      powerWh = powerWh +  powerT * ((WhMillis - last_time) / 3600000.0);
      Icycle = 0;
    }
    RunCounting = false;                                                    	        // Aproximadamente 20ms                  	
  }
  //No se tendría que ejecutar nunca, en caso de que lo haga se elimina la función
  vTaskDelete(NULL);
}
