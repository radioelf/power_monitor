#ifndef _MQTTEsp32_
#define _MQTTEsp32_
/*
  Envió al servidor mosquitto de los datos a través de los topic

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
*/

#include <PubSubClient.h>                                                     	      // Librería para MQTT https://github.com/knolleary/pubsubclient

const char*       mqttServer = "192.168.***.***";
const int         mqttPort = 1883 ;
const char*       mqttUser = "######";
const char*       mqttPassword = "*********";

const char* topicTension = "POWER-A2144E/sensor/tension";
const char* topicCorriente1 = "POWER-A2144E/sensor/enchufes";
const char* topicCorriente2 = "POWER-A2144E/sensor/cocina";
const char* topicCorriente3 = "POWER-A2144E/sensor/terraza";
const char* topicCorriente4 = "POWER-A2144E/sensor/a-a";
const char* topicCorriente5 = "POWER-A2144E/sensor/luces";
const char* topicCorrienteT = "POWER-A2144E/sensor/corriente";
const char* topicPotenciaT = "POWER-A2144E/sensor/W";
const char* topicPotencia = "POWER-A2144E/sensor/kwh";
const char* topicTcpu = "POWER-A2144E/cpu/temp";
const char* topicTemDHT = "POWER-A2144E/sensor/temperatura_ext";
const char* topicHumDHT = "POWER-A2144E/sensor/humedad_ext";
const char* topicClock = "POWER-A2144E/clock";
bool MQTT_RETAIN = false;                                                     	      // MQTT bandera retain

WiFiClient        wmqttclient;                                                	      // Instancia para mqtt
PubSubClient      mqttclient (wmqttclient);

//************************************************************************************
// Recepción mqtt
//************************************************************************************
void mqttCallback (char* topic, byte* message, unsigned int len) {
  String messageRx;
  for (int i = 0; i < len; i++) {                                             	      // creamos el mensaje carácter a carácter
    messageRx += (char)message[i];
  }
}

//************************************************************************************
// Conexión o reconexión al servidor mqtt
//************************************************************************************
bool mqttConnect() {
  if (!mqttclient.connect(mqttServer, mqttUser, mqttPassword)) {
    return false;
  }
  mqttclient.subscribe("POWER-A2144E/#");
  return true;
}
//************************************************************************************
// Publicamos  el topic y el valor double pasado
//************************************************************************************
void PublishMqtt(const char* topic, double value) {
  char topicState [strlen(topic) + 7];
  String StateTopic = String (topic) + "/state";
  StateTopic.toCharArray(topicState, StateTopic.length() + 1);
  char string_mqtt [9];
  dtostrf (value, 6, 2, string_mqtt);                                         	      // Double, número de caracteres, número de decimales, char resultante
  mqttclient.publish(topicState, string_mqtt);
}

//************************************************************************************
// Publicamos  el topic y la trama pasada
//************************************************************************************
void PublishMqttT(const char* topic, char* string_mqtt) {
  mqttclient.publish(topic, string_mqtt);
}

//************************************************************************************
// Envió periódico y autodeterminación pata Home Assistant
//Auto detección para Home Assistant <discovery_prefix>/<component>/[<node_id>/]<object_id>/config
//************************************************************************************
void discovery() {
  static bool autoDetec = 0;
  PublishMqttT("POWER-A2144E/app", (char*) "Radioelf");
  PublishMqttT("POWER-A2144E/version", (char*)  VERSION);
  PublishMqttT("POWER-A2144E/board", (char*) "Power_RADIOELF");
  PublishMqttT("POWER-A2144E/host", (char*) "Monitor Consumo");
  PublishMqttT("POWER-A2144E/desc", (char*) "POWER_MONITOR");
  PublishMqttT("POWER-A2144E/ssid", (char*) ssid);
  PublishMqttT("POWER-A2144E/ip", (char*) WiFi.localIP().toString().c_str());
  PublishMqttT("POWER-A2144E/mac", (char*) WiFi.macAddress().c_str());
  char payload[5];
  snprintf(payload, 4, "%d", WiFi.RSSI());
  PublishMqttT("POWER-A2144E/rssi", payload);

  if (autoDetec)
    PublishMqttT ("POWER-A2144E/status", (char*) "online");
  else
    PublishMqttT ("POWER-A2144E/status", (char*) "offline");

  uint8_t nTopic = 0;
  String ValueName = "Tension Red";
  String Name = "tension";
  String Unit = "V";
  String topicName = String (topicTension);
  String Icon = "transmission-tower";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String Mac = String(mac[0], 16) + String(mac[1], 16) + String(mac[2], 16) + String(mac[3], 16) + String(mac[4], 16) + String(mac[5], 16);
  while (nTopic < 11) {
    String message = "{";
    message += String(F("\"unit_of_measurement\":\"")) + Unit + String(F("\",\"icon\":\"mdi:")) + Icon + String(F("\",\""));
    message += String(F("name\":\"")) + ValueName + String(F("\",\""));
    message += String(F("state_topic\":\"")) + topicName  + String(F("/state\",\""));
    message += String(F("availability_topic\":\"")) + String(F("POWER-A2144E/status\",\""));
    message += String(F("unique_id\":\"")) + String(F("ESP32")) + Name + String(F("\",\""));
    message += String(F("device\":{\"identifiers\":\"")) + Mac + String(F("\",\""));
    message += String(F("name\":\"Power Monitor")) + String(F("\",\"sw_version\":\"PowerMonitor ")) + String (VERSION) + String(F("\",\"model\":\"Power1\",\"manufacturer\":\"Radioelf\"}"));
    message += "}";
   
    String topic = ("homeassistant/sensor/POWER-A2144E/" + Name + "/config"); 
    autoDetec = mqttclient.publish((char*) topic.c_str(), (char*) message.c_str(), MQTT_RETAIN);
    nTopic ++;
    switch (nTopic) {
      case 1:
        ValueName = "Intensidad Enchufes";
        Name = "corriente1";
        Unit = "A";
        topicName = String (topicCorriente1);
        Icon = "flash";
        break;
      case 2:
        ValueName = "Intensidad Cocina";
        Name = "corriente2";
        Unit = "A";
        topicName = String (topicCorriente2);
        Icon = "flash";
        break;
      case 3:
        ValueName = "Intensidad Terraza";
        Name = "corriente3";
        Unit = "A";
        topicName = String (topicCorriente3);
        Icon = "flash";
        break;
      case 4:
        ValueName = "Intensidad Climatizador";
        Name = "corriente4";
        Unit = "A";
        topicName = String (topicCorriente4);
        Icon = "flash";
        break;
      case 5:
        ValueName = "Intensidad Luces";
        Name = "corriente5";
        Unit = "A";
        topicName = String (topicCorriente5);
        Icon = "flash";
        break;
      case 6:
        ValueName = "Intensidad Total";
        Name = "corrienteT";
        Unit = "A";
        topicName = String (topicCorrienteT);
        Icon = "flash";
        break;
      case 7:
        ValueName = "Potencia W";
        Name = "potencia";
        Unit = "W";
        topicName = String (topicPotenciaT);
        Icon = "gauge";
        break;
      case 8:
        ValueName = "Potencia Kwh";
        Name = "potencia1";
        Unit = "Kwh";
        topicName = String (topicPotencia);
        Icon = "chart-bar";
        break;
      case 9:
        ValueName = "Temperatura Externa";
        Name = "temperaturaext";
        Unit = "°C";
        topicName = String (topicTemDHT);
        Icon = "thermometer";
        break;
      case 10:
        ValueName = "Humedad Externa";
        Name = "humedadext";
        Unit = "%";
        topicName = String (topicHumDHT);
        Icon = "water-percent";
        break;
    }
    delay (25);
  }
  if (autoDetec) {
    PublishMqttT ("POWER-A2144E/status", (char*) "online"); 
  }
}

//************************************************************************************
// Inicializamos mqtt
//************************************************************************************
void mqttIni() {
  mqttclient.setServer(mqttServer, mqttPort);
  //mqttclient.setCallback(mqttCallback);                                   	        // NO RX
  if (mqttclient.connect(mqttServer, mqttUser, mqttPassword)) {             	        // Acceder al broker MQTT
    discovery();
  }
}

#endif
