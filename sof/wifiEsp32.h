#ifndef _WifiEsp32_
#define _WifiEsp32_
/*
Gestión del uso del modulo wifi, conexión, escaneo, modo estación, modo punto de acceso

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
#include <WiFi.h>
#include <ESPAsyncWebServer.h>                                           		        // Librería para implementar un servidor web https://github.com/me-no-dev/ESPAsyncWebServer
#include <ESPmDNS.h>                                                     		        // Librería para servido DNS

#define NAME "Consumo"                                                   		        // Nombre del punto de acceso de la red WiFi
#define PASS "esp32Consumo"                                              		        // Contraseña del punto de acceso si falla la conexión a la red WiFi

const char*       ntpServer = "hora.roa.es";                             		        // 150.214.94.5
const long        gmtOffset_sec = 3600;
const int         daylightOffset_sec = 3600;

const char*       ssid = "#########";                                     		        // Nombre del punto de accesos a conectar
const char*       password =  "********";                               		        // Contraseña para el punto de accesos a conectar        
bool              NetworkFound = false;                                   		        // True si la red WiFi está conectada
bool              APmode = false;                                         		        // Indicación modo Ap
bool              http_response_flag = false;                             		        // Solicitud de respuesta
char              timetxt[9];                                             		        // Hora info. NTP
char              datetxt[9];                                             		        // Fecha info. NTP
struct tm         timeinfo;

uint8_t diaActual;

#include "mqtt.h"
AsyncWebServer server(80);

void callback(char* topic, byte* payload, unsigned int length);

//************************************************************************************
// Recuperar la hora local del servidor NTP y convertir  a una cadena.
//************************************************************************************
bool gettime(bool force = false) {
  static int16_t delaycount = 0;                                            	        // Para reducir el número de solicitudes NTP
  static int16_t retrycount = 100;
  if (!NetworkFound) return false;
  //if (timeinfo.tm_isdst == 0) {                                           	        // horario verano =1, invierno=0
  //  Inv = 1;
  //}

  if (--delaycount <= 0 || force) {                                         	        // Sync. cada pocas horas
    delaycount = 7200;                                                      	        // Reset contador
    if (timeinfo.tm_year) {                                                 	        // Hora valida encontrada?
    }

    if (!getLocalTime (&timeinfo)) {                                        	        // Leer del servidor NTP
      timeinfo.tm_year = 0;                                                 	        // Establecer la hora actual como ilegal
      if (retrycount) {                                                     	        // Sincronizar?
        retrycount--;                                                       	        // No, re-intentar de nuevo
        delaycount = 5;                                                     	        // Re-intentar pasados 5 segundos
      }
      return false;
    }
    else {
      sprintf (timetxt, "%02d:%02d:%02d",                                   	        // Formato de nueva hora a una cadena
               timeinfo.tm_hour,                                            	        // horas 0-23
               timeinfo.tm_min,                                             	        // minutos 0-59
               timeinfo.tm_sec);                                            	        // segundos 0-59
      sprintf (datetxt, "%02d/%02d/%02d",
               timeinfo.tm_mday,                                            	        // día 1-31
               timeinfo.tm_mon + 1,                                         	        // mes 0-11
               timeinfo.tm_year - 100);                                     	        // año (inicio 1900)
    }
  }
  return true;
}

//************************************************************************************
// Listar las redes disponibles.
// Las redes aceptables son aquellas que tienen una entrada en las preferencias.
// Los SSID de las redes disponibles se guardarán para su uso en la interfaz web.
//************************************************************************************
bool listNetworks() {
  bool OK = false;
  int8_t  numSsid;
  numSsid = WiFi.scanNetworks();

  if (numSsid <= 0) {
    return OK;
  }

  for (uint8_t i = 0; i < numSsid; i++) {
    if (strcmp (WiFi.SSID(i).c_str(), ssid) == 0) {
      OK = true;
      if (APmode) {
        WiFi.softAPdisconnect(true);
        APmode = false;
      }
    }
  }
  return OK;
}
//************************************************************************************
// Activamos y configuramos el modo AP
//************************************************************************************
void modeAP() {
  // Canal RF 6, ISSD ON, 1 conexión
  WiFi.softAP(NAME, PASS, 6, 0, 1);                                         	        // Este ESP32 será un AP
  NetworkFound = false;
  APmode = true;
  error = 90;
}
//************************************************************************************
// Conectar a WiFi usando el SSID
// Si la conexión falla, se crea un AP y la función devuelve falso.
//************************************************************************************
bool connectwifi(bool connec) {
  if (connec) {
    if (!APmode) {                                                          	        // si NO estamos en modo AP
      WiFi.softAPdisconnect(true);                                          	        // Si todavía se conserva la conexión antigua
    }
    WiFi.begin (ssid, password);                                            	        // Conectar a SSID único

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {                      	        // Tratar de conectar
      if (!APmode) modeAP();                                                	        // Error al conectar, configuración AP local
      return false;
    } else {
      uint8_t macAddr[6];
      WiFi.macAddress(macAddr);

      delay (300);
      gettime();
      diaActual = timeinfo.tm_mday;
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      mqttIni();
      APmode = false;
      return true;
    }
  } else {
    if (!APmode)
      modeAP();                                                               	      // ISSD NO encontrado, configuración AP local
    return false;
  }
}
//************************************************************************************
// Configuración conexión WIFI
//************************************************************************************
bool confiWifi() {
  WiFi.mode (WIFI_STA);                                                       	      // Este ESP32 es una estación
  WiFi.persistent (false);                                                    	      // No se guardar SSID y contraseña
  WiFi.disconnect();                                                          	      // Después de reiniciar el router todavía podría estar conectado
  delay (100);

  NetworkFound = connectwifi(listNetworks());                                 	      // Buscamos redes disponible y conectar a red WiFi
  return NetworkFound;
}
#endif
