#ifndef _HTML_
#define _HTML_
/*
  Gestión pagina WEB en modo punto de acceso
  
  Variables a enviar a pagina WEB:
  Tensión RED = tensión
  Intensidad total = correntT
  Potencia = powerkwh

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
bool conenctWebAP = false;
uint32_t tWebAP = 0;

double corrent [5] = {0.0}, powerT = 0.0, correntT = 0.0, tension = 230.0, powerWh = 0.0;

//************************************************************************************
// Gestión de la salida después de una solicitud http.
//************************************************************************************
String GestionWeb(uint8_t orden) {
  tWebAP = millis();
  error = 90;
  switch (orden) {
    case 1:
      digitalWrite (LedPin, HIGH);
      conenctWebAP = true;
      return String();
    case 2:
      return String (round (correntT));
    case 3:
      return String (round (powerWh/1000.0));
    case 4:
      return String(tension) + "V";
    default:
      break;
  }
  return String();
}

//************************************************************************************
// Inicializamos servidor http
//************************************************************************************
void IniHtml() {
  server.on("/js/highcharts.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/js/highcharts.js", "text/javascript");
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    GestionWeb(1);
    request->send(SPIFFS, "/index.html");
  });
  server.on("/corriente", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GestionWeb(2).c_str());
  });
  server.on("/potencia", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GestionWeb(3).c_str());
  });
  server.on("/tension", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GestionWeb(4).c_str());
  });
  server.onNotFound([](AsyncWebServerRequest * request) {
    request->send(404, "text/plain", "404: Not found");
  });
  server.begin();
}
#endif
