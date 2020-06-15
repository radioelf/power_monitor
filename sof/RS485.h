#ifndef _RS485_
#define _RS485_
/*
  Gestión de petición  y envió de datos a través de RS485
  la trama de petición está compuesta de 3 bytes (STX ID ETX)
  la trama de envió de los datos está compuesta de 52 bytes
  STX - ID - 49 bytes datos - CRC - ETX

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
#define STX  '\2'
#define ETX  '\3'
#define ID   0x01

union double_Byte {
  double    datoF;
  uint8_t  datoB[4];
} unionFB;

/*************************************************************************************
  CRC-8
*************************************************************************************/
uint8_t CRC8(const uint8_t *dataTx, uint8_t dataLength) {
  uint8_t crc = 0x00;
  while (dataLength--) {
    uint8_t x = *dataTx++;
    for (uint8_t i = 8; i; i--) {
      uint8_t sum = (crc ^ x) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      x >>= 1;
    }
  }
  return crc;
}

/*************************************************************************************
  Enviar un mensaje a través de RS485, con todos los datos en una sola trama
  STX,  ID, datos, crc y EXT (52 bytes)
*************************************************************************************/
void sendMsg () {
  uint8_t dataTx[49], x, i = 0;
  unionFB.datoF = tension;
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 0 - 3
  }
  for (uint8_t y = 0; y < 5; y++) {
    unionFB.datoF = corrent [y];
    for (x = 0; x < 4; x++) {
      dataTx [i++] = unionFB.datoB[x];                                        	      // dataTx 4 - 23
    }
  }
  unionFB.datoF = correntT;
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 24 - 27
  }
  unionFB.datoF = powerT;
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 28 - 31
  }
  unionFB.datoF = (powerWh / 1000.0);
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 32 - 35
  }
  unionFB.datoF = ((temprature_sens_read() - 32) / 1.8);
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 36 - 39
  }
  unionFB.datoF = temDHT;
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 40 - 43
  }
  unionFB.datoF = humDHT;
  for (x = 0; x < 4; x++) {
    dataTx [i++] = unionFB.datoB[x];                                          	      // dataTx 44 - 47
  }
  dataTx[48] = CRC8(dataTx, 48);                                              	      // CRC de dataTx de 0 a 47
  
  digitalWrite (RS485Pin, HIGH);
  digitalWrite (LedPin, HIGH);
  Serial.write (STX);                                                         	      // STX ->inicio del paquete
  Serial.write (ID);                                                          	      // ID
  for (x = 0; x < 49; x++) {
    Serial.write(dataTx [x]);                                                 	      // datos + crc, 0-48 (TX 49 bytes)
  }
  Serial.write(ETX);                                                          	      // ETX -> fin del paquete
  digitalWrite (LedPin, LOW);
  digitalWrite (RS485Pin, LOW);
}

/*************************************************************************************
  Recepción de la trama de petición de datos, STX ID ETX (0x02 0x01 0x03)
  retorna false si no se recibió nada o un error sino retorna true
*************************************************************************************/
bool RS485recvMsg () {
  uint32_t start_time = millis ();
  bool Ini = false, IdOk = false;
  while ((millis () - start_time) < 10) {                                     	      // Permanecerá como máximo 10ms
    if (Serial.available () > 0) {
      int8_t inByte = Serial.read ();
      switch (inByte) {
        case STX:                                                             	      // Inicio
          if (!Ini) {
            Ini = true;
            start_time = millis ();                                           	      // reiniciamos el tiempo de espera
            break;
          }
          break;
        case ETX:                                                             	      // Fin
          if (IdOk) return true;
          break;
        default:
          if (!Ini)                                                           	      // Tenemos el inicio de la trama?
            break;                                                            	      // NO
          if  (inByte == ID && IdOk == false) {
            IdOk = true;
            break;
          }
          else
            return false;
      }
    }
    yield();
  }
  return false;                                                               	      // Timeout
}
#endif
