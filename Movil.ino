#include <SoftwareSerial.h>

// Pines recomendados (ajustar según su wiring):
// Conecte TX del LoRa al pin RX del ESP8266 (por ejemplo, D7=GPIO13)
// Conecte RX del LoRa al pin TX del ESP8266 (por ejemplo, D8=GPIO15)
SoftwareSerial lora(13, 15); // RX=13, TX=15 para ESP8266

// Parámetros para cálculo de distancia
float RSSI_ref = -57.46; // RSSI a 1 m (ajustar experimentalmente)
float n = 1.8;           // Factor ambiental (ajustar experimentalmente)

// Arreglos para almacenar info de A, B, C
String messages[3];
bool received[3] = {false, false, false};
int messageCount = 0;

void setup() {
  Serial.begin(115200);
  lora.begin(9600); // Debe coincidir con AT+IPR=9600 en el módulo LoRa
  
  delay(1000);
  Serial.println("Iniciando M (Movil)...");

  // Configurar módulo LoRa una sola vez. Si ya configurado, comentar estas líneas.
  sendLoRaCmd("AT+ADDRESS=2");
  sendLoRaCmd("AT+NETWORKID=10");
  sendLoRaCmd("AT+BAND=915000000");
  sendLoRaCmd("AT+PARAMETER=10,7,1,7");

  Serial.println("Configuración completada. Esperando mensajes de A, B, C...");
}

void loop() {
  // Esperar hasta recibir 3 mensajes (A,B,C)
  while (messageCount < 3) {
    if (lora.available()) {
      String receivedMessage = lora.readStringUntil('\n');
      receivedMessage.trim();

      // Ejemplo esperado: +RCV=3,1,A,-57.0
      if (receivedMessage.startsWith("+RCV=")) {
        Serial.println("Mensaje recibido: " + receivedMessage);

        // Buscar comas
        int pos1 = receivedMessage.indexOf(',');
        int pos2 = (pos1 != -1) ? receivedMessage.indexOf(',', pos1 + 1) : -1;
        int pos3 = (pos2 != -1) ? receivedMessage.indexOf(',', pos2 + 1) : -1;
        int pos4 = (pos3 != -1) ? receivedMessage.indexOf(',', pos3 + 1) : -1;

        if (pos4 != -1) {
          // Extracto de campos
          // pos2+1 a pos3: identificador (A,B,C)
          String remitente = receivedMessage.substring(pos2 + 1, pos3);
          remitente.trim();
          // pos3+1 a pos4: RSSI
          String rssiStr = receivedMessage.substring(pos3 + 1, pos4);
          rssiStr.trim();

          float rssiValue = rssiStr.toFloat();
          float diff = RSSI_ref - rssiValue;
          float distance = pow(10, (diff / (10 * n)));

          // Índice según remitente
          int idx = -1;
          if (remitente == "A") idx = 0;
          else if (remitente == "B") idx = 1;
          else if (remitente == "C") idx = 2;

          if (idx != -1 && !received[idx]) {
            messages[idx] = "ID:" + remitente + ",Distancia:" + String(distance, 2);
            received[idx] = true;
            messageCount++;
            Serial.println("Guardado: " + messages[idx]);
          }
        } else {
          Serial.println("Formato de mensaje no válido.");
        }
      } else {
        Serial.println("Mensaje no reconocido: " + receivedMessage);
      }
    }
    delay(50); // Pequeño retardo para no saturar CPU
  }

  // Al tener A,B,C
  String distA = extraerDistancia(messages[0]);
  String distB = extraerDistancia(messages[1]);
  String distC = extraerDistancia(messages[2]);

  // Enviar a T (ADDRESS=1)
  String msgToT = "A:" + distA + ",B:" + distB + ",C:" + distC;
  String cmdToT = "AT+SEND=1," + String(msgToT.length()) + "," + msgToT;

  Serial.println("Enviando datos a T: " + cmdToT);
  lora.print(cmdToT + "\r\n");
  delay(500);

  // Reiniciar para siguiente ciclo
  messageCount = 0;
  for (int i = 0; i < 3; i++) {
    received[i] = false;
    messages[i] = "";
  }

  // Pausa antes de siguiente lectura
  delay(5000);
}

// Función para extraer la distancia de la cadena "ID:X,Distancia:YY.YY"
String extraerDistancia(String mensaje) {
  int pos = mensaje.indexOf("Distancia:");
  if (pos != -1) {
    return mensaje.substring(pos + 10); 
  }
  return "0.00";
}

// Función para enviar comandos AT y leer respuesta
void sendLoRaCmd(String cmd) {
  lora.print(cmd + "\r\n");
  delay(200);
  while (lora.available()) {
    String resp = lora.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.println("LoRa Respuesta: " + resp);
    }
  }
}

