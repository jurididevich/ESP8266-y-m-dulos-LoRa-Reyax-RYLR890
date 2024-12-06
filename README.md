A continuación se muestra el código traducido al español y las instrucciones para cada dispositivo. Todos los comentarios, instrucciones y texto están ahora en español. La estructura del proyecto asume que tendrás archivos .ino separados para cada nodo: TX_A.ino, TX_B.ino, TX_C.ino (transmisores), Movil.ino (móvil) y Central.ino (central).

**Estructura general del proyecto:**


- **TX_A.ino** - Código para el transmisor A  
- **TX_B.ino** - Código para el transmisor B (igual a A, solo cambia `deviceID`)  
- **TX_C.ino** - Código para el transmisor C (igual a A, solo cambia `deviceID`)  
- **Movil.ino** - Código para el nodo móvil (M)  
- **Central.ino** - Código para el nodo central (T: central nodo)

Antes de descargar el código, asegúrese de que:

- Configurado direcciones, NetworkID, Banda y Parámetros en cada módulo LoRa.
- Reemplazado el `deviceID` en TX_B.ino (en 'B') y TX_C.ino (en 'C').
- Añadida la lógica de medición RSSI al código del transmisor (ahora soporta el ejemplo con `random()`).
- Comentada la reconfiguración LoRa si es necesario, si ya está guardada en el módulo.
- Calibrados RSSI_ref y n en Movil.ino para cálculos precisos de distancia.

---

### Código de los transmisores (A, B, C)

**TX_A.ino** (para A; para B y C solo cambie `deviceID` y `AT+ADDRESS`):

```cpp
#include <SoftwareSerial.h>

// Conexión del módulo LoRa (RX, TX):
// Ajuste los pines según su circuito. Ejemplo: RX LoRa al pin 12 (TX ESP), TX LoRa al pin 14 (RX ESP)
SoftwareSerial lora(12, 14);  // Ajustar si es necesario

// Dirección del nodo móvil (M)
const int MOBILE_ADDRESS = 2; // M = 2

// Identificador del transmisor (A, B o C)
// IMPORTANTE: Cambie este valor en cada archivo de transmisor
char deviceID = 'A'; // En TX_B.ino usar 'B', en TX_C.ino usar 'C'

void setup() {
  Serial.begin(115200);
  lora.begin(9600); // Ajuste la velocidad si es necesario
  
  // Configuración inicial del módulo LoRa (ejecutar una sola vez)
  // Si el módulo ya está configurado y guarda la configuración, puede comentar estas líneas
  sendLoRaCmd("AT+ADDRESS=3"); // Para A = 3, B = 4, C = 5 (Ajuste según el archivo correspondiente)
  sendLoRaCmd("AT+NETWORKID=10");
  sendLoRaCmd("AT+BAND=915000000");
  sendLoRaCmd("AT+PARAMETER=10,7,1,7");
  
  Serial.print("Transmisor ");
  Serial.print(deviceID);
  Serial.println(" inicializado.");
}

void loop() {
  // Medición de RSSI (EJEMPLO)
  // IMPORTANTE: Reemplace esta parte con la lógica real para leer el RSSI de su módulo LoRa.
  // Algunos módulos LoRa permiten leer el RSSI con un comando AT (p.ej. "AT+RSSI?")
  // Revise la documentación del módulo para obtener el RSSI real.
  
  int rssi = random(-80, -50); // Reemplace con la lectura real del RSSI
  
  // Formar el mensaje a enviar
  // Formato: <ID>,<RSSI>
  String data = String(deviceID) + "," + String(rssi);
  String cmd = "AT+SEND=" + String(MOBILE_ADDRESS) + "," + String(data.length()) + "," + data + "\r\n";
  
  lora.print(cmd);
  Serial.print("Enviado: ");
  Serial.println(cmd);
  
  delay(7000); // Espera 7 segundos antes del próximo envío
}

// Función para enviar comandos AT y leer la respuesta
void sendLoRaCmd(String cmd) {
  lora.print(cmd + "\r\n");
  delay(200);
  while (lora.available()) {
    String resp = lora.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.println("Respuesta LoRa: " + resp);
    }
  }
}
```

**TX_B.ino**: Igual que TX_A.ino, pero con `deviceID = 'B'` y `AT+ADDRESS=4`.

```cpp
#include <SoftwareSerial.h>
SoftwareSerial lora(12, 14);

const int MOBILE_ADDRESS = 2; 
char deviceID = 'B'; // Cambiado a 'B'

void setup() {
  Serial.begin(115200);
  lora.begin(9600);

  // Configuración inicial LoRa (si no está ya hecha)
  sendLoRaCmd("AT+ADDRESS=4"); // B = 4
  sendLoRaCmd("AT+NETWORKID=10");
  sendLoRaCmd("AT+BAND=915000000");
  sendLoRaCmd("AT+PARAMETER=10,7,1,7");
  
  Serial.println("Transmisor B inicializado.");
}

void loop() {
  int rssi = random(-80, -50); // Ejemplo, reemplace con lectura real
  String data = String(deviceID) + "," + String(rssi);
  String cmd = "AT+SEND=" + String(MOBILE_ADDRESS) + "," + String(data.length()) + "," + data + "\r\n";

  lora.print(cmd);
  Serial.print("Enviado: ");
  Serial.println(cmd);

  delay(7000);
}

void sendLoRaCmd(String cmd) {
  lora.print(cmd + "\r\n");
  delay(200);
  while (lora.available()) {
    String resp = lora.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.println("Respuesta LoRa: " + resp);
    }
  }
}
```

**TX_C.ino**: Igual que TX_A.ino, pero con `deviceID = 'C'` y `AT+ADDRESS=5`.

```cpp
#include <SoftwareSerial.h>
SoftwareSerial lora(12, 14);

const int MOBILE_ADDRESS = 2; 
char deviceID = 'C'; // Cambiado a 'C'

void setup() {
  Serial.begin(115200);
  lora.begin(9600);

  // Configuración inicial LoRa (si no está ya hecha)
  sendLoRaCmd("AT+ADDRESS=5"); // C = 5
  sendLoRaCmd("AT+NETWORKID=10");
  sendLoRaCmd("AT+BAND=915000000");
  sendLoRaCmd("AT+PARAMETER=10,7,1,7");
  
  Serial.println("Transmisor C inicializado.");
}

void loop() {
  int rssi = random(-80, -50); // Ejemplo, reemplace con lectura real
  String data = String(deviceID) + "," + String(rssi);
  String cmd = "AT+SEND=" + String(MOBILE_ADDRESS) + "," + String(data.length()) + "," + data + "\r\n";

  lora.print(cmd);
  Serial.print("Enviado: ");
  Serial.println(cmd);

  delay(7000);
}

void sendLoRaCmd(String cmd) {
  lora.print(cmd + "\r\n");
  delay(200);
  while (lora.available()) {
    String resp = lora.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.println("Respuesta LoRa: " + resp);
    }
  }
}
```

---

### Código del nodo central (T) - Central.ino

```cpp
#include <SoftwareSerial.h>

// Conexión del módulo LoRa (RX, TX), ajuste pines según necesidad
SoftwareSerial lora(12, 14);

void setup() {
  Serial.begin(115200);
  lora.begin(9600);

  // Configuración inicial LoRa (una sola vez)
  // Comente estas líneas si el módulo ya está configurado
  sendLoRaCmd("AT+ADDRESS=1"); // T = 1
  sendLoRaCmd("AT+NETWORKID=10");
  sendLoRaCmd("AT+BAND=915000000");
  sendLoRaCmd("AT+PARAMETER=10,7,1,7");

  Serial.println("Nodo Central (T) inicializado.");
}

void loop() {
  if (lora.available()) {
    String receivedMessage = lora.readStringUntil('\n');
    receivedMessage.trim();

    if (receivedMessage.startsWith("+RCV=")) {
      Serial.print("Recibido desde M: ");
      Serial.println(receivedMessage);

      // Extracción de valores A, B, C del mensaje en formato A:XX.XX,B:YY.YY,C:ZZ.ZZ
      float distA = getValue(receivedMessage, "A");
      float distB = getValue(receivedMessage, "B");
      float distC = getValue(receivedMessage, "C");

      Serial.print("A: ");
      Serial.print(distA);
      Serial.print(", B: ");
      Serial.print(distB);
      Serial.print(", C: ");
      Serial.println(distC);

      // Aquí puede agregar lógica adicional, por ejemplo, enviar estos datos a internet.
    }
  }
  delay(50); // Pequeño retardo para evitar uso excesivo de CPU
}

// Función para extraer el valor asociado a una etiqueta (A, B o C)
float getValue(String data, String label) {
  int start = data.indexOf(label + ":") + 2; // +2 para saltar la etiqueta y ':'
  if(start == 1) return 0; // No se encontró la etiqueta
  int end = data.indexOf(',', start);
  if (end == -1) {
    end = data.length();
  }
  return data.substring(start, end).toFloat();
}

// Función para enviar comandos AT y leer la respuesta
void sendLoRaCmd(String cmd) {
  lora.print(cmd + "\r\n");
  delay(200);
  while (lora.available()) {
    String resp = lora.readStringUntil('\n');
    resp.trim();
    if (resp.length() > 0) {
      Serial.println("Respuesta LoRa: " + resp);
    }
  }
}
```

---

### Movil.ino

A continuación se muestra el ejemplo de código final para el nodo móvil (M) en el entorno Arduino IDE. El código está escrito en ESP8266 (por ejemplo, NodeMCU) con módulo LoRa Reyax RYLR890 y tiene en cuenta las recomendaciones anteriores. 

**Predisposiciones y condiciones:**

- Nodos:
    
    - T (Central): `AT+ADDRESS=1`.
    - M (Móvil): `AT+DIRECCIÓN=2`.
    - A: `EN+DIRECCIÓN=3`.
    - B: `EN+DIRECCIÓN=4`.
    - C: `EN+DIRECCIÓN=5`.
- Todos los nodos tienen el mismo `NETWORKID`, `BAND`, `PARAMETER`. Por ejemplo
    
    - `AT+NETWORKID=10`.
    - AT+BAND=915000000
    - `AT+PARAMETER=10,7,1,7`.
- Ajuste la velocidad UART a 9600 baudios para un funcionamiento más fiable con SoftwareSerial. En todos los módulos, ejecuta el comando `AT+IPR=9600` una vez (puede ser a través de Serial Monitor) para mantener la velocidad en el módulo LoRa.
    
- Formato del mensaje:  
    Los nodos A,B,C envían un identificador simple («A», «B», «C»). El módulo LoRa en M en la recepción genera una cadena de la forma:  
    `+RCV=<dirección de origen>,<longitud>,<contenido>,<RSSI>`.  
    Por ejemplo `+RCV=3,1,A,-57.0`.  
    Aquí `-57.0` es el RSSI añadido automáticamente por el módulo.
    
- El nodo móvil (M) recibe 3 mensajes de A,B,C, calcula las distancias y las envía al nodo T en el formato:  
    `A:XX.XX,B:YY.YY,C:ZZ.ZZ`.  
    Utilizando el comando `AT+SEND=1,<longitud>,<mensaje>`.
    
- Después de la primera configuración exitosa de los módulos (DIRECCIÓN, NETWORKID, BANDA, PARÁMETRO), estos comandos pueden ser comentados en el código para evitar instalaciones repetidas. Si el módulo ya está configurado, comentar las llamadas a `sendLoRaCmd(«AT+...»)` en `setup()`.
    
- La depuración se puede observar en el Serial Monitor Arduino IDE.
    

**Código (Movil.ino):**

```cpp
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
```

Recomendaciones:**

- Si el módulo LoRa ya está preconfigurado, comenta las líneas en `setup()` con `sendLoRaCmd(«AT+...»)` para ahorrar tiempo y evitar configuraciones repetidas.
- Calibre RSSI_ref y n si es necesario.
- Considera el uso de una UART hardware o una librería RadioLib para mejorar la estabilidad.
- Los transmisores (A,B,C) envían sólo un identificador («A», «B», «C»). El módulo LoRa en M añade automáticamente RSSI en la recepción, simplificando la lógica.
- El nodo T recibirá un mensaje de M con el formato `+RCV=2,<longitud>,A:XX.XX,B:YY.YY,C:ZZ.ZZ,<RSSI>` y podrá mostrarlo en el monitor.

Este código está listo para ser cargado en el entorno Arduino IDE para ESP8266 (NodeMCU).


---

### Instrucciones finales:

1. **Estructura del proyecto:**  
   - Cree cinco archivos .ino: TX_A.ino, TX_B.ino, TX_C.ino, Movil.ino, Central.ino.  
   - Pegue el código correspondiente en cada archivo.

2. **Cambios para cada transmisor:**  
   - En TX_A.ino: `deviceID = 'A'` y `AT+ADDRESS=3` en setup().  
   - En TX_B.ino: `deviceID = 'B'` y `AT+ADDRESS=4` en setup().  
   - En TX_C.ino: `deviceID = 'C'` y `AT+ADDRESS=5` en setup().

3. **Configuración del LoRa:**  
   - Asegúrese que todos los nodos tienen el mismo `NETWORKID`, `BAND` y `PARAMETER`.  
   - Ajuste la dirección (ADDRESS) única para cada nodo (A=3, B=4, C=5, M=2, T=1).

4. **Calibración y RSSI real:**  
   - En los transmisores (A,B,C), reemplace la lógica de `rssi = random(...)` con la medición real del RSSI según la documentación de su módulo LoRa.  
   - En el nodo M, ajuste RSSI_ref y n para obtener distancias precisas.  
   - En el nodo central T, si desea enviar los datos a internet u otra plataforma, agregue el código correspondiente.

5. **Probar el sistema:**  
   - Suba cada archivo al dispositivo correspondiente (A,B,C,M,T).  
   - Encienda todos los nodos.  
   - A,B,C enviarán datos a M. M calculará distancias y enviará a T. T mostrará en el monitor serial las distancias A,B,C.







