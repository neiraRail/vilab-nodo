#include <WiFi.h>
#include <NTPClient.h>  //sincronización de relojes
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <RAK12033-IIM42652.h>  // Librería sensor
#include <LittleFS.h>           // Sistema de archivos
#include <PubSubClient.h>       // MQTT client

#define FORMAT_LITTLEFS_IF_FAILED true

// Constantes para sincronizar la hora y enviar en formato UNIX
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;      // GMT-4 (invierno)
const int daylightOffset_sec = 0;  // GMT-3 (verano)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);


// Variables y constantes globales para UDP envio registros
IPAddress udpServerIP = IPAddress(200, 13, 4, 208);
uint16_t udpServerPort = 8080;  // Puerto del servidor


// Confuguración para MQTT
const char* mqtt_broker = "200.13.4.208";
const char* topic = "lab/data";
const char* mqtt_username = "RAK";
const char* mqtt_password = "_";
const int mqtt_port = 1883;

// Clientes udp, http, wifi y mqtt
WiFiUDP udp;
HTTPClient api;
WiFiClient espClient;
PubSubClient mqttclient(espClient);

// Variables y constantes globales de CONFIGURACION
int node = 2;  // Cambiar para cada nuevo nodo
int start = 0;
int batch_id = 0;
int time_update = 1;
int time_sensor = 500;  //ms
int time_reset = 0;
int active = 0;
int batch_size = 10;
String protocol = "http";
String send_mode = "stream";
String ssid = "oficinaCEIS";
String password = "cGr3ECF2rZq42EB";
String rest_server = "http://200.13.4.208:8080";  // server configuración

// Otras variables
time_t timeout_lec = 0, time_passed_update = 0;
int time_lap = 0;  //ms


float gyro_x_old = 0.0, gyro_y_old = 0.0, gyro_z_old = 0.0;
float acc_x_old = 0.0, acc_y_old = 0.0, acc_z_old = 0.0;
float sense_a = 0.00, sense_g = 0.00;  // misma unidad que la medida del accelerometro
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

SemaphoreHandle_t bigLock = NULL;

DynamicJsonDocument doc(2048);  // gestión JSON de configuración

IIM42652 IMU;

// defición segundo hilo de ejecución
TaskHandle_t task_loop1;

// Prototipos de funciones
bool wifi_connect(const String& ssid2, const String& password2);
bool load_setup_server();
bool load_setup();
void esploop1(void* pvParameters);
void loop1();  // Cada minuto actualiza la config, cada time_reset horas reinicia.
void loop();   // Lee una medición lo más rapido que puede (sobre time_sensor) y la envía por udp.
void setup();  //Inicio, Serial, FS, semáforo, wifi, config, reloj, wire, IMU y el segundo hilo.
void setup1();

// Inicio del programa, con el setup
void setup() {
  // Inicialización de los LEDs
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  timeout_lec = millis();  // registro de ventana de tiempo
  // Inicialización del puerto serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Esperar a que se establezca la conexión serial.
  }

// cuando no esta formateada la memoria flash
#ifdef TWOPART
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED, "/lfs2", 5, "part2")) {
    Serial.println("Fallo hacer la particion");
    return;
  }
  Serial.println("Partición formateada...");
#endif

  // monta la memoria flash
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("Fallo montar memoria flash");
    return;
  }

  // inicia semaforo
  bigLock = xSemaphoreCreateMutex();

  //carga de datos de configuración si hay le da preferencia, en caso contario usa por defecto

  Serial.println("Inicio Configuración:");
  load_setup();

  // Conexión a la red WiFi, intenta 3 veces sin tener que volver a reiniciarse, sino intenta con red por defecto.
  Serial.println("Inicio WIFI");
  if (!wifi_connect(ssid, password)) {
    if (!wifi_connect(ssid, password)) {
      if (!wifi_connect(ssid, password)) {
        if (!wifi_connect("sensor", "1234567890")) {
          Serial.print("Error WIFI");
          delay(2000);
          ESP.restart();  // Intentar conexión a una red por defecto con datos móviles para cargar el setup en línea.
        }
      }
    }
  }

  start += 1;
  // actualiza configuración
  delay(100);
  if (load_setup_server()) {
    load_setup();
  } else {
    Serial.println("Error al cargar Setup");
  }

  // Inicializar el cliente NTP y sincronizar la hora
  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }

  //HTTP para enviar data
  if (!api.begin(rest_server + "/lectura")) {
    Serial.println("Error API");
    for (;;)
      ;
  }

  //Inicializar MQTT
  mqttclient.setServer(mqtt_broker, mqtt_port);
  while (!mqttclient.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (mqttclient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(mqttclient.state());
      delay(2000);
    }
  }


    Wire.begin();
    Wire.setClock(400000);

    // Inicialización del sensor TDK
    if (IMU.begin() == false) {
      while (1) {
        Serial.println("IIM-42652 no está conectado.");
        delay(5000);
      }
    }

    IMU.accelerometer_enable();
    IMU.gyroscope_enable();
    IMU.temperature_enable();

    IMU.set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_16g);
    IMU.set_accel_frequency(IIM42652_ACCEL_CONFIG0_ODR_50_HZ);

    //IMU.enable_accel_low_power_mode();
    //IMU.wake_on_motion_configuration( ACCEL_X_THR, ACCEL_Y_THR, ACCEL_Z_THR);
    Serial.println("Sensor READY");

  // crea segundo hilo de ejecución
  #if defined(ESP_PLATFORM)
    xTaskCreatePinnedToCore(
      esploop1,               /* Task function. */
      "loop1",                /* name of task. */
      10000,                  /* Stack size of task */
      NULL,                   /* parameter of the task */
      1,                      /* priority of the task */
      &task_loop1,            /* Task handle to keep track of created task */
      !ARDUINO_RUNNING_CORE); /* pin task to core 0 */
  #endif

    // Apagar el LED para indicar el final del setup
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
  }

void setup1() {}

// Lectura y envío de mediciones según el parámetro de delay
void loop() {
  if (active == 1) {
    if (protocol != "http" && protocol != "udp" && protocol != "mqtt0" && protocol != "mqtt1" && protocol != "mqtt2") {
      Serial.println("No es un protocolo válido, utiliza 'http', 'udp', 'mqtt0', 'mqtt1' o 'mqtt2'");
      Serial.print("Se esperará: ");
      Serial.println(time_update);
      delay(time_update * 1000);
    }

    if (send_mode == "stream") {

      time_lap = (millis() - timeout_lec);  // la primera vez es un valor mayor para marcar la refrencia de valores de inicio
      // Serial.print("Tiempo pasado entre iteracion: ");
      // Serial.println(time_lap);
      if ((time_lap >= time_sensor)) {
        Serial.println("MODO STREAAAAM");

        digitalWrite(LED_GREEN, HIGH);  //LED verde de lectura se enciende
        timeout_lec = millis();

        IIM42652_axis_t accel_data;
        IIM42652_axis_t gyro_data;
        float temp;

        IMU.get_accel_data(&accel_data);
        IMU.get_gyro_data(&gyro_data);
        IMU.get_temperature(&temp);
        //  time_t now = timeClient.getEpochTime(); //Yo lo pondría aqui para que sea mas preciso
        acc_x = (float)accel_data.x / 2048;  //Por qué divide por 2048?
        acc_y = (float)accel_data.y / 2048;
        acc_z = (float)accel_data.z / 2048;
        gyro_x = (float)gyro_data.x / 16.4;
        gyro_y = (float)gyro_data.y / 16.4;
        gyro_z = (float)gyro_data.z / 16.4;

        if (((gyro_x_old - gyro_x) >= sense_g) or ((gyro_y_old - gyro_y) >= sense_g) or ((gyro_z_old - gyro_z) >= sense_g) or ((acc_x_old - acc_x) >= sense_a) or ((acc_y_old - acc_y) >= sense_a) or ((acc_z_old - acc_z) >= sense_a)) {
          gyro_x_old = gyro_x;
          gyro_y_old = gyro_y;
          gyro_z_old = gyro_z;
          acc_x_old = acc_x;
          acc_y_old = acc_y;
          acc_z_old = acc_z;

          time_t now = timeClient.getEpochTime();  // registra el tiempo de medición

          String json = "{\"time\": " + (String)now + ",";
          json += " \"delta\":" + (String)time_lap + ",";
          json += " \"node\":" + (String)node + ",";
          json += " \"acc_x\":" + (String)(acc_x) + ",";
          json += " \"acc_y\":" + (String)(acc_y) + ",";
          json += " \"acc_z\":" + (String)(acc_z) + ",";
          json += " \"gyr_x\":" + (String)(gyro_x) + ",";
          json += " \"gyr_y\":" + (String)(gyro_y) + ",";
          json += " \"gyr_z\":" + (String)(gyro_z) + ",";
          json += " \"mag_x\":0,";
          json += " \"mag_y\":0,";
          json += " \"mag_z\":0,";
          json += " \"temp\":" + (String)temp + ",";
          json += " \"start\":" + (String)start + "}";

          if (protocol == "http") {
            api.addHeader("Content-Type", "application/json");
            int httpResponseCode = api.POST(json);
            String json_update = api.getString();
            if (httpResponseCode != 200) {
              Serial.println("Error sending");
            }
            Serial.println(json_update);
          }
          if (protocol == "udp") {
            udp.beginPacket(udpServerIP, udpServerPort);
            udp.print(json);
            udp.endPacket();
          }
          if (protocol == "mqtt0") {
            // const int length = json.length();
            // char* payload = new char[length + 1];
            // json.toCharArray(payload, json.length() + 1);
            bool success = mqttclient.publish(topic, json.c_str());
            Serial.print("MQTT success: ");
            Serial.println(success);
          }
          if (protocol == "mqtt1") {
            Serial.println("MQTT1 no soportado");
            return;
          }
          if (protocol == "mqtt2") {
            Serial.println("MQTT2 no soportado");
            return;
          }

          if (time_lap > 150) { Serial.println(json); }  // imprime al inicio VALOR DE REGERENCIA
          digitalWrite(LED_GREEN, LOW);                  //LED verde de lectura se apaga
        }
      }
      return;
    } else if (send_mode == "batch") {
      Serial.println("MODO BATCH!!!");
      DynamicJsonDocument data(4096);
      batch_id++;
      data["id"] = batch_id;
      data["len"] = batch_size;
      JsonArray batch = data.createNestedArray("batch");
      int i = 0;
      while (i < batch_size) {

        time_lap = (millis() - timeout_lec);  // la primera vez es un valor mayor para marcar la refrencia de valores de inicio
        if ((time_lap >= time_sensor)) {

          digitalWrite(LED_GREEN, HIGH);  //LED verde de lectura se enciende
          timeout_lec = millis();

          IIM42652_axis_t accel_data;
          IIM42652_axis_t gyro_data;
          float temp;

          IMU.get_accel_data(&accel_data);
          IMU.get_gyro_data(&gyro_data);
          IMU.get_temperature(&temp);
          //  time_t now = timeClient.getEpochTime(); //Yo lo pondría aqui para que sea mas preciso
          acc_x = (float)accel_data.x / 2048;  //Por qué divide por 2048?
          acc_y = (float)accel_data.y / 2048;
          acc_z = (float)accel_data.z / 2048;
          gyro_x = (float)gyro_data.x / 16.4;
          gyro_y = (float)gyro_data.y / 16.4;
          gyro_z = (float)gyro_data.z / 16.4;

          if (((gyro_x_old - gyro_x) >= sense_g) or ((gyro_y_old - gyro_y) >= sense_g) or ((gyro_z_old - gyro_z) >= sense_g) or ((acc_x_old - acc_x) >= sense_a) or ((acc_y_old - acc_y) >= sense_a) or ((acc_z_old - acc_z) >= sense_a)) {
            gyro_x_old = gyro_x;
            gyro_y_old = gyro_y;
            gyro_z_old = gyro_z;
            acc_x_old = acc_x;
            acc_y_old = acc_y;
            acc_z_old = acc_z;

            time_t now = timeClient.getEpochTime();  // registra el tiempo de medición


            JsonObject lectura = batch.createNestedObject();
            lectura["time"] = now;
            lectura["delta"] = time_lap;
            lectura["node"] = node;
            lectura["acc_x"] = acc_x;
            lectura["acc_y"] = acc_y;
            lectura["acc_z"] = acc_z;
            lectura["gyr_x"] = gyro_x;
            lectura["gyr_y"] = gyro_y;
            lectura["gyr_z"] = gyro_z;
            lectura["mag_x"] = 0;
            lectura["mag_y"] = 0;
            lectura["mag_z"] = 0;
            lectura["temp"] = 0;
            lectura["start"] = start;
          }
          i++;

        }  //if time_lap>time_sensor
      }    //For loop
      String jsonString;
      serializeJson(data, jsonString);
      // Serial.println(jsonString);
      if (protocol == "http") {
        api.addHeader("Content-Type", "application/json");
        int httpResponseCode = api.POST(jsonString);
        String json_update = api.getString();
        if (httpResponseCode != 200) {
          Serial.println("Error sending");
        }
        Serial.println(json_update);
      }
      if (protocol == "udp") {
        udp.beginPacket(udpServerIP, udpServerPort);
        Serial.println(jsonString);
        udp.print(jsonString);
        udp.endPacket();
      }
      if (protocol == "mqtt0") {
        Serial.println("Sending with MQTT0");
        // Length (with one extra character for the null terminator)
        int str_len = jsonString.length() + 1;
        // Prepare the character array (the buffer)
        char char_array[str_len];
        // Copy it over
        jsonString.toCharArray(char_array, str_len);
        // Serial.println(jsonString.c_str());
        Serial.println(char_array);
        bool success = mqttclient.publish(topic, char_array);
        Serial.print("MQTT success: ");
        Serial.println(success);
      }
      if (protocol == "mqtt1") {
        Serial.println("MQTT1 no soportado");
        return;
      }
      if (protocol == "mqtt2") {
        Serial.println("MQTT2 no soportado");
        return;
      }
      digitalWrite(LED_GREEN, LOW);
    }  //if send_mode==batch
  }    // if active==1
}

// segundo hilo de ejecución dedicado a la configuración, no cambia con sensor
void loop1() {
  vTaskDelay(pdMS_TO_TICKS(1000));                           // retrado de 1 seg el watchdog de la tarea
  if ((millis() - time_passed_update) > time_update * 1000)  //Repetir cada minuto: Actualizar la configuración.
  {
    digitalWrite(LED_BLUE, HIGH);
    Serial.println("Update");
    time_passed_update = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (!wifi_connect(ssid, password)) { ESP.restart(); }
    }
    if (load_setup_server()) {
      load_setup();
    } else {
      Serial.println("Error al cargar Setup");
    }
    Serial.print("Tiempo para conectar y cargar setup: ");
    Serial.println(String(millis() - time_passed_update));  //Cuánto demoró en conectar y cargar setup
    digitalWrite(LED_BLUE, LOW);
  }

  else {
    delay(time_sensor * 4);
  }

  if ((time_reset > 0) && (millis() > time_reset * 3600000)) {  // reinicia cada time_reset horas
    ESP.restart();
  }
}



// Implementación de funciones

// defición para la segunda tarea
void esploop1(void* pvParameters) {
  setup1();
  for (;;)
    loop1();
}

// Función para conectar a una red WiFi
bool wifi_connect(const String& ssid2, const String& password2) {
  int connAttempts = 0;
  Serial.println("Conectando a:");
  Serial.println(ssid2);
  Serial.println(password2);
  WiFi.begin(ssid2.c_str(), password2.c_str());
  while (WiFi.status() != WL_CONNECTED && connAttempts < 30) {
    delay(200);
    Serial.print(".");
    connAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Conectado");
    return true;
  } else {
    return false;
  }
}

bool load_setup_server() {

  HTTPClient http;  // para conectarse al servicio de configuración
  String json_node = "{\"node\":" + (String)node + ",\"start\":" + (String)start + "}";
  Serial.println(json_node);
  if (!http.begin(rest_server + "/nodes/init")) {
    Serial.println("Error Update");
    return (false);
  }
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(json_node);
  String json_update = http.getString();
  if (httpResponseCode == 200) {
    http.end();
    DeserializationError error = deserializeJson(doc, json_update);
    if (error) {
      Serial.print(F("deserializeJson() FALLO: "));
      Serial.println(error.c_str());
      delay(50);
      return (false);
    } else {
      // Abre el archivo en modo de escritura
      File file = LittleFS.open("/config.json", "w");
      if (!file) {
        Serial.println("Error al abrir el archivo");
        return (false);
      }
      // Guarda el objeto JSON en el archivo
      serializeJson(doc, file);
      // Cierra el archivo
      file.close();
      Serial.println("Datos guardados en REGISTRO");
      return (true);

      return (false);
    }
  } else {
    Serial.print("Código al pedir config: ");
    Serial.println(httpResponseCode);
  }
  return (false);
}


bool load_setup() {
  // Abrir el archivo de configuración desde la memoria flash

  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  // Tamaño del archivo de configuración
  size_t size = configFile.size();

  // Leer el contenido del archivo en un buffer
  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);

  // Crear un objeto JSON a partir del buffer
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return false;
  }

  // Cerrar el archivo de configuración
  configFile.close();
  // Leer y asignar los valores del objeto JSON a las variables globales

  while ((bigLock == NULL) || (xSemaphoreTake(bigLock, (TickType_t)0) == pdFALSE)) {
    delay(1);
  }
  //  udpServerIP.fromString(doc["udpServerIP"].as<String>());
  //  udpServerPort = doc["udpServerPort"].as<uint16_t>();
  ssid = doc["ssid"].as<String>();
  password = doc["password"].as<String>();
  rest_server = doc["rest_server"].as<String>();
  node = doc["node"].as<int>();
  time_update = doc["time_update"].as<int>();
  time_sensor = doc["time_sensor"].as<int>();
  batch_size = doc["batch_size"].as<int>();
  //  sense_a = doc["sense_a"].as<float>();
  //  sense_g = doc["sense_g"].as<float>();
  time_reset = doc["time_reset"].as<int>();
  protocol = doc["protocol"].as<String>();
  send_mode = doc["send_mode"].as<String>();
  active = doc["active"].as<int>();
  start = doc["start"].as<int>();
  xSemaphoreGive(bigLock);
  Serial.println("Setup Cargado");
  return true;
}
