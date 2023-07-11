#include <Arduino.h>

#define ESP32

#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <SPIFFS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <Hash.h>
  #include <FS.h>
#endif
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

#define Working_LED 16
#define Pulse_LED  5

int NumRepeat  = 0;                   // el numero de pulsos a generar: min 1 ... max 20
int StartPulsos = 0;

unsigned int PulseDur_ms = 0;         // la duracion de un pulso en ms: min 100ms max 2500ms (2,5sec)
unsigned int PauseDur_ms = 0;         // la duracion de una pause en ms: min 100ms max 2500ms (2,5sec)

unsigned int PulseDuration = 0;       // la duracion de un pulso en us (micro sec.) = PulseDur_ms * 1.000
unsigned int PauseDuration = 0;       // la duracion de una pause en us (micro sec.) = PauseDur_ms * 1.000

const char* ssid = "IoTnet";
const char* password = "darksecret";


/*
const char* ssid = "In%T$en56tEl!1o2";
const char* password = "2@RM1ZkY!V5x6";
*/

const char* parameter_durPulso = "input_durPulso";
const char* parameter_durPausa = "input_durPausa";
const char* parameter_NumPulsos = "input_NumPulsos";
const char* parameter_StartPulsos = "input_StartPulsos";

String printData;

// ---------------------------------------------------------------------------------------------------------

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>HTML Form to Input Data</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  
  <style>
    html {font-family: Sans-Serif; display: inline-block; text-align: left;}
    h3 {font-size: 1.0rem; color: #FFA000;}
  </style>
  
  <script>
    function message_popup() {
      alert("grabando datos en la EEPROM del procesador SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script>
  
  </head>
  
  <body>
      <h3>generador de pulsos de desactivador</h3>
      <hr size="1px" color="black" />
 <fieldset>
    <legend> parametros de pulsos </legend>
      <form action="/get" target="hidden-form"><br>
            - la duracion del pulso (valor actual %input_durPulso% mseg) :&emsp;&emsp;<input type="range" name="input_durPulso" value="100" step=50 min="50" max="2500" 
            oninput="document.getElementById('fPulso').innerHTML = this.value" />
           <label id="fPulso"></label> ms  
           &emsp;<input type="submit" value="grabar" onclick="message_popup()">
      </form><br>
      <form action="/get" target="hidden-form">
            - la duracion de la pause (valor actual %input_durPausa% mseg) :&emsp;<input type="range" name="input_durPausa" value="100" step=50 min="50" max="2000" 
            oninput="document.getElementById('fPause').innerHTML = this.value" />
           <label id="fPause"></label> ms  
           &emsp;<input type="submit" value="grabar" onclick="message_popup()">
      </form><br>
      <form action="/get" target="hidden-form">
            - el numero de pulsos a enviar (valor actual %input_NumPulsos%) :&emsp;&emsp;&emsp;<input type="range" name="input_NumPulsos" value="1" step=1 min="1" max="15" 
            oninput="document.getElementById('fTren').innerHTML = this.value" />
           <label id="fTren"></label>
           &emsp;&emsp;<input type="submit" value="grabar" onclick="message_popup()">
      </form>
   </fieldset><br>
   
   <fieldset>
      <legend> Pulsos </legend>
      <form action="/get" target="hidden-form"><br>
            - envio de pulsos (valor actual %input_StartPulsos%) :&emsp;&emsp;<input type="range" name="input_StartPulsos" value="0" min="0" step=1 max="1" 
            oninput="document.getElementById('fStartPulsos').innerHTML = this.value" />
           <label id="fStartPulsos"></label>
           &emsp;<input type="submit" value=" start/ stop pulsos" onclick="message_popup()">
      </form><br>
   </fieldset><br>
    <hr size="1px" color="black" /><br>     

    <iframe style="display:none" name="hidden-form"></iframe>
  </body>
</html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// ---------------------------------------------------------------------------------------------------------

String read_file(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("Empty file/Failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void write_file(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("SUCCESS in writing file");
  } else {
    Serial.println("FAILED to write file");
  }
  file.close();
}

String processor(const String& var){
  if(var == "input_durPulso"){
    return read_file(SPIFFS, "/input_durPulso.txt");
  }
  else if(var == "input_durPausa"){
    return read_file(SPIFFS, "/input_durPausa.txt");
  }
  else if(var == "input_NumPulsos"){
    return read_file(SPIFFS, "/input_NumPulsos.txt");
  }
  else if(var == "input_StartPulsos"){
    return read_file(SPIFFS, "/input_StartPulsos.txt");
  }
  return String();
}

// ---------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  #ifdef ESP32
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #else
    if(!SPIFFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connecting...");
    return;
  }
  Serial.println();
  Serial.print("=================================================");
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("-------------------------------------------------");
  Serial.println();
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;

    if (request->hasParam(parameter_durPulso)) {
      inputMessage = request->getParam(parameter_durPulso)->value();
      write_file(SPIFFS, "/input_durPulso.txt", inputMessage.c_str());
    }

    else if (request->hasParam(parameter_durPausa)) {
      inputMessage = request->getParam(parameter_durPausa)->value();
      write_file(SPIFFS, "/input_durPausa.txt", inputMessage.c_str());
    }
    else if (request->hasParam(parameter_NumPulsos)) {
      inputMessage = request->getParam(parameter_NumPulsos)->value();
      write_file(SPIFFS, "/input_NumPulsos.txt", inputMessage.c_str());
    }
    else if (request->hasParam(parameter_StartPulsos)) {
      inputMessage = request->getParam(parameter_StartPulsos)->value();
      write_file(SPIFFS, "/input_StartPulsos.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound(notFound);
  server.begin();

   pinMode(Working_LED, OUTPUT);
   pinMode(Pulse_LED, OUTPUT);
                                      // reset ports
   digitalWrite(Working_LED, LOW);
   digitalWrite(Pulse_LED, LOW);
   

   StartPulsos   = 0;
   NumRepeat     = 10;
   PulseDur_ms   = 500;
   PauseDur_ms   = 500;

   PulseDuration = PulseDur_ms;
   PauseDuration = PauseDur_ms;
}

void loop() 
  {
    if (StartPulsos<1)
        {
        int myPulso = read_file(SPIFFS, "/input_durPulso.txt").toInt();
        Serial.print("- duracion de PULSO: ");
        Serial.println(myPulso);
      
        Serial.println();
      
        int myPause = read_file(SPIFFS, "/input_durPausa.txt").toInt();
        Serial.print("- duracion de PAUSE: ");
        Serial.println(myPause);
      
        Serial.println();
      
        int myNumPulsos = read_file(SPIFFS, "/input_NumPulsos.txt").toInt();
        Serial.println("- numero de pulsos a enviar: ");
        Serial.println(myNumPulsos);
      
        
        Serial.println();
      
        int myStartPulsos = read_file(SPIFFS, "/input_StartPulsos.txt").toInt();
        Serial.println("- envio de pulsos: ");
        Serial.println(myStartPulsos);
                                                      // write the pulse train data
        PulseDuration = myPulso;
        PauseDuration = myPause;
        NumRepeat     = myNumPulsos;
        StartPulsos   = myStartPulsos;
   
        Serial.print("-------------------------------------------------");
        Serial.println();
        
        delay(500);
        }
      else
        {
                                                // reset ports
         digitalWrite(Working_LED, LOW);
         digitalWrite(Pulse_LED, LOW);
 
         delay (500);
                                                // monitoring LED ON.     
         digitalWrite(Working_LED, HIGH);
    
         for (int i=NumRepeat; i > 0; --i)
             {  
             digitalWrite(Pulse_LED, HIGH);
             delay(PulseDuration);
             digitalWrite(Pulse_LED, LOW);
             delay(PauseDuration);
             }
                                             // monitoring LED OFF.     
         digitalWrite(Working_LED, LOW);
         digitalWrite(Pulse_LED, LOW);
         
         StartPulsos =0;
         write_file(SPIFFS, "/input_StartPulsos.txt", "0");
         }
  }