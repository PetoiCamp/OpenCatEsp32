#include <WiFi.h>
#include <WebServer.h>

// const char* ssid = "len-AP";
// const char* password = "qwertyuiop";
const char *ssid = "U5-5303";
const char *password = "y53035303";
WebServer webServer(80);
long connectWebTime;
bool webServerConnected = false;
String webResponse = "";
bool cmdFromWeb = false;
void handleCommand() {
  // Get the cmd parameter
  String webCmd = webServer.arg("cmd");

  if (webCmd == "") {
    webServer.send(400, "text/plain", "Missing cmd parameter");
    return;
  }

  // Print the received command to Serial
  Serial.print("Received web command: ");
  Serial.println(webCmd);
  token = webCmd[0];
  strcpy(newCmd, webCmd.c_str() + 1);

  Serial.print("Main program get: ");
  Serial.println(newCmd);
  newCmdIdx = 4;
  cmdFromWeb = true;
  // Process the command and prepare response
  // if (webCmd == "kwk" || webCmd == "ksit"){
  //     response = webCmd;
  // }
  // else if (webCmd == "A1") {
  //     response = "101";
  // }
  // else if (webCmd == "D1") {
  //     response = "1";
  // }
  // else if (webCmd == "Ultrasonic") {
  //     response = "30cm";
  // }
  // else if (webCmd == "?") {
  //     response = "bittle_X";
  // }
  // else {
  //     response = "unused cmd";
  // }

  // Print the response to Serial
  // while (response == "") {
  //   Serial.print('.');
  //   delay(1);
  // }
  // webServer.send(200, "text/plain", text);
  // cmdFromWeb = false;
}

void setupWebServer() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  connectWebTime = millis();
  while (millis() - connectWebTime < 5000) {

    if (WiFi.status() == WL_CONNECTED) {
      webServerConnected = true;
      break;
    }
    delay(100);
    Serial.print(".");
  }
  if (webServerConnected) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Enable CORS
    webServer.enableCORS(true);
    // Set up server routes
    webServer.on("/", HTTP_GET, handleCommand);
    // Start server
    webServer.begin();
    Serial.println("HTTP server started");
  } else
    Serial.println("Timeout: Fail to connect web server!");
}

void WebServerLoop() {
  if (webServerConnected)
    webServer.handleClient();
}
