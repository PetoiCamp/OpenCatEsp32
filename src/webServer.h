#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>

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
  if (token == '?') {
    webResponse = "bittle_X";  // 设备标识
  }
  webServer.send(200, "text/plain", webResponse);
  cmdFromWeb = false;
}

// WiFi设置函数
void setupWiFi() {
  // 创建 WiFiManager 实例
  WiFiManager wm;

  // 设置配网页面的超时时间（可选，默认120秒）
  wm.setConfigPortalTimeout(180);

  // 设置热点名称并尝试连接WiFi
  if (!wm.autoConnect("BittleWirelessCfg")) {
    Serial.println("配网失败，重启设备");
    delay(3000);
    ESP.restart();  // 如果配网失败，重启设备
  }
  webServerConnected = true;

  // 打印连接成功信息和IP地址
  Serial.println("已成功连接到WiFi");
  Serial.println(WiFi.localIP());
}

void setupWebServer() {
  // Connect to WiFi
  setupWiFi();
  if (webServerConnected) {
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
