#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>

// const char* ssid = "len-AP";
// const char* password = "qwertyuiop";
// const char *ssid = "U5-5303";
// const char *password = "y53035303";
String ssid = "";
String password = "";
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

// 通过串口配置WiFi的函数
bool connectWifi(String ssid, String password) {
  WiFi.begin(ssid.c_str(), password.c_str());
  // 等待WiFi连接，最多等待10秒
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 100) {
    delay(50);
    PT('.');
    timeout++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("connection failed");
    return false;
  }
}
bool configureWiFiViaSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("wifi%")) {
      // 解析WiFi配置字符串
      int firstDelimiter = input.indexOf('%', 5);                    // 找到第二个%的位置
      int secondDelimiter = input.indexOf('%', firstDelimiter + 1);  // 找到第三个%的位置

      if (firstDelimiter != -1 && secondDelimiter != -1) {
        ssid = input.substring(5, firstDelimiter);
        password = input.substring(firstDelimiter + 1, secondDelimiter);

        // 尝试连接WiFi
        WiFi.begin(ssid.c_str(), password.c_str());

        // 等待WiFi连接，最多等待10秒
        int timeout = 0;
        while (WiFi.status() != WL_CONNECTED && timeout < 100) {
          delay(100);
          timeout++;
        }

        if (WiFi.status() == WL_CONNECTED) {
          Serial.print("IP Address: ");
          Serial.println(WiFi.localIP());
          return true;
        } else {
          Serial.println("connection failed");
          return false;
        }
      }
    }
  }
  return false;
}

// 修改WiFi设置函数
void setupWiFi() {
  // 等待串口配置
  unsigned long startTime = millis();
  // while (millis() - startTime < 200) {  // 等待5秒钟接收串口配置
  //     if (configureWiFiViaSerial()) {
  //         return;  // 如果通过串口配置成功，直接返回
  //     }
  // }

  // 如果没有收到串口配置，则使用WiFiManager
  // WiFiManager wm;
  // wm.setConfigPortalTimeout(180);

  // if(!wm.autoConnect("BittleWirelessCfg")) {
  //     Serial.println("配网失败，重启设备");
  //     delay(3000);
  //     ESP.restart();
  // }

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
