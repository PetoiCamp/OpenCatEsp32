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
void handleCommand() {
  // Get the cmd parameter
  String webCmd = webServer.arg("cmd");
  if (webCmd == "") {
    webServer.send(400, "text/plain", "Missing cmd parameter");
    return;
  }
  // Print the received command to Serial
  PTHL("web command: ", webCmd);
  cmdFromWeb = true;
  token = webCmd[0];
  strcpy(newCmd, webCmd.c_str() + 1);
  cmdLen = strlen(newCmd);
  newCmd[cmdLen + 1] = '\0';
  newCmdIdx = 4;
  while (cmdFromWeb)
    delay(1);
  webServer.send(200, "text/plain", webResponse);
  webResponse = "";
}

// 通过串口配置WiFi的函数
bool connectWifi(String ssid, String password) {
  WiFi.begin(ssid.c_str(), password.c_str());
  // 等待WiFi连接，最多等待10秒
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 100) {
    delay(100);
    PT('.');
    timeout++;
  }
  PTL();
  if (WiFi.status() == WL_CONNECTED) {
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
}


void startWifiManager() {
  //if the wifi manager is not connected this time, it won't enter wifi manager next time
#ifdef I2C_EEPROM_ADDRESS
  i2c_eeprom_write_byte(EEPROM_WIFI_MANAGER, false);
#else
  config.putBool("WifiManager", false);
#endif
  // Connect to WiFi
  WiFiManager wm;
  wm.setConfigPortalTimeout(60);  // timeout after 60 seconds
  //if it fails to connect, it won't open wifi manager during next bootup
  if (!wm.autoConnect((uniqueName + " WifiConfig").c_str())) {
    PTLF("Fail to connect Wifi. Rebooting.");
    delay(3000);
    ESP.restart();
  } else
    webServerConnected = true;
  if (webServerConnected) {
    // Enable CORS
    webServer.enableCORS(true);
    // Set up server routes
    webServer.on("/", HTTP_GET, handleCommand);
    // Start server
    webServer.begin();
    PTLF("HTTP server started");
  } else
    PTLF("Timeout: Fail to connect web server!");
#ifdef I2C_EEPROM_ADDRESS
  i2c_eeprom_write_byte(EEPROM_WIFI_MANAGER, webServerConnected);
#else
  config.putBool("WifiManager", webServerConnected);
#endif
}
void resetWifiManager() {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  //load the flash-saved configs
  esp_wifi_init(&cfg);                                  //initiate and allocate wifi resources (does not matter if connection fails)
  delay(2000);                                          //wait a bit
  if (esp_wifi_restore() != ESP_OK) {
    PTLF("\nWiFi is not initialized by esp_wifi_init ");
  } else {
    PTLF("\nWiFi Configurations Cleared!");
  }
  delay(2000);
  ESP.restart();
}

void webServerTask(void *pvParameters) {
  while (true) {
    webServer.handleClient();
    vTaskDelay(1);  // Small delay to prevent watchdog issues
  }
}
void WebServerLoop() {
  if (webServerConnected)
    webServer.handleClient();
}
