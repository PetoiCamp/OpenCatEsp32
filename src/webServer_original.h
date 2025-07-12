#include "esp32-hal.h"
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
    delayMicroseconds(100);
  webServer.send(200, "text/plain", webResponse + "\n");
  // PTHL("res: ",webResponse+"\n"+token+"\n")
  webResponse = "";
}

// Function to configure WiFi via serial port
bool connectWifi(String ssid, String password) {
  WiFi.begin(ssid.c_str(), password.c_str());
  // Wait for WiFi connection, up to 10 seconds
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
      // Parse WiFi configuration string
      int firstDelimiter = input.indexOf('%', 5);                   // Find the position of the second %
      int secondDelimiter = input.indexOf('%', firstDelimiter + 1); // Find the position of the third %

      if (firstDelimiter != -1 && secondDelimiter != -1) {
        ssid = input.substring(5, firstDelimiter);
        password = input.substring(firstDelimiter + 1, secondDelimiter);

        // Try to connect to WiFi
        WiFi.begin(ssid.c_str(), password.c_str());

        // Wait for WiFi connection, up to 10 seconds
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

// Modified WiFi setup function
void setupWiFi() {
  // Wait for serial configuration
  // unsigned long startTime = millis();
  // while (millis() - startTime < 200) {  // Wait 5 seconds to receive serial configuration
  //     if (configureWiFiViaSerial()) {
  //         return;  // If serial configuration succeeds, return directly
  //     }
  // }

  // If no serial configuration is received, use WiFiManager
}

void startWifiManager() {
  // if the wifi manager is not connected this time, it won't enter wifi manager next time
#ifdef I2C_EEPROM_ADDRESS
  i2c_eeprom_write_byte(EEPROM_WIFI_MANAGER, false);
#else
  config.putBool("WifiManager", false);
#endif
  // Connect to WiFi
  WiFiManager wm;
  wm.setConfigPortalTimeout(60); // timeout after 60 seconds
  // if it fails to connect, it won't open wifi manager during next bootup
  String wifiConfigName = (uniqueName.length() > 0) ? (uniqueName + " WifiConfig") : "Robot WifiConfig";
  if (!wm.autoConnect(wifiConfigName.c_str())) {
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
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // load the flash-saved configs
  esp_wifi_init(&cfg); // initiate and allocate wifi resources (does not matter if connection fails)
  delay(2000);         // wait a bit
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
    vTaskDelay(1); // Small delay to prevent watchdog issues
  }
}
void WebServerLoop() {
  if (webServerConnected)
    webServer.handleClient();
}
