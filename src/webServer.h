#include "esp32-hal.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>

#include <map>

// WiFi配置
String ssid = "";
String password = "";
WebServer webServer(80);
long connectWebTime;
bool webServerConnected = false;

// Async task management
struct WebTask {
  String taskId;
  String command;
  String status;  // "pending", "running", "completed", "error"
  String result;
  unsigned long timestamp;
  unsigned long startTime;
  bool resultReady;
};

std::map<String, WebTask> webTasks;
String currentWebTaskId = "";
bool webTaskActive = false;

// Function declarations
String generateTaskId();
void startWebTask(String taskId);
void handleTaskStatus(String taskId);
void completeWebTask();
void errorWebTask(String errorMessage);
void processNextWebTask();
void handleTaskList();

// Generate task ID
String generateTaskId() {
  return String(millis()) + "_" + String(esp_random() % 1000);
}

// Start executing web task
void startWebTask(String taskId) {
  if(webTasks.find(taskId) == webTasks.end()) {
    return;
  }

  WebTask &task = webTasks[taskId];
  String webCmd = task.command;

  // Set global flags and commands - maintain compatibility with existing code
  cmdFromWeb = true;
  currentWebTaskId = taskId;
  webTaskActive = true;
  webResponse = "";  // Clear response buffer

  // Parse command - same logic as before
  token = webCmd[0];
  strcpy(newCmd, webCmd.c_str() + 1);
  cmdLen = strlen(newCmd);
  newCmd[cmdLen + 1] = '\0';
  newCmdIdx = 4;

  // Update task status
  task.status = "running";
  task.startTime = millis();

  PTHL("starting web task: ", taskId);
}

// Task status query handler
void handleTaskStatus(String taskId) {
  if(webTasks.find(taskId) == webTasks.end()) {
    webServer.send(404, "text/plain", "Task not found");
    return;
  }

  WebTask &task = webTasks[taskId];

  // Build response
  String response = task.status;
  if(task.status == "completed") {
    response += "\n" + task.result;
  } else if(task.status == "error") {
    response += "\nERROR: " + task.result;
  } else if(task.status == "running") {
    unsigned long elapsed = millis() - task.startTime;
    response += "\nRunning for " + String(elapsed) + "ms";
  }

  webServer.send(200, "text/plain", response);

  // Clean up completed old tasks (after 30 seconds)
  if((task.status == "completed" || task.status == "error") && (millis() - task.timestamp > 30000)) {
    webTasks.erase(taskId);
  }
}

// Process next waiting task
void processNextWebTask() {
  for(auto &pair : webTasks) {
    WebTask &task = pair.second;
    if(task.status == "pending") {
      startWebTask(task.taskId);
      break;
    }
  }
}

// Complete web task - called in reaction.h
void completeWebTask() {
  if(!webTaskActive || currentWebTaskId == "") {
    return;
  }

  if(webTasks.find(currentWebTaskId) != webTasks.end()) {
    WebTask &task = webTasks[currentWebTaskId];
    task.status = "completed";
    task.result = webResponse;  // Save collected response
    task.resultReady = true;

    PTHL("web task completed: ", currentWebTaskId);
    PTHL("result length: ", task.result.length());
  }

  // Reset global state
  cmdFromWeb = false;
  webTaskActive = false;
  currentWebTaskId = "";

  // Check if there are waiting tasks
  processNextWebTask();
}

// Web task error handling
void errorWebTask(String errorMessage) {
  if(!webTaskActive || currentWebTaskId == "") {
    return;
  }

  if(webTasks.find(currentWebTaskId) != webTasks.end()) {
    WebTask &task = webTasks[currentWebTaskId];
    task.status = "error";
    task.result = errorMessage;
    task.resultReady = true;
  }

  // Reset state
  cmdFromWeb = false;
  webTaskActive = false;
  currentWebTaskId = "";

  // Process next task
  processNextWebTask();
}

// Async command handler - replaces original handleCommand
void handleCommandAsync() {
  // Get command parameters
  String webCmd = webServer.arg("cmd");
  if(webCmd == "") {
    webServer.send(400, "text/plain", "Missing cmd parameter");
    return;
  }

  // Check if there's a task status query request
  String taskId = webServer.arg("taskId");
  if(taskId != "") {
    // Return task status
    handleTaskStatus(taskId);
    return;
  }

  // Generate new task ID
  String newTaskId = generateTaskId();

  // Create task record
  WebTask task;
  task.taskId = newTaskId;
  task.command = webCmd;
  task.status = "pending";
  task.result = "";
  task.timestamp = millis();
  task.startTime = 0;
  task.resultReady = false;

  // Store task
  webTasks[newTaskId] = task;

  // If there's no active web task currently, start immediately
  if(!webTaskActive) {
    startWebTask(newTaskId);
  }

  // Return task ID immediately - this is the key to async
  webServer.send(200, "text/plain", "TASK_ID:" + newTaskId);

  PTHL("web command async: ", webCmd);
  PTHL("task ID: ", newTaskId);
}

// Compatibility API - return current web task ID (if any)
String getCurrentWebTaskId() {
  return currentWebTaskId;
}

// Check if there's an active web task
bool hasActiveWebTask() {
  return webTaskActive;
}

// Get task list (for debugging)
void handleTaskList() {
  String response = "Active tasks:\n";
  for(const auto &pair : webTasks) {
    const WebTask &task = pair.second;
    response += task.taskId + ": " + task.status + " (" + task.command + ")\n";
  }
  webServer.send(200, "text/plain", response);
}

// WiFi配置函数（保持原来的逻辑）
bool connectWifi(String ssid, String password) {
  WiFi.begin(ssid.c_str(), password.c_str());
  int timeout = 0;
  while(WiFi.status() != WL_CONNECTED && timeout < 100) {
    delay(100);
    PT('.');
    timeout++;
  }
  PTL();
  if(WiFi.status() == WL_CONNECTED) {
    return true;
  } else {
    Serial.println("connection failed");
    return false;
  }
}

bool configureWiFiViaSerial() {
  if(Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if(input.startsWith("wifi%")) {
      int firstDelimiter = input.indexOf('%', 5);
      int secondDelimiter = input.indexOf('%', firstDelimiter + 1);

      if(firstDelimiter != -1 && secondDelimiter != -1) {
        ssid = input.substring(5, firstDelimiter);
        password = input.substring(firstDelimiter + 1, secondDelimiter);

        WiFi.begin(ssid.c_str(), password.c_str());

        int timeout = 0;
        while(WiFi.status() != WL_CONNECTED && timeout < 100) {
          delay(100);
          timeout++;
        }

        if(WiFi.status() == WL_CONNECTED) {
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

void setupWiFi() {
  // Keep original logic
}

void startWifiManager() {
#ifdef I2C_EEPROM_ADDRESS
  i2c_eeprom_write_byte(EEPROM_WIFI_MANAGER, false);
#else
  config.putBool("WifiManager", false);
#endif

  WiFiManager wm;
  wm.setConfigPortalTimeout(60);
  String wifiConfigName = (uniqueName.length() > 0) ? (uniqueName + " WifiConfig") : "Robot WifiConfig";
  if(!wm.autoConnect(wifiConfigName.c_str())) {
    PTLF("Fail to connect Wifi. Rebooting.");
    delay(3000);
    ESP.restart();
  } else
    webServerConnected = true;

  if(webServerConnected) {
    webServer.enableCORS(true);

    // Set up async routes
    webServer.on("/", HTTP_GET, handleCommandAsync);
    webServer.on("/cmd", HTTP_GET, handleCommandAsync);  // Compatibility route
    webServer.on("/status", HTTP_GET, []() {
      String taskId = webServer.arg("taskId");
      if(taskId != "") {
        handleTaskStatus(taskId);
      } else {
        webServer.send(400, "text/plain", "Missing taskId parameter");
      }
    });
    webServer.on("/tasks", HTTP_GET, handleTaskList);  // Debug interface

    webServer.begin();
    PTLF("HTTP server started (async mode)");
  } else
    PTLF("Timeout: Fail to connect web server!");

#ifdef I2C_EEPROM_ADDRESS
  i2c_eeprom_write_byte(EEPROM_WIFI_MANAGER, webServerConnected);
#else
  config.putBool("WifiManager", webServerConnected);
#endif
}

void resetWifiManager() {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  delay(2000);
  if(esp_wifi_restore() != ESP_OK) {
    PTLF("\nWiFi is not initialized by esp_wifi_init ");
  } else {
    PTLF("\nWiFi Configurations Cleared!");
  }
  delay(2000);
  ESP.restart();
}

// 移除webServerTask - 不再需要独立任务
// 主循环调用函数
void WebServerLoop() {
  if(webServerConnected) {
    webServer.handleClient();

    // 检查任务超时（可选）
    unsigned long currentTime = millis();
    for(auto &pair : webTasks) {
      WebTask &task = pair.second;
      if(task.status == "running" && task.startTime > 0) {
        if(currentTime - task.startTime > 30000) {  // 30秒超时
          PTHL("web task timeout: ", task.taskId);
          task.status = "error";
          task.result = "Task timeout";
          task.resultReady = true;

          if(task.taskId == currentWebTaskId) {
            cmdFromWeb = false;
            webTaskActive = false;
            currentWebTaskId = "";
            processNextWebTask();
          }
        }
      }
    }
  }
}
