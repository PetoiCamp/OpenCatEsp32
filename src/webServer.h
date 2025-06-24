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

// 异步任务管理
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

// 函数声明
String generateTaskId();
void startWebTask(String taskId);
void handleTaskStatus(String taskId);
void completeWebTask();
void errorWebTask(String errorMessage);
void processNextWebTask();
void handleTaskList();

// 生成任务ID
String generateTaskId() {
  return String(millis()) + "_" + String(esp_random() % 1000);
}

// 开始执行web任务
void startWebTask(String taskId) {
  if(webTasks.find(taskId) == webTasks.end()) {
    return;
  }

  WebTask &task = webTasks[taskId];
  String webCmd = task.command;

  // 设置全局标志和命令 - 保持与现有代码兼容
  cmdFromWeb = true;
  currentWebTaskId = taskId;
  webTaskActive = true;
  webResponse = "";  // 清空响应缓冲区

  // 解析命令 - 与原来的逻辑相同
  token = webCmd[0];
  strcpy(newCmd, webCmd.c_str() + 1);
  cmdLen = strlen(newCmd);
  newCmd[cmdLen + 1] = '\0';
  newCmdIdx = 4;

  // 更新任务状态
  task.status = "running";
  task.startTime = millis();

  PTHL("starting web task: ", taskId);
}

// 任务状态查询处理器
void handleTaskStatus(String taskId) {
  if(webTasks.find(taskId) == webTasks.end()) {
    webServer.send(404, "text/plain", "Task not found");
    return;
  }

  WebTask &task = webTasks[taskId];

  // 构建响应
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

  // 清理已完成的旧任务（30秒后）
  if((task.status == "completed" || task.status == "error") && (millis() - task.timestamp > 30000)) {
    webTasks.erase(taskId);
  }
}

// 处理下一个等待的任务
void processNextWebTask() {
  for(auto &pair : webTasks) {
    WebTask &task = pair.second;
    if(task.status == "pending") {
      startWebTask(task.taskId);
      break;
    }
  }
}

// 完成web任务 - 在reaction.h中调用
void completeWebTask() {
  if(!webTaskActive || currentWebTaskId == "") {
    return;
  }

  if(webTasks.find(currentWebTaskId) != webTasks.end()) {
    WebTask &task = webTasks[currentWebTaskId];
    task.status = "completed";
    task.result = webResponse;  // 保存收集到的响应
    task.resultReady = true;

    PTHL("web task completed: ", currentWebTaskId);
    PTHL("result length: ", task.result.length());
  }

  // 重置全局状态
  cmdFromWeb = false;
  webTaskActive = false;
  currentWebTaskId = "";

  // 检查是否有等待的任务
  processNextWebTask();
}

// Web任务错误处理
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

  // 重置状态
  cmdFromWeb = false;
  webTaskActive = false;
  currentWebTaskId = "";

  // 处理下一个任务
  processNextWebTask();
}

// 异步命令处理器 - 替换原来的handleCommand
void handleCommandAsync() {
  // 获取命令参数
  String webCmd = webServer.arg("cmd");
  if(webCmd == "") {
    webServer.send(400, "text/plain", "Missing cmd parameter");
    return;
  }

  // 检查是否有查询任务状态的请求
  String taskId = webServer.arg("taskId");
  if(taskId != "") {
    // 返回任务状态
    handleTaskStatus(taskId);
    return;
  }

  // 生成新任务ID
  String newTaskId = generateTaskId();

  // 创建任务记录
  WebTask task;
  task.taskId = newTaskId;
  task.command = webCmd;
  task.status = "pending";
  task.result = "";
  task.timestamp = millis();
  task.startTime = 0;
  task.resultReady = false;

  // 存储任务
  webTasks[newTaskId] = task;

  // 如果当前没有活跃的web任务，立即开始执行
  if(!webTaskActive) {
    startWebTask(newTaskId);
  }

  // 立即返回任务ID - 这是异步的关键
  webServer.send(200, "text/plain", "TASK_ID:" + newTaskId);

  PTHL("web command async: ", webCmd);
  PTHL("task ID: ", newTaskId);
}

// 兼容性API - 返回当前web任务ID（如果有）
String getCurrentWebTaskId() {
  return currentWebTaskId;
}

// 检查是否有活跃的web任务
bool hasActiveWebTask() {
  return webTaskActive;
}

// 获取任务列表（调试用）
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
  // 保持原来的逻辑
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

    // 设置异步路由
    webServer.on("/", HTTP_GET, handleCommandAsync);
    webServer.on("/cmd", HTTP_GET, handleCommandAsync);  // 兼容性路由
    webServer.on("/status", HTTP_GET, []() {
      String taskId = webServer.arg("taskId");
      if(taskId != "") {
        handleTaskStatus(taskId);
      } else {
        webServer.send(400, "text/plain", "Missing taskId parameter");
      }
    });
    webServer.on("/tasks", HTTP_GET, handleTaskList);  // 调试接口

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
