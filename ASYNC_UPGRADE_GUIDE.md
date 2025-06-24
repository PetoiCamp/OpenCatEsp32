# PetoiWebBlock 异步通信架构升级方案

## 问题
原webServer.h中的同步通信导致看门狗重启：
```cpp
while (cmdFromWeb) delayMicroseconds(100);  // 阻塞等待
```

## 解决方案
采用异步任务管理架构，消除所有阻塞等待。

## 核心修改文件

### 1. 替换 webServer.h
将 `src/webServer.h` 替换为 `src/webServer_async.h`

**关键变化：**
- 移除阻塞等待循环
- 实现异步任务队列管理
- Web请求立即返回任务ID
- 后台异步执行命令

### 2. 修改 reaction.h
在 `src/reaction.h` 中添加异步处理函数：

```cpp
// 异步web服务器函数声明
#ifdef WEB_SERVER
void completeWebTask();
void errorWebTask(String errorMessage);
void finishWebCommand();
void finishWebCommandWithError(String errorMsg);
#endif
```

在 `reaction()` 函数末尾添加：
```cpp
void finishWebCommand()
{
#ifdef WEB_SERVER
  if (cmdFromWeb)
  {
    completeWebTask(); // 调用异步完成函数
  }
#endif
}
```

### 3. 修改 OpenCatEsp32.ino
在 `loop()` 函数末尾添加：
```cpp
#ifdef WEB_SERVER
  WebServerLoop(); // 处理异步Web请求
#endif
```

## API使用方式

### HTTP请求
```
GET /?cmd=kup
返回: TASK_ID:123456_789
```

### 状态查询
```
GET /status?taskId=123456_789
返回: completed\nup\nk
```

### JavaScript客户端
参考 `PetoiWebBlock/js/petoi_async_client.js`

## 编译配置
使用 `huge_app` 分区方案：
```bash
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app
```

## 结果
- ✅ 消除看门狗重启
- ✅ 支持严格时序控制  
- ✅ 保持完全兼容性
- ✅ 提升系统稳定性 
