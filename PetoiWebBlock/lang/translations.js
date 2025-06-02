const TRANSLATIONS = {
  zh: {
    // 按钮和标题
    "appTitle": "陪拓网页编程积木",
    "showCode": "显示代码",
    "runCode": "运行代码",
    "saveProgram": "保存程序",
    "loadProgram": "加载程序",
    "clearAll": "清除全部",
    "clearConsole": "清除日志",
    "consoleLog": "控制台日志",
    "serialConnect": "连接串口",
    "quickConnect": "快速连接",
    "closeSerial": "关闭连接",
    "clearDisplay": "清除输出",
    "send": "发送",
    "serialOutput": "串口监视器",
    "serialInputPlaceholder": "输入要发送的内容",
    "wifiConfig": "WiFi配置",
    "resetPrompt": "请在配置WiFi前按下RESET按钮",
    "ssidPlaceholder": "输入WiFi名称",
    "passwordPlaceholder": "输入WiFi密码",
    "cancel": "取消",
    "confirm": "确认",
    "scanWifi": "扫描WiFi",
    "scanning": "扫描中...",
    "serialConfiguredIP": "IP地址: ",
    "serialReadError": "读取串口数据错误:",
    "serialSendError": "发送数据错误:",
    "serialConnectionError": "串口连接错误:",
    "enterWifiName": "请输入WiFi名称",
    "wifiCommandError": "发送WiFi命令错误:",
    "wifiCommandFailed": "发送WiFi命令失败",
    "connectedToDevice": "已连接到设备: ",
    "textCopied": "文本已复制到剪贴板",
    "serialPortBusy": "串口已被占用，请关闭其他正在使用此串口的程序",
    "taskEnded": "任务结束",
    "showSentCommands": "显示发送指令",
    "commandCompleted": "指令已完成",
    "commandNoReturnWarning": "警告：未收到预期的返回值：",
    "fileLoaded": "已加载：{filename}",
    "loadFileFailed": "无法载入文件",
    "currentFileLabel": "当前文件：",

    // 命令消息
    "sendingCommand": "发送命令: ",
    "commandFailed": "命令执行失败:",
    "httpError": "HTTP错误: ",

    // 显示代码窗口
    "generatedCode": "生成的JavaScript代码",
    "copyCode": "复制代码",
    "codeCopied": "复制成功！",

    // 积木分类
    "categoryLogic": "逻辑",
    "categoryLoops": "循环",
    "categoryMath": "数学",
    "categoryText": "文本",
    "categoryVariables": "变量",
    "categoryFunctions": "函数",
    "categoryCommunication": "通信",
    "categoryMotion": "动作",
    "categoryControl": "控制",
    "categoryConsole": "控制台",
    "categoryMusic": "音乐",

    // 积木文本 - 通信
    "connectWithIP": "连接IP地址 %1",
    "getDigitalInput": "获取数字输入 %1",
    "getAnalogInput": "获取模拟输入 %1",
    "getSensorInput": "获取传感器 %1",
    "setDigitalOutput": "设置数字输出 引脚 %1 状态 %2",
    "setAnalogOutput": "设置模拟输出 引脚 %1 数值 %2",
    "sendCustomCommand": "发送自定义命令 %1",

    // 积木文本 - 动作
    "setMotorAngle": "设置关节 %1 角度为 %2",
    "getJointAngle": "获取关节 %1 的角度",
    "getAllJointAngles": "获取所有关节角度",
    "gait": "步态 %1",
    "posture": "姿势 %1",
    "acrobatic_moves": "杂技动作(小心使用) %1",
    "localAction": "原地动作 %1",
    "highDifficultyAction": "高难度特技动作(小心使用) %1",
    "setMotorAngleWithDelay": "设置关节 %1 角度为 %2 后延时 %3 秒",
    "gaitWithDelay": "步态 %1 后延时 %2 秒",
    "postureWithDelay": "姿势 %1 后延时 %2 秒",
    "acrobaticWithDelay": "杂技动作(小心使用) %1 后延时 %2 秒",

    // 积木文本 - 控制
    "delayMs": "延时 %1 秒",
    "delayMessage": "延时 {delay} 秒...",

    // 积木文本 - 传感器
    "gyroControl": "陀螺仪 %1",

    // 积木文本 - 控制台
    "consoleLogVariable": "在控制台输出变量 %1",

    // 积木文本 - 音乐
    "playNote": "播放音符 %1 时长 %2 拍",

    // 动作选项
    "stand": "站立",
    "sit": "坐下",
    "rest": "休息",
    "pee": "尿尿",
    "backflip": "后空翻",
    "jump": "跳跃",
    "handstand": "倒立",
    "boxing": "打拳",
    "frontflip": "前空翻",
    "step": "原地踏步",
    "rotateLeft": "左旋",
    "rotateRight": "右旋",
    "walkForward": "向前走",
    "walkLeft": "向左走",
    "walkRight": "向右走",
    "walkBackward": "向后走",
    "backLeft": "向左后走",
    "backRight": "向右后走",
    "trotForward": "向前跑",
    "trotLeft": "向左跑",
    "trotRight": "向右跑",
    "crawlForward": "向前爬",
    "crawlLeft": "向左爬",
    "crawlRight": "向右爬",
    "gapForward": "向前跨",
    "gapLeft": "向左跨",
    "gapRight": "向右跨",
    "moonwalk": "太空步",

    // 传感器选项
    "ultrasonic": "超声波",
    "touch": "触摸",
    "distance": "距离",
    "light": "光线",
    "temperature": "温度",
    "humidity": "湿度",
    "gyroEnable": "启用",
    "gyroDisable": "禁用",

    // 连接和错误相关消息
    "connectingDevice": "正在连接设备: ",
    "deviceResponseInfo": "设备返回信息: ",
    "deviceModelInfo": "设备型号: ",
    "errorMockData": "错误：使用了模拟数据而非真实请求！",
    "connectionFailedMock": "连接失败：系统使用了模拟数据而非真实网络请求。请检查网络设置。",
    "connectionFailedCheck": "连接机器人失败！请确认IP地址正确并且设备已开启。",
    "connectionError": "连接出错: ",
    "connectionTimeout": "连接超时：无法连接到 {ip}，请检查设备是否开启并在同一网络中。",
    "networkError": "网络错误：无法连接到 {ip}，请检查网络连接。",
    "connectionErrorDetails": "连接机器人出错: {error}",
    "programExecutionStopped": "程序已中断执行，后续指令将不会运行。",
    "errorInvalidIP": "错误：无效的IP地址",
    "receivedResponse": "收到响应: ",
    "requestFailedStatusCode": "请求失败，状态码: {status}",
    "requestError": "请求错误: ",
    "invalidIPAddress": "无效的IP地址",
    "requestTimeout": "请求超时",
    "asyncSendingCommand": "异步发送命令: ",
    "asyncReceivedResponse": "异步收到响应: ",
    "networkRequestError": "网络请求错误",
    "programEndingRestCommand": "程序结束，自动发送休息指令...",
    "restCommandFailed": "发送休息指令失败: ",

    // 代码生成器注释
    "connectingIPAddress": "连接IP地址",
    "connectionFailedComment": "连接失败时中断后续程序执行，但不再抛出异常，因为makeConnection函数已经显示了错误提示",
    "executionStoppedComment": "直接返回，中断后续代码执行",
    "mockRequest": "模拟请求: ",
    "usingMockHttpRequest": "注意：使用了模拟HTTP请求，返回的是假设备型号",

    // Joint names - 关节名称
    "jointHeadPanning": "头偏转角",
    "jointHeadTiltingNybble": "头俯仰角(狸宝)",
    "jointTailNybble": "尾巴(狸宝)",
    "jointLFArm": "左前臂",
    "jointRFArm": "右前臂",
    "jointRBArm": "右后臂",
    "jointLBArm": "左后臂",
    "jointLFKnee": "左前膝",
    "jointRFKnee": "右前膝",
    "jointRBKnee": "右后膝",
    "jointLBKnee": "左后膝",
    "jointReserved": "预留",

    // Joint block UI text
    "setJointLabel": "设置关节",
    "angleTo": "角度为",
    "thenDelay": "后延时",
    "secUnit": "秒",

    // Note block UI text
    "playNote": "播放音符",
    "forDuration": "时长",
    "beatUnit": "拍",

    // Motion blocks UI text
    "gaitLabel": "步态",
    "postureLabel": "姿势",
    "acrobaticMovesLabel": "杂技动作(小心使用)",

    // Note names
    "noteRest": "休止符",
    "noteLowC": "低音C",
    "noteLowCSharp": "低音C#",
    "noteLowD": "低音D",
    "noteLowDSharp": "低音D#",
    "noteLowE": "低音E",
    "noteLowF": "低音F",
    "noteLowFSharp": "低音F#",
    "noteLowG": "低音G",
    "noteLowGSharp": "低音G#",
    "noteLowA": "低音A",
    "noteLowASharp": "低音A#",
    "noteLowB": "低音B",
    "noteMiddleC": "中音C",
    "noteMiddleCSharp": "中音C#",
    "noteMiddleD": "中音D",
    "noteMiddleDSharp": "中音D#",
    "noteMiddleE": "中音E",
    "noteMiddleF": "中音F",
    "noteMiddleFSharp": "中音F#",
    "noteMiddleG": "中音G",
    "noteMiddleGSharp": "中音G#",
    "noteMiddleA": "中音A",
    "noteMiddleASharp": "中音A#",
    "noteMiddleB": "中音B",
    "noteHighC": "高音C",

    // Gait options
    "gaitStep": "原地踏步",
    "gaitRotateLeft": "左旋",
    "gaitRotateRight": "右旋",
    "gaitWalkForward": "向前走",
    "gaitWalkLeft": "向左走",
    "gaitWalkRight": "向右走",
    "gaitWalkBackward": "向后走",
    "gaitBackLeft": "向左后走",
    "gaitBackRight": "向右后走",
    "gaitTrotForward": "向前跑",
    "gaitTrotLeft": "向左跑",
    "gaitTrotRight": "向右跑",
    "gaitCrawlForward": "向前爬",
    "gaitCrawlLeft": "向左爬",
    "gaitCrawlRight": "向右爬",
    "gaitGapForward": "向前跨",
    "gaitGapLeft": "向左跨",
    "gaitGapRight": "向右跨",
    "gaitMoonwalk": "太空步",

    // Posture options
    "postureStand": "站立",
    "postureSit": "坐下",
    "postureRest": "休息",
    "posturePee": "尿尿",

    // Acrobatic options
    "acrobaticHandstand": "倒立",
    "acrobaticBoxing": "打拳",
    "acrobaticBackflip": "后空翻",
    "acrobaticFrontflip": "前空翻",
    "acrobaticJump": "跳跃",

    // Code dialog UI text
    "generatedJSCode": "生成的JavaScript代码",
    "copyCode": "复制代码",
    "copySuccess": "复制成功！",

    // Save dialog UI text
    "currentFile": "当前文件: \"{filename}\"",
    "noFileSaved": "尚未保存文件",
    "save": "保存",
    "saveAs": "另存为",
    "saveWarning": "注意：如果下载文件夹中已有同名文件，浏览器会自动添加数字后缀。",
    "saveProgram": "保存程序",

    // Save success dialog UI text
    "saveSuccess": "保存成功",
    "savedToDownloads": "文件已保存到您的\"下载\"文件夹。",
    "filenameNote": "文件名: \"{filename}\"(如下载文件夹中已有同名文件，浏览器可能自动添加数字后缀)",
    "close": "关闭",

    // Debug messages
    "incompleteIPDetected": "检测到可能不完整的IP地址: {ip}，后面还有数字",
    "newIPDetected": "检测到新的IP地址: {ip}，更新自当前IP: {oldIP}",
    "invalidIPFormat": "IP地址格式不正确: {ip}",

    // Undo/Redo buttons
    "undo": "撤销",
    "redo": "重做",
  },

  en: {
    // Buttons and titles
    "appTitle": "Petoi Web Coding Blocks",
    "showCode": "Show Code",
    "runCode": "Run Code",
    "saveProgram": "Save Program",
    "loadProgram": "Load Program",
    "clearAll": "Clear All",
    "clearConsole": "Clear Log",
    "consoleLog": "Console Log",
    "serialConnect": "Connect Serial Port",
    "quickConnect": "Quick Connect",
    "closeSerial": "Close Serial",
    "clearDisplay": "Clear Output",
    "send": "Send",
    "serialOutput": "Serial Monitor",
    "serialInputPlaceholder": "Enter content to send",
    "wifiConfig": "WiFi Configuration",
    "resetPrompt": "Please press RESET button before configuring WiFi",
    "ssidPlaceholder": "Enter WiFi SSID",
    "passwordPlaceholder": "Enter WiFi Password",
    "cancel": "Cancel",
    "confirm": "Confirm",
    "scanWifi": "Scan WiFi",
    "scanning": "Scanning...",
    "serialConfiguredIP": "IP address: ",
    "serialReadError": "Serial read error: ",
    "serialSendError": "Serial send error: ",
    "serialConnectionError": "Serial connection error: ",
    "enterWifiName": "Please enter WiFi name",
    "wifiCommandError": "WiFi command error:",
    "wifiCommandFailed": "Failed to send WiFi command",
    "connectedToDevice": "Connected to device: ",
    "textCopied": "Text copied to clipboard",
    "serialPortBusy": "Serial port is busy, please close other programs using this serial port",
    "taskEnded": "Task ended",
    "showSentCommands": "Show Sent Commands",
    "commandCompleted": "Command completed",
    "commandNoReturnWarning": "Warning: Expected return value not received: ",
    "fileLoaded": "Loaded: {filename}",
    "loadFileFailed": "Failed to load file",
    "currentFileLabel": "Current file:  ",

    // Command messages
    "sendingCommand": "Sending command: ",
    "commandFailed": "Command execution failed:",
    "httpError": "HTTP error: ",

    // Code Display Window
    "generatedCode": "Generated JavaScript Code",
    "copyCode": "Copy Code",
    "codeCopied": "Copied!",

    // Block categories
    "categoryLogic": "Logic",
    "categoryLoops": "Loops",
    "categoryMath": "Math",
    "categoryText": "Text",
    "categoryVariables": "Variables",
    "categoryFunctions": "Functions",
    "categoryCommunication": "Communication",
    "categoryMotion": "Motion",
    "categoryControl": "Control",
    "categoryConsole": "Console",
    "categoryMusic": "Music",

    // Block text - Communication
    "connectWithIP": "Connect with IP %1",
    "getDigitalInput": "Get Digital Input %1",
    "getAnalogInput": "Get Analog Input %1",
    "getSensorInput": "Get Sensor %1",
    "setDigitalOutput": "Set Digital Output Pin %1 State %2",
    "setAnalogOutput": "Set Analog Output Pin %1 Value %2",
    "sendCustomCommand": "Send Custom Command %1",

    // Block text - Motion
    "setMotorAngle": "Set Joint %1 Angle to %2",
    "getJointAngle": "Get Joint %1 Angle",
    "getAllJointAngles": "Get All Joint Angles",
    "gait": "Gait %1",
    "posture": "Posture %1",
    "acrobatic_moves": "Acrobatic Moves (Use with Caution) %1",
    "localAction": "Static Action %1",
    "highDifficultyAction": "High Difficulty Action (Use with Caution) %1",
    "setMotorAngleWithDelay": "Set Joint %1 Angle to %2 then delay %3 sec",
    "gaitWithDelay": "Gait %1 then delay %2 sec",
    "postureWithDelay": "Posture %1 then delay %2 sec",
    "acrobaticWithDelay": "Acrobatic Moves (Use with Caution) %1 then delay %2 sec",

    // Block text - Control
    "delayMs": "Delay %1 seconds",
    "delayMessage": "Delay {delay} seconds...",

    // Block text - Sensors
    "gyroControl": "Gyroscope %1",

    // Block text - Console
    "consoleLogVariable": "Log variable to console %1",

    // Block text - Music
    "playNote": "Play note %1 for %2 beat",

    // Action options
    "sit": "Sit",
    "stand": "Stand",
    "rest": "Rest",
    "pee": "Pee",
    "backflip": "Backflip",
    "jump": "Jump",
    "handstand": "Handstand",
    "boxing": "Boxing",
    "frontflip": "Frontflip",
    "step": "Step",
    "rotateLeft": "Rotate Left",
    "rotateRight": "Rotate Right",
    "walkForward": "Walk Forward",
    "walkLeft": "Walk Left",
    "walkRight": "Walk Right",
    "walkBackward": "Walk Backward",
    "backLeft": "Back Left",
    "backRight": "Back Right",
    "trotForward": "Trot Forward",
    "trotLeft": "Trot Left",
    "trotRight": "Trot Right",
    "crawlForward": "Crawl Forward",
    "crawlLeft": "Crawl Left",
    "crawlRight": "Crawl Right",
    "gapForward": "Gap Forward",
    "gapLeft": "Gap Left",
    "gapRight": "Gap Right",
    "moonwalk": "Moonwalk",

    // Sensor options
    "ultrasonic": "Ultrasonic",
    "touch": "Touch",
    "distance": "Distance",
    "light": "Light",
    "temperature": "Temperature",
    "humidity": "Humidity",
    "gyroEnable": "Enable",
    "gyroDisable": "Disable",

    // Connection and error related messages
    "connectingDevice": "Connecting to device: ",
    "deviceResponseInfo": "Device response info: ",
    "deviceModelInfo": "Device model: ",
    "errorMockData": "Error: Using mock data instead of real request!",
    "connectionFailedMock": "Connection failed: System used mock data instead of real network request. Please check network settings.",
    "connectionFailedCheck": "Failed to connect to the robot! Please verify the IP address is correct and the device is powered on.",
    "connectionError": "Connection error: ",
    "connectionTimeout": "Connection timeout: Unable to connect to {ip}, please check if the device is on and in the same network.",
    "networkError": "Network error: Unable to connect to {ip}, please check your network connection.",
    "connectionErrorDetails": "Robot connection error: {error}",
    "programExecutionStopped": "Program execution has been stopped, subsequent instructions will not run.",
    "errorInvalidIP": "Error: Invalid IP address",
    "receivedResponse": "Received response: ",
    "requestFailedStatusCode": "Request failed, status code: {status}",
    "requestError": "Request error: ",
    "invalidIPAddress": "Invalid IP address",
    "requestTimeout": "Request timeout",
    "asyncSendingCommand": "Async sending command: ",
    "asyncReceivedResponse": "Async received response: ",
    "networkRequestError": "Network request error",
    "programEndingRestCommand": "Program ended, sending rest command...",
    "restCommandFailed": "Failed to send rest command: ",

    // Code generator comments
    "connectingIPAddress": "Connecting to IP address",
    "connectionFailedComment": "Stop execution on connection failure, without throwing exception as makeConnection function already showed error",
    "executionStoppedComment": "Return directly to stop subsequent code execution",
    "mockRequest": "Mock request: ",
    "usingMockHttpRequest": "Note: Using mock HTTP request, returning fake device model",

    // Joint names - 关节名称
    "jointHeadPanning": "Head Panning",
    "jointHeadTiltingNybble": "Head Tilting (Nybble)",
    "jointTailNybble": "Tail (Nybble)",
    "jointLFArm": "Left Front Arm",
    "jointRFArm": "Right Front Arm",
    "jointRBArm": "Right Back Arm",
    "jointLBArm": "Left Back Arm",
    "jointLFKnee": "Left Front Knee",
    "jointRFKnee": "Right Front Knee",
    "jointRBKnee": "Right Back Knee",
    "jointLBKnee": "Left Back Knee",
    "jointReserved": "Reserved",

    // Joint block UI text
    "setJointLabel": "Set Joint",
    "angleTo": "Angle to",
    "thenDelay": "then delay",
    "secUnit": "sec",

    // Note block UI text
    "playNote": "Play note",
    "forDuration": "for",
    "beatUnit": "beat",

    // Motion blocks UI text
    "gaitLabel": "Gait",
    "postureLabel": "Posture",
    "acrobaticMovesLabel": "Acrobatic Moves (Use with Caution)",

    // Note names
    "noteRest": "Rest",
    "noteLowC": "Low C",
    "noteLowCSharp": "Low C#",
    "noteLowD": "Low D",
    "noteLowDSharp": "Low D#",
    "noteLowE": "Low E",
    "noteLowF": "Low F",
    "noteLowFSharp": "Low F#",
    "noteLowG": "Low G",
    "noteLowGSharp": "Low G#",
    "noteLowA": "Low A",
    "noteLowASharp": "Low A#",
    "noteLowB": "Low B",
    "noteMiddleC": "Middle C",
    "noteMiddleCSharp": "Middle C#",
    "noteMiddleD": "Middle D",
    "noteMiddleDSharp": "Middle D#",
    "noteMiddleE": "Middle E",
    "noteMiddleF": "Middle F",
    "noteMiddleFSharp": "Middle F#",
    "noteMiddleG": "Middle G",
    "noteMiddleGSharp": "Middle G#",
    "noteMiddleA": "Middle A",
    "noteMiddleASharp": "Middle A#",
    "noteMiddleB": "Middle B",
    "noteHighC": "High C",

    // Gait options
    "gaitStep": "Step",
    "gaitRotateLeft": "Rotate Left",
    "gaitRotateRight": "Rotate Right",
    "gaitWalkForward": "Walk Forward",
    "gaitWalkLeft": "Walk Left",
    "gaitWalkRight": "Walk Right",
    "gaitWalkBackward": "Walk Backward",
    "gaitBackLeft": "Back Left",
    "gaitBackRight": "Back Right",
    "gaitTrotForward": "Trot Forward",
    "gaitTrotLeft": "Trot Left",
    "gaitTrotRight": "Trot Right",
    "gaitCrawlForward": "Crawl Forward",
    "gaitCrawlLeft": "Crawl Left",
    "gaitCrawlRight": "Crawl Right",
    "gaitGapForward": "Gap Forward",
    "gaitGapLeft": "Gap Left",
    "gaitGapRight": "Gap Right",
    "gaitMoonwalk": "Moonwalk",

    // Posture options
    "postureStand": "Stand",
    "postureSit": "Sit",
    "postureRest": "Rest",
    "posturePee": "Pee",

    // Acrobatic options
    "acrobaticHandstand": "Handstand",
    "acrobaticBoxing": "Boxing",
    "acrobaticBackflip": "Backflip",
    "acrobaticFrontflip": "Frontflip",
    "acrobaticJump": "Jump",

    // Code dialog UI text
    "generatedJSCode": "Generated JavaScript Code",
    "copyCode": "Copy Code",
    "copySuccess": "Copied!",

    // Save dialog UI text
    "currentFile": "Current file: \"{filename}\"",
    "noFileSaved": "No file saved",
    "save": "Save",
    "saveAs": "Save As",
    "saveWarning": "Note: If a file with the same name already exists in the download folder, the browser will automatically add a number suffix.",
    "saveProgram": "Save Program",

    // Save success dialog UI text
    "saveSuccess": "Save successful",
    "savedToDownloads": "File saved to your \"Downloads\" folder.",
    "filenameNote": "Filename: \"{filename}\" (If a file with the same name already exists in the download folder, the browser may automatically add a number suffix)",
    "close": "Close",

    // Debug messages
    "incompleteIPDetected": "Incomplete IP address detected: {ip}, more digits may follow",
    "newIPDetected": "New IP address detected: {ip}, updated from current IP: {oldIP}",
    "invalidIPFormat": "Invalid IP address format: {ip}",

    // Undo/Redo buttons
    "undo": "Undo",
    "redo": "Redo",
  },

  ja: {
    // ボタンとタイトル
    "appTitle": "Petoiウェブコーディングブロック",
    "showCode": "コードを表示",
    "runCode": "コードを実行",
    "saveProgram": "プログラムを保存",
    "loadProgram": "プログラムを読み込む",
    "clearAll": "すべてクリア",
    "clearConsole": "ログをクリア",
    "consoleLog": "コンソールログ",
    "serialConnect": "シリアルポートに接続",
    "quickConnect": "クイック接続",
    "closeSerial": "接続を閉じる",
    "clearDisplay": "表示をクリア",
    "send": "送信",
    "serialOutput": "シリアルモニター",
    "serialInputPlaceholder": "送信する内容を入力",
    "wifiConfig": "WiFi設定",
    "resetPrompt": "WiFiを設定する前にRESETボタンを押してください",
    "ssidPlaceholder": "WiFi名を入力",
    "passwordPlaceholder": "WiFiパスワードを入力",
    "cancel": "キャンセル",
    "confirm": "確認",
    "scanWifi": "WiFiをスキャン",
    "scanning": "スキャン中...",
    "serialConfiguredIP": "IPアドレス: ",
    "serialReadError": "シリアルデータ読み取りエラー:",
    "serialSendError": "データ送信エラー:",
    "serialConnectionError": "シリアル接続エラー:",
    "enterWifiName": "WiFi名を入力してください",
    "wifiCommandError": "WiFiコマンドエラー:",
    "wifiCommandFailed": "WiFiコマンド送信に失敗しました",
    "connectedToDevice": "デバイスに接続しました: ",
    "textCopied": "テキストがクリップボードにコピーされました",
    "serialPortBusy": "シリアルポートが使用中です。このシリアルポートを使用している他のプログラムを閉じてください",
    "taskEnded": "タスク終了",
    "showSentCommands": "送信コマンドを表示",
    "commandCompleted": "コマンドが完了しました",
    "commandNoReturnWarning": "警告：期待される戻り値が受信されませんでした：",
    "fileLoaded": "読み込み完了：{filename}",
    "loadFileFailed": "ファイルの読み込みに失敗しました",
    "currentFileLabel": "現在のファイル：",

    // コマンドメッセージ
    "sendingCommand": "コマンド送信中: ",
    "commandFailed": "コマンド実行失敗:",
    "httpError": "HTTPエラー: ",

    // コード表示ウィンドウ
    "generatedCode": "生成されたJavaScriptコード",
    "copyCode": "コードをコピー",
    "codeCopied": "コピー成功！",

    // ブロックカテゴリ
    "categoryLogic": "論理",
    "categoryLoops": "ループ",
    "categoryMath": "数学",
    "categoryText": "テキスト",
    "categoryVariables": "変数",
    "categoryFunctions": "関数",
    "categoryCommunication": "通信",
    "categoryMotion": "動作",
    "categoryControl": "制御",
    "categoryConsole": "コンソール",
    "categoryMusic": "音楽",

    // ブロックテキスト - 通信
    "connectWithIP": "IPアドレスに接続 %1",
    "getDigitalInput": "デジタル入力を取得 %1",
    "getAnalogInput": "アナログ入力を取得 %1",
    "getSensorInput": "センサーを取得 %1",
    "setDigitalOutput": "デジタル出力ピン %1 状態 %2 を設定",
    "setAnalogOutput": "アナログ出力ピン %1 値 %2 を設定",
    "sendCustomCommand": "カスタムコマンドを送信 %1",

    // ブロックテキスト - モーション
    "setMotorAngle": "関節 %1 の角度を %2 に設定",
    "getJointAngle": "関節 %1 の角度を取得",
    "getAllJointAngles": "すべての関節角度を取得",
    "gait": "歩行パターン %1",
    "posture": "姿勢 %1",
    "acrobatic_moves": "アクロバティック動作（注意して使用） %1",
    "localAction": "静的アクション %1",
    "highDifficultyAction": "高難度アクション（注意して使用） %1",
    "setMotorAngleWithDelay": "関節 %1 の角度を %2 に設定し %3 秒待機",
    "gaitWithDelay": "歩行パターン %1 の後 %2 秒待機",
    "postureWithDelay": "姿勢 %1 の後 %2 秒待機",
    "acrobaticWithDelay": "アクロバティック動作（注意して使用）%1 の後 %2 秒待機",

    // ブロックテキスト - 制御
    "delayMs": "%1 秒待機",
    "delayMessage": "{delay} 秒待機中...",

    // ブロックテキスト - センサー
    "gyroControl": "ジャイロスコープ %1",

    // ブロックテキスト - コンソール
    "consoleLogVariable": "変数をコンソールに出力 %1",

    // ブロックテキスト - 音楽
    "playNote": "音符 %1 を %2 拍子分再生",

    // アクションオプション
    "stand": "立つ",
    "sit": "座る",
    "rest": "休息",
    "pee": "おしっこ",
    "backflip": "バック宙返り",
    "jump": "ジャンプ",
    "handstand": "逆立ち",
    "boxing": "ボクシング",
    "frontflip": "前方宙返り",
    "step": "足踏み",
    "rotateLeft": "左回転",
    "rotateRight": "右回転",
    "walkForward": "前進",
    "walkLeft": "左歩き",
    "walkRight": "右歩き",
    "walkBackward": "後退",
    "backLeft": "左後方",
    "backRight": "右後方",
    "trotForward": "前方トロット",
    "trotLeft": "左トロット",
    "trotRight": "右トロット",
    "crawlForward": "前方這い進み",
    "crawlLeft": "左這い進み",
    "crawlRight": "右這い進み",
    "gapForward": "前方跨ぎ",
    "gapLeft": "左跨ぎ",
    "gapRight": "右跨ぎ",
    "moonwalk": "ムーンウォーク",

    // センサーオプション
    "ultrasonic": "超音波",
    "touch": "タッチ",
    "distance": "距離",
    "light": "光",
    "temperature": "温度",
    "humidity": "湿度",
    "gyroEnable": "有効",
    "gyroDisable": "無効",

    // 接続およびエラー関連メッセージ
    "connectingDevice": "デバイスに接続中: ",
    "deviceResponseInfo": "デバイス応答情報: ",
    "deviceModelInfo": "デバイスモデル: ",
    "errorMockData": "エラー：実際のリクエストではなくモックデータが使用されています！",
    "connectionFailedMock": "接続失敗：システムは実際のネットワークリクエストではなくモックデータを使用しました。ネットワーク設定を確認してください。",
    "connectionFailedCheck": "ロボットへの接続に失敗しました！IPアドレスが正しいことと、デバイスの電源が入っていることを確認してください。",
    "connectionError": "接続エラー: ",
    "connectionTimeout": "接続タイムアウト：{ip}に接続できません。デバイスの電源が入っていて同じネットワーク上にあることを確認してください。",
    "networkError": "ネットワークエラー：{ip}に接続できません。ネットワーク接続を確認してください。",
    "connectionErrorDetails": "ロボット接続エラー: {error}",
    "programExecutionStopped": "プログラムの実行が停止しました。後続の指示は実行されません。",
    "errorInvalidIP": "エラー：無効なIPアドレス",
    "receivedResponse": "受信したレスポンス: ",
    "requestFailedStatusCode": "リクエストに失敗しました。ステータスコード: {status}",
    "requestError": "リクエストエラー: ",
    "invalidIPAddress": "無効なIPアドレス",
    "requestTimeout": "リクエストタイムアウト",
    "asyncSendingCommand": "非同期コマンド送信中: ",
    "asyncReceivedResponse": "非同期レスポンス受信: ",
    "networkRequestError": "ネットワークリクエストエラー",
    "programEndingRestCommand": "プログラム終了、休息コマンドを送信中...",
    "restCommandFailed": "休息コマンドの送信に失敗しました: ",

    // コードジェネレーターコメント
    "connectingIPAddress": "IPアドレスに接続中",
    "connectionFailedComment": "接続失敗時に後続プログラムの実行を停止します。makeConnection関数がすでにエラーを表示しているため、例外はスローしません",
    "executionStoppedComment": "後続のコード実行を停止するために直接戻ります",
    "mockRequest": "モックリクエスト: ",
    "usingMockHttpRequest": "注：モックHTTPリクエストを使用しています。偽のデバイスモデルを返します",

    // 関節名
    "jointHeadPanning": "頭部パン",
    "jointHeadTiltingNybble": "頭部チルト（Nybble）",
    "jointTailNybble": "しっぽ（Nybble）",
    "jointLFArm": "左前腕",
    "jointRFArm": "右前腕",
    "jointRBArm": "右後腕",
    "jointLBArm": "左後腕",
    "jointLFKnee": "左前膝",
    "jointRFKnee": "右前膝",
    "jointRBKnee": "右後膝",
    "jointLBKnee": "左後膝",
    "jointReserved": "予約済み",

    // ジョイントブロックUIテキスト
    "setJointLabel": "関節を設定",
    "angleTo": "角度を",
    "thenDelay": "その後遅延",
    "secUnit": "秒",

    // 音符ブロックUIテキスト
    "playNote": "音符を再生",
    "forDuration": "時間",
    "beatUnit": "拍",

    // モーションブロックUIテキスト
    "gaitLabel": "歩行パターン",
    "postureLabel": "姿勢",
    "acrobaticMovesLabel": "アクロバティック動作（注意して使用）",

    // 音符名
    "noteRest": "休符",
    "noteLowC": "低いC",
    "noteLowCSharp": "低いC#",
    "noteLowD": "低いD",
    "noteLowDSharp": "低いD#",
    "noteLowE": "低いE",
    "noteLowF": "低いF",
    "noteLowFSharp": "低いF#",
    "noteLowG": "低いG",
    "noteLowGSharp": "低いG#",
    "noteLowA": "低いA",
    "noteLowASharp": "低いA#",
    "noteLowB": "低いB",
    "noteMiddleC": "中央C",
    "noteMiddleCSharp": "中央C#",
    "noteMiddleD": "中央D",
    "noteMiddleDSharp": "中央D#",
    "noteMiddleE": "中央E",
    "noteMiddleF": "中央F",
    "noteMiddleFSharp": "中央F#",
    "noteMiddleG": "中央G",
    "noteMiddleGSharp": "中央G#",
    "noteMiddleA": "中央A",
    "noteMiddleASharp": "中央A#",
    "noteMiddleB": "中央B",
    "noteHighC": "高いC",

    // 歩行オプション
    "gaitStep": "足踏み",
    "gaitRotateLeft": "左回転",
    "gaitRotateRight": "右回転",
    "gaitWalkForward": "前進",
    "gaitWalkLeft": "左歩き",
    "gaitWalkRight": "右歩き",
    "gaitWalkBackward": "後退",
    "gaitBackLeft": "左後方",
    "gaitBackRight": "右後方",
    "gaitTrotForward": "前方トロット",
    "gaitTrotLeft": "左トロット",
    "gaitTrotRight": "右トロット",
    "gaitCrawlForward": "前方這い進み",
    "gaitCrawlLeft": "左這い進み",
    "gaitCrawlRight": "右這い進み",
    "gaitGapForward": "前方跨ぎ",
    "gaitGapLeft": "左跨ぎ",
    "gaitGapRight": "右跨ぎ",
    "gaitMoonwalk": "ムーンウォーク",

    // 姿勢オプション
    "postureStand": "立つ",
    "postureSit": "座る",
    "postureRest": "休息",
    "posturePee": "おしっこ",

    // アクロバティックオプション
    "acrobaticHandstand": "逆立ち",
    "acrobaticBoxing": "ボクシング",
    "acrobaticBackflip": "バック宙返り",
    "acrobaticFrontflip": "前方宙返り",
    "acrobaticJump": "ジャンプ",

    // コードダイアログUIテキスト
    "generatedJSCode": "生成されたJavaScriptコード",
    "copyCode": "コードをコピー",
    "copySuccess": "コピー完了！",

    // 保存ダイアログUIテキスト
    "currentFile": "現在のファイル: \"{filename}\"",
    "noFileSaved": "ファイルが保存されていません",
    "save": "保存",
    "saveAs": "名前を付けて保存",
    "saveWarning": "注：ダウンロードフォルダに同名のファイルがすでに存在する場合、ブラウザは自動的に数字の接尾辞を追加します。",
    "saveProgram": "プログラムを保存",

    // 保存成功ダイアログUIテキスト
    "saveSuccess": "保存成功",
    "savedToDownloads": "ファイルが「ダウンロード」フォルダに保存されました。",
    "filenameNote": "ファイル名: \"{filename}\"（ダウンロードフォルダに同名のファイルがすでに存在する場合、ブラウザは自動的に数字の接尾辞を追加することがあります）",
    "close": "閉じる",

    // デバッグメッセージ
    "incompleteIPDetected": "不完全なIPアドレスが検出されました: {ip}、後に続く数字がある可能性があります",
    "newIPDetected": "新しいIPアドレスが検出されました: {ip}、現在のIP: {oldIP} から更新",
    "invalidIPFormat": "無効なIPアドレス形式: {ip}",

    // Undo/Redo buttons
    "undo": "元に戻す",
    "redo": "やり直し",

    // ファイル操作
    "enterFileName": "ファイル名を入力してください:",

    // コードダイアログUIテキスト
    "generatedJSCode": "生成されたJavaScriptコード"
  }
};

let currentLang = 'en'; // 默认语言

function setLanguage(lang)
{
  currentLang = lang;
  updateBlocklyTranslations();
  updateUITranslations();
}

function getText(key)
{
  return TRANSLATIONS[currentLang][key] || key;
}

function updateBlocklyTranslations()
{
  // 更新所有积木的文本
  workspace.getAllBlocks().forEach(block =>
  {
    block.render();
  });
}

function updateUITranslations()
{
  // 更新UI元素的文本
  document.querySelectorAll('[data-i18n]').forEach(element =>
  {
    const key = element.getAttribute('data-i18n');
    if (element.tagName.toLowerCase() === 'input')
    {
      element.placeholder = getText(key);
    } else
    {
      // 特殊处理currentFileLabel，在英文模式下使用不间断空格
      if (key === 'currentFileLabel' && currentLang === 'en')
      {
        element.innerHTML = 'Current file:&nbsp;&nbsp;';
      } else
      {
        element.textContent = getText(key);
      }
    }
  });
} 
