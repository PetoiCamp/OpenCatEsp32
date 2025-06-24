/**
 * JavaScript代码生成器 - 为所有自定义积木生成JavaScript代码
 */

// 代码生成:发送步态动作命令
Blockly.JavaScript.forBlock['gait'] = function (block)
{
  const code = block.getFieldValue('COMMAND');
  const delay = block.getFieldValue('DELAY');
  const delayMs = Math.round(delay * 1000);
  return `console.log(await httpRequest(deviceIP, "${code}", 2000, true));\n` +
    `await new Promise(resolve => setTimeout(resolve, ${delayMs}));\n`;
};

// 代码生成:发送姿势动作命令
Blockly.JavaScript.forBlock['posture'] = function (block)
{
  const code = block.getFieldValue('COMMAND');
  const delay = block.getFieldValue('DELAY');
  const delayMs = Math.round(delay * 1000);
  return `console.log(await httpRequest(deviceIP, "${code}", 2000, true));\n` +
    `await new Promise(resolve => setTimeout(resolve, ${delayMs}));\n`;
};

// 代码生成:发送杂技动作命令
Blockly.JavaScript.forBlock['acrobatic_moves'] = function (block)
{
  const code = block.getFieldValue('COMMAND');
  const delay = block.getFieldValue('DELAY');
  const delayMs = Math.round(delay * 1000);
  return `console.log(await httpRequest(deviceIP, "${code}", 2000, true));\n` +
    `await new Promise(resolve => setTimeout(resolve, ${delayMs}));\n`;
};

// 代码生成:设置马达角度代码生成器
Blockly.JavaScript.forBlock['set_motor_angle'] = function (block)
{
  const motorId = block.getFieldValue('MOTOR');
  const angle = block.getFieldValue('ANGLE');
  const delay = block.getFieldValue('DELAY');
  const delayMs = Math.round(delay * 1000);
  // 生成唯一的随机后缀，防止变量名冲突
  const uniqueSuffix = Math.floor(Math.random() * 10000);

  return `// 设置关节角度并获取响应
console.log(await httpRequest(deviceIP, "m ${motorId} " + Math.min(125, Math.max(-125, ${angle})), 2000, true));\n` +
    `await new Promise(resolve => setTimeout(resolve, ${delayMs}));\n`;
};

// 代码生成:获取关节角度的代码生成器
Blockly.JavaScript.forBlock['get_joint_angle'] = function (block)
{
  const jointId = block.getFieldValue('JOINT');
  return [`parseInt(await httpRequest(deviceIP, "m ${jointId} ?", 2000, true)) || 0`, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

// 代码生成:获取所有关节角度的代码生成器
Blockly.JavaScript.forBlock['get_all_joint_angles'] = function (block)
{
  const code = `
    (async function() {
      let angles = {};
      for(let i = 0; i <= 11; i++) {
        angles["joint" + i] = parseInt(await httpRequest(deviceIP, "m " + i + " ?", 2000, true)) || 0;
      }
      return JSON.stringify(angles);
    })()
  `;
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

// 代码生成:延时代码生成器
Blockly.JavaScript.forBlock['delay_ms'] = function (block)
{
  const delay = block.getFieldValue('DELAY');
  const delayMs = Math.round(delay * 1000); // 将秒转换为毫秒
  return `console.log(getText("delayMessage").replace("{delay}", ${delay}));\nawait new Promise(resolve => setTimeout(resolve, ${delayMs}));\n`;
};

// 代码生成:陀螺仪控制代码生成器
Blockly.JavaScript.forBlock['gyro_control'] = function (block)
{
  const state = block.getFieldValue('STATE');
  return `console.log(await httpRequest(deviceIP, "g ${state}", 2000, true));\n`;
};

// 代码生成:连接代码生成器
Blockly.JavaScript.forBlock['make_connection'] = function (block)
{
  const ip = block.getFieldValue('IP_ADDRESS');
  return `// ${getText("connectingIPAddress")}
const connectionResult = await makeConnection("${ip}");
if(connectionResult) {
  deviceIP = "${ip}";
  // console.log(getText("connectedToDevice") + deviceIP);
} else {
  // ${getText("connectionFailedComment")}
  return; // ${getText("executionStoppedComment")}
}\n`;
};

// 代码生成:获取数字输入代码生成器 - 移除重复定义，改为异步
Blockly.JavaScript.forBlock['get_digital_input'] = function (block)
{
  const pin = block.getFieldValue('PIN');
  const code = `await (async function() {
    const rawResult = await httpRequest(deviceIP, "Rd" + String.fromCharCode(${pin}) + "\\\\n", 2000, true);
    
    // 首先尝试提取=号后的数字
    if (rawResult && rawResult.includes('=')) {
      const lines = rawResult.split('\\\\n');
      for (let i = 0; i < lines.length; i++) {
        if (lines[i].trim() === '=' && i + 1 < lines.length) {
          const num = parseInt(lines[i + 1].trim());
          if (!isNaN(num)) {
            return num;
          }
        }
      }
    }
    
    // 尝试从单行格式中提取数字，如"4094 R"
    const words = rawResult.trim().split(/\\\\s+/);
    for (const word of words) {
      const num = parseInt(word);
      if (!isNaN(num)) {
        return num;
      }
    }
    
    return 0;
  })()`;
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

// 代码生成:获取模拟输入代码生成器 - 移除重复定义，改为异步
Blockly.JavaScript.forBlock['get_analog_input'] = function (block)
{
  const pin = block.getFieldValue('PIN');
  const code = `await (async function() {
    const rawResult = await httpRequest(deviceIP, "Ra" + String.fromCharCode(${pin}) + "\\\\n", 2000, true);
    
    // 首先尝试提取=号后的数字
    if (rawResult && rawResult.includes('=')) {
      const lines = rawResult.split('\\\\n');
      for (let i = 0; i < lines.length; i++) {
        if (lines[i].trim() === '=' && i + 1 < lines.length) {
          const num = parseInt(lines[i + 1].trim());
          if (!isNaN(num)) {
            return num;
          }
        }
      }
    }
    
    // 尝试从单行格式中提取数字，如"4094 R"
    const words = rawResult.trim().split(/\\\\s+/);
    for (const word of words) {
      const num = parseInt(word);
      if (!isNaN(num)) {
        return num;
      }
    }
    
    return 0;
  })()`;
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

// 代码生成:获取传感器输入代码生成器
Blockly.JavaScript.forBlock['get_sensor_input'] = function (block)
{
  const sensor = block.getFieldValue('SENSOR');
  return [`parseInt(await httpRequest(deviceIP, "i ${sensor}", 2000, true)) || 0`, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

// 代码生成:设置数字输出代码生成器
Blockly.JavaScript.forBlock['set_digital_output'] = function (block)
{
  const pin = block.getFieldValue('PIN');
  const state = block.getFieldValue('STATE');
  return `console.log(await httpRequest(deviceIP, "o ${pin} d ${state}", 2000, true));\n`;
};

// 代码生成:设置模拟输出代码生成器
Blockly.JavaScript.forBlock['set_analog_output'] = function (block)
{
  const pin = block.getFieldValue('PIN');
  const value = block.getFieldValue('VALUE');
  return `console.log(await httpRequest(deviceIP, "o ${pin} a ${value}", 2000, true));\n`;
};

// 代码生成:发送自定义命令代码生成器
Blockly.JavaScript.forBlock['send_custom_command'] = function (block)
{
  const command = block.getFieldValue('COMMAND');
  return `console.log(await httpRequest(deviceIP, "${command}", 2000, true));\n`;
};

// 代码生成:控制台输出变量代码生成器
Blockly.JavaScript.forBlock['console_log_variable'] = function (block)
{
  const variable = Blockly.JavaScript.valueToCode(block, 'VARIABLE', Blockly.JavaScript.ORDER_NONE) || '""';
  return `console.log(${variable});\n`;
};

// 代码生成:播放音符代码生成器
Blockly.JavaScript.forBlock['play_note'] = function (block)
{
  const note = block.getFieldValue('NOTE');
  const duration = block.getFieldValue('DURATION');
  return `console.log(await httpRequest(deviceIP, "b ${note} ${duration}", 2000, true));\n`;
};

// HTTP请求函数，用于在生成的代码中使用 - 仅供模拟测试
function mockHttpRequest(ip, command, returnResult = false)
{
  // 在命令前添加标识前缀，用于调试，但不改变原始命令行为
  const debugCommand = "[MOCK]" + command;
  // console.log(getText("mockRequest") + `${debugCommand} -> ${ip}`);

  // 针对不同命令返回不同模拟值
  if (returnResult)
  {
    // 模拟设备型号查询
    if (command === '?')
    {
      // console.warn(getText("usingMockHttpRequest"));
      return "PetoiModel-v1.0";
    }

    // 模拟传感器、数字和模拟输入的响应
    if (command.startsWith("Ra") || command.startsWith("Rd") || command.startsWith("i ") || command.includes(" ?"))
    {
      return "123";
    }
  }

  return returnResult ? "0" : true; // 默认返回值
}

// 调试时可以通过以下方式启用模拟请求
// window.httpRequest = mockHttpRequest;
