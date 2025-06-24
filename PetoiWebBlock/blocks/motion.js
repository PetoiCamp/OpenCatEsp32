// 发送原地动作
Blockly.defineBlocksWithJsonArray([
  {
    type: 'local_action',
    message0: "原地动作 %1",
    args0: [
      {
        type: "field_dropdown",
        name: "COMMAND",
        options: [
          ['站立', 'kup'],
          ['坐下', 'ksit'],
          ['休息', 'd'],
          ['尿尿', 'kpee'],
        ],
      }
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 230,
    tooltip: "Send HTTP request with command parameter",
    helpUrl: ""
  },
]);

// 代码生成:发送原地动作命令
javascript.javascriptGenerator.forBlock['local_action'] = function (block)
{
  const code = block.getFieldValue('COMMAND');
  return `console.log(await httpRequest(deviceIP, "${code}", 2000, true));\n`;
};

// 发送高难度特技动作
Blockly.defineBlocksWithJsonArray([
  {
    type: 'high_difficulty_action',
    message0: "高难度特技动作(小心使用) %1",
    args0: [
      {
        type: "field_dropdown",
        name: "COMMAND",
        options: [
          ['后空翻', 'kbkf'],
          ['跳跃', 'kjmp'],
        ],
      }
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 230,
    tooltip: "Send HTTP request with command parameter",
    helpUrl: ""
  },
]);

// 代码生成:发送高难度特技动作命令
javascript.javascriptGenerator.forBlock['high_difficulty_action'] = function (block)
{
  const code = block.getFieldValue('COMMAND');
  return `console.log(await httpRequest(deviceIP, "${code}", 2000, true));\n`;
};

// 定义设置马达角度积木块
Blockly.defineBlocksWithJsonArray([
  {
    type: 'set_motor_angle',
    message0: "设置马达 %1 角度为 %2",
    args0: [
      {
        type: "field_number",
        name: "MOTOR",
        value: 0,
        min: 0,
        max: 11,
        precision: 1
      },
      {
        type: "field_number",
        name: "ANGLE",
        value: 90,
        min: 0,
        max: 180
      }
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 230,
    tooltip: "设置指定马达的角度（0-11号，0-180度）",
    helpUrl: ""
  }
]);

// 代码生成:设置马达角度代码生成器
javascript.javascriptGenerator.forBlock['set_motor_angle'] = function (block)
{
  const motorId = block.getFieldValue('MOTOR');
  const angle = block.getFieldValue('ANGLE');
  return `console.log(await httpRequest(deviceIP, "m ${motorId} " + Math.min(125, Math.max(-125, ${angle})), 2000, true));\n`;
};


// 定义获取关节角度积木块
Blockly.defineBlocksWithJsonArray([
  {
    type: 'get_joint_angle',
    message0: "获取关节 %1 的角度",
    args0: [
      {
        type: "field_dropdown",
        name: "JOINT",
        options: [
          ["0", "0"],
          ["1", "1"],
          ["2", "2"],
          ["3", "3"],
          ["4", "4"],
          ["5", "5"],
          ["6", "6"],
          ["7", "7"],
          ["8", "8"],
          ["9", "9"],
          ["10", "10"],
          ["11", "11"]
        ]
      }
    ],
    output: "Number",
    colour: 230,
    tooltip: "获取指定关节的当前角度值",
    helpUrl: ""
  }
]);

// 代码生成:获取关节角度的代码生成器
javascript.javascriptGenerator.forBlock['get_joint_angle'] = function (block)
{
  const jointId = block.getFieldValue('JOINT');
  return [`parseInt(await httpRequest(deviceIP, "m ${jointId} ?", 2000, true)) || 0`, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};


// 定义获取所有关节角度积木块
Blockly.defineBlocksWithJsonArray([
  {
    type: 'get_all_joint_angles',
    message0: "获取所有关节角度",
    output: "String",  // 改为返回字符串类型（JSON字符串）
    colour: 230,
    tooltip: "获取所有关节(1-16号)的当前角度值，返回JSON格式",
    helpUrl: ""
  }
]);

// 代码生成:获取所有关节角度的代码生成器
javascript.javascriptGenerator.forBlock['get_all_joint_angles'] = function (block)
{
  const code = `
    (async function() {
      let angles = {};
      for(let i = 1; i <= 16; i++) {
        angles["joint" + i] = parseInt(await httpRequest(deviceIP, "m " + i + " ?", 2000, true)) || 0;
      }
      return JSON.stringify(angles);
    })()
  `;
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};
