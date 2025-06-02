// 定义陀螺仪控制积木块
Blockly.defineBlocksWithJsonArray([
  {
    type: 'gyro_control',
    message0: "陀螺仪 %1",
    args0: [
      {
        type: "field_dropdown",
        name: "ACTION",
        options: [
          ["启用", "enable"],
          ["禁用", "disable"]
        ]
      }
    ],
    inputsInline: true,
    previousStatement: null,
    nextStatement: null,
    colour: 230,
    tooltip: "控制陀螺仪模块的启用和禁用",
    helpUrl: ""
  }
]);

// 生成陀螺仪控制的代码
javascript.javascriptGenerator.forBlock['gyro_control'] = function(block) {
  const action = block.getFieldValue('ACTION');
  const value = action === 'enable' ? 'B' : 'b';
  const message = action === 'enable' ? '已启用' : '已禁用';
  
  return `httpRequest(deviceIP, "G${value}");
  console.log("陀螺仪${message}");`;
};

// 定义传感器输入积木
Blockly.defineBlocksWithJsonArray([
  {
    type: 'get_sensor_input',
    message0: "获取传感器 %1",
    args0: [
      {
        type: "field_dropdown",
        name: "SENSOR",
        options: [
          ["超声波", "ultrasonic"],
          ["触摸", "touch"],
          ["距离", "distance"]
        ]
      }
    ],
    output: true,
    outputType: "Number",
    colour: 230,
    tooltip: "读取各种传感器的值",
    helpUrl: ""
  }
]);

// 代码生成器保持不变
javascript.javascriptGenerator.forBlock['get_sensor_input'] = function(block) {
  var sensor = block.getFieldValue('SENSOR');
  return [`httpRequest(deviceIP, "${sensor}")`, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};
