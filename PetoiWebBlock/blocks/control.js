// 延迟积木
Blockly.defineBlocksWithJsonArray([
    {
      type: 'delay_ms',
      message0: "delay %1 ms",
      args0: [
        {
          type: "field_input",
          name: "DELAY_MS",
          text: "1000",
        }
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 230,
      tooltip: "delay milliseconds",
      helpUrl: ""
    },
  ]);


// delay function block the process
function delay(time_ms) {
    const start = Date.now();
    while (Date.now() - start < time_ms) {
      // Wait
    }
}
  
// 延迟积木代码生成
javascript.javascriptGenerator.forBlock['delay_ms'] = function(block) {
  
    // Use valueToCode for input_value
    const time = block.getFieldValue('DELAY_MS');    
    // Generate the code that will be executed
    const code = `delay(${time});\n`;
    return code;
};
