/**
 * PetoiWebBlock 异步通信客户端
 * 解决看门狗重启问题，支持严格时序控制
 */

class PetoiAsyncClient
{
    constructor(baseUrl = null)
    {
        this.baseUrl = baseUrl || `http://${window.location.hostname}`;
        this.taskTimeout = 10000; // 10秒超时
    }

    /**
     * 发送异步命令
     */
    async sendCommand(cmd)
    {
        try
        {
            const response = await fetch(`${this.baseUrl}/?cmd=${encodeURIComponent(cmd)}`);
            const taskId = (await response.text()).replace('TASK_ID:', '').trim();
            return await this.waitForCompletion(taskId);
        } catch (error)
        {
            throw new Error(`命令发送失败: ${error.message}`);
        }
    }

    /**
     * 等待任务完成
     */
    async waitForCompletion(taskId)
    {
        const startTime = Date.now();

        while (Date.now() - startTime < this.taskTimeout)
        {
            try
            {
                const response = await fetch(`${this.baseUrl}/status?taskId=${taskId}`);
                const text = await response.text();
                const lines = text.trim().split('\n');
                const status = lines[0];
                const result = lines.slice(1).join('\n');

                if (status === 'completed')
                {
                    return result;
                } else if (status === 'error')
                {
                    throw new Error(result);
                }

                await this.delay(100);
            } catch (error)
            {
                throw new Error(`状态查询失败: ${error.message}`);
            }
        }

        throw new Error(`任务${taskId}执行超时`);
    }

    /**
     * 延迟函数
     */
    delay(ms)
    {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    /**
     * 连续读取传感器
     */
    async readSensorContinuous(pin, count, intervalMs = 500)
    {
        const readings = [];

        for (let i = 0; i < count; i++)
        {
            const result = await this.sendCommand(`Ra${pin}`);
            readings.push(result);

            if (i < count - 1)
            {
                await this.delay(intervalMs);
            }
        }

        return readings;
    }

    /**
     * 执行带时序的命令序列
     */
    async executeSequence(sequence)
    {
        for (const step of sequence)
        {
            if (step.type === 'command')
            {
                await this.sendCommand(step.cmd);
            } else if (step.type === 'delay')
            {
                await this.delay(step.duration);
            }
        }
    }
}

// 使用示例
async function exampleUsage()
{
    const client = new PetoiAsyncClient();

    // 基本命令
    await client.sendCommand('kup');

    // 时序控制序列
    const sequence = [
        { type: 'command', cmd: 'kwkF' },     // 向前走
        { type: 'delay', duration: 3000 },    // 等待3秒
        { type: 'command', cmd: 'i0 45' },    // 头部左转
        { type: 'delay', duration: 1000 },    // 等待1秒
        { type: 'command', cmd: 'kup' }       // 停止站立
    ];

    await client.executeSequence(sequence);

    // 连续传感器读取
    const readings = await client.readSensorContinuous('35', 5, 200);
}

// 创建全局实例
window.PetoiAsyncClient = PetoiAsyncClient;

// 兼容性：创建默认实例
if (typeof window !== 'undefined')
{
    window.petoiClient = new PetoiAsyncClient();
}

export default PetoiAsyncClient; 
