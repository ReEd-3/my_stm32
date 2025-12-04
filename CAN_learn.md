# CAN总线通信

## 一. 通信原理

### 1. 帧格式

|帧类型   |作用    |
|**数据帧**  |发送设备进行数据发送|
|**遥控帧**  |接收设备进行数据请求|
|**错误帧&过载帧&帧间隔**|略|

1. 数据帧
    数据帧分为**标准格式**和**扩展格式**两种  
    1. 标准格式：
        - 一个标准格式数据帧由**帧起始、仲裁段、控制段、数据段、CRC段、ACK段、帧结束**顺序组成
        ```

        帧起始  | 仲裁段 | 控制段   |数据段  | CRC段  |ACK段          |帧结束
        SOF     |ID,RTR |IDE,r0,DLC|DATA   |CRC     |ACK槽，ACK界定符|

        ```
        - **帧起始**之前总线上一直为空闲（隐形电平1），帧起始为**1位**的**显性电平0**

        - **仲裁段**由**11位报文ID**和**1位RTR**组成
        >   **RTR**用于区分**遥控帧**和**数据帧**，数据帧为显性0，遥控帧为隐性1   

        - **控制段**由**1位IDE**，**1位r0**和**4位DLC**组成
        >   **IDE**用于区分标准格式和扩展格式，标准格式为显性0，扩展格式为隐形1  
        >   **r0**用于占位为显性0，无实际意义  
        >   **DLC**表示数据段的长度  

        - **数据段**由要发送的数据组成

        - **CRC段**由**15位CRC校验码**和**1位CRC界定符**
        >**CRC校验码**是由CRC算法对此前进行的数据进行计算得到的校验码  
        >**CRC界定符**必须是隐形1，跟在**CRC校验码**之后

        - **ACK段**由**1位ACK槽**和**1位ACK界定符**组成
        >**ACK槽**段时，发送方不做发送，总线默认收紧为隐性1，如果有接收方接收到，则拉开总线变为显性0，否则仍然为隐性1  
        >**ACK界定符**必须是隐性1

        - **帧结束**由7个隐性1组成
    2. 扩展格式：
        - 扩展格式有18位额外ID
        - 前11位ID后接SRR和IDE，都为**一位隐性电平1**， SRR用来替代RTR位
        - **IDE**后是**18位扩展ID**和**1位RTR**
        - **RTR**后接**1位r0**和**1位r1**
        - 之后接DLC，后面与标准格式相同

2. 遥控帧
    遥控帧也分为**标准格式**和**扩展格式**两种  
    - 遥控帧**没有数据段**，**RTR位**为隐性0
    - 遥控帧用来**请求数据**，响应请求的一方通过相同的ID反馈数据

### 2. 位填充

发送方发送五个相同电平时，自动追加一个相反电平进行填充，接收方会自动移除填充位

### 3. 位同步

1. 位时序

每个数据位时长分为**同步段（SS）、传播时间段（PTS）、相位缓冲段1（PBS1）、相位缓冲段2（PBS2）**,每个段由若干个最小时间位组成  
> SS    ->  1Tq  
> PTS   ->  1~8Tq  
> PBS1  ->  1~8Tq  
> PBS2  ->  2~8Tq  

**数据跳变沿**控制在**SS段**  
**采样点**在**PTS1和PTS2**之间  

2. 硬同步

发送方在**SS段**进行数据发送，接收方在收到**SOF段**下降沿时，接收方会将**位时序计时器**拨到**SS段**  

硬同步只在**SOF下降**沿有效  

3. 再同步

发送方和接收方收发数据不同步时，接收方根据**SWJ（再同步补偿宽度值）**，**加长PBS1段**或者**缩短PBS2段**进行再同步

4. 波特率计算

波特率 = 1 / 一个数据位的时长

### 4. 仲裁

1. 资源分配规则

    - 若当前有设备操作总线，则其他任何设备**不能再发送数据帧或者遥控帧**
    - 检测到**11位连续的隐性连续电平**则认为总线空闲

2. 非破坏性仲裁
    - 设备发送一个电平会**回读**总线当前电平状态
    - 同时发送的数据，**ID小**的在仲裁胜出
    - 仲裁失利的设备转变为接收状态，等待下一次总线空闲
    - **数据帧**优先级高于**遥控帧**
    - **标准格式**优先级高于**扩展格式**

### 3. 代码编写

1. 配置**CAN过滤器**
```c
    CAN_FilterTypeDef canfilterconfig; // 结构体变量声明
  
    // 配置CAN过滤器
    canfilterconfig.FilterBank = 0;                         // 使用过滤器组0
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;     // 掩码模式
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;    // 32位模式
    canfilterconfig.FilterIdHigh = 0x0000;                  // 标准ID高16位
    canfilterconfig.FilterIdLow = 0x0000;                   // 标准ID低16位
    canfilterconfig.FilterMaskIdHigh = 0x0000;              // 屏蔽位高16位
    canfilterconfig.FilterMaskIdLow = 0x0000;               // 屏蔽位低16位
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;    // 使用FIFO0
    canfilterconfig.FilterActivation = ENABLE;              // 启用过滤器
    canfilterconfig.SlaveStartFilterBank = 14;              // 指定CAN2过滤器组

```

2. 初始化
    初始化需要先**开启过滤器**，再**启用CAN**，再**启动中断回调**

```c
    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig); // 开启过滤器
    HAL_CAN_Start(&hcan); // 启用CAN
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 启用中断回调
```
正常情况下这三个函数返回值均为`HAL_OK`

3. 发送数据

```c

    CAN_TxHeaderTypeDef TxHeader; // 定义结构体变量
    uint8_t TxData[8]; // 数据缓冲区

    TxHeader.StdId = 0x123;            // 标准ID
    TxHeader.ExtId = 0x01;             // 扩展ID
    TxHeader.RTR = CAN_RTR_DATA;       // 数据帧
    TxHeader.IDE = CAN_ID_STD;         // 使用标准ID
    TxHeader.DLC = 8;                  // 数据长度8字节
    TxHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox); // 返回值也为HAL_OK

```
需要使用`TxData[7] = 0x0a;`将缓冲区最后一位写为`\n`，使得VOFA+能够显示这一段数据

4. 接收数据

在**中断**中接收数据，并且改变接收标志`RX_flag`的值
```c

    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        RX_flag = 1;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData); // RxData为数据接收缓冲区
    }

```
主循环中对数据进行处理
```c
	if(RX_flag){
		HAL_UART_Transmit(&huart1, RxData, 8,1000);
		RX_flag = 0;
	}
```

### 注意事项

中断中不能使用串口进行发送数据，遵循“快进快出原则”

向串口发送数据以`\n`结尾才能使数据在VOFA+中被打印
