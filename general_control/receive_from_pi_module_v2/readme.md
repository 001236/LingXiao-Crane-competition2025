### **`receive_from_pi` 模块使用指南 (V3 - 终极版)**

本模块用于从树莓派接收初始场地布局信息，采用轮询方式工作。它提供了三种接收函数：一个简单阻塞函数，一个带修复和超时的健壮被动接收函数，以及一个功能最全、能主动向树莓派请求数据的高级函数。

#### **1. CubeMX 配置 (UART4)**

这部分配置保持不变。

- **外设选择**: `Connectivity` -> `UART4`
- **模式**: `Asynchronous`
- **引脚**: CubeMX自动分配 (如 `PA0` UART4_TX, `PA1` UART4_RX)，请根据硬件核对。
- **参数设置**:
  - 波特率 (Baud Rate): `115200` Bits/s
  - 字长 (Word Length): `8 Bits`
  - 校验位 (Parity): `None`
  - 停止位 (Stop Bits): `1`
  - 数据方向 (Data Direction): `Receive and Transmit`
- **NVIC 中断**: 轮询方式**无需**勾选 `UART4 global interrupt`。

#### **2. 模块功能与函数**

##### **主要函数:**

1. **`void rpi_init_polling(UART_HandleTypeDef \*huart);`**
   - **功能**: 初始化本模块，必须在调用任何其他接收函数前调用一次。
   - **调用**: 在 `main()` 函数中，`MX_UART4_Init()` 之后调用。
2. **`void rpi_receive_data_blocking_polling(RecognitionResult_t \*result);`**
   - **功能**: **简单阻塞接收**。程序会一直在此处等待，直到收到一个所有位置都有有效数字（即不含'x'）的完美数据串。
   - **用途**: 适用于期望通信质量极好，且必须等待完美数据的最简单场景。
3. **`bool rpi_receive_data_configurable_polling(...);`**
   - **功能**: **健壮的被动接收**。它在简单阻塞的基础上，增加了数据修复和超时机制，但不会主动发送任何请求。
   - **用途**: 当您希望被动接收数据，但需要处理数据不完美或树莓派长时间不发送数据的情况时使用。
4. **`bool rpi_request_and_receive_configurable_polling(...);`**
   - **功能**: **功能最全的主动请求接收函数 (推荐)**。它在函数开始时会**主动发送一次请求信号**，然后等待接收数据，同时具备与上面函数完全相同的**数据修复和超时机制**。
   - **参数**:
     - `result`: (输出) 指向 `RecognitionResult_t` 结构体的指针。
     - `request_signal` (const char*): 您想要在函数开始时发送的请求信号字符串，例如 `"GET_LAST_RESULT\n"`。
     - `attempt_repair` (bool): `true`则启用修复逻辑。
     - `max_attempts_before_repair` (uint32_t): 见下面的修复逻辑详解。
     - `enable_timeout` (bool): `true`则启用超时机制。
     - `timeout_ms` (uint32_t): 整个接收过程的最大等待时间（毫秒）。
   - **返回值 (`bool`)**:
     - `true`: 成功获取到**完美数据**（原始或修复后），并且是在超时前。
     - `false`: **超时发生**。此时 `result` 中可能是最后一次接收到的不完整数据。

##### **数据修复逻辑详解:**

当 `attempt_repair` 为 `true` 时，修复逻辑如下：

- **置物区 (Area) - 立即修复**:
  - **条件**: 只要收到一条消息，其中置物区部分**仅有1个位置为'x'**，且**当前没有'0'（空位）**，同时其他5个位置为1-6中不重复的数字。
  - **行为**: 程序会**立刻**（无需等待多次）尝试将这个'x'修复为'0'。
- **货架区 (Shelf) - 延迟修复**:
  - **条件**: 收到不完美消息的累计次数达到 `max_attempts_before_repair` 后，程序会检查货架区是否满足修复条件（仅有1个位置为'x'，且其他5个位置为1-6中不重复的数字）。
  - **行为**: 如果满足条件，则自动填充缺失的那个1-6数字。

#### **3. 如何在 `main.c` 中使用**

```c
/* USER CODE BEGIN Includes */
#include "receive_from_pi.h" // 包含头文件
#include <stdio.h>           // 如果需要打印调试
#include <string.h>          // 如果需要字符串操作
/* USER CODE END Includes */

int main(void)
{
    /* ... MCU 初始化 ... */
    MX_UART4_Init();

    /* USER CODE BEGIN 2 */
    
    // 1. 定义一个结构体变量来存储结果
    RecognitionResult_t recognition_data;
    
    // 2. 初始化我们的接收模块
    rpi_init_polling(&huart4); // 将 huart4 句柄传递给模块

    // --- 您可以根据需求，选择以下三种调用方式之一 ---

    // === 方式一：最简单的阻塞接收 (不推荐，不够健壮) ===
    // rpi_receive_data_blocking_polling(&recognition_data);
    // 程序会卡在这里直到收到完美数据


    // === 方式二：被动等待，带修复和超时 (推荐用于常规接收) ===
    // bool is_ok = rpi_receive_data_configurable_polling(&recognition_data,
    //                                                    true,  // 启用修复
    //                                                    5,     // 尝试5次后修复货架
    //                                                    true,  // 启用超时
    //                                                    30000);// 30秒超时
    // if (is_ok) { /* 处理完美数据 */ } else { /* 处理超时 */ }


    // === 方式三：主动请求，带修复和超时 (推荐用于开机后首次获取数据) ===
    bool is_ok_requested = rpi_request_and_receive_configurable_polling(
                                &recognition_data,
                                "GET_LAST_RESULT\n", // 主动发送这个信号给树莓派
                                true,    // 启用修复
                                5,       // 尝试5次后修复货架
                                true,    // 启用超时
                                30000);  // 30秒超时

    if (is_ok_requested) {
        // 成功获取到完美数据
        // 在这里，您可以使用 recognition_data 中的数据来初始化您的机器人状态
    } else {
        // 超时了，未能从树莓派获取到有效布局
        // 在这里处理超时情况，例如使用一套预设的默认布局或报错
    }

    /* USER CODE END 2 */

    /* ... 主循环 ... */
}
```

#### **4. 数据结构 (`RecognitionResult_t`)**

定义在 `receive_from_pi.h` 中：

```c
typedef struct {
    int shelf_positions[6]; // 货架位置信息
    int area_positions[6];  // 置物区信息
} RecognitionResult_t;
```

- `shelf_positions[0-5]`: 对应 **1-6号货架位**，值为货箱编号 (1-6)，或 `-1` (代表'x')。
- `area_positions[0-5]`: 对应 **a-f区**，值为纸垛编号 (1-6)，空位 (`0`)，或 `-1` (代表'x')。

#### **5. 预期字符串格式 (来自树莓派)**

格式要求保持不变，必须是12对由逗号分隔的 "键:值"，并以单个换行符 \n (LF) 结尾。

示例: 1:3,2:x,3:5,4:6,5:2,6:4,a:5,b:0,c:1,d:x,e:6,f:2\n

#### **6. 注意事项**

- 模块使用 **UART4**，波特率 **115200**。
- 所有接收函数均为**阻塞式**，在获取到数据或超时前，程序会停在函数调用处。