# Modbus 主机回归测试

测试直接编译 `HardLib/modbus.c`、`HardLib/rs485.c`，用内存中的 UART / DMA / DWT
替代硬件。传感器源码也参与编译，用于检查其初始化不会重置通信共用的计数器。
这些测试不访问串口，不会烧录或操作开发板。

需要 CMake、Ninja 和能生成本机可执行程序的 C 编译器，例如 MinGW GCC。
这里不能使用 `arm-none-eabi-gcc`，因为生成的测试需要在电脑上运行。

在项目根目录运行；编译器不在 PATH 中时，将 `gcc` 换成其完整路径：

```powershell
cmake -S tests -B build/HostTests -G Ninja '-DCMAKE_C_COMPILER=gcc'
cmake --build build/HostTests
ctest --test-dir build/HostTests --output-on-failure
```

覆盖 14 组行为：温度及负温度报文、1.75 ms 静默时间、DWT 回绕、DMA 尚有接收字节、
长分片、多寄存器写入、中断屏蔽状态保持、CRC / 从站地址过滤、异常应答、保持寄存器、
线圈 / 离散输入、广播写入、缓冲溢出恢复、RS485 收发状态和共用 DWT 初始化。
八种已实现的功能码均参与测试。

固定示例：温度寄存器为 2500，即 25.00 ℃ 时：

```text
请求：01 04 00 00 00 01 31 CA
应答：01 04 02 09 C4 BE F3
```

测试不能验证实际引脚、A/B 极性、转换器、电路及板内烧录的程序。
开发板实际收发仍需上板验证。
