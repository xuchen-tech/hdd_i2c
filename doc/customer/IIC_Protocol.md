# I2C address
Slave address: 0x50(7bit address)
# I2C Register Address Define List
|Reg Address|Command|C/R/W|bytes|comment|
|-|-|-|-|-|
|0x80|mode|R/W|1|0(Normal mode), 1(mode 1), 2(mode 2), etc|
|0x81|Ready|R/W|1|0(Not ready), 1~255(1~255 Data is ready)|
|0x82|Data|R/W|256|Data|

* 0x80, 0x81, 0x82 寄存器地址以及数据交互方式：

以 0x80, 0x81, 0x82 三个地址组合为一个通道，用于按照约定的协议交互目标数据。把寄存器地址命名为：

0x80: Mode
0x81: Ready
0x82: Data

周期性数据交互步骤如下
1. 记录时间 A
2. 主机读取 Mode（地址 0x80）一个字节，如果该值不为目标模式 0xD1，则写入目标模式（0xD1）, 如果已经是目标模式 0xD1，表示这个通道正按照约定的协议模式，用于传输某些特定数据，我们可以称之为目标数据，进行下一步。
3. 主机读取 Ready（地址 0x81）一个字节，如果该值为 0，则等 20ms 后，再回到第 1 步，直到读取的 Ready值不为 0。比如读取到的数据是 n（意味着从机已经把一个周期的目标数据写入到了 Data（地址 0x82）寄存器地址。
4. 主机读取 Data(地址 0x82) n 个字节，即为协议约定的 n 字节的数据。数据解析参见 《Mode Define List》
5. 主机把 Ready 写 0 值。
6. 查询计时器，当前时间为 B，如果 B- A 不小于 20ms，则从第 1 步开始，重新下一次数据读取。

# Mode Define List
|Mode |Ready(bytes)|Data 定义数据及存放顺序|
|-|-|-|
|0xD1|n>0 时 n=8/16/24/32|Unit 1 <br>Int32_t force signal (μV) 小端<br>Int16_t Temp_x10(10℃) 小端<br>Uint16_t ModebusCRC 小端(该 Unit 前面数据的CRC16_Modbus 校验值)<br><br>Unit 2 <br>Int32_t force signal (μV) 小端<br>Int16_t Temp_x10(10℃) 小端<br>Uint16_t ModebusCRC 小端(该 Unit 前面数据的CRC16_Modbus 校验值)<br><br>Unit n <br>Int32_t force signal (μV) 小端<br>Int16_t Temp_x10(10℃) 小端<br>Uint16_t ModebusCRC 小端(该 Unit 前面数据的CRC16_Modbus 校验值)|
|扩展区|扩展区|扩展区|

