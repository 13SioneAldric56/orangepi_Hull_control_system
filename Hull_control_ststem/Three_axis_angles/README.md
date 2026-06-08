# DDM350B/DDM360B 三维电子罗盘 Python 接口

基于瑞芬科技 DDM350B/DDM360B 高精度动态三维电子罗盘的 Python 通讯接口。

## 安装

```bash
pip install pyserial
```

## 快速开始

```python
from ddm350b import DDM350B, OutputMode

# 连接罗盘
compass = DDM350B('/dev/ttyS0', baudrate=9600)
compass.connect()

# 设置模式并读取
compass.set_mode(OutputMode.POLLING)
data = compass.read()

print(f"横滚角: {data.roll:.2f}°")
print(f"俯仰角: {data.pitch:.2f}°")
print(f"航向角: {data.heading:.2f}°")

compass.disconnect()
```

## 使用方式

### 方式1: 上下文管理器（推荐）

```python
from ddm350b import DDM350B, OutputMode

with DDM350B('/dev/ttyS0') as compass:
    compass.set_mode(OutputMode.POLLING)
    
    data = compass.read()
    if data:
        print(f"航向: {data.heading:.1f}°")
```

### 方式2: 快捷函数

```python
from ddm350b import read_compass, close

data = read_compass('/dev/ttyS0')
if data:
    print(f"横滚={data.roll:.1f}° 俯仰={data.pitch:.1f}° 航向={data.heading:.1f}°")

close()
```

### 方式3: 连续读取

```python
from ddm350b import DDM350B, OutputMode

compass = DDM350B('/dev/ttyS0')
compass.connect()
compass.set_mode(OutputMode.AUTO_10HZ)  # 10Hz自动输出

for data in compass.read_continuous(interval=0.1):
    print(f"{data.roll:.1f} {data.pitch:.1f} {data.heading:.1f}")
```

## 输出模式

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| 问答式 | `OutputMode.POLLING` | 需主动发送读取命令 |
| 5Hz | `OutputMode.AUTO_5HZ` | 自动输出 5次/秒 |
| 15Hz | `OutputMode.AUTO_15HZ` | 自动输出 15次/秒 |
| 25Hz | `OutputMode.AUTO_25HZ` | 自动输出 25次/秒 |
| 35Hz | `OutputMode.AUTO_35HZ` | 自动输出 35次/秒 |
| 50Hz | `OutputMode.AUTO_50HZ` | 自动输出 50次/秒 |
| 100Hz | `OutputMode.AUTO_100HZ` | 自动输出 100次/秒 |

## API 参考

### DDM350B 类

#### `__init__(port, baudrate=9600, timeout=1.0)`
初始化罗盘对象

- `port`: 串口号，如 `'/dev/ttyS0'` 或 `'/dev/ttyUSB1'`
- `baudrate`: 波特率，默认 9600
- `timeout`: 超时时间（秒）

#### `connect() -> bool`
连接到罗盘，返回连接状态

#### `disconnect()`
断开连接

#### `set_mode(mode: OutputMode) -> bool`
设置输出模式

#### `read() -> CompassData | None`
读取一次角度数据，返回 `CompassData(roll, pitch, heading)` 或 `None`

#### `read_continuous(interval=0.1)`
连续读取生成器，yield `CompassData` 对象

#### `set_magnetic_declination(degrees: float) -> bool`
设置磁偏角（正数东偏，负数西偏）

#### `start_calibration() -> bool`
开始校准

#### `save_calibration() -> bool`
保存校准结果

### CompassData 数据结构

```python
from ddm350b import CompassData

data = CompassData(roll=1.5, pitch=-2.3, heading=125.6)

# 属性访问
data.roll    # -> 1.5
data.pitch   # -> -2.3
data.heading # -> 125.6

# 索引访问
data[0]  # -> 1.5
data[1]  # -> -2.3
data[2]  # -> 125.6

# 解包
roll, pitch, heading = data
```

## 示例程序

运行示例：

```bash
cd examples
python demo.py
```

示例列表：

| 示例 | 功能 | 对应函数 |
|------|------|----------|
| 示例1 | 问答模式读取 | `example_1_polling_mode()` |
| 示例2 | 自动输出模式 | `example_2_continuous_mode()` |
| 示例3 | 高速采集 | `example_3_high_speed()` |
| 示例4 | 数据保存到文件 | `example_4_stream_to_file()` |
| 示例5 | 数据结构使用 | `example_5_data_class()` |

## 数据说明

### 角度含义

| 角度 | 名称 | 说明 | 范围 |
|------|------|------|------|
| Roll | 横滚角 | 绕X轴旋转 | -180° ~ +180° |
| Pitch | 俯仰角 | 绕Y轴旋转 | -90° ~ +90° |
| Heading | 航向角 | 绕Z轴旋转（北向） | 0° ~ 360° |

### 坐标系

```
        北 (Heading=0°)
          ↑
          │
    西 ←──┼──→ 东
          │
          ↓
         南 (Heading=180°)
```

## 通讯参数

| 参数 | 默认值 |
|------|--------|
| 波特率 | 9600 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 超时 | 1秒 |

## 常见问题

### 1. 连接失败
- 检查串口号是否正确
- 确认设备已连接
- 检查驱动程序是否安装

### 2. 读取超时
- 确认波特率设置正确
- 检查数据线连接
- 问答模式下确认已调用 `read()`

### 3. 数据异常
- 可能是磁场干扰，尝试校准
- 确认罗盘已水平放置

## 文件结构

```
Three_axis_angles/
├── ddm350b.py              # 主接口文件
├── mode_command.py         # 模式命令生成
├── examples/
│   ├── demo.py             # 演示程序
│   ├── example_basic.py    # 基础示例
│   └── example_ddm350b.py  # 使用示例
└── README.md               # 本文档
```
