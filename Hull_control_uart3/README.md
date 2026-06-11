# Hull control (uart3 精简版)

从原项目剥离出的 `lan_control_hull.py` 运行所需的最小文件集合。
默认电机驱动为 `uart3`（`lan_control_config.yaml` 中 `motor_driver: "uart3"`，
对应 `control_car/uart_dual_drive.py` 的 0x06 串口帧协议）。

## 运行

```bash
pip install -r requirements.txt
python lan_control_hull.py                 # 默认配置 lan_control_config.yaml (cli_gps + uart3)
python lan_control_hull.py --legacy        # 备案配置 lan_control_config_legacy.yaml
python lan_control_hull.py --config xxx.yaml
```

## 目录结构（按功能分子文件夹）

```
Hull_control_uart3/
├── lan_control_hull.py            # 入口：船体端局域网 TCP 控制服务
├── heading_lock_control.py        # 航向锁定 + GPS 导航集成核心
├── lan_control_config.yaml        # 默认配置（cli_gps，uart3 驱动）
├── lan_control_config_legacy.yaml # 备案配置（--legacy）
├── requirements.txt
│
├── compass/                       # 电子罗盘：读取、协议解析、卡尔曼滤波、航向解算
│   ├── __init__.py
│   ├── config.py
│   ├── device.py
│   ├── filter.py
│   ├── heading.py
│   ├── protocol.py
│   ├── transport.py
│   └── wrap.py
│
├── Gps/                           # GPS：定位读取 + 导航控制 + 往返任务
│   ├── gps.py
│   ├── gps_navigation_controller.py
│   └── gps_roundtrip_mission.py
│
└── control_car/                   # 电机驱动（uart3 / esc / hbridge 三种均被导入）
    ├── uart_dual_drive.py         # uart3 驱动（默认使用）
    ├── uart_motor_protocol.py     # uart3 0x06 帧协议
    ├── esc_dual_drive.py          # ESC 驱动
    ├── esc_motor_control.py
    ├── dual_motor_control.py      # H 桥驱动
    └── pwm_sysfs.py               # PWM sysfs 底层
```

## 说明

- `compass`、`Gps`、`control_car` 沿用原包名，`heading_lock_control.py` 与入口同级，
  以保证 `from compass ...` / `from Gps.gps ...` / `from control_car... ` /
  `from heading_lock_control ...` 这些导入路径不变，无需改动任何源码。
- `heading_lock_control.py` 在导入时会同时加载 uart3 / esc / hbridge 三种驱动模块，
  因此三者全部保留；运行时由配置 `motor_driver` 选择实际驱动。
- 已剥离原项目中与本入口无关的文件（如 `lan_control_server.py`、`gps_navigation.py`、
  `Three_axis_angles/`、`compass_calibration.py`、`orange_pi_motor_driver.py` 等）。
```
