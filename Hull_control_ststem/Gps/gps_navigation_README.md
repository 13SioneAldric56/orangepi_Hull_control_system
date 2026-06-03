# GPS导航匹配算法

基于NMEA协议解析的经纬度数据进行导航计算，支持航点管理和自动匹配功能。

## 功能特性

- **距离计算**：使用Haversine公式计算两点间球面距离
- **方位角计算**：计算从起点到终点的航向角（0-360°，正北为0°）
- **航点管理**：支持从JSON文件加载、保存航点
- **自动匹配**：根据当前位置自动匹配最近的航点
- **多接口支持**：静态接口和实例接口

## 快速开始

### 静态接口（两点间导航）

```python
from gps_navigation import GPSNavigation

# 计算两点间的相对距离和角度
result = GPSNavigation.navigation_between(
    lat1, lon1,    # 起点经纬度（度）
    lat2, lon2     # 终点经纬度（度）
)

print(f"相对距离: {result.distance:.2f} 米")
print(f"相对角度: {result.bearing:.2f}°")
```

### 实例接口（基于基准点）

```python
from gps_navigation import GPSNavigation

# 创建导航器实例
nav = GPSNavigation()

# 设置基准点/目标点
nav.set_home(31.2304, 121.4737)

# 计算当前位置到基准点的导航
result = nav.get_relative_position(current_lat, current_lon)
```

## 航点管理

### 1. 航点文件格式

JSON格式的航点文件示例：

```json
[
    {
        "name": "码头A",
        "latitude": 31.2304,
        "longitude": 121.4737,
        "description": "起始码头"
    },
    {
        "name": "港口B",
        "latitude": 31.2400,
        "longitude": 121.4800,
        "description": "目标港口"
    }
]
```

### 2. 读取预设置的航点

```python
from gps_navigation import GPSNavigation

# 创建导航器
nav = GPSNavigation()

# 从文件加载航点
waypoints_file = "waypoints.json"
nav.load_waypoints_from_file(waypoints_file)

# 列出所有航点
for wp in nav.list_waypoints():
    print(f"{wp['name']}: ({wp['latitude']}, {wp['longitude']})")
```

### 3. 保存航点��文件

```python
# 添加航点
nav.add_waypoint("锚地C", 31.2350, 121.4650, "临时锚地")

# 保存到文件
nav.save_waypoints_to_file("my_waypoints.json")
```

### 4. 计算到所有航点的导航

```python
# 当前位置（从GPS获取）
current_lat, current_lon = 31.2280, 121.4700

# 计算到所有航点的距离和角度（自动按距离排序）
results = nav.calculate_all_waypoints(current_lat, current_lon)

for r in results:
    print(f"{r['name']}: {r['distance']:.1f}米, {r['bearing']:.1f}°({r['direction']})")
```

### 5. 匹配最近的航点

```python
# 查找最近的航点
nearest = nav.find_nearest_waypoint(current_lat, current_lon)
if nearest:
    waypoint, nav_result = nearest
    print(f"最近航点: {waypoint.name}")
    print(f"距离: {nav_result.distance:.1f}米")
    print(f"方位: {nav_result.bearing:.1f}°")

# 在指定范围内匹配航点（例如5000米内）
match = nav.match_nearest_waypoint(current_lat, current_lon, max_distance=5000)
if match:
    wp, res = match
    print(f"匹配到: {wp.name}，距离: {res.distance:.1f}米")
else:
    print("范围内无航点")
```

## 导航结果

`NavigationResult` 数据类包含以下字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `distance` | float | 相对距离（米） |
| `bearing` | float | 相对角度（度，0-360，正北为0°） |
| `distance_km` | float | 相对距离（公里） |
| `east_distance` | float | 东向位移（米，正东为正） |
| `north_distance` | float | 北向位移（米，正北为正） |

## 完整使用示例

### 示例1：船舶导航到预设置航点

```python
from gps_navigation import GPSNavigation

# 1. 创建导航器并加载航点文件
nav = GPSNavigation()
nav.load_waypoints_from_file("ship_waypoints.json")

# 2. 从GPS模块获取当前位置
# 假设从GPS解析得到
current_lat, current_lon = 31.2280, 121.4700

# 3. 计算到所有航点，按距离排序
results = nav.calculate_all_waypoints(current_lat, current_lon)

print("所有航点导航信息（按距离排序）:")
for r in results:
    print(f"  {r['name']}: {r['distance']:.0f}米, {r['bearing']:.1f}°({r['direction']})")

# 4. 获取最近航点
nearest = nav.find_nearest_waypoint(current_lat, current_lon)
if nearest:
    wp, res = nearest
    print(f"\n最近航点是 {wp.name}，距离{res.distance:.0f}米")
```

### 示例2：实时导航回调

```python
from gps import GPSReader
from gps_navigation import GPSNavigation

# 加载航点
nav = GPSNavigation()
nav.load_waypoints_from_file("waypoints.json")

def navigation_callback(position):
    """GPS数据回调函数"""
    if position.fix_quality > 0:  # 有效定位
        # 计算到所有航点
        results = nav.calculate_all_waypoints(
            position.latitude, position.longitude
        )

        if results:
            nearest = results[0]  # 最近的航点
            print(f"最近航点: {nearest['name']}")
            print(f"距离: {nearest['distance']:.1f}米")
            print(f"方位: {nearest['bearing']:.1f}°({nearest['direction']})")

# 启动GPS
gps = GPSReader(port='/dev/ttyS0', baudrate=38400)
gps.start_reading(callback=navigation_callback)
```

### 示例3：航点增删改查

```python
from gps_navigation import GPSNavigation

nav = GPSNavigation()

# 添加航点
nav.add_waypoint("起点", 31.2304, 121.4737, "起始位置")
nav.add_waypoint("终点", 31.2400, 121.4800, "目标位置")

# 查询航点
wp = nav.get_waypoint("起点")
if wp:
    print(f"起点坐标: {wp.latitude}, {wp.longitude}")

# 删除航点
nav.remove_waypoint("临时点")

# 保存航点
nav.save_waypoints_to_file("my_points.json")
```

### 示例4：范围匹配（进入区域告警）

```python
from gps_navigation import GPSNavigation

nav = GPSNavigation()
nav.load_waypoints_from_file("ports.json")

current_lat, current_lon = 31.2350, 121.4750

# 匹配5000米内的航点（例如港口、禁入区）
matched = nav.match_nearest_waypoint(current_lat, current_lon, max_distance=5000)
if matched:
    wp, res = matched
    print(f"警告：前方{res.distance:.0f}米处有{wp.name}")
    print(f"建议转向: {res.bearing:.1f}°")
else:
    print("安全区域，无匹配航点")
```

## API 参考

### GPSNavigation 类

#### 主要方法

| 方法 | 说明 |
|------|------|
| `navigation_between()` | 静态方法，计算两点间导航 |
| `get_relative_position()` | 实例方法，计算到目标点的导航 |
| `calculate_distance()` | 静态方法，计算距离和位移分量 |
| `calculate_bearing()` | 静态方法，计算方位角 |
| `get_cardinal_direction()` | 静态方法，角度转方位名称 |

#### 航点管理

| 方法 | 说明 |
|------|------|
| `load_waypoints_from_file()` | 从JSON文件加载航点 |
| `save_waypoints_to_file()` | 保存航点到JSON文件 |
| `add_waypoint()` | 添加航点 |
| `remove_waypoint()` | 删除航点 |
| `get_waypoint()` | 获取指定航点 |
| `list_waypoints()` | 列出所有航点 |
| `find_nearest_waypoint()` | 查找最近航点 |
| `match_nearest_waypoint()` | 范围内匹配最近航点 |
| `calculate_all_waypoints()` | 计算到所有航点导航信息 |

### 辅助函数

| 函数 | 说明 |
|------|------|
| `create_navigation_target()` | 创建导航目标点字典 |

## 算法说明

### 距离计算（Haversine公式）

两点间球面距离计算公式：

```
a = sin²(Δlat/2) + cos(lat1) × cos(lat2) × sin²(Δlon/2)
c = 2 × atan2(√a, √(1-a))
d = R × c
```

其中 R = 6371000 米（地球平均半径）

### 方位角计算

```
θ = atan2(sin(Δlon) × cos(lat2),
           cos(lat1) × sin(lat2) - sin(lat1) × cos(lat2) × cos(Δlon))
```

方位角定义：
- 0°/360° = 正北
- 90° = 正东
- 180° = 正南
- 270° = 正西

## 安装与依赖

- Python 3.6+
- 无需第三方库（仅使用标准库）

## 文件说明

```
gps_navigation.py          # 主模块
gps_navigation_README.md   # 文档
waypoints.json             # 航点配置文件（示例）
```

## 注意事项

1. 输入坐标单位为**十进制度**（decimal degrees）
2. 纬度范围：-90° 到 +90°（南纬负，北纬正）
3. 经度范围：-180° 到 +180°（西经负，东经正）
4. 距离精度适用于中等距离（几百公里内）
5. 方位角基于正北方向，顺时针增加
6. 航点文件必须为UTF-8编码的JSON格式

## 与GPS模块集成

本模块与 `gps.py` 中的 `GPSPosition` 数据格式完全兼容：

```python
from gps import GPSReader, GPSPosition
from gps_navigation import GPSNavigation

# 获取GPS数据
gps = GPSReader(port='/dev/ttyS0', baudrate=38400)

def on_position(position: GPSPosition):
    # 使用position.latitude和position.longitude
    result = nav.navigation_between(
        position.latitude, position.longitude,
        target_lat, target_lon
    )

gps.start_reading(callback=on_position)
```

## License

MIT License

# 农庄 坐标 
113.581055
22.336825

#公司楼下 坐标
113.539615
22.400980


纬度: 22.4048°
经度: 113.5347°

纬度: 22.4047°
经度: 113.5350°

22.404399
113.534844

22.404601
113.534753

22.404421
113.534883

22.340250
113.576989

22.340684
113.577533

远点gps定位
22.340490
113.576880
