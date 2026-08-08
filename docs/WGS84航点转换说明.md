# WGS84 航点转换说明

`scripts/convert_wgs84_waypoints.py` 在本地电脑运行，把上游
`track_output.txt` 转换为控制器可直接读取的 odom 平面航点
`watpoints_odom.csv`。脚本不依赖 ROS 2，也不会启动或控制车辆。

## 坐标假设

- LIO 启动时 odom 原点为 `(0,0)`。
- odom `+x` 指向车辆启动时车头方向，`+y` 指向车辆左侧。
- 输入第一点的 WGS84 经纬高定义 odom 原点；第一点不会被删除，输出第一行
  为 `(0,0)`。控制器会立即确认该点，再按序前往第二点。
- 初始航向定义为“从真北开始顺时针”：北 `0°`、东 `90°`、南
  `180°`、西 `270°`。

任一假设不成立时，不得直接使用转换结果。

## 输入与输出

默认输入文件是仓库根目录的 `track_output.txt`，采用分号分隔：

```text
序号;经度;纬度;高程
1;120.00000000;30.00000000;0
2;120.00010000;30.00000000;0
```

程序也接受英文表头，以及旧的 5 列格式；旧格式第 5 列会被忽略。序号必须
严格递增，经纬度使用 WGS84 十进制度，高度单位为米。高度参与三维坐标
转换，但不会写入二维控制 CSV。

默认输出 `watpoints_odom.csv`：

```csv
x_m,y_m
0.000000,0.000000
9.648628,-0.000005
```

文件名中的 `watpoints` 是当前上下游约定的拼写。上传到 RDK 时再改名为
控制 launch 使用的 `waypoints.csv`。

## 最常用命令

在仓库根目录运行，并按车辆启动时车头方向选择一个命令：

```powershell
python .\scripts\convert_wgs84_waypoints.py --heading N
python .\scripts\convert_wgs84_waypoints.py --heading E
python .\scripts\convert_wgs84_waypoints.py --heading S
python .\scripts\convert_wgs84_waypoints.py --heading W
```

已有输出时，只有人工确认要替换后才加 `--force`：

```powershell
python .\scripts\convert_wgs84_waypoints.py --heading E --force
```

任意真航向可用 `--heading-deg`：

```powershell
python .\scripts\convert_wgs84_waypoints.py --heading-deg 37.5 --force
```

`--heading` 与 `--heading-deg` 必须二选一。自定义路径示例：

```powershell
python .\scripts\convert_wgs84_waypoints.py `
  --input D:\data\track_output.txt `
  --output D:\data\watpoints_odom.csv `
  --heading E `
  --force
```

## 核对与上传

程序会打印输入 SHA-256、原点、航向、逐点局部坐标、相邻航段长度、总长和
输出路径。它不会自行删点、改点或改变顺序。请先检查：

1. 第一行是 `(0,0)`，点数和序号与输入一致；
2. odom `+x/+y` 方向、车头真航向和路线左右关系正确；
3. 相邻距离、总长和路线形状符合现场预期。

确认后才上传：

```powershell
scp .\watpoints_odom.csv sunrise@192.168.188.227:/home/sunrise/waypoints.csv
```

控制器只在启动时读取一次 CSV；替换文件后必须在车辆安全停止的前提下重启
控制 launch。脚本默认拒绝覆盖现有输出，并拒绝超过 2 MiB、单行超过 4096
字节、超过 10000 点或全部输出点重合的异常文件。
