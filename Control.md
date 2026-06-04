# SCARA_F103 控制与串口调试说明

本文档用于课程设计现场调试：构建、烧录、串口协议、错误码、回零、上位机测试 UI。

## 1. 工程路径

```text
C:\Users\22602\Desktop\SCARA\SCARA_F103
```

## 2. 构建

```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
cmake --build --preset Debug
```

自检：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1
```

当前验证重点：

```text
固件版本：0.23.2
串口波特率：115200 8N1
通信看门狗：默认关闭
正式轨迹限位：上位机负责
```

## 3. 烧录

使用 VS Code 任务或现有 CMSIS-DAP/OpenOCD 配置烧录 `build\Debug\SCARA_F103.hex`。

烧录后串口参数：

```text
115200 baud
8 data bits
No parity
1 stop bit
Newline: \n 或 \r\n
```

## 4. 基础串口命令

```text
VERSION          查询固件版本。
HOSTCAP          查询上下位机职责边界。
PING             链路测试，返回 OK PONG。
STATUS           长状态查询。
?                立即请求一帧 <...> 状态。
ERRORS           展开底层错误 bit。
HOME_SENSOR      查看 HOME1/HOME2 输入。
ENABLE 1         使能双轴输出。
ENABLE 0         关闭双轴输出。
STOP             受控停止。
ESTOP            急停。
CLEAR_ERROR      清除错误位。
ZERO             软件清零。
WATCHDOG OFF     关闭通信看门狗。
WATCHDOG ON 3000 开启 3000 ms 通信看门狗。
```

推荐手动调试顺序：

```text
VERSION
HOSTCAP
WATCHDOG OFF
CLEAR_ERROR
ENABLE 1
G21
G90
G0 X-35.000 Y145.000 F600 ;ID=SEED LIM=1
G1 X-34.900 Y145.000 F800 ;ID=0001 LIM=1
?
```

使用 `SCARA_UI` 点动时，上位机会在第一条点动 G-code 前自动发送：

```text
CLEAR_ERROR
ENABLE 1
```

这样可以避免刚烧录或刚急停后电机未使能，导致下位机在启动运动块时返回 `error:15`。`OK ENABLE 1`、`OK CLEAR_ERROR`、`OK ZERO` 只表示系统命令执行完成，不属于点动 G-code 的 `ok seq/cs/line` 回显，上位机不会用这些系统 OK 推进点动队列。

## 5. G-code 通信协议

上位机逐行发送 G-code。下位机成功接收、解析并入队后，返回完整回显：

```text
ok seq=<n> cs=<hex> line=<原始接收行>
```

示例：

```text
TX: G1 X-34.900 Y145.000 F800 ;ID=0001 LIM=1
RX: ok seq=12 cs=5A line=G1 X-34.900 Y145.000 F800 ;ID=0001 LIM=1
```

上位机必须检查：

```text
seq   ACK 序号递增。
cs    与发送行 ASCII 累加和低 8 位一致。
line  与发送行完全一致。
```

上位机必须等到 `ok seq/cs/line` 后才能发送下一条正式轨迹指令。

## 6. 支持的 G-code 子集

```text
G0 X Y F      快速定位。
G1 X Y F      直线插补点。
G20           英寸单位，不推荐使用。
G21           毫米单位，推荐。
G90           绝对坐标。
G91           相对坐标。
G4 P          暂停，占位支持。
M0/M2/M30     停止/程序结束，占位支持。
$X            清除报警。
$H            启动回零。
$G            查询 G-code 模态。
?             立即状态查询。
!             暂停。
~             恢复。
```

正式轨迹推荐格式：

```text
G1 X120.050 Y80.010 F1200 ;ID=0123 LIM=1
```

字符含义：

```text
G      G-code 指令前缀。
0/1    G0 快速定位，G1 直线插补点。
X      末端 X 坐标，单位 mm。
Y      末端 Y 坐标，单位 mm。
F      进给速度，单位 mm/min，由上位机规划。
;      注释开始，MCU 不参与运动解析，但会完整回显。
ID     上位机轨迹点编号，便于日志匹配。
LIM    上位机已完成限位检查标记，建议 `LIM=1`。
\n     一行命令结束。
```

## 7. 状态回传

自动状态帧约 5 Hz 推送，也可以发送 `?` 立即查询：

```text
<Idle|M:x,y|P:p1,p2|Bf:planner_free,rx_free|Q:planner_used|E:n|H:h1,h2|HS:home_state|A1:en,run,cur_pps,tgt_pps|A2:en,run,cur_pps,tgt_pps>
```

字段含义：

```text
Idle/Run 当前是否空闲或有运动/队列。
M        MCU 根据软件脉冲正解估算的末端 XY。
P        双电机软件脉冲计数。
Bf       规划缓冲剩余、RX 行队列剩余。
Q        规划缓冲已用段数。
E        步进底层错误 bit。
H        HOME1/HOME2 输入，1 表示触发。
HS       回零状态机阶段。
A1/A2    轴状态：使能、运行、当前 pps、目标 pps。
```

状态帧不是某条 G-code 的 ACK，可能穿插在 `ok` 前后。上位机需要按行区分：

```text
ok ...       指令应答。
error:<n>    指令错误。
<...>        状态推送。
```

## 8. 错误码速查

固件有两类错误：

```text
error:<code>   G-code 流协议错误，表示当前这一行没有被正常接受。
E:<bits>       状态帧里的步进底层错误位，可以叠加。
```

`error:<code>`：

```text
error:2    数字字段解析失败，例如 Xabc、G 后面没有数字。
error:3    不支持的 $ 命令，当前只支持 $X、$H、$G。
error:4    F 速度字段非法，例如 F0、F-100 或 F 后面不是数字。
error:5    $H 回零启动失败，通常是正在运动、回零未空闲或错误未清除。
error:8    上位机发送太快，已有一条 pending 行，必须等 ok 后再发下一条。
error:15   运动目标被拒绝，常见原因是几何逆解失败、电机未使能、急停或当前有错误位。
error:20   不支持的 G/M/字段。
error:25   同一行重复出现 X 或 Y。
```

`E:<bits>`：

```text
E:0    无底层错误。
E:1    软限位错误。v0.23.1 正式轨迹默认不再由下位机做软限位。
E:2    急停错误。
E:4    通信看门狗超时。
E:8    未使能时尝试运动。
```

组合值：

```text
E:3    1 + 2，软限位 + 急停。
E:5    1 + 4，软限位 + 通信看门狗超时。
E:12   4 + 8，通信看门狗超时 + 未使能运动。
```

特别注意：

```text
error:4  是 G-code 的 F 字段错误。
E:4      是通信看门狗超时。
error:5  是 $H 回零启动失败。
E:5      是底层错误位组合。
```

## 9. 限位职责

从 `v0.23.1` 开始，正式轨迹限位由上位机负责：

```text
APP_HOST_OWNS_LIMIT_CHECKS = 1
HOSTCAP ... host_limit=1 mcu_soft_limit=0
```

上位机发送轨迹前必须遍历所有点，完成：

```text
XY 工作空间检查。
五连杆逆解是否存在。
正解回代误差检查。
关节角范围检查。
电机脉冲范围检查。
轨迹段是否穿越禁区。
速度、加速度、拐角速度是否保守。
限位开关状态是否允许继续运动。
```

下位机只保留：

```text
G-code 语法检查。
五连杆几何逆解是否存在。
电机是否使能。
STOP/ESTOP。
回零输入和状态回传。
```

## 10. `$H` 第二次 `error:5`

`$H` 返回 `ok` 只表示“回零流程已经启动”，不表示已经完成。

如果第一次 `$H` 后限位开关没有触发，状态可能停留在：

```text
HS:Axis1Search
HS:Axis1Backoff
HS:Axis2Search
HS:Axis2Backoff
```

此时第二次发送 `$H` 会返回：

```text
error:5
```

这是正常现象，表示回零流程或电机运动仍未结束。正确流程：

```text
1. 发送 $H。
2. 持续发送 ? 或观察自动状态。
3. 等待 HS:Done。
4. 如果卡住，发送 HOME_SENSOR 检查限位输入。
5. 需要中断时发送 STOP 或 ESTOP，再 CLEAR_ERROR。
```

`v0.23.1` 已允许在 `HS:Done` 或 `HS:Error` 状态下重新 `$H`；如果仍在搜索/回退过程中，第二次 `$H` 仍会返回 `error:5`。

## 11. 高频轨迹测试

```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
powershell -NoProfile -ExecutionPolicy Bypass -File tools\host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion
```

逐条显示 TX/RX/MATCH。只看进度可加：

```powershell
-QuietLines
```

## 12. PyQt 上位机仿真 UI

路径：

```text
C:\Users\22602\Desktop\SCARA\SCARA_F103\tools\robot_upper_sim
```

运行：

```powershell
conda activate robot
cd C:\Users\22602\Desktop\SCARA\SCARA_F103\tools\robot_upper_sim
python upper_sim.py
```

缺少依赖：

```powershell
pip install PyQt5 pyserial
```

UI 功能：

```text
生成直线 + 圆弧 3000 点轨迹。
上位机侧做五连杆正逆解和速度规划。
实时显示 TX/RX。
绘制末端轨迹、速度曲线、脉冲曲线、五连杆姿态和状态字段。
支持手动发送串口命令。
```
