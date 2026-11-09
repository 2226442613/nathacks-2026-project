# NatHack Robot：电脑端对接指导与任务流程

版本：NATHACK_V2_SIMPLE · 2026-11-09

本文根据当前 `/Users/meilinz/Documents/Arduino/NatHack_Robot` 源码整理。它描述目前的简化版固件，取代此前要求 HEARTBEAT、ESTOP、RESET_ESTOP、SAFETY、授权门禁的对接文档。本文不代表全部硬件流程已经实测通过。

## 1. 给电脑端 Codex 的工作要求

请按本文对接电脑控制程序和 Dashboard，保留现有摄像头标定、视觉识别及定位模块。电脑负责目标识别、坐标转换、路径规划、移动时长计算、到达判断、返航和归位判断；ESP32只执行指令和固定清洁动作。

当前版本连接后直接允许下发动作，不要求心跳、急停解锁或运动授权。不要继续使用旧 Module 07 的 CAPS 安全能力检查阻止本版本，也不要在后台发送不支持的安全指令。修改用户操作流程时应明确显示这是 SIMPLE 固件，不能继续声称具有旧版锁存急停或通信心跳看门狗。

保持 ASCII 文本协议和 TCP 3333，不改成 JSON。MOVE 第三个字段始终为毫秒，绝不能解释成 PWM。不要自动连接实物或执行运动测试。

## 2. 连接与启动

- ESP32为TCP服务端，电脑为客户端，两者连接同一热点。
- 端口固定3333；IP以ESP32本次连接所得地址为准，历史地址172.20.10.12不能写死为永久地址。
- 每条指令以LF（`\n`）结束。建议大写，无多余空格，单条不超过120字节。
- TCP是字节流：一次读取可能包含多行或半行，必须缓存并按换行切分。
- 一次只使用一条控制连接；不要让多个工具同时争用。
- 连接欢迎消息为 `CONNECTED,NATHACK_V2`，随后为 `STATUS,IDLE` 或当前状态。欢迎消息仍叫V2，不能仅凭它识别SIMPLE。
- 启动串口标识为 `NatHack Robot V2 SIMPLE`；两个舵机上电设置为180°。
- 使用CAPS识别当前版本，完整回复为：

```text
CAPS,NATHACK_V2_SIMPLE,MOVE_DURATION_MS,FIXED_PWM_170,STOP,TARGET,ARRIVED
```

不支持 HEARTBEAT、SAFETY、ESTOP、RESET_ESTOP，会返回 `ERR,UNKNOWN_COMMAND`。

## 3. 指令表

| 电脑发送 | ESP32回复 | 实际作用 |
|---|---|---|
| `PING` | `PONG` | 连通性查询，无动作 |
| `CAPS` | 上述完整能力行 | 查询协议，不自动执行任务 |
| `STATUS` | `STATUS,<状态>` | 查询清洁状态，不含坐标或运动剩余时间 |
| `MOVE,F,1000` | `OK,MOVE`，正常结束后`DONE,MOVE` | 前进1000毫秒 |
| `MOVE,B,500` | 同上 | 后退500毫秒 |
| `MOVE,L,300` | 同上 | 原地左转300毫秒 |
| `MOVE,R,1500` | 同上 | 原地右转1500毫秒 |
| `TARGET,GROUND,BLOOD` | `OK,TARGET` | 记录地面目标，选择泵A |
| `TARGET,GROUND,OTHER` | `OK,TARGET` | 记录地面目标，选择泵B |
| `TARGET,WALL,BLOOD` | `OK,TARGET` | 记录墙面目标，选择泵A |
| `TARGET,WALL,OTHER` | `OK,TARGET` | 记录墙面目标，选择泵B |
| `ARRIVED` | 先`EVENT,CLEANING_STARTED`，再`OK,ARRIVED` | 停止导航运动，启动已记录目标的清洁动作 |
| `STOP`或`ABORT` | `OK,STOP` | 停电机、关泵、取消目标与清洁，两个舵机归位180° |

TARGET不携带坐标，也不会自行导航；目标坐标保存在电脑端。BLOOD/OTHER是清洁液选择，不代表固件能识别污渍。泵A/B分别由GPIO17/18控制，不能同时运行。

## 4. MOVE的准确语义

- 所有运动固定PWM170，20kHz，8位；包括地面自动擦拭。
- `MOVE,<方向>,<duration_ms>`，方向只允许F/B/L/R；时间为1～10000的整数，文本最多5位。
- 不传速度、不传距离、不传角度。每个MOVE只执行一次，不需要持续重发。
- 当前MOVE未结束，或正在自动清洁时，新MOVE返回`ERR,BUSY`，不会排队或延长时间。
- IDLE和APPROACH均可接受MOVE。
- `OK,MOVE`只表示接受；`DONE,MOVE`表示该次定时动作停止，不能理解为到达目标或车身已完全静止。
- STOP/ABORT、断线或故障中止不补发`DONE,MOVE`。电脑必须取消等待，区分完成、中止和结果未知。
- 协议没有动作ID。保持单个在途动作，不能并发发送多个MOVE或对不明确的执行结果自动重试。
- `STATUS,IDLE`只表示没有清洁任务，不能据此推断电机没在执行MOVE。

实测参考：PWM170下右转1500毫秒约90°。可初估：

```text
right_turn_ms = round(right_angle_deg * 1500 / 90)
```

右转数据不能直接套用左转或直行。电脑应在动作完成、车身停稳后重新定位并修正。不要声称固定PWM等于固定实际转速。

## 5. 完整任务状态流程

```text
连接电脑 → 初始位置待命 → 电脑选择一个目标
→ TARGET记录类型与清洁液
→ 电脑通过若干MOVE导航到目标
→ 摄像头确认到达并停止
→ ARRIVED
→ ESP32自动清洁
→ DONE,CLEANING
→ 电脑重新定位小车
→ 电脑通过若干MOVE返回初始位置并调整朝向/贴墙状态
→ 电脑确认已归位 → 等待下一个目标
```

要求电脑记录初始位姿 `(x0,y0,theta0)`，用当前位姿 `(x,y,theta)`闭环导航。地面与墙面用各自的平面标定，不能直接混用像素到世界坐标转换。顶部标记高度对定位的影响应由现有定位模块处理。

地面任务返航后要恢复初始位置、车头方向及侧面贴墙状态；墙面目标沿墙导航也由电脑负责。初始位置容差、朝向容差、停稳等待时间和靠墙判断需要实际标定，固件没有这些阈值。

固件没有HOME、RETURN、已归位或自动返航指令。电脑收到DONE,CLEANING后必须自行规划返回。归位确认是电脑端状态，建议在Dashboard显示“清洁完成→返回中→已归位/待命”，不能把DONE,CLEANING直接显示成整个任务已归位。

## 6. 固件当前自动清洁动作

### 地面

1. ARRIVED停止导航电机。
2. 对应泵喷洒1200毫秒，然后关闭。
3. 前进350毫秒、后退350毫秒，重复3组，固定PWM170。
4. 停止电机和泵，两个舵机命令归位180°，清除目标。
5. 发送`DONE,CLEANING`，回到IDLE。

理论动作计时约3.3秒，不包括机械停稳及舵机归位时间。不要用这个估算代替真实完成事件。

### 墙面

1. ARRIVED停止导航电机。
2. GPIO9喷管舵机设置90°，等待700毫秒。
3. 对应泵喷洒1200毫秒，然后关闭。
4. 喷管舵机回180°，等待700毫秒。
5. GPIO10海绵舵机按180°→0°→180°往复3组；每15毫秒步进2°。
6. 电机/泵停止，舵机命令归位180°，清除目标，发送`DONE,CLEANING`。

理论计时约10.7秒，主循环和机械响应会有额外时间。

**墙面清洁期间车轮不动。当前实现是停在目标处用舵机擦，不是沿墙边走边擦。** 如需要边走边擦，属于后续固件功能变更，电脑不能靠发送MOVE实现，因为清洁阶段会拒绝MOVE。

`DONE,CLEANING`只证明预设动作结束，不证明污渍消失，也不证明舵机实际到位。电脑如需清洁效果验证，应另外读取画面；不要未确认便自动重复喷洒。

## 7. 状态、拒绝与故障

清洁状态：IDLE、APPROACH、SPRAY_LIFT、SPRAYING、SPRAY_LOWER、GROUND_FORWARD、GROUND_BACKWARD、WALL_WIPE、FINISHED。FINISHED立即转IDLE，通常无法靠轮询观察到。

| 回复 | 意义 |
|---|---|
| `ERR,BUSY` | 新MOVE遇到运动/清洁忙碌；或非IDLE时试图设置TARGET |
| `ERR,NO_TARGET` | ARRIVED时没有处于APPROACH的目标；也可能是重复ARRIVED |
| `ERR,MOVE_FORMAT` | MOVE字段格式错误 |
| `ERR,DIRECTION` | 方向不是F/B/L/R |
| `ERR,DURATION` | 时长为空、非数字或数字文本过长 |
| `ERR,DURATION_RANGE` | 时长不在1～10000毫秒 |
| `ERR,TARGET_FORMAT` | TARGET字段格式错误 |
| `ERR,UNKNOWN_TARGET` | 不支持的表面/清洁液 |
| `ERR,UNKNOWN_COMMAND` | 不支持的指令，包括旧安全协议指令 |
| `EVENT,CLEAN_MOTOR_TIMEOUT` | 内部清洁电机连续运行达到约600毫秒上限，取消任务 |
| `EVENT,PUMP_TIMEOUT` | 水泵连续运行达到约1700毫秒上限，取消任务 |
| `EVENT,LINK_LOST_ABORTED` | 检测到TCP断线，取消任务；通常只能在串口看到 |

独立清洁时长上限仍存在，但不锁存、不要求复位。普通MOVE按自身时长停止，不受600毫秒清洁上限限制。停止/超时中止清除目标，不能继续发ARRIVED恢复；新任务需重新TARGET。

没有心跳超时保护。若连接假死但TCP尚未检测到，MOVE可能运行到指定时长结束，清洁也可能继续推进。STOP不是锁存急停，不是硬件电源切断。固件停止动作含舵机回180°，因此STOP不等于所有执行器完全静止。

## 8. 电脑程序建议结构

1. 后台接收线程持续解析所有行，将ACK、完成事件、故障及状态分别处理。
2. 单个普通请求在途，保持单个MOVE在途；STOP可打断等待。
3. ACK超时不等于未执行，DONE丢失也不等于未执行。标记未知并重新定位，不重放动作。
4. 普通移动完成后等实测停稳时间，再读最新摄像头画面，避免使用执行前的旧帧。
5. 成功TARGET后才能ARRIVED。重复TARGET不会替换挂起目标；需要取消时先STOP。
6. ARRIVED事件可先于ACK到达，不能把EVENT误当指令回复。
7. DONE,CLEANING到来后转换为返航状态；返航期间不要设置下一个清洁目标。
8. 断线取消本地任务，重连不重放旧队列；先查询并重新定位，再由操作员/既定任务逻辑明确安排新动作。
9. 更新Dashboard旧安全门禁与提示，不后台发送旧版安全指令，不用旧SAFETY解析失败阻断SIMPLE。
10. 本文不授权自动修改视觉标定、自动执行硬件运动或自动烧录。

## 9. 参考交互

```text
PC > PING
ESP > PONG
PC > TARGET,GROUND,OTHER
ESP > OK,TARGET
PC > MOVE,F,100
ESP > OK,MOVE
ESP > DONE,MOVE
# 电脑重新定位，按需继续MOVE；下面仅在确认到达后发送
PC > ARRIVED
ESP > EVENT,CLEANING_STARTED
ESP > OK,ARRIVED
ESP > DONE,CLEANING
# 电脑重新定位并计算真实返程，此处不提供假定返程距离
PC > MOVE,B,100
ESP > OK,MOVE
ESP > DONE,MOVE
# 电脑确认位置、方向和贴墙状态，确认归位后才开始下个目标
```

## 10. 调试入口与验收

简易工具：`python3 pc_simple.py ESP32_IP`，直接输入完整指令并回车；quit结束时尽力STOP再断开。该脚本只用于手动通信，没有摄像头导航或自动返航。

先验收PING/CAPS，再架空车轮试MOVE,F,100及STOP；分别核对各方向、自动停止和断线停止。清洁测试涉及水泵及舵机，需另行确认机械行程和供电；模拟或编译通过不能视为硬件动作通过。

电脑端完成修改后，请交付修改文件、协议测试结果，以及一次完整“目标→导航→清洁→返航→归位待命”的状态日志，区分模拟结果和真实硬件结果。
