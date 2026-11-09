# Module 07 最终协议交接与源码核对

## 核对范围与结论

本文件全文随交付包提供。已核对当前 protocol.py、client.py、simulator.py、cli.py
和通信测试，并对照用户本轮提供的 ESP32 行为逐项检查。

**结论：线协议基本一致，不能宣称整体已兼容。** 上轮“固件缺少安全能力”来自旧本机
参考副本，不应继续套用于用户报告的新版。当前 Downloads/NatHack_Robot 2/Protocol.cpp
仍是未加入安全指令的旧副本，不能代表新烧录固件。本次未获得新版固件源码或固件哈希，
因此下文的 ESP32 行为是用户提供的协议事实，尚非本次源码／实机独立验证结果。

本轮保留四个生产源码文件的原样语义，不静默修改模拟器或放宽客户端门禁。
新增独立通信台架工具及核验测试，更新本交接文件。没有连接真实机器人、发送实机运动、
改动或上传固件。Modules 01–06、相机标定、视觉、Dashboard 和现有 Module 02 工作未改。

## 完整线协议

TCP 端口 **3333**；ASCII，每条以 LF `\n` 结束。电脑单行限制120个可打印字节，
兼容接收CRLF；新版固件的长度／非法字符处理还需固件端核对。协议不是JSON。

共同CAPS：

```text
CAPS,NATHACK_V1_1,MOVE_DURATION_MS,FIXED_PWM_170,STOP,ESTOP,RESET_ESTOP,HEARTBEAT,SAFETY,TARGET,ARRIVED
```

生产客户端要求以上全部能力；允许额外无冲突能力，但拒绝旧 `MOVE_PWM_0_255`，包括
同时广告新旧两种MOVE语义。CAPS通过不代表已经验证真实保护，更不自动授权运动。
台架拒绝探针使用严格相等的CAPS检查。

| 指令 | 预期回复／事件 |
| --- | --- |
| PING | PONG |
| STATUS | STATUS,<原有清洁状态> |
| CAPS | 上述能力行 |
| SAFETY | SAFETY,ESTOP=<0或1>,WATCHDOG_MS=1000,WATCHDOG=<OK或EXPIRED> |
| HEARTBEAT | OK,HEARTBEAT |
| MOVE,<F或B或L或R>,<duration_ms> | 接受OK,MOVE；正常计时结束且电机停止后DONE,MOVE |
| STOP / ABORT | OK,STOP；取消动作，不发被取消MOVE的DONE |
| ESTOP | OK,ESTOP；锁存，重复调用不恢复任务 |
| RESET_ESTOP | OK,RESET_ESTOP；条件不满足ERR,RESET_UNSAFE |
| TARGET,<GROUND或WALL>,<BLOOD或OTHER> | 现有约定OK,TARGET／ERR,BUSY；新版应再核对，生产客户端物理模式仍禁止 |
| ARRIVED | 现有约定EVENT,CLEANING_STARTED及OK,ARRIVED，否则ERR,NO_TARGET／ERR,BUSY；新版应再核对，生产客户端物理模式仍禁止 |

MOVE时长为1～10000整数毫秒，方向只允许F/B/L/R；电脑和旧参考解析器还限制数字最多5位。
所有电机固定PWM170（8位0～255）、20kHz，电脑只发时长。
例：MOVE,F,1000；MOVE,B,500；MOVE,L,300；MOVE,R,1500。
移动或清洁忙碌时返回ERR,BUSY。锁存时拒绝MOVE应实际返回ERR,ESTOP_LATCHED。

原有清洁状态：IDLE、APPROACH、SPRAY_LIFT、SPRAYING、SPRAY_LOWER、GROUND_FORWARD、
GROUND_BACKWARD、WALL_WIPE、FINISHED。DONE,CLEANING与DONE,MOVE分开；连接欢迎消息只是遥测。
ERR,DIRECTION／MOVE_FORMAT／DURATION／DURATION_RANGE等具体参数错误名称仍需新版固件确认；
生产客户端将任意ERR回复作为拒绝，不当作接受或完成。

## 与用户提供新版固件逐项比较

| 项目 | 电脑客户端／现有模拟器 | 结论 |
| --- | --- | --- |
| 3333、ASCII、LF | 相同 | 一致 |
| CAPS、毫秒MOVE、PWM170／20kHz、1～10000 | 解析与模拟器一致 | 一致，真实保护未独立验证 |
| OK与DONE分开、忙碌拒绝、取消不补DONE | 客户端分开记录，模拟器有对应行为；客户端一次只允许一个在途MOVE，第二个先本地拒绝 | 一致或电脑端有意更严格 |
| CAPS成功不解锁 | 客户端仍DISARMED | 一致 |
| 上电ESTOP锁定 | 模拟器初始estop=False | **不一致，保留并标出** |
| 上电舵机PWM关闭 | 模拟器只有角度，没有PWM使能状态 | **未建模** |
| 仅HEARTBEAT刷新1000ms计时 | 模拟器还在PING、STATUS、CAPS、STOP、ESTOP、MOVE、TARGET等更新计时 | **不一致** |
| 连接后开始1000ms计时 | 模拟器connected()开始计时 | 一致，但心跳专属计时不一致 |
| 查询／心跳／重连不清除锁存超时 | 客户端与模拟器均不自动清除故障 | 一致 |
| 复位需当前TCP、近期心跳、无动作／挂起任务 | 模拟器用last_seen，未要求真正心跳；另有舵机180°及reset_allowed条件 | **复位条件不一致** |
| 客户端首次复位 | 生产客户端握手后约200ms才自动发第一条心跳；立即RESET可能收到ERR,RESET_UNSAFE | 错误会保留锁定；先显式HEARTBEAT或等日志确认，再手动复位 |
| 正常清洁电机每段350ms，独立600ms上限 | 模拟器用350ms独立停机定时，没有另一个600ms故障上限 | **不等价**；不能验证新版600ms保护 |
| 正常喷洒1200ms，独立1700ms上限 | 模拟器只有1200ms清洁状态推进，没有独立1700ms泵保护 | **不一致** |
| 故障停电机、关泵、取消清洁 | 模拟器有对应停止行为 | 基本一致 |
| 故障停止舵机PWM、不摆动归位 | 模拟器abort()将角度设180°，未建模PWM关闭 | **不一致** |
| 正常归位180° | 模拟器正常任务结束归位180° | 角度一致 |
| Wi-Fi断开 | 模拟器仅建模TCP断开，没有独立Wi-Fi事件 | **未独立建模** |
| 软件保护不是硬件急停 | 客户端不宣称硬件急停保障 | 一致 |

普通STOP的舵机PWM策略、独立上限触发后的具体事件／错误字符串，本轮描述未完整给出，
不能自行推定。旧模拟器不是新版固件的精确替身；通过旧模拟器测试不能消除表中差异。

## 授权、完成和故障结果

物理授权必须同时满足完整CAPS、有效SAFETY、无锁存故障、authorized=True，及两项独立限制：

- validated_max_pwm：整数170～255；低于170拒绝。它是操作员认可的功率上限，不设置输出。
- validated_max_duration_ms：独立整数1～9999，必须由操作员实测认可。只有旧PWM参数不能授权，
  也绝不把PWM值解释成时间。超时长在发送前拒绝。

STOP、故障、断线或成功复位后仍需重新授权。RESET失败保持锁定。
TARGET／ARRIVED物理模式限制继续保留。

```python
accepted = client.send_command('MOVE,R,1500')  # OK仅表示接受
outcome = client.wait_for_move(accepted)       # 不占请求锁，心跳继续
latest = client.get_move_result(accepted)      # 返回更新后的不可变快照
```

结果明确区分accepted、completed、aborted、unknown、rejected。
只有有效DONE,MOVE才标completed。STOP/ESTOP ACK或已识别故障标aborted；通信失效或缺少
完成依据标unknown，不推断完成，不重发MOVE。完成等待默认上限为时长加0.8s回复余量。
协议没有动作编号，停止后使用PING回复屏障排空旧回复，不自动恢复旧任务。

当前立即识别的故障依据包括EVENT,WATCHDOG_STOP、EVENT,LINK_LOST_ABORTED、
ERR,ESTOP_LATCHED、ERR,WATCHDOG_LATCHED及SAFETY中的锁存／超时。
新版600/1700ms故障若用其他消息名，当前客户端只记录遥测；没有DONE会在截止后标unknown。
请固件端交付实际故障消息清单，才能确认所有故障都能立即结束等待。不能以现有测试宣称覆盖未知消息。

右转1500ms约90°只可作round(angle_deg*1500/90)初估，不套用于左转或直行。
DONE不代表到达目标或完全停稳；等待实际验证的稳定时间后再读取摄像头。未新增导航。

## 通信专用入口：保持TCP并暂停／恢复心跳

新增tools/module07_bench.py，独立于生产RobotClient。物理模式要求显式确认执行器电源隔离。
只自动发200ms心跳，不自动RESET、授权、MOVE或重连；不提供普通MOVE/TARGET/ARRIVED入口。
工具是台架诊断程序，不是安全控制器或硬件急停。

在操作员确认电机、泵、舵机执行器电源均已隔离后，手动运行（本次没有执行）：

```bash
.venv/bin/python -m tools.module07_bench --mode physical --host ESP32_IP \
  --actuators-isolated --log-file NEW_BENCH_LOG.log
```

同一个TCP会话中：

1. 输入caps、safety，确认CAPS和初始ESTOP=1；日志应有OK,HEARTBEAT。
2. 输入pause，停止后续HEARTBEAT但保持TCP。等待至少1.2s；期间可输入ping、status。
   输入safety应看到WATCHDOG=EXPIRED、ESTOP=1。以最后一条心跳日志计时，不能以按键时刻当精确测量。
3. 输入resume，只恢复HEARTBEAT；再输入safety，故障仍应锁存。reset仅在操作员明确输入时发送；
   成功也不启动任务，工具没有授权运动功能。quit关闭连接，绝不自动重连。

工具逐条记录真实TX/RX。pause不会由其他保活替代。若现有模拟器在ping下不超时，这正是已标出的差异，
不能据此判新版固件失败。台架查询是同步的，回复等待期间不会并发插入心跳；一般ACK应很快，慢／丢回复
可能真实触发看门狗。台架日志用于核对实际时序，不是200ms实时调度保证。

## 区分本地拦截与固件实际拒绝

- 生产RobotClient在ESTOP后调用send_command('MOVE,F,1')：本地PermissionError，线上没有MOVE。
  这只能证明电脑门禁，不能证明固件拒绝。
- 隔离电源的台架工具中，明确输入probe-estop：先检查完整CAPS，取得OK,ESTOP并查询ESTOP=1，
  然后只向线上发送一次MOVE,F,1。必须看到[TX] MOVE,F,1和[RX] ERR,ESTOP_LATCHED才判通过。
  不进行RESET，不解除锁存，不重复MOVE。无隔离确认／CAPS错误／急停未确认则不发送该探针。
  如果收到OK,MOVE或其他意外回复，立即判失败并尽力发送ESTOP、STOP后退出；若响应超时也不重试。
  即使电源隔离，这仍是有效MOVE字符串，因此必须由操作员明确发起，绝不能在带电执行器上使用。

完成通信、超时和拒绝检查后结束第一阶段；本交付不安排带电运动或固件上传。

## 文件与本机验证

交付包含robot_comm/protocol.py、client.py、simulator.py、cli.py、相关包文件、
tests/test_robot_communication.py、tools/module07_bench.py和本文全文，附SHA-256清单。
依赖现有项目环境；不是独立固件，也不包含凭据、相机资料或整项目备份。

```bash
.venv/bin/python -m pytest -q tests/test_robot_communication.py
.venv/bin/python -m pytest -q
```

785项完整回归／91项通信测试是上一轮基线，不直接作为本轮结论。
本轮完整回归794项通过（1个既有警告，42.68s），通信专项100项通过（11.24s）。
详细结果记录在随包TEST_REPORT.md中；测试仅使用本机回环端点、假时钟及既有旧固件C++桩。
新增ReportedSafetyPeer是对用户描述的测试替身，不是新版ESP32源码；新版真实硬件兼容性仍待隔离台架核验。
