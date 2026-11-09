# Hospital Guardian

基于固定摄像头、视觉定位与 ESP32 的医院环境清洁机器人 Hackathon MVP。

系统在电脑端识别新增污渍、转换场地坐标、定位机器人并规划接近位置，通过 Wi-Fi 向 ESP32 发送有时长上限的移动指令。摄像头持续反馈实际位姿；机器人到达后，由 ESP32 执行完整清洁动作，电脑等待完成事件，再从新观测位姿继续下一目标。返航保留为可选模式。Dashboard 提供实时标注画面、任务队列和进度展示。

**当前范围：Modules 01–07、Dashboard、离线模拟与人工监督的实机流程。** 红色／非红色分类只用于演示优先级和清洁液类别，不代表医学物质鉴定；本项目尚未验证临床清洁效果。

> 文档依据 2026-11-09 的实际源码与本机配置整理。模拟通过、局部实机验证和整机端到端验证分别记录，不互相替代。

## 1. 系统分工

```text
固定摄像头（iPhone Continuity Camera / USB 相机）
  → Module 02：新增污渍检测、分类、连续帧确认
  → Module 03：污渍像素 → 地面／墙面坐标
  + Module 04：ArUco → 机器人位置与朝向
  → Module 05：停车／工具几何与可达性
  → Module 06：闭环导航、到达核验、等待清洁、返航
  → Module 07：TCP 3333、ASCII 指令与异步回复
  → ESP32：执行 MOVE 和完整清洁预设

Module 01 提供场地标定；Dashboard 只展示画面、任务与进度。
```

| 模块 | 负责内容 | 主要源码 |
| --- | --- | --- |
| 01 · Camera Calibration | 相机读取、地面／墙面独立标定、坐标转换 | `vision/camera.py`、`vision/calibration.py`、`main.py` |
| 02 · Spill Detection | 对比已登记合法背景，检测新增区域，RED／NON_RED 分类，候选与确认状态 | `vision/spill_detection.py`、`vision/spill_tracking.py` |
| 03 · Surface Localization | 判断 FLOOR／WALL／BOUNDARY，输出坐标、轮廓、尺寸和质量信息 | `vision/localization.py`、`vision/target_geometry.py` |
| 04 · Robot Localization | 单个 ArUco 标记、机器人中心和朝向、固定高度实测映射、位姿新鲜度 | `vision/robot_tracking.py`、`vision/robot_plane.py` |
| 05 · Cleaning Geometry | 独立地面／墙面工具几何、接近位置、接触覆盖与可达性规划 | `vision/cleaning_planner.py`、`navigation/adapters.py` |
| 06 · Navigation | 短实用路线、相机反馈、MOVE／STOP、修正、清洁交接；可选原有 Y→X／X→Y 返航模式 | `navigation/controller.py`、`navigation/automatic.py` |
| 07 · Communication | CAPS 协商、ASCII 分帧、真实执行结果、日志、软件模拟器 | `robot_comm/` |
| Dashboard | DEMO、LIVE 标注视频、任务队列、历史和进度 | `dashboard/` |

电脑不逐步控制喷泵、海绵、舵机或清洁时的前后擦拭。目标坐标留在电脑端；ESP32 接收表面／清洁液类别、执行动作并报告完成。

## 2. 当前实现与验证状态

- 已实现自动识别、红色优先、坐标定位、停车规划、闭环导航、ESP32 清洁交接及返航的模拟流程。
- Module 04 已有操作员确认的实机定位 PASS：4 个训练点、3 个独立验证点，RMS **1.045 cm**，最大误差 **1.253 cm**。保存映射中的误差与此一致；有效范围限于对应固定相机、标记高度和场地。
- 实机已有 TCP、PING／CAPS 和 MOVE 执行记录。**尚不能据此宣称自动识别→清洁→下一目标的实机完整流程通过。** 历史运行出现过 `NO_REACHABLE_TARGET` 和 `NO_CONFIRMED_SPILL`；可复现的软件问题及新验证结果见 [自主导航 MVP 报告](AUTONOMOUS_MVP_REPORT.md)。原始证据缺失的位姿不会补造。
- Dashboard LIVE 直接复用 `test-vision` 的 OpenCV 标注帧，经过 MJPEG 显示；没有另一套 JavaScript 视觉标注算法。
- 物理工具配置仍有缺项。地面 MVP 可交给 ESP32 执行固定清洁预设，但只核验停车与车体／工具碰撞范围，**不证明实际接触或喷洒覆盖**。墙面精确接触几何缺失时仍会拒绝相应清洁计划。

## 3. 环境与安装

主要开发环境为 macOS / Apple Silicon，摄像头由 macOS 作为本地设备提供。建议使用 Python 3.12；原项目也支持 Python 3.11。无需 Raspberry Pi、额外 iPhone 视频服务器、CUDA 或深度学习框架。

在项目根目录运行：

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dashboard.txt
python main.py --help
```

`requirements-dashboard.txt` 同时安装基础依赖和 Flask；只使用视觉／导航 CLI 时可改装 `requirements.txt`。依赖包括 OpenCV contrib、NumPy、pytest，Dashboard 额外使用 Flask。

只安装 `opencv-contrib-python`，不要同时安装另一个提供 `cv2` 的 OpenCV 包；`test-vision` 等原生窗口需要非 headless 版本。macOS 首次使用时，需要允许启动 Python 的应用访问摄像头。

下文命令均在仓库根目录、已激活 `.venv` 的终端执行。尚未激活时，可将 `python` 替换为 `.venv/bin/python`。

## 4. 快速开始：先展示，再决定是否运动

### 4.1 Dashboard DEMO：无需相机或 ESP32

使用仓库自带的合成场地标定，适合新检出项目的电脑：

```bash
python main.py dashboard --mode demo \
  --calibration data/module03_demo/calibration.json --port 5050
```

打开 <http://127.0.0.1:5050>，使用 Play／Step 查看模拟检测、优先队列和任务历史。DEMO 不打开相机、不连接机器人。当前 Dashboard 启动仍需读取标定文件，因此新检出项目时应使用上面的合成文件；已有本机场地标定时也可使用默认配置。

### 4.2 OpenCV 双检测：只看相机，不运动

前提：本机已有对应场地的标定、背景和机器人映射文件。

```bash
python main.py test-vision --camera 0
```

一个相机采集流程将每帧交给原有污渍检测和 ArUco 跟踪，显示：

- 机器人标记轮廓、ID、中心、X／Y、朝向及定位状态；
- 污渍框、ID、RED／NON_RED、候选／确认状态与坐标；
- FPS、几何状态和具体错误原因。

终端约每秒打印一次坐标。按 **Q** 退出；不会连接 ESP32。

### 4.3 Dashboard LIVE：真实相机标注画面

先关闭 `test-vision`、预览或其他占用同一相机的程序：

```bash
python main.py dashboard --mode live --camera 0 \
  --width 1280 --height 720 --port 5050
```

打开 <http://127.0.0.1:5050>。这条命令会打开真实相机；LIVE 直接显示与 `test-vision` 相同的 OpenCV 绘图输出，经 `/api/stream.mjpg` 传输。单个服务进程共用一个采集／检测流程，刷新网页或增加标签页不会再开一个相机。

相机断开时保留任务历史并显示错误；恢复设备后点击 **Retry monitoring**。切回 DEMO 或 Ctrl+C 停止服务会释放相机。Dashboard 不发送 MOVE、TARGET、ARRIVED，也不启动清洁；已有手动任务状态操作仅记录管理状态。

**当前相机共享范围是同一个 Dashboard 进程内部。** Dashboard LIVE 和独立 `auto-clean` 都会持有自己的相机，不能同时争用同一设备。运行实机导航时，将 Dashboard 切为 DEMO 或停止它；`--navigation-events` 可只读加载导航进度，不能解决跨进程相机共享。

### 4.4 完整自动流程模拟：无需硬件

```bash
python main.py auto-clean --mode simulation --max-tasks 2
```

内置合成画面和机器人模型用于验证红色优先、导航、清洁完成事件和连续目标。成功时摘要为 `READY`、`completed=2`。增加 `--return-home` 可测试原有逐目标返航流程。该结果不代表实际机器人到达或清洁效果通过。

## 5. 相机、背景与定位配置

### 5.1 关键文件

| 文件 | 用途与注意事项 |
| --- | --- |
| `config/camera_config.json` | Module 01 地面／墙面标定；本机文件，Git 默认忽略 |
| `data/iphone_reference.png` 与同名 `.json` | 已登记合法背景及元数据；两者需要一起保存 |
| `config/robot_config.json` | ArUco 字典、尺寸、高度、中心偏移和车身尺寸 |
| `config/robot_marker_plane.json` | 已保存的固定高度实测映射及独立验证误差 |
| `data/robot_plane_points_marker9.json` | 当前 9 cm 标记配置对应的 4 训练＋3 验证测量会话 |
| `data/robot_plane_points.json` | 原始中断会话，当前为 4 训练＋1 验证；保留原始数据，不能当成最新完整验证集 |
| `config/spill_config.json` | Module 02 检测、分类与确认参数 |
| `config/localization_config.json` | Module 03 定位与质量参数 |
| `config/movement_timing.json` | 分方向实测运动时间估计 |
| `config/navigation_config.json` | 位姿新鲜度、容差、时长上限、停稳与修正限制 |
| `config/cleaning_hardware.json` | 实机车身与工具几何；未知测量保持空值 |
| `config/cleaning_demo.json` | 模拟用工具几何，不可代替真实硬件测量 |

默认画面为 **1280×720**；本机场地地面为 **121.6×68.4 cm**，墙面为 **121.6×31.5 cm**。这些是当前装置的数据，不是通用默认场地尺寸。

坐标以厘米计：地面使用 `(X,Y)`，墙面使用 `(X,H)`，H 从墙脚向上。墙面高度不能当成地面 Y；墙脚投影也不自动等于可停车位置。

### 5.2 当前机器人测量

| 项目 | 当前记录 |
| --- | --- |
| 车身长 × 宽 | 26 × 15.2 cm |
| ArUco | `DICT_4X4_50`，ID 0 |
| 标记黑色正方形边长 | 9.0 cm，不含白色边框 |
| 标记安装高度 | 16.6 cm |
| 前进 F | 约 16 cm/s |
| 后退 B | 约 18.5 cm/s |
| 左转 L | 约 4000 ms / 90°；Module 04 theta 减少 |
| 右转 R | 约 3000 ms / 90°；Module 04 theta 增加 |

运动换算使用对应方向的测量；早期“R 1500 ms ≈ 90°”只保留为历史参考，不替代当前估计。固定 PWM 不等于恒定真实速度，导航仍需摄像头反馈。

左侧墙面海绵记录为 8×13.5 cm、向车身左侧外伸 5 cm、最低边离地 5 cm、前缘比车头后退 6 cm。它不是地面接触工具。喷嘴约在前左角、高 16.5 cm，落水点约在车前 10 cm；精确安装偏移、喷洒范围及旋转扫掠仍未全部核实，不能由这些近似值推定覆盖。

### 5.3 新场地初始化与原有配置保护

已有通过验证的装置应先复用并检查保存文件，**不要把重新标定或重建背景当作日常启动步骤**。新克隆通常不包含被忽略的真实相机资料，需要恢复对应备份或由操作员重新采集。

只读检查入口：

```bash
python main.py preview --camera 0
python main.py verify-calibration --camera 0 --grid
python main.py track-robot --camera 0 \
  --marker-plane config/robot_marker_plane.json \
  --verification-points data/robot_plane_points_marker9.json \
  --confirm-fixed-camera --confirm-ground-reference
```

仅在确实建立**新装置／新场地**时，才使用以下创建入口；场地尺寸需由操作员测量：

```bash
python main.py calibrate --camera 0 --dual-plane --floor-boundary top \
  --config config/camera_config_NEW.json --grid
python main.py background-reference --camera 0 --confirm-clean \
  --output data/legal_reference_NEW.png
python main.py calibrate-robot-interactive --help
```

`NEW` 是提醒使用新文件名，不表示可覆盖已有文件。背景登记会将当时画面中的物品作为合法背景；不要把待识别污渍登记进去。机器人交互标定支持 `--resume`，但只有会话元数据、相机分辨率、标记高度／尺寸与配置兼容时才能续采；不可强行把旧会话改成当前配置。

移动相机、改变镜头／裁剪／缩放、分辨率或标记安装后，需要重新核对映射兼容性。`UNVERIFIED` 不能当作已验证导航位姿；相同分辨率本身也不能证明相机没有移动。

## 6. 自动清洁与实机导航

### 6.1 自动任务顺序

1. 读取新鲜、已验证机器人位姿，保存 HOME 位置与朝向。
2. 自动检测并确认污渍，RED_SPILL／HIGH 优先于 NON_RED_SPILL／LOW。
3. 按污渍完整边界计算停车位置，检查车身、侧面工具、墙面与运动扫掠范围。
4. 默认在直达和两个单拐点 F／B 路线中，选择通过全部扫掠检查、按分方向标定预计用时较短的方案；使用有上限的 MOVE、实时位姿监控、早停与短脉冲修正。
5. 地面目标停车时车头朝向污渍，前保险杠距最近污渍边缘约 **5 cm**；车身中心需计入 13 cm 的中心到车头距离。到达后重新读取污渍和机器人，核验间距与朝向。
6. 电脑发送 `TARGET`，再发送 `ARRIVED`；ESP32 自行完成清洁。
7. 只在收到准确的 `DONE,CLEANING` 后，等待停稳并重新定位。
8. 从新鲜实测位姿重新扫描并继续下一确认目标，不强制返回 HOME。`--return-home` 保留原有 **Y→X 出发、X→Y 返航、恢复朝向并验证 HOME** 的行为；`--navigation-only` 仍执行监督式往返。

墙面目标采用独立的平行墙面接近方式，不把地面正面接近策略套用到墙上。停车能放下车身，也不一定表示清洁期间前后移动的扫掠范围安全。

间距按有限宽度的前保险杠与实际污渍外轮廓计算，不把包围框当作污渍边缘。位置容差造成朝向与真实污渍偏离时，进行小幅面对目标修正后再次读取相机核验。检测仍需原有连续 3 帧确认；扫描最多观察 12 帧以容纳短暂漏检，不降低检测或确认阈值。

当前初始配置：位置容差 ±2.5 cm、朝向 ±7°；HOME 位置误差上限 3 cm、朝向 10°。这些是软件配置，不是已经证实的机械精度。单段 MOVE 上限为 **1000 ms**；缺少验证过的停止数据时使用更短的监控脉冲，当前短脉冲上限为 500 ms，不能自动采用固件 10000 ms 最大值。

定位短暂丢失时先 STOP，只有停止已确认、且有限静止重捕获窗口内取得新位姿后才能重新计算动作。持续丢失、相机失败、STOP 无确认、通信不确定或清洁失败均停止任务；不会用旧坐标继续移动、重发不确定的 MOVE，或在未知清洁状态下自动返航。

### 6.2 监督式实机导航测试：不清洁

下列命令**会操作真实机器人**，仅供操作员检查场地、隔离清洁执行器电源并明确授权后手动运行。它自动选择检测目标，不输入手工目标坐标：

```bash
python main.py auto-clean --mode physical --navigation-only \
  --host 172.20.10.12 --port 3333 --camera 0 --confirm-navigation-test
```

流程为检测→Y→X→核验停车→X→Y→恢复 HOME 朝向；不发送 TARGET／ARRIVED。`--confirm-navigation-test` 同时确认当前背景、相机、中心参考、路线与静止工具检查，不能把它当作绕过真实检查的开关。IP 是本次装置使用过的地址，换热点／重新分配后应以 ESP32 当前地址为准。

### 6.3 完整实机自动清洁

同样会产生真实移动和清洁动作；确认已完成通信核验、监督导航及清洁预设检查后，由操作员明确启动：

```bash
python main.py auto-clean --mode physical \
  --host 172.20.10.12 --port 3333 --camera 0 \
  --background data/iphone_reference.png --confirm-clean \
  --authorize-motion --confirm-fixed-camera \
  --confirm-ground-reference --confirm-static-tools
```

默认一个批次上限为 100 个目标，可用 `--max-tasks` 减少。正常持续到没有未处理的可见确认目标；达到上限仍有目标时报告 `BATCH_LIMIT`。已结束预设的同一轮廓不会因颜色变化自动重复喷洒；此前不可达的目标会在处理其他任务后按新位姿重新评估。所有物理动作仍由真实回复和新鲜实测位姿确认。

首次 TCP 连接默认等待 5 秒，可用 `--connect-timeout` 单独调整；它不改变指令回复、MOVE 完成或 STOP 核验的时限。收到 `DONE,MOVE` 后先静止等待再获取新位姿。连续 3 次转向进展低于 0.5° 仍停止；`[MOTION]` 显示计划动作与实测转角。`POSE_DIAGNOSTIC` 同时记录所选 marker ID、角点和坐标；其中帧龄从电脑收到画面时计算，不能证明 iPhone 传输前没有延迟。STOP/PING 核验失败后保持故障状态，退出摘要保留最初错误及再次 STOP 的失败信息，不自动恢复。

运行生成 `data/auto_navigation_<运行标识>/`：

- `wire.log`：完整 `[TX]`／`[RX]`、定位、背景诊断及停车候选日志；
- `progress.jsonl`：导航状态、位姿、候选拒绝阶段、间距及到达核验；
- `summary.json`：完成、跳过、拒绝原因与批次结果。

`NO_CONFIRMED_SPILL` 表示观察窗口内没有确认目标；`NO_REACHABLE_TARGET` 表示有目标但没有安全方案。部分完成后仍有拒绝目标时报告 `PARTIAL`，不会隐藏为 READY。默认模式 READY 表示当前队列的预设任务处理结束，不表示已回 HOME 或污渍已被视觉确认清除；是否核验 HOME 单独记录在 `home_verified`。

## 7. ESP32 通信协议

当前自动导航明确使用 **NATHACK_V2_SIMPLE**：TCP **3333**、ASCII、每条以 LF `\n` 结束。不是 JSON；电脑内部的 JSON 坐标和任务数据不会原样发送给固件。

```text
CAPS,NATHACK_V2_SIMPLE,MOVE_DURATION_MS,FIXED_PWM_170,STOP,TARGET,ARRIVED
```

| 指令 | 回复／含义 |
| --- | --- |
| `PING` | `PONG` |
| `CAPS` | 上述能力行，连接成功本身不代表运动获授权 |
| `STATUS` | `STATUS,<清洁状态>`；不能代替定位或 MOVE 完成判断 |
| `MOVE,<F或B或L或R>,<duration_ms>` | `OK,MOVE` 表示接受；正常结束后 `DONE,MOVE` |
| `STOP` / `ABORT` | `OK,STOP`；中止 MOVE 不补发 `DONE,MOVE` |
| `TARGET,<GROUND或WALL>,<BLOOD或OTHER>` | `OK,TARGET`，只登记类别，不传目标坐标 |
| `ARRIVED` | 清洁开始事件与 `OK,ARRIVED`；预设结束发送 `DONE,CLEANING` |
| `ERR,BUSY` 或其他 `ERR,...` | 拒绝／失败，不能当成正常完成 |

所有电机运动的协议约定为 **PWM170、20 kHz、8 位**，MOVE 第三字段永远是毫秒，固件范围 1～10000。RED_SPILL 映射为 BLOOD，NON_RED_SPILL 映射为 OTHER；这是预设选择标签。

`OK,MOVE`、`DONE,MOVE`、已中止和结果未知分别处理。`DONE,MOVE` 不表示目标到达，也不表示车身已经完全停稳；`DONE,CLEANING` 不表示机器人已回 HOME。

SIMPLE **没有**旧版 HEARTBEAT 看门狗或锁存 ESTOP 能力，客户端不会向它发送 HEARTBEAT／SAFETY／ESTOP／RESET_ESTOP。物理动作仍需电脑端明确授权，但不能宣称具有 SIMPLE 固件未提供的保护；软件 STOP 也不是硬件急停。

项目另外保留 `safe_v1_1` 客户端／模拟器，不能与 SIMPLE 的能力协商混用。[旧安全协议交接](MODULE07_DURATION_PROTOCOL.md) 是该独立版本的历史核对文档，不是当前 SIMPLE 启动指南。

只查询、不运动的当前 SIMPLE 连接示例：

```bash
python main.py robot-client --mode physical --firmware-profile simple \
  --host 172.20.10.12 --port 3333 \
  --command PING --command CAPS --command STATUS
```

完整固件对接参考见 [SIMPLE 对接指导](docs/NatHack_电脑端对接指导.md)，其中较早的右转时间示例以当前 `movement_timing.json` 为准。仓库 `firmware/NatHack_Robot/` 保存的是较早导入副本，存在缺失 Protocol 源码及参数差异；见 [固件说明](firmware/NatHack_Robot/README.md)。不能把该目录等同于当前车上烧录版本，也不要直接按它重新上传固件。

## 8. 其他常用入口

```bash
# 单独查看方向计时数据，不产生运动
python main.py movement-calibration show

# 单目标导航／清洁／返航的确定性模拟
python main.py simulate-navigation --output-dir data/module06_demo_NEW

# 只根据实时视觉打印坐标、停车点和 MOVE 计划；不连接 ESP32
python main.py quick-plan --camera 0

# 已有目标事件的独立清洁规划；具体参数见帮助
python main.py plan-cleaning --help

# 检测与地面／墙面定位，不执行导航
python main.py detect-spills --camera 0 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries
```

带相机的命令不要同时占用同一个设备。所有子命令的当前参数以 `python main.py <命令> --help` 为准。

## 9. 测试

完整自动测试入口：

```bash
python -m pytest -q
```

完整套件含本机 TCP 回环通信测试，受限环境可能不允许绑定端口。通过模拟或测试替身验证不等于真实执行器验证。

Dashboard／双视觉专项：

```bash
python -m pytest -q tests/test_dashboard_live_stream.py \
  tests/test_dashboard.py tests/test_dashboard_isolation.py \
  tests/test_live_vision.py tests/test_live_detection_regressions.py
```

最近该专项结果为 **86 passed**，覆盖标注帧逐像素一致、MJPEG 内容、单相机、多标签页／刷新、断线释放和重新打开。导航相关最近专项记录为 **251 passed、21 deselected**，未运行的网络集成项不计作通过。以上为各次专项记录，存在覆盖重叠，不相加，也不是本次完整套件的新结果。

核心用例还包括方向计时、MOVE／DONE 区分、STOP 竞态、相机丢失、新鲜位姿、越界／障碍、修正限制、清洁完成与 HOME 生命周期。详见 `tests/` 及对应模块报告。

## 10. 目录与资料

```text
main.py                    统一 CLI
vision/                    Modules 01–05、双视觉处理与绘图
navigation/                Module 06、自动队列、计时、碰撞与模拟
robot_comm/                Module 07 客户端、协议、模拟器、线日志
dashboard/                 Flask 服务、LIVE 视频、DEMO、任务展示
config/                    场地、机器人、工具、检测与导航配置
data/                      本机背景、测量、运行证据及少量合成测试资料
firmware/                  导入的固件参考与历史安全补丁；版本需核对
tools/                     离线诊断、样例生成、通信台架工具
tests/                     自动化回归测试
docs/                      固件对接等补充文档
backups/                   本机备份，不默认进入 Git
```

| 资料 | 内容 |
| --- | --- |
| [Module 04 报告](MODULE04_REPORT.md) / [交互标定说明](MODULE04_WIZARD_REPORT.md) | 机器人定位、固定高度映射与测量流程 |
| [Module 05 报告](MODULE05_REPORT.md) | 独立地面／墙面工具规划及配置要求 |
| [Module 06 报告](MODULE06_REPORT.md) / [混合导航说明](MODULE06_HYBRID_REPORT.md) | 导航设计与历史测试记录 |
| [监督导航测试报告](MODULE06_NAVIGATION_TEST_REPORT.md) | 实机导航入口和前置条件 |
| [Dashboard 报告](DASHBOARD_REPORT.md) | LIVE／DEMO、API、修改文件及专项验证 |
| [最近目标拒绝调查](SKIPPED_TARGET_20261109_073050_REPORT.md) | 离线测量、证据缺口与最小规划修复 |
| [自主导航 MVP 报告](AUTONOMOUS_MVP_REPORT.md) | 当前连续目标流程、已修复软件问题、完整回归及实机验证限制 |
| [旧版模块操作参考](README_MODULE_REFERENCE.md) | 旧 README 完整归档，包含详细 CLI／API 说明；历史状态需对照当前源码 |

### 配置与数据恢复

`.gitignore` 排除虚拟环境、缓存、凭据、大录像、运行日志以及部分本机标定／背景资料；**Git 提交不等于这些资料已备份**。换电脑或做 checkpoint 前，分别核对 Git 跟踪和独立备份：

- 相机标定、已验证机器人映射、机器人设置与运动计时；
- 合法背景 PNG **及其 JSON 元数据**；
- 原始 4＋1 中断会话、完整验证会话和 9 cm 标记关联会话；
- 当前工具几何、必要运行证据和实际固件版本资料。

保留原始文件，使用新目录备份，并通过文件哈希核对备份与源文件一致。不要为恢复演示而静默重写标定、重新登记背景或把模拟配置当成实测数据。凭据不应提交到仓库。

## 11. 常见问题

| 现象 | 首先检查 |
| --- | --- |
| 相机打不开／黑屏 | macOS 权限、相机索引、其他程序占用、Continuity Camera 连接；索引 0 不保证总是 iPhone |
| LIVE 无画面 | 服务终端错误、标定／背景路径、相机权限；恢复设备后 Retry monitoring |
| `UNVERIFIED` | 映射与测量会话、9 cm 标记尺寸、16.6 cm 高度、中心偏移与当前固定相机上下文是否兼容 |
| `MARKER_NOT_VISIBLE`／`STALE_POSE` | 遮挡、转动模糊、反光、标记是否仍在画面；不要放宽位姿新鲜度代替排查 |
| 背景 `MOVED` 等质量告警 | 查看原始诊断与坐标变换是否有效；自动清洁当前将非关键背景告警保留为诊断，真正无效变换仍拒绝 |
| `NO_REACHABLE_TARGET` | `wire.log` 的 `[PLAN]`、候选停车／清洁扫掠／出发及返航检查；不要关闭碰撞边界 |
| 有 `OK,MOVE` 但没有完成 | 接受不等于执行完成；查看 STOP、ERR、断线和超时，禁止自动重发 |
| CAPS 不匹配 | 核对 SIMPLE／safe_v1_1 及实际烧录固件，不能根据欢迎字符串猜测 |
| `CONFIGURATION_REQUIRED` | 补充对应工具的真实尺寸／偏移；不使用 DEMO 尺寸替代 |

本项目优先保持 Hackathon 演示链路简单、可检查、可停止。进一步实机验证应基于保存的测量与真实日志进行，不能把预计位姿或软件模拟当作已发生的物理结果。
