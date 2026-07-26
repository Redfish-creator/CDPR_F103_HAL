# DATA_DICTIONARY

本文定义当前串口、标定和现场参数的字段含义。数值来源分为：现场观察、串口日志、当前代码常量、待确认。

## 当前比赛精简输出

默认 `Q2-CAL-JOG-v12-Q3-DMA-COMPACT1` 不输出周期视觉、
`PATROL SEG/NODE/CAL/AVG` 或正常电机事务明细，也不为遥测执行逐段、角点或
回零后的编码器读取。正常串口只保留启动配置、最终巡点顺序、角点开始/命中、
回中心和最终结果；任何通信失败仍保留地址、功能码、TX/RX、长度、耗时、
CRC/exception 与 `uart_error`。

下述完整字段仍适用于已保存的历史证据和 `Debug-dense` 诊断构建。精简输出不
改变字段含义，也不把编码器回零等同于视觉中心归零。

## 单位和符号

| 字段 | 单位 | 定义 |
| --- | --- | --- |
| `x,y` | cm | 平面坐标，中心为 `(0,0)`，右为 `+x`，上为 `+y` |
| `soft` | cm | 控制器内部目标/估计坐标 |
| `laser` | cm | 视觉识别出的激光点坐标 |
| `circle` | cm | 视觉识别出的圆心坐标，可能是目标圆，也可能是其他圆 |
| `err` | cm | `laser - circle` |
| `d` | cm | 到目标或中心的距离 |
| `dL` | cm | 理论绳长变化量；负数为收线，正数为放线 |
| `cmd` | cm | 经过限幅/平衡后的实际命令量，打印为正值 |
| `enc` | 0.1 deg | 电机编码器位置读数，按电机零点相对值理解 |
| `enc_before/enc_after` | 0.1 deg | 一个 segment 前一有效读数/运动后读数 |
| `denc` | 0.1 deg | `enc_after - enc_before`；任一侧读取失败时为 `NA` |
| `enc_src=SEG/READ` | enum | 历史/诊断 `PATROL NODE` 复用末段读数，或在末段不完整时补读 |
| `enc_sample=DEFERRED_TO_CORNER` | enum | 上一版稳定日志字段，表示该段没有读取编码器；不是 `0`、不是读取失败，也不能由 `cmd` 反推。当前精简比赛构建不再打印该行，角点也不额外读取遥测编码器 |
| `PATROL SEG BASE enc=DEFERRED profile=CORNER_ONLY` | enum | 上一版稳定日志字段；当前精简比赛构建不再打印。中心电机零点保存和视觉中心归零始终是两个独立步骤 |
| `age` | ms | 视觉数据年龄，越大越可能过期 |
| `tilt` | 待确认 | 视觉或姿态输出的倾斜量；数值常在 `-1.00..1.00` |
| `vmax` | 0.1 RPM | 电机命令最大速度 |
| `mag` | 0.1 deg | 电机命令幅值 |
| `bal` | boolean | `1` 表示巡检平衡启发式启用 |
| `release<=` | cm | 当前 segment 允许的单根放绳上限；用于解释分段安全约束，不是实际绳长 |

## 视觉数据字段

示例：

```text
MANUAL vision patrol latest LASER cnt=1363 x=-19.80 y=20.20 age=98ms CIRCLE cnt=1366 x=-20.00 y=20.00 age=98ms TILT cnt=1250 x=-0.27 y=0.49 age=98ms err=(0.20,0.20) soft=(-19.47,14.27)
```

| 字段 | 含义 | 风险 |
| --- | --- | --- |
| `LASER cnt` | 激光视觉帧计数 | 计数不变且 age 增大表示视觉可能卡住 |
| `CIRCLE cnt` | 圆心视觉帧计数 | 识别到的圆不一定是当前目标圆 |
| `TILT cnt` | 倾斜/姿态帧计数 | age 过大时不可用于调速 |
| `HOME src=LASER-CIRCLE` | 归零使用激光和中心圆误差 | 依赖视觉在线 |
| `&mode,initial#` | 请求中心/初始视觉模式 | 串口命令 |
| `&mode,total#` | 请求全局/四角视觉模式 | 串口命令 |

数据质量规则：

- `age <= 500 ms` 是当前归零使用的在线判断阈值。
- `age` 达到数秒或数十秒时，不能再作为位置闭环数据。
- 如果目标应为 LT/RT/RB/LB，但 `circle=(0,0)`，只能说明看到中心圆，不能确认目标角点。
- 如果视觉 `cnt` 长时间不变，即使位置数值看似合理，也要判为待确认。

## 电机和通信字段

| 字段 | 含义 |
| --- | --- |
| `home buf mN: OK` | 第 N 个电机的缓冲命令成功 |
| `home trigger: OK` | 四电机同步触发成功 |
| `TIMEOUT` | 等应答超时 |
| `CRC_BAD` | 应答 CRC 校验失败 |
| `REJECTED` | 驱动器返回异常或拒绝 |
| `TX_FAIL` | UART 发送失败 |
| `UART_BUSY` | HAL 在本次阻塞发送开始前发现 UART 忙；未发送，可短延时重试 |
| `FRAME_BAD` | 收到字节但地址或功能码不属于当前请求，或事务参数非法 |
| `manual trigger` | 手动回零同步触发 |
| `pos=0 zero=0 err=0` | 电机位置和保存零点一致 |

失败事务诊断字段：

| 字段 | 含义 |
| --- | --- |
| `tx/rx` | 本次事务实际记录的十六进制发送/接收字节 |
| `exception` | CRC 正确的 Modbus 异常响应码；具体含义按实际驱动固件确认 |
| `crc_calc/crc_recv` | 本机计算值/帧内接收值 |
| `elapsed` | 从事务开始到返回的耗时，单位 ms |
| `trigger_attempted` | `1` 表示已尝试同步触发；缓冲失败时应为 `0` |
| `attempted` | 四电机缓冲写入尝试位掩码，bit0..bit3 对应 M1..M4 |

最新失败链路定义为通信/驱动层失败：

```text
home buf m3 attempt=1 REJECTED
home buf m3 attempt=2 CRC_BAD
home buf m3 attempt=3 CRC_BAD
home buffer FAIL -> stop all, abort
```

## 当前关键阈值

| 名称 | 数值 | 单位 | 来源 |
| --- | ---: | --- | --- |
| `HOME_OK_CM` | `0.6` | cm | 当前代码 |
| `HOME_STEP_MAX_CM` | `0.8` | cm | 当前代码 |
| `HOME_MAX_ITER` | `30` | count | 当前代码 |
| `HOME_VISION_AGE_MS` | `500` | ms | 当前代码 |
| `HOME_ROUGH_DONE_CM` | `12.0` | cm | 当前代码 |
| `HOME_ROUGH_STEP_CM` | `0.6` | cm | 当前代码 |
| `HOME_TOTAL_CORNER_CM` | `20.0` | cm | 当前代码 |
| `HOME_NOHOME_CALC_LIMIT_CM` | `21.8` | cm | 当前代码 |
| `MANUAL_AUTO_HOME_FAST_STEP_CM` | `2.0` | cm | 当前代码 |
| `MANUAL_AUTO_HOME_FINE_STEP_CM` | `0.5` | cm | 当前代码 |
| `MANUAL_AUTO_HOME_FINE_ZONE_CM` | `3.0` | cm | 当前代码 |
| `MANUAL_AUTO_HOME_MAX_ITER` | `30` | count | 当前代码 |
| `MANUAL_AUTO_HOME_MAX_START_CM` | `16.0` | cm | 当前代码 |
| `MANUAL_RETURN_TOL_COUNTS` | `20` | 0.1 deg | 当前代码 |
| `MANUAL_RETURN_BASE_VMAX` | `600` | 0.1 RPM | 当前代码 |
| `MANUAL_RETURN_ENCODER_SUSPECT_COUNTS` | `5000` | 0.1 deg | 当前代码 |
| `MANUAL_RETURN_VISUAL_NEAR_CM` | `8.0` | cm | 当前代码 |
| `MANUAL_CENTER_TENSION_COUNTS` | `40` | 0.1 deg | 当前代码 |
| `MANUAL_CENTER_TENSION_VMAX` | `60` | 0.1 RPM | 当前代码 |
| `MANUAL_CENTER_TENSION_SETTLE_MS` | `300` | ms | 当前代码 |
| `EDGE_PATROL_TARGET_CM` | `16.0` | cm | 当前代码 |
| `EDGE_PATROL_APPROACH_CM` | `13.0` | cm | 当前代码 |
| `EDGE_PATROL_MID_CM` | `10.5` | cm | 当前代码 |
| `EDGE_PATROL_SEG_MAX_CM` | `2.0` | cm | 当前代码 |
| `EDGE_PATROL_DWELL_MS` | `700` | ms | 当前代码 |
| `EDGE_PATROL_SEG_SETTLE_MS` | `300` | ms | 当前比赛代码；删日志后仍明确保留 |
| `EDGE_PATROL_OK_CM` | `0.8` | cm | 当前代码 |
| `EDGE_PATROL_FINE_STEP_CM` | `0.45` | cm | 当前代码 |
| `EDGE_PATROL_FINE_MAX_ITER` | `12` | count | 当前代码 |
| `EDGE_PATROL_TILT_SLOW` | `0.80` | tilt | 当前代码 |
| `EDGE_PATROL_CAL_MAX_COUNT` | `30` | samples | 当前代码 |
| `EDGE_PATROL_SEGMENT_READ_GAP_MS` | `80` | ms | 诊断构建使用，V9 巡检编码器轮询间隔 |
| `EDGE_PATROL_POST_READ_QUIET_MS` | `120` | ms | 诊断构建使用，V9 四台编码器读完后的总线静默 |
| `EDGE_PATROL_SEGMENT_ENCODER_READS` | `0` | boolean | 当前比赛默认；`1` 仅用于专门高密度总线诊断 |
| `EDGE_PATROL_CAL_ENCODER_READS` | `0` | boolean | 当前比赛默认；视觉确认角点不为遥测额外读取四台编码器 |
| `MANUAL_RETURN_POST_ENCODER_READS` | `0` | boolean | 当前比赛默认；返回完成后不做只用于打印的四次位置读 |
| `manual return pre-read` | `4` | requests/return | 每角回中心前必须读取四台当前位置，直接用于计算返回量和安全保护，不可删除 |
| `APP_COMPACT_LOG` | `1` | boolean | 当前比赛默认；关闭上层正常过程刷屏，故障始终保留 |
| `MOTION_COMPACT_LOG` | `1` | boolean | 当前比赛默认；关闭运动层成功事务刷屏，故障始终保留 |
| `MANUAL_VISION_LOG` | `0` | boolean | 当前比赛默认；关闭 USART1 周期视觉输出，USART2 DMA 接收仍持续工作 |
| `EDGE_PATROL_NODE_LOG` | `0` | boolean | 当前比赛默认 |
| `EDGE_PATROL_SEGMENT_LOG` | `0` | boolean | 当前比赛默认 |
| `EDGE_PATROL_CAL_LOG` | `0` | boolean | 当前比赛默认；RAM 内视觉学习不受影响 |
| `MOTION_PATROL_M2_FIRST` | `1` | boolean | 当前巡检缓冲写顺序为 `M2 -> M1 -> M3 -> M4`，同步触发不变 |
| `EDGE_PATROL_VISION_RETRY_MAX` | `3` | attempts | 当前代码 |
| `EDGE_PATROL_VISION_RETRY_MS` | `120` | ms | 当前代码 |
| `MOTION_BUFFER_RETRY_MAX` | `3` | attempts | 当前代码 |
| `MOTION_BUFFER_RETRY_DELAY_MS` | `90` | ms | 当前代码 |
| `MOTION_PATROL_BALANCE_RADIUS_CM` | `8.0` | cm | 保留作历史对照，当前补偿停用 |
| `MOTION_PATROL_DOM_PAYOUT_RATIO` | `0.70` | ratio | 保留作历史对照，当前补偿停用 |
| `MOTION_PATROL_DOM_PAYOUT_GAIN` | `1.00` | gain | 当前安全配置；旧配置为 `0.86` |
| `MOTION_PATROL_SIDE_PAYOUT_GAIN` | `1.00` | gain | 当前安全配置；旧配置为 `1.03` |
| `MOTION_PATROL_BALANCE_ENABLE` | `0` | boolean | 当前代码，未经验证的巡检平衡补偿停用 |
| `MOTION_PATROL_BASE_VMAX` | `300` | 0.1 RPM | 当前代码，FIT360 提速试验后已恢复 V12 速度 |
| `MOTION_PATROL_M1..M4_PAYOUT_SCALE` | `1.00` | gain | 当前代码默认值 |
| `MOTION_MANUAL_JOG_BASE_VMAX` | `600` | 0.1 RPM | 当前代码，沿用第一问点动基础速度 |
| `MOTION_MANUAL_JOG_INTER_CMD_DELAY_MS` | `120` | ms | 当前代码，V11 四电机点动缓冲写入基础间隔 |
| `MOTION_PATROL_INTER_CMD_DELAY_MS` | `160` | ms | 当前代码，V11 巡检四电机缓冲写入基础间隔 |
| `MOTION_NOHOME_INTER_CMD_DELAY_MS` | `160` | ms | 当前代码，V12 视觉归零/nohome 四电机缓冲写入基础间隔 |
| `MOTION_M2_PRE_CMD_EXTRA_DELAY_MS` | `120` | ms | 当前代码，V11 M2 缓冲写入前额外静默 |
| `MOTION_M2_POST_CMD_EXTRA_DELAY_MS` | `80` | ms | 当前代码，V11 M2 缓冲写入成功后额外静默 |
| `TASK_M2_BUFFER_PRE_CMD_EXTRA_DELAY_MS` | `120` | ms | 当前代码，V12 上层手动回零/预紧缓冲写入前 M2 额外静默 |
| `TASK_M2_BUFFER_POST_CMD_EXTRA_DELAY_MS` | `80` | ms | 当前代码，V12 上层手动回零/预紧缓冲写入成功后 M2 额外静默 |
| `MOTOR_MODBUS_POLL_MODE` | `0` | boolean | 当前代码，V12-M1M3-POLL-TIM2 轮询调度已回滚 |
| `MOTOR_MODBUS_POLL_TIMER_HZ` | `N/A` | Hz | 当前代码不使用 TIM2 作为 Modbus 轮询调度源 |
| `MOTOR_MODBUS_MIN_FRAME_GAP_MS` | `N/A` | ms | 当前代码不启用全局轮询 slot；仍保留各路径原有写间隔 |
| `MOTOR_MODBUS_PREWRITE_STATUS_POLL` | `0` | boolean | 当前代码不在相对缓冲写入前轮询同一电机状态 |
| `MOTOR_MODBUS_PREWRITE_POLL_RETRY_MAX` | `0` | attempts | 当前代码不启用写前状态轮询重试 |
| `MOTOR_MODBUS_PREWRITE_POLL_RETRY_MS` | `N/A` | ms | 当前代码不启用写前状态轮询重试 |
| `MOTOR_MODBUS_PREWRITE_POLL_SETTLE_MS` | `N/A` | ms | 当前代码不启用写前状态轮询 settle |
| `EDGE_PATROL_CENTER_RESET_RETRY_MAX` | `3` | attempts | 当前代码，巡检起步/角点后/最终中心重置归零失败后的自动恢复次数 |
| `EDGE_PATROL_CENTER_RESET_RETRY_WAIT_MS` | `800` | ms | 当前代码，V12 失败后停机等待再回零的静默时间 |
| `EDGE_PATROL_MOVE_RECOVERY_RETRY_MAX` | `3` | attempts | 当前代码，角点任务内通信失败后回中心并重启当前角点任务的次数 |
| `EDGE_PATROL_MOVE_RECOVERY_WAIT_MS` | `800` | ms | 当前代码，角点任务通信失败后停机等待再回零的静默时间 |
| `EDGE_PATROL_USE_TARGET_SEED` | `0` | boolean | 当前代码，FIT360 经验 soft 目标 seed 已禁用 |
| `EDGE_PATROL_ORDER_MODE_PIN` | `PA15 / KEY1` | GPIO | 当前代码，Q3/OLED 角点顺序选择模式切换键；板载 KEY1，按下为低电平 |
| `EDGE_PATROL_ORDER_COUNT` | `4` | points | 当前代码，当前 Q2/Q3 顺序选择对象为 LT/RT/RB/LB 四个边缘圆 |
| `MANUAL_JOG_RUN_CM` | `4.0` | cm | 当前代码，全区域固定段长；边界最后一段除外 |
| `MANUAL_JOG_SAFE_LIMIT_CM` | `20.0` | cm | 当前代码，soft 点动边界；不等于实际视觉坐标 |
| `EDGE_PATROL_TAKEUP_GAIN` | `0.99` | gain | 当前代码，巡检小幅减少 1% 收绳 |
| `EDGE_PATROL_PAYOUT_GAIN` | `1.01` | gain | 当前代码，巡检小幅增加 1% 放绳 |
| `EDGE_PATROL_LB_TAKEUP_GAIN` | `1.02` | gain | 当前代码，角点1/M1所在角/LB 专用，略多收线 |
| `EDGE_PATROL_LB_PAYOUT_GAIN` | `0.99` | gain | 当前代码，角点1/M1所在角/LB 专用，略少放线 |
| `MANUAL_JOG_TAKEUP_GAIN` | `1.00` | gain | 当前代码，P4 起始点动恢复中性 |
| `MANUAL_JOG_PAYOUT_GAIN` | `1.00` | gain | 当前代码，P4 起始点动恢复中性 |
| `MANUAL_JOG_GAP_MS` | `20` | ms | 当前代码，相邻点动段间隔 |
| `MANUAL_JOG_X_UP_COMP` | `0.42` | ratio | 当前代码，保留现场直线性较好的水平运动补偿 |

`MOTION_BUFFER_RETRY_MAX=3` 只适用于发送开始前的 `UART_BUSY`。相对缓冲写入
一旦出现 `TX_FAIL/TIMEOUT/CRC_BAD/REJECTED/FRAME_BAD`，其发送或接收状态
不确定，当前代码不会重复相对命令，也不会同步触发。

V11 针对 M2 在手动点动和巡检中均出现的 `0x10` 缓冲写入 `rx_len=0` 超时，
只增加总线静默窗口；相对缓冲命令的安全边界不变。也就是说，M2 若继续
`TIMEOUT/CRC_BAD/REJECTED/FRAME_BAD`，仍然不重发、不同步触发。

V12 现场依据是 v11 已连续完成 `LT`、`RT` 和 `CENTER-RT` 前的巡检段，
但在 `CENTER-RT` 后的视觉精归零 `nohome move` 中 M2 `rx_len=0`、`402 ms`
超时。该路径原基础间隔仍为 `20 ms`，M1 成功后到 M2 之前只有约 `140 ms`
静默，短于巡检路径约 `280 ms` 的等效 M1→M2 间隔。V12 将 `nohome` 基础
间隔改为 `160 ms`，并在中心重置归零失败时最多做 `3` 次安全恢复：停机、等待
`800 ms`、按编码器回电机零点、再视觉归零。仍不重发同一条不确定的相对缓冲命令。

Q3/OLED 角点顺序选择只改变四个独立角点 job 的执行顺序，不改变每个 job 内部
`IN1 -> IN2 -> APP -> target -> fine -> return center` 的结构。选择模式内
PC0 为上一个，PC1 为下一个，PC2 确认，PC3 返回；退出选择模式后 PC0-PC3 恢复手动点动。
每次选择键动作会通过 USART1 调试串口打印 `PATROL order key ...` 日志。

V13 的 `profile=...` 日志、`PATROL profile ...` 表，以及 V14 的
`PATROL move recovery after waypoint ...` 日志均为历史试验记录；当前已回到 V12，
正常巡检串口不应再出现这些字段。若现场新日志仍出现这些字段，说明烧录的不是当前 V12 hex。

V15 曾把上述运动通信失败恢复扩展为“回中心 + 临时松线 + 重新进入角点”，并新增
`PATROL RECOVERY VISION`、`PATROL retry relax start`、`PATROL RETRY RELAX DATA`
日志。run10 显示该策略在 LT 方向退化：LT 的临时松线为 `[0,0,0,0]`，没有改变张力，
却把同一 LT 路径重复执行到恢复上限。当前代码已回到 V12，以上 V15 日志不再出现。

手动点动的 `soft` 是按预计运动时间积分的控制坐标，不是实时编码器位置。点动
分组或停止出现非 `OK` 后，当前代码设置“需要回零”锁存并禁止 PC0-PC3；只有
KEY0 成功完成电机返回与视觉归零后才清除；`PA0` 可在自动回中心后重试巡检，
不会等待自动清除。这样不能修复总线硬件，但可防止在位置状态不确定时继续累计
相对命令。

USART3 普通写应答的完整长度为 `8` 字节；驱动异常帧为 `5` 字节。当前对读命令
和 `0x10` 多寄存器写会在发送前预开 USART3_RX 对应的 DMA1 Channel 3，允许
DMA 同时收到本机回显和驱动应答，再按地址/功能码/CRC 找有效应答；`0x06`
单寄存器写仍发完后再接收，因为它的本机回显与正常应答格式完全相同，不能用
回显判定驱动确认。诊断中继续记录 `uart_error`；例如 ORE、接收超时和 CRC
错误不再混为同一种现象。完整应答窗口当前为 `400 ms`。

## 必须继续采集的数据

每个运动段至少保存：

- 时间或串口计数。
- `tag/name/step/seg`。
- `from_x/from_y/to_x/to_y`。
- `dL_m1..dL_m4`。
- `cmd_m1..cmd_m4`。
- `bal`。
- `vmax/gain/release limit`。
- 四个电机命令结果。
- 触发结果。
- 当前比赛精简模式不保存本段明细；诊断模式可保存完成后的
  `enc_m1..enc_m4` 和 `PATROL CAL/AVG`。这些是诊断样本，不是比赛动作的
  完成条件。
- 完成后的 `laser/circle/err/tilt/age`。
- 是否为目标圆，是否视觉确认。

历史/诊断日志质量标签（默认精简比赛构建不打印逐段质量行）：

| `quality` | 含义 |
| --- | --- |
| `GOOD` | 运动成功、四电机读回完整、LASER/CIRCLE 均新鲜 |
| `CMD_OK_VISION_FRESH_ENC_DEFERRED` | 四台缓冲写和同步触发成功、LASER/CIRCLE 新鲜，但本段按稳定模式未读取编码器；不能解释为电机实际位移已由编码器验证 |
| `ENC_PARTIAL` | 至少一个运动后编码器读回失败 |
| `VISION_STALE` | LASER 缺失或 `age > 500 ms` |
| `CIRCLE_STALE` | CIRCLE 缺失或 `age > 500 ms` |
| `SEEN_OTHER` | CIRCLE 新鲜但识别到的不是当前目标圆，例如 RT 目标时看到中心圆 `(0,0)` |
| `COMM_FAIL` | 缓冲写入或同步触发失败，不能作为拟合样本 |

`PATROL SEG SKIP ... reason=VISION_STALE` 表示运动尚未下发，因此不能把该
segment 的目标 `to` 当成已到达位置。

没有张力传感器时，不能把编码器数据直接等同为绳张力。它只能用于拟合实际绳长/运动响应；张力状态需要通过松垂观察、过紧现象、可能的电流/驱动报警或新增传感器间接判断。
