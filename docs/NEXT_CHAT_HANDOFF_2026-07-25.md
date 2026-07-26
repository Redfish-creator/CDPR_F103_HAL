# NEXT_CHAT_HANDOFF_2026-07-25

本文件原本给 2026-07-25 后续新对话接手使用。当前代码已被后续
`Q2-CAL-JOG-v12-Q3-ORDER-PC2OK-RETRY3` 覆盖；最新状态以
`docs/PROJECT_HANDOFF.md`、`docs/DECISIONS.md`、`docs/DATA_DICTIONARY.md`
和 `data/site_parameters.csv` 为准。

## 直接给下一个新对话的关键词

```text
CDPR Q2-CAL-JOG-v12-Q3-ORDER-PC2OK-RETRY3；工作区 D:\CDPR_F103_HAL；
先读 AGENTS.md 指定的 docs/SITE_LAYOUT.md、docs/PROJECT_HANDOFF.md、docs/DECISIONS.md、docs/DATA_DICTIONARY.md、data/site_parameters.csv；
不要回退用户已有修改；不要猜现场信息；soft/laser/circle/err/enc 按 DATA_DICTIONARY.md 解释；
当前代码状态：独立四角点任务；默认顺序 LT>RT>RB>LB；KEY1/PA15 进入 OLED 顺序选择模式，PC0 上一个，PC1 下一个，PC2 确认，PC3 返回，PA0/WK_UP 按已选顺序启动；选择键动作通过 USART1 调试串口打印；
V12 当前保留：nohome 视觉归零基础写间隔 160ms，M2 额外 120ms 前静默和 80ms 后静默，中心重置和角点任务通信失败后的安全恢复机会均为 3 次；
最新完整四角现场结果来自 run15，confirmed=0x0F，unconfirmed=0x00；LT/RT/RB/LB 均视觉确认；LB 为 M1 所在角点，当前有 LB 专用拉紧 gain；
构建已通过：cmake --build --preset Debug --target CDPR_F103_HAL；hex 在 build\Debug\CDPR_F103_HAL.hex；RAM 5136B/48KB，FLASH 99964B/256KB。
```

## 当前工程状态

- 当前工作目录：`D:\CDPR_F103_HAL`。
- 当前分支曾为：`backup-current`。
- 工作区是脏工作区，已有多处未提交修改；不要执行 `git reset --hard`，不要回退用户已有修改。
- 当前启动标记：`[BOOT] CDPR backup-current app start Q2-CAL-JOG-v12-Q3-ORDER-PC2OK-RETRY3`。
- 当前构建命令已通过：`cmake --build --preset Debug --target CDPR_F103_HAL`。
- 当前 hex：`D:\CDPR_F103_HAL\build\Debug\CDPR_F103_HAL.hex`。
- 当前资源占用：RAM `5136 B / 48 KB (10.45%)`，FLASH `99964 B / 256 KB (38.13%)`。

## 新对话开始必须先读

按 `AGENTS.md`，继续开发前必须先读：

- `docs/SITE_LAYOUT.md`
- `docs/PROJECT_HANDOFF.md`
- `docs/DECISIONS.md`
- `docs/DATA_DICTIONARY.md`
- `data/site_parameters.csv`

关键规则：

- 不要猜现场信息；无法确认写“待确认”。
- 数值保留原单位和原精度。
- `soft` 是控制器内部目标/估计坐标，不等于真实激光坐标。
- `laser` 是视觉激光点坐标。
- `circle` 是视觉圆心坐标，可能是目标圆，也可能是中心圆或其他圆。
- `err = laser - circle`。
- `enc` 是电机编码器位置，单位 `0.1 deg`。
- 编码器回零不等于视觉中心归零。
- 理论绳长模型不等于真实绳长模型；现场存在松线、摩擦和绕线误差。
- 相对缓冲写 `TIMEOUT/CRC_BAD/REJECTED/FRAME_BAD/TX_FAIL` 后不能原地重发同一相对命令，不能强行同步触发。
- 继续开发前保存新串口数据，尤其是每个 waypoint/segment 的 `dL/cmd/enc/vision`。

## 历史保留的 V12 运动策略

本文件为 2026-07-25 交接快照。当前代码已被后续
`Q2-CAL-JOG-v12-Q3-ORDER-PC2OK-RETRY3` 覆盖；最新状态以
`docs/PROJECT_HANDOFF.md`、`docs/DECISIONS.md` 和 `data/site_parameters.csv`
为准。

- 巡检基础收/放绳为 T=`0.99`、P=`1.01`。
- 中心轻预紧仍为 `40` counts、`vmax=60`、稳定 `300 ms`。
- 手动点动收/放绳为 `1.00/1.00`。
- 巡检四台缓冲写入基础间隔为 `160 ms`。
- 视觉归零/nohome 小步运动四台缓冲写入基础间隔为 `160 ms`。
- M2 每次 `0x10` 缓冲写前额外静默 `120 ms`，成功后额外静默 `80 ms`。
- 上层手动回零/预紧也对 M2 使用同样的 `120 ms` 前静默和 `80 ms` 后静默。
- PA0 起步、中心重置、最终中心刷新都使用带恢复的自动归零入口；该历史版本首次归零失败后，只做一次停机、等待 `800 ms`、电机回零、视觉归零。
- 普通巡检段/细调段运动通信失败时，当前 V12 不自动重走普通段，也不原地重发失败的相对缓冲命令；只记录失败段并中止。

## 已回退的试验

- V13 `Q2-CAL-JOG-v13-CORNER-TENSION`：角点 profile 已从当前源码移除，不再打印 `PATROL profile ...`，也不再使用 `profile=...`。
- V14 `Q2-CAL-JOG-v14-SAFE-RETRY`：普通巡检段/细调段通信失败后的自动回中心重走已移除。
- V15 `Q2-CAL-JOG-v15-AUTO-RELAX`：retry-relax 自动松线已移除。

V15 run10 失败证据保存在 `data/patrol_evidence_2026-07-25-v15-run10.csv`。关键链路：

```text
LT-APP step=3/28 seg=2/5 M3 TIMEOUT
回中心重走，LT retry relax 实际为 [0,0,0,0]
LT-IN2 step=2/28 seg=3/3 M2 TIMEOUT
再次回中心重走
LT-APP step=3/28 seg=1/5 M2 TIMEOUT
recovery limit count=2/2，abort
```

判断：V15 没有解决线松紧问题，反而让 LT 路径重复执行并变差。当前不要沿 V15 自动松线/同角点重复重走方向继续。

## 下一次现场测试建议

1. 烧录 `build\Debug\CDPR_F103_HAL.hex`。
2. 上电确认启动串口含：

   ```text
   Q2-CAL-JOG-v12-ZERO-RECOVER
   ```

3. KEY0 在中心视觉归零并保存电机零点。
4. PA0 开始巡检。
5. 保存完整串口，重点关注：

   - `PATROL tension profile: center=40@60, balance=OFF, path gain(T=0.99 P=1.01), vmax=300`
   - 不应再出现 `corner_profile=ON`、`PATROL profile ...`、`profile=...`
   - 不应再出现 `PATROL move recovery after waypoint ...`
   - `PATROL SEG ... status=TIMEOUT/OK`
   - `PATROL SEG BUS ... attempted=... failed_m=... trigger_attempted=...`
   - `PATROL SEG DATA ... laser/circle/err/tilt/enc`
   - `PATROL CAL TABLE`

## 可直接复制给新对话的请求

```text
请继续处理 D:\CDPR_F103_HAL 的 CDPR 固件。先阅读 AGENTS.md 指定的 docs/SITE_LAYOUT.md、docs/PROJECT_HANDOFF.md、docs/DECISIONS.md、docs/DATA_DICTIONARY.md、data/site_parameters.csv，不要回退已有修改。

当前代码已更新到 Q2-CAL-JOG-v12-Q3-ORDER-PC2OK-RETRY3，已构建通过，hex 在 build\Debug\CDPR_F103_HAL.hex。当前为独立四角点任务；默认顺序 LT>RT>RB>LB，KEY1/PA15 进入 OLED 顺序选择模式，PC0 上一个，PC1 下一个，PC2 确认，PC3 返回，PA0/WK_UP 按已选顺序启动。中心重置和角点任务通信失败后的安全恢复机会均为 3 次；仍不原地重发失败的不确定相对缓冲命令。

下一步请先用新的现场完整串口验证 Q3 顺序选择和三次恢复行为。重点看是否启动标记为 Q2-CAL-JOG-v12-Q3-ORDER-PC2OK-RETRY3，KEY1/PA15 菜单是否与 PC0-PC3 jog 隔离，PC2/PC3 键位是否正确，且通信失败恢复是否从当前角点 job 的 IN1 重新开始。继续修改前先保存每个 waypoint/segment 的 dL/cmd/enc/vision。
```
