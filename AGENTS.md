# AGENTS

新会话开始前必须先阅读：

- `docs/SITE_LAYOUT.md`
- `docs/PROJECT_HANDOFF.md`
- `docs/DECISIONS.md`
- `docs/DATA_DICTIONARY.md`
- `data/site_parameters.csv`

关键规则：

- 用户已要求“暂停所有代码修改”时，不得修改 `Core/`、构建脚本或固件源码。
- 不要猜测现场信息；无法确认的一律标记为“待确认”。
- 数值保留原单位和原精度。
- 串口数据中 `soft`、`laser`、`circle`、`err`、`enc` 的含义必须按 `docs/DATA_DICTIONARY.md` 解释。
- 不要把编码器回零等同于视觉中心归零。
- 不要把理论绳长模型当作真实绳长模型；现场存在松线、摩擦和绕线误差。
- 处理电机通信失败时，连续 `REJECTED/CRC_BAD/TIMEOUT` 不能强行继续同步触发。
- 不要回退用户已有修改；需要改代码前先检查当前工作区。
- 继续开发前先保存新串口数据，尤其是每个 waypoint/segment 的 `dL/cmd/enc/vision`。
