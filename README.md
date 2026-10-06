# 航摄姿态插值服务（Attitude Interpolation Service）

把低频惯导（INS）姿态样本投影到相机曝光时刻。姿态按**单位四元数**解释，
分量顺序固定为 **[w, x, y, z]**；由于 `q` 与 `-q` 表示同一旋转，插值始终沿
**最短旋转弧**进行，返回序列按曝光顺序重新定号，保证相邻帧影像足迹连续。

## 快速开始

```bash
# 构建并启动服务（宿主机端口默认 8080，可用 HOST_PORT 覆盖）
docker compose up app
HOST_PORT=9000 docker compose up app

# 一次性验证：app 健康检查通过后自动运行 verify，结束后自行退出
docker compose up --exit-code-from verify verify
echo $?   # 0 = 全部通过
```

`verify` 是一次性服务：仅在 `app` 通过健康检查后启动，依次执行
**构建检查**（字节码编译 + 应用导入）、**代码测试**（pytest 套件）、
**API 冒烟**（插值精度/连续性与间隙拒绝等），随后以位掩码退出码汇总：

| 退出码位 | 含义 |
|---|---|
| 0 | 全部通过 |
| 1 | 构建/编译检查失败 |
| 2 | 单元/接口测试失败 |
| 4 | API 冒烟失败 |

本地开发（无需 Docker）：

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/uvicorn app.main:app --port 8000
API_BASE_URL=http://127.0.0.1:8000 .venv/bin/python verify.py
```

## API

### `POST /api/attitudes/interpolate`

请求体：

```json
{
  "samples": [
    {"t": 1700000000000000000, "q": [1.0, 0.0, 0.0, 0.0]},
    {"t": 1700000000000010000, "q": [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]}
  ],
  "queries": [1700000000000002500, 1700000000000005000, 1700000000000007500],
  "max_gap_ns": 10000,
  "extrapolation_limit_ns": 0
}
```

| 字段 | 约束 |
|---|---|
| `samples` | 2–2000 个样本；`t` 为纳秒整数时间戳，**严格递增**；`q` 为 4 个有限数（w, x, y, z），整体非零（按单位四元数解释，自动归一化） |
| `queries` | 1–500 个纳秒整数时间戳，**严格递增**；默认须落在 `[samples[0].t, samples[-1].t]` 闭区间内（可位于端点）。启用外推后，可越过首/末样本各不超过 `extrapolation_limit_ns` 纳秒 |
| `max_gap_ns` | 非负整数；每个查询的**包围样本间隔**（外推时为首/末两项支撑样本的间隔）不得超过该上限 |
| `extrapolation_limit_ns` | 可选，非负整数；**省略或为 0 时行为与旧版完全一致**（区间外查询仍以 `QUERY_OUT_OF_RANGE` 拒绝）。为正时，首端取最早两项、末端取最后两项样本的**相对旋转与精确纳秒间隔**作**恒角速度外推**，查询越过端点的距离不得超过该值 |

时间戳请使用 JSON 整数字面量（历元纳秒 ~1.7e18 超出 float64 精确整数
范围 2^53，浮点字面量会被拒绝以避免静默舍入）。

**外推约定**（仅 `extrapolation_limit_ns > 0` 时）：

- 外推沿对应端点相邻样本的**最短旋转方向**继续，等价于在首/末两项
  确定的大圆上延长 SLERP（首端参数 < 0，末端参数 > 1）。
- 支撑样本的间隔仍须满足 `max_gap_ns`，否则以 `SAMPLE_GAP_EXCEEDED`
  整体拒绝（错误索引为触发的 query 索引，`side` 标明 `before`/`after`）。
- 从端点样本到外推姿态的旋转必须**严格小于 180°**；达到 180° 时以
  `EXTRAPOLATION_180_DEGREE_ROTATION` 拒绝（样本对本身为 180° 仍按
  既有 `AMBIGUOUS_180_DEGREE_ROTATION` 在样本校验阶段拒绝）。
- 越过端点超过 `extrapolation_limit_ns`（限界本身闭区间、恰好等于时接受）
  以可定位 query 索引的 `QUERY_OUT_OF_RANGE` 拒绝。
- 任一 query 失败则整架次请求 4xx，不产生部分结果；外推帧与区间内帧
  共用同一套单位化与整列符号连续约定，形成可复核的连续姿态序列。

响应 `200`：

```json
{
  "attitudes": [
    {"t": 1700000000000002500, "q": [0.9807852804032305, 0.0, 0.0, 0.19509032201612828]}
  ]
}
```

- 按查询顺序返回单位四元数（模长误差 ≤ 1e-9，插值误差 ≤ 1e-9）。
- **符号约定**：首项的第一个非零分量为正；后续项选取使与前项内积
  非负的等价符号（`q` / `-q`），保证相邻曝光姿态连续、无正负翻转。

### 错误响应（4xx，不产生部分结果）

任何输入越界、零四元数、非递增时间、180° 歧义或超长间隙都会返回
`400`，且可定位到具体索引：

```json
{
  "detail": {
    "code": "SAMPLE_GAP_EXCEEDED",
    "message": "queries[0]=... is enclosed by samples[0] (t=...) and samples[1] (t=...) whose gap 10000000 ns exceeds max_gap_ns=1000",
    "index": 0,
    "path": "queries[0]",
    "sample_index": 0,
    "gap_ns": 10000000,
    "max_gap_ns": 1000
  }
}
```

主要错误码：`SAMPLE_COUNT_OUT_OF_RANGE`、`QUERY_COUNT_OUT_OF_RANGE`、
`MISSING_FIELD`、`INVALID_TYPE`、`NON_INTEGER_TIMESTAMP`、
`TIMESTAMP_PRECISION_LOSS`、`NON_FINITE_COMPONENT`、`ZERO_QUATERNION`、
`NON_INCREASING_SAMPLE_TIME`、`NON_INCREASING_QUERY_TIME`、
`AMBIGUOUS_180_DEGREE_ROTATION`（相邻旋转恰为 180°，最短弧不唯一）、
`QUERY_OUT_OF_RANGE`（含越过 `extrapolation_limit_ns`）、`NEGATIVE_MAX_GAP`、
`SAMPLE_GAP_EXCEEDED`（含外推支撑间隔过长）、`NEGATIVE_EXTRAPOLATION_LIMIT`、
`EXTRAPOLATION_180_DEGREE_ROTATION`（外推旋转达到 180°）、
`INVALID_JSON` / `INVALID_BODY`。

> 注：省略 `extrapolation_limit_ns`（或传 0）时，`QUERY_OUT_OF_RANGE`
> 的消息与负载与旧版逐字一致——不含任何外推上下文字段。

### `GET /health`

返回 `{"status": "ok"}`，供容器健康检查使用。

## 数值约定

- 输入四元数按 `math.hypot` 稳健归一化（极小/极大分量不溢出）。
- 相邻样本先做符号对齐（内积为负则取 `-q`），再按 SLERP 沿最短弧插值；
  相邻旋转 |dot| ≤ 1e-12 视为 180° 歧义并拒绝（该阈值远高于 float64
  归一化噪声 ~1e-16，远低于任何可用的小于 180° 间隔）。
- 小角度（dot > 1-1e-9）使用级数权重避免 `acos`/`sin` 相消，全角度
  范围内插值误差 ~1e-15，远优于 1e-9 的交付容差。
- 插值参数 `u = (t - t_i) / (t_{i+1} - t_i)` 由整数纳秒精确计算，
  历元级时间戳（~1.7e18 ns）不损失精度。
- 启用外推时，先由端点相邻样本求相对旋转 `r = q0^{-1} q1`（半角
  θ < 90°），再按 `q(t) = q0 · r^u` 延长同一大圆：首端
  `u = -(t0 - t) / (t1 - t0)`，末端 `u = 1 + (t - t_n) / (t_n - t_{n-1})`。
  幂次直接由相对旋转的矢量部构造（`sin(uθ)/sinθ`），即使支撑角极小、
  外推倍数很大（如 1e-6 rad/100 ns 外推 100 µs）也不发生相消。

## 项目结构

```
app/
  attitude.py   # 核心：校验 + 最短弧 SLERP + 输出符号约定（纯标准库）
  main.py       # FastAPI HTTP 层
tests/
  test_attitude.py  # 核心数值与校验单测
  test_api.py       # API 接口测试
verify.py           # 一次性验证：构建 + 测试 + 冒烟，位掩码退出码
Dockerfile
docker-compose.yml  # app（健康检查）+ verify（service_healthy 后运行）
requirements.txt
```
