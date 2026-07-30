# MIU 私有 JSON 协议规范

> 版本：v1.0  
> 适用固件：miu-ftd（Leader）、miu-socket、miu-light、miu-radar、miu-rgb

---

## 1. 整体架构

```
┌──────────────┐  UDP 5678 / JSON  ┌─────────────────────────────────┐
│  子设备      │ ◄───────────────► │         Leader (miu-ftd)        │
│  socket_01   │                   │  app_device_table（路由表）      │
│  light_01    │                   │  app_uart_pc（UART0 桥接）       │
│  radar_01    │                   │  app_udp_comm_json_process()    │
│  rgb_01      │                   └──────────────┬──────────────────┘
└──────────────┘                                  │ UART0（115200）
                                                  │ JSON + '\n'
                                         ┌────────▼────────┐
                                         │   PC 上位机      │
                                         │  设备表展示      │
                                         │  绑定 / 联动规则 │
                                         │  ThingsBoard    │
                                         └─────────────────┘
```

- **Thread 内部**：子设备 ↔ Leader，UDP port **5678**，JSON 格式
- **Leader ↔ PC**：UART0，115200 baud，每条消息一行 JSON + `\n`
- **Leader 职责**：维护设备路由表；透传子设备消息到 PC；将 PC 下发的 CONTROL 路由到目标子设备
- **联动逻辑**：完全在 PC 侧实现，固件不感知

---

## 2. 传输层约定

| 属性 | 值 |
|------|----|
| Thread 端口 | UDP 5678 |
| 区分方式 | payload 首字节 `{` = JSON；首字节 `0x88` = 原有二进制协议（OTA/Path/mgm 等） |
| 编码 | UTF-8 |
| 单包上限 | 建议 ≤ 512 字节（Thread 6LoWPAN MTU 1280 字节内均安全） |
| UART 格式 | JSON 字符串 + `\n`（LF），无 BOM |

---

## 3. 通用包结构

所有消息共享相同的顶层字段：

```json
{
  "ver"     : 1,
  "type"    : "<消息类型>",
  "dev_type": "<设备类型>",
  "dev_name": "<预烧录设备名>",
  "dev_id"  : "AABBCC",
  "ip"      : "<ML-EID IPv6 字符串>",
  "rloc16"  : 1025,
  "seq"     : 42,
  "data"    : { }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `ver` | int | 是 | 协议版本，当前固定为 `1` |
| `type` | string | 是 | 见第 4 节 |
| `dev_type` | string | 是 | 见第 5 节 |
| `dev_name` | string | 是 | 出厂预烧录，全网唯一标识 |
| `dev_id` | string | 是 | 设备 MAC 地址末 6 位（大写十六进制），用于唯一识别硬件 |
| `ip` | string | child→leader 必填 | 发送方 ML-EID |
| `rloc16` | int | child→leader 必填 | 发送方 RLOC16（十进制） |
| `seq` | int | 是 | 单调递增序列号，ACK 用于匹配 |
| `data` | object | 是 | 见第 6 节 |

> `ip` 和 `rloc16` 在 leader→child 的 CONTROL 消息中可省略（leader 已知目标地址）。

---

## 4. 消息类型（`type` 字段）

| type | 方向 | 说明 | 是否需要 ACK |
|------|------|------|------------|
| `REGISTER` | child → leader | 入网注册，上报设备基本信息 | ✅ leader 回 ACK |
| `REPORT` | child → leader | 状态/数据上报（事件驱动） | ❌ |
| `CONTROL` | leader → child（经由 PC 下发） | 控制命令 | ✅ child 回 ACK |
| `ACK` | 双向 | 对 REGISTER / CONTROL 的应答 | ❌ |
| `QUERY_TABLE` | PC → leader（UART） | 请求 Leader 重播当前设备表 | ❌（leader 逐条发 REGISTER） |
| `DEV_ONLINE` | leader → PC（UART） | 设备在线/离线状态变更通知 | ❌ |

---

## 5. 设备类型（`dev_type` 字段）

| dev_type | 产品 | 支持的操作 |
|----------|------|-----------|
| `SOCKET` | 插座 | REPORT（状态变化）、CONTROL（翻转 / 强制开 / 强制关继电器） |
| `LIGHT` | 光照传感器 | REPORT（lux 值） |
| `RADAR` | 雷达 | REPORT（presence）、CONTROL（串口配置参数） |
| `RGB` | RGB 灯带 | REPORT（各通道值）、CONTROL（各通道 PWM） |

---

## 6. `data` 字段定义

### 6.1 REGISTER — 所有设备通用

```json
"data": {
  "fw_ver": "1.0.0",
  "hw_ver": "A"
}
```

### 6.2 REPORT

**SOCKET**
```json
"data": { "state": 1 }
```
`state`: `0`=off，`1`=on

**LIGHT**
```json
"data": { "lux": 312 }
```
`lux`: 光照值，单位 lux，uint32

**RADAR**
```json
"data": { "presence": 1 }
```
`presence`: `0`=无人，`1`=有人

**RGB**
```json
"data": { "r": 100, "g": 50, "b": 0 }
```
各通道 PWM 占空比，范围 `0`～`100`（百分比）

### 6.3 CONTROL

**SOCKET**
```json
"data": { "toggle": 1 }
```
`toggle`: 固定为 `1`，触发一次继电器翻转（GPIO1 取反），无论当前状态如何。

```json
"data": { "on": 1 }
```
`on`: 固定为 `1`，强制拉高 GPIO1（继电器吸合，ON）。

```json
"data": { "off": 1 }
```
`off`: 固定为 `1`，强制拉低 GPIO1（继电器断开，OFF）。

三条命令互斥，同一包只携带其中一个字段。执行完毕后子设备均回 ACK 并上报最新状态（REPORT）。

**RGB**
```json
"data": { "r": 0, "g": 0, "b": 50 }
```

**RADAR**（通过串口写入雷达模组的配置参数）
```json
"data": { "delay_s": 30, "dist_m": 5 }
```
`delay_s`: 有人离开后延迟报无人的秒数  
`dist_m`: 检测距离上限，单位米  
两个字段可同时出现，也可单独出现；同时出现时子设备串行执行，全部完成后回一次 ACK。

**RADAR**（查询当前配置参数）
```json
"data": { "get": "delay_s" }
```
`get` 支持的值：
- `"delay_s"` — 查询离开延迟
- `"dist_m"` — 查询检测距离
- `"all"` — 同时查询两项（子设备依次请求，各返回一条 REPORT）

子设备收到 GET 后**立即回 ACK**（code=0），随后通过 **REPORT** 消息携带读取到的值：
```json
"data": { "delay_s": 30 }
```
或
```json
"data": { "dist_m": 5 }
```

**LIGHT**：只读设备，不接受 CONTROL

### 6.4 ACK

```json
"data": {
  "seq_ack": 42,
  "code"   : 0,
  "msg"    : "ok"
}
```

| `code` | 含义 |
|--------|------|
| `0` | 成功 |
| `1` | 未知命令 |
| `2` | 执行失败 |

### 6.5 QUERY_TABLE

PC 发往 Leader，`data` 字段为空对象（或省略）：

```json
{"ver":1,"type":"QUERY_TABLE"}
```

Leader 收到后，针对设备表中每一条有效条目，向 UART0 发送一条 `REGISTER` 格式 JSON（字段含义与第 6.1 节相同，`seq` 固定为 `0` 表示重播）。若设备表为空则无任何回复。

### 6.6 DEV_ONLINE

由 Leader 主动发往 PC（UART0），用于通知设备在线状态变更，共两种触发场景：

| 触发条件 | `online` 值 |
|----------|-------------|
| OpenThread 邻居表 `CHILD_REMOVED`（Leader 检测到 child 离开，如 MLE 超时 / 主动 detach） | `0`（离线） |
| 收到 REGISTER 且设备表中已存在该设备的注册信息（重新入网） | `1`（上线） |

```json
"data": {
  "online": 0
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `online` | int | `1` = 设备上线，`0` = 设备离线 |

> `dev_type`、`dev_name`、`dev_id` 字段照常填写，PC 可据此定位到对应设备条目并更新在线状态。  
> 此消息无需 ACK，PC 侧按最新状态覆盖即可。

---

## 7. 完整消息示例

### 雷达入网注册（child → leader）
```json
{"ver":1,"type":"REGISTER","dev_type":"RADAR","dev_name":"radar_01","dev_id":"AABBCC","ip":"fd11:ab::3","rloc16":1025,"seq":1,"data":{"fw_ver":"1.0.0","hw_ver":"A"}}
```

### Leader 回 ACK（leader → child）
```json
{"ver":1,"type":"ACK","dev_type":"RADAR","dev_name":"radar_01","seq":2,"data":{"seq_ack":1,"code":0,"msg":"ok"}}
```

### 雷达上报有人（child → leader → PC）
```json
{"ver":1,"type":"REPORT","dev_type":"RADAR","dev_name":"radar_01","ip":"fd11:ab::3","rloc16":1025,"seq":3,"data":{"presence":1}}
```

### 插座上报继电器状态（child → leader → PC）
```json
{"ver":1,"type":"REPORT","dev_type":"SOCKET","dev_name":"socket_01","ip":"fd11:ab::4","rloc16":2049,"seq":5,"data":{"state":1}}
```
`state`: `0`=断开（OFF），`1`=吸合（ON）；由 GPIO1 电平变化触发。

### PC 下发插座翻转（PC → leader UART → child UDP）
```json
{"ver":1,"type":"CONTROL","dev_type":"SOCKET","dev_name":"socket_01","seq":10,"data":{"toggle":1}}
```

### PC 下发插座强制开（PC → leader UART → child UDP）
```json
{"ver":1,"type":"CONTROL","dev_type":"SOCKET","dev_name":"socket_01","seq":11,"data":{"on":1}}
```

### PC 下发插座强制关（PC → leader UART → child UDP）
```json
{"ver":1,"type":"CONTROL","dev_type":"SOCKET","dev_name":"socket_01","seq":12,"data":{"off":1}}
```

### 灯带调色（PC → leader → child）
```json
{"ver":1,"type":"CONTROL","dev_type":"RGBCW","dev_name":"rgbcw_01","seq":13,"data":{"r":0,"g":0,"b":50}}
```

### 雷达配置参数下发（PC → leader → child）
```json
{"ver":1,"type":"CONTROL","dev_type":"RADAR","dev_name":"radar_01","seq":14,"data":{"delay_s":30,"dist_m":5}}
```

### 雷达查询离开延迟（PC → leader → child）
```json
{"ver":1,"type":"CONTROL","dev_type":"RADAR","dev_name":"radar_01","seq":15,"data":{"get":"delay_s"}}
```

### Leader 上报设备离线（leader → PC UART）
```json
{"ver":1,"type":"DEV_ONLINE","dev_type":"RADAR","dev_name":"radar_01","dev_id":"AABBCC","seq":20,"data":{"online":0}}
```
Leader 检测到 radar_01 从 Thread 邻居表移除（CHILD_REMOVED），向 PC 上报离线状态。

### Leader 上报设备重新上线（leader → PC UART）
```json
{"ver":1,"type":"DEV_ONLINE","dev_type":"RADAR","dev_name":"radar_01","dev_id":"AABBCC","seq":21,"data":{"online":1}}
```
radar_01 重新入网并发送 REGISTER，Leader 发现设备表中已有该条目，向 PC 上报上线状态。

### PC 请求设备表（PC → leader UART）
```json
{"ver":1,"type":"QUERY_TABLE"}
```

### Leader 逐条重播 REGISTER（leader → PC UART，每条设备发一行）
```json
{"ver":1,"type":"REGISTER","dev_type":"RADAR","dev_name":"radar_01","dev_id":"AABBCC","ip":"fd11:ab::3","rloc16":1025,"seq":0,"data":{"fw_ver":"1.0.0","hw_ver":"A"}}
{"ver":1,"type":"REGISTER","dev_type":"SOCKET","dev_name":"socket_01","dev_id":"DDEEFF","ip":"fd11:ab::4","rloc16":2049,"seq":0,"data":{"fw_ver":"1.0.0","hw_ver":"A"}}
```

### 子设备立即回 ACK（child → leader → PC）
```json
{"ver":1,"type":"ACK","dev_type":"RADAR","dev_name":"radar_01","seq":5,"data":{"seq_ack":13,"code":0,"msg":"ok"}}
```

### 子设备 REPORT 查询结果（child → leader → PC）
```json
{"ver":1,"type":"REPORT","dev_type":"RADAR","dev_name":"radar_01","ip":"fd11:ab::3","rloc16":1025,"seq":6,"data":{"delay_s":30}}
```

---

## 8. 消息流程图

```
子设备入网（首次注册）
  Child ──[REGISTER]──► Leader  新建设备路由表条目
  Leader ──[ACK]──────► Child
  Leader ──[REGISTER + '\n']──► PC UART0  （PC 新增设备）

子设备重新入网（已有记录）
  Child ──[REGISTER]──► Leader  命中已有条目，刷新 ip/rloc16/last_seen
  Leader ──[ACK]──────► Child
  Leader ──[DEV_ONLINE online=1 + '\n']──► PC UART0  （PC 更新在线状态）

子设备离线
  OpenThread 检测到 child 离开（MLE 超时 / detach）
  Leader ──[DEV_ONLINE online=0 + '\n']──► PC UART0  （PC 更新离线状态）

子设备状态变化
  Child ──[REPORT]──► Leader  更新状态缓存
  Leader ──[REPORT + '\n']──► PC UART0

PC 发出联动控制
  PC ──[CONTROL + '\n']──► Leader UART0
  Leader 查设备路由表找目标 IP
  Leader ──[CONTROL]──► Child UDP
  Child 执行 → Child ──[ACK]──► Leader
  Leader ──[ACK + '\n']──► PC UART0

PC 请求设备表
  PC ──[QUERY_TABLE + '\n']──► Leader UART0
  Leader 遍历设备表，逐条发送 REGISTER + '\n'──► PC UART0
```

---

## 9. Leader 设备路由表结构体（`app_device_table.h`）

```c
typedef enum {
    MIU_DEV_TYPE_SOCKET  = 0,
    MIU_DEV_TYPE_LIGHT   = 1,
    MIU_DEV_TYPE_RADAR   = 2,
    MIU_DEV_TYPE_RGBCW   = 3,
    MIU_DEV_TYPE_UNKNOWN = 0xFF,
} miu_dev_type_t;

typedef struct {
    bool             valid;
    miu_dev_type_t   dev_type;
    char             dev_name[32];   /* 预烧录，全网唯一 */
    char             dev_id[7];      /* MAC 末 6 位，大写十六进制，如 "AABBCC" */
    otIp6Address     ip;             /* ML-EID，UDP 路由目标 */
    uint16_t         rloc16;
    char             fw_ver[16];
    char             hw_ver[8];
    uint32_t         last_seen_ms;   /* FreeRTOS tick ms */
    bool             online;         /* OT CHILD_REMOVED → false；重新 REGISTER → true */
    union {
        struct { uint8_t  state;               } socket;
        struct { uint32_t lux;                 } light;
        struct { uint8_t  presence;            } radar;
        struct { uint8_t  r, g, b, c, w;      } rgbcw;
    } data;
} miu_device_info_t;

#define MIU_MAX_DEVICES 16

typedef struct {
    miu_device_info_t devices[MIU_MAX_DEVICES];
    uint8_t           count;
} miu_device_table_t;
```

---

## 10. 新增文件清单

| 文件 | 位置 | 说明 |
|------|------|------|
| `miu_json.h` | `miu-common/include/` | 轻量 JSON 字段提取（static inline，无外部依赖） |
| `app_device_table.h/.c` | `miu-ftd/include,src/` | Leader 设备路由表 |
| `app_uart_pc.h/.c` | `miu-ftd/include,src/` | UART0 ↔ PC JSON 桥接 |

### 修改的文件

| 文件 | 改动摘要 |
|------|---------|
| `miu-common/include/app_uart.h` | 新增 `app_uart0_rx_read()` 公开声明 |
| `miu-common/src/app_uart.c` | `uart0_rx_read` 改为公开；FTD 模式下调用 `app_uart_pc_json_recv()` |
| `miu-common/include/app_protocol.h` | 新增 `app_udp_comm_json_process()` 声明 |
| `miu-common/src/app_udp.c` | 增加 JSON dispatch 分支（首字节 `{`） |
| `miu-common/src/app_udp_comm.c` | FTD 实现 `app_udp_comm_json_process()`；MTD 弱符号占位 |
| `miu-ftd/src/app_net_mgm.c` | Leader 初始化时调用 `app_device_table_init()` + `app_uart_pc_init()` |
| `miu-ftd/CMakeLists.txt` | 加入新源文件；禁用 OTA；开启 `CONFIG_APP_UART_PC_ENABLE` |
