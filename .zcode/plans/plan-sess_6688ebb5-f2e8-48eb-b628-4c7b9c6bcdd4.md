## QMC5883P 磁力计驱动移植计划

**目标**：在 `5883p-test-branch` 上新增 QMC5883P 驱动（QMC5883L 已停产，P 是其 pin-to-pin 继任者但寄存器不兼容），完全遵循仓库现有的自定义 vtable + 运行时扫描框架，与现有 QMC5883L 驱动并存互不影响（地址 0x2C vs 0x0D，天然区分）。模板选 `QMC6309.c`（寄存器风格与 P 最接近）。

### 第 0 步：提取规格书核对寄存器（实现前必须做）
计划模式拦截了 PDF 提取，退出后先用 `pdftotext -enc UTF-8 <PDF> -` 完整提取两份规格书：
- `E:\download\C2847467_3D磁传感器_QMC5883P_规格书_WJ90930.PDF`（主要依据）
- `E:\download\C976032_3D磁传感器_QMC5883L_规格书_WJ314275.PDF`（对照）
逐项核对（目前仅有二手对比报告，须以原规格书为准）：I2C 地址 0x2C、CHIPID（reg 0x00=0x80）、数据寄存器 0x01–0x06 小端补码、STATUS 0x09（DRDY bit0/OVFL bit1）、CTRL1 0x0A（MD/OSR1/LPF）与 CTRL2 0x0B（SET/RNG/ODR/SRT）的精确位域、量程码与灵敏度（预期 ±2/8/12/30G = 15000/3750/2500/1000 LSB/G）、ODR 码、模式切换是否必须经 suspend、软复位方式、上电时序。如与报告冲突以规格书为准。

### 第 1 步：新驱动 `src/sensor/mag/QMC5883P.c` + `.h`
照 QMC6309.c 的结构与代码风格（文件级静态状态、`last_config_code` 防重写、STATUS DRDY/OVL 门控、6 字节突发读、字节级去重启发式、经 IMU 外部代理时跳过独立 STATUS 读）：
- 寄存器 #define 按核对结果写；量程默认固定 **±8G**（3750 LSB/G，`sensitivity = 1/3750`，与现有 QMC 系干扰余量一致，注释说明 ±2G 可选）
- `init`：复位/模式初始化 → `update_odr`（period_s ≤0/INFINITY → suspend，否则取 10/50/100/200Hz 档）
- `shutdown`：按规格书写 suspend 或软复位（同 QMC6309 做法）
- `mag_process`：小端拼 16 位 → `int16_t` → 乘灵敏度得高斯
- vtable：`{init, shutdown, update_odr, oneshot, read, mag_none_temp_read, process, 6, 6}`（P 无温度输出寄存器，挂 none）

### 第 2 步：`src/sensor/sensors_enum.h`
- 顶部"芯片知识库"注释 QST 段加一行 `-QMC5883P:2C,00,80`
- `enum dev_mag` 在 `MAG_QMC5883L` 之后插入 `MAG_QMC5883P`（枚举值不落盘持久化，插中间安全；`SENSOR_DEV_MAG_COUNT = MAG_ICT153XX + 1` 自动 +1）

### 第 3 步：`src/sensor/sensors_tables.c`（扫描接线）
- `SENSOR_MAG_DRV_COUNT` 加 `SLIME_DRV1(CONFIG_SENSOR_DRV_QMC5883P)`
- `dev_mag_names[]`、`sensor_mags[]` 在 QMC5883L 之后对应位置插入（`#if IS_ENABLED(...)` → `&sensor_mag_qmc5883p`，否则 `&sensor_mag_none`）
- 新增地址组 **G11**（0x2C 不属于现有 G0–G10 任何组）：`#define SLIME_MAG_G11 (IS_ENABLED(CONFIG_SENSOR_DRV_QMC5883P))`；四张表各加 `#if SLIME_MAG_G11` 段——addr: `1, 0x2C`；reg: `1, 0x00`；id: `1, 0x80`；dev: `MAG_QMC5883P`；`i2c_dev_mag_addr_count` 追加 `+ SLIME_MAG_G11`

### 第 4 步：`Kconfig.sensors_drivers`
Magnetometer drivers 菜单内加 `config SENSOR_DRV_QMC5883P`（`default y if !SENSOR_MAG_DRIVERS_MINIMAL`，`default n`）。

### 第 5 步：`CMakeLists.txt`
照 QMC5883L 的写法（214–216 行）加 `if(CONFIG_SENSOR_DRV_QMC5883P) target_sources(app PRIVATE src/sensor/mag/QMC5883P.c)`。

### 第 6 步：主机测试 `tests/host/sensor_drivers/test_mag_failures.c`
include 新头文件，照 `qmc5883_writes`/条目 302 的格式加 QMC5883P 的期望写序列与驱动清单条目，覆盖超时/触发/突发失败不发布等既有用例模式。

### 第 7 步：验证
- `west build -b nini_slimevr_mag_uf2/nrf52833`（本分支主力板）编译通过；再跑 `tests/host`（Makefile）通过
- 实机验证留给用户硬件：上电日志应出现 `Valid device found at address: 0x2C (register: 0x00, value: 0x80)`，Web 控制台可见 mag 采样
- 不改任何板级 defconfig（NiNi 板未启用 MINIMAL 白名单，默认全编译；启用 MINIMAL 的 3 块板也没白名单 QMC5883L，保持一致）

**不做的事**：不改 QMC5883L 现有驱动；不改融合/校准/轴对齐层（新驱动输出与其它 mag 同为单位为高斯的三轴向量，直接进既有数据流）；不动 devicetree。