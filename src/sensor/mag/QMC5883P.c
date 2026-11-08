#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "QMC5883P.h"
#include "sensor/sensor_none.h"

// QMC5883P Rev E（QST-PD-B002-22）。接口骨架与 QMC6309.c 同源，差异见 QMC5883P.h 头注。

#define QMC5883P_CHIPID_REG 0x00 // = 0x80

#define QMC5883P_OUTX_L_REG 0x01

// Status register {DRDY:1;OVFL:1;...}（与 6309 同风格）
#define QMC5883P_STAT_REG 0x09

#define STAT_DATA_RDY_MASK 0b01
#define STAT_OVERFLOW_MASK 0b10

// Control register 1 {OSR2:2;OSR1:2;ODR:2;MODE:2} —— P 特有布局：ODR 在这里
#define QMC5883P_CTRL_REG_1 0x0A

#define MODE_SUSPEND 0b00
#define MODE_NORMAL 0b01
#define MODE_SINGLE 0b10
#define MODE_CONTINUOUS 0b11
#define MODE_MASK 0b11

// OSR1 过采样（位 [5:4]）：8x=00（噪声换带宽：磁融合 tau 秒级，取最高过采样）
#define OSR1_8X 0b00
// OSR2 降采样抗混叠（位 [7:6]）：÷8=11（datasheet 示例 0xCD 同款）
#define OSR2_DIV8 0b11
#define OSR_MASK (OSR2_DIV8 << 6 | OSR1_8X << 4)

// ODR（位 [3:2]，2 位档）：10/50/100/200Hz。我方工作点 100Hz（对齐
// CONFIG_SENSOR_MAG_ODR=100；200Hz 徒增功耗——Normal 模式电流随 ODR 上涨）
#define ODR_10Hz 0b00
#define ODR_50Hz 0b01
#define ODR_100Hz 0b10
#define ODR_200Hz 0b11
#define ODR_MASK(odr) ((odr) << 2)

// Control register 2 {SOFT_RST:1;SELF_TEST:1;-:1;-:1;INT_ENB:1;RNG:2;SET_RST:2}
#define QMC5883P_CTRL_REG_2 0x0B

#define SET_RESET_ON 0b00 // 测量中周期性 SET/RESET 消桥路零漂/温漂——勿为省电关闭
#define RNG_8G 0b10       // 8G 档（地磁 0.25-0.65G + 身体佩戴硬磁偏移，余量与灵敏度平衡）
#define RNG_MASK(rng) ((rng) << 2)
// CTRL2 常规值 = RNG_8G<<2 | SET_RESET_ON = 0x08（golden value，勿写 L 的 0x41/0x01）
#define CTRL2_VALUE (RNG_MASK(RNG_8G) | SET_RESET_ON)

#define SOFT_RESET_MASK 0x80
#define SOFT_RESET_CLEAR 0x00
#define SELF_TEST_MASK 0x40 // 自测位：置 1 等 5ms 读普通数据寄存器（不在 init 使用）

// 符号寄存器（官方上电序列 0x29=0x06；且每次进 Normal/Continuous 前重写一次，
// 借鉴社区实现 darkk-knightt 的低成本保险，防模式切换后极性回翻）
#define QMC5883P_SIGN_REG 0x29
#define SIGN_VALUE 0x06

// 8G 档 3750 LSB/G（P 与 6309 的 4000 不同！）
static const float sensitivity = 1 / 3750.0f;

// OVFL 软件判据：|raw| >= 30000 判饱和（与 STATUS OVFL 位双保险；报错丢弃、
// 绝不自动扩量程——VR 校准的 LS 拟合与 VQF 磁场基准不能中途换灵敏度）
#define MAG_SATURATION_LSB 30000

static uint8_t last_state = 0xff;
static bool last_overflow = false;
static int64_t oneshot_trigger_ms = 0;
static bool oneshot_pending;
static bool oneshot_failed;
// Normal 模式数据寄存器保持上一样本；字节全同 + 周期窗口判重复（同 6309 启发式）
static uint8_t last_raw_sample[6];
static bool last_raw_sample_valid = false;
static int64_t last_mag_time_ms;
static int32_t mag_period_ms = 10; // default 100Hz

LOG_MODULE_REGISTER(QMC5883P, LOG_LEVEL_INF);

int qmc5883p_init(float period_s, float *actual_period_s)
{
	last_state = 0xff; // init state
	last_overflow = false;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	last_mag_time_ms = 0;
	// 官方上电序列：0x29=0x06 → 0x0B=0x08 → 0x0A 配置（后两步在 update_odr 内完成）
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_SIGN_REG, SIGN_VALUE);
	if (err) {
		LOG_ERR("Communication error");
		return err;
	}
	err = qmc5883p_update_odr(period_s, actual_period_s);
	return (err < 0 ? err : 0);
}

void qmc5883p_shutdown(void)
{
	last_state = 0xff;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;
	// 软复位（0x80）后保守补写 0x00 —— 复位位自清但官方序列建议显式回写
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, SOFT_RESET_MASK);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL_REG_2, SOFT_RESET_CLEAR);
	if (err) {
		LOG_ERR("Communication error");
	}
}

int qmc5883p_update_odr(float period_s, float *actual_period_s)
{
	int requested_odr_hz; // Truncate fractional Hz before selecting a supported rate.
	uint8_t odr_code;
	uint8_t mode_code;

	if (period_s <= 0 || period_s == INFINITY) // suspend / oneshot mode
	{
		mode_code = MODE_SUSPEND; // oneshot will set SINGLE after
		requested_odr_hz = 0;
	} else {
		mode_code = MODE_NORMAL;
		requested_odr_hz = 1 / period_s;
	}

	if (mode_code == MODE_SUSPEND) {
		odr_code = ODR_200Hz; // for oneshot
		period_s = INFINITY;  // signal oneshot mode available
	} else if (requested_odr_hz > 100) {
		odr_code = ODR_200Hz;
		period_s = 1.f / 200;
	} else if (requested_odr_hz > 50) {
		odr_code = ODR_100Hz;
		period_s = 1.f / 100;
	} else if (requested_odr_hz > 10) {
		odr_code = ODR_50Hz;
		period_s = 1.f / 50;
	} else {
		odr_code = ODR_10Hz;
		period_s = 1.f / 10;
	}

	uint8_t config_code = ODR_MASK(odr_code) | mode_code;
	if (last_state == config_code) {
		*actual_period_s = period_s;
		return 0; /* already configured — success for err|= callers */
	}

	// P 特有约束：活动模式互切必须经 Suspend 中转（先写 MODE=00，再写目标模式）
	int err = 0;
	if (last_state != 0xff && (last_state & MODE_MASK) != MODE_SUSPEND
		&& mode_code != MODE_SUSPEND) {
		err = ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_MAG,
			QMC5883P_CTRL_REG_1,
			(last_state & ~MODE_MASK) | MODE_SUSPEND);
	}

	if (!err) {
		// 每次（重）进 Normal 前重写符号寄存器（0x29=0x06）——社区实测的防回翻保险
		err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_SIGN_REG, SIGN_VALUE);
	}
	if (!err) {
		err = ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_MAG,
			QMC5883P_CTRL_REG_2,
			CTRL2_VALUE);
	}
	if (!err) {
		// 我方 100Hz 工作点：OSR_MASK|ODR_100Hz<<2|MODE_NORMAL = 1100 1001b = 0xC9
		// （golden value；datasheet 示例 0xCD 为 200Hz 档，单测两者都断言）
		err = ssi_reg_write_byte(
			SENSOR_INTERFACE_DEV_MAG,
			QMC5883P_CTRL_REG_1,
			OSR_MASK | config_code);
	}
	if (err) {
		LOG_ERR("Communication error");
		last_state = 0xff;
		return err;
	}

	last_state = config_code;
	oneshot_trigger_ms = 0;
	oneshot_pending = false;
	oneshot_failed = false;
	last_raw_sample_valid = false;

	if (mode_code != MODE_SUSPEND) {
		mag_period_ms = (int32_t)(period_s * 1000);
	}

	*actual_period_s = period_s;
	return 0;
}

void qmc5883p_mag_oneshot(void)
{
	// oneshot 由 Suspend 态发起（P 的 Single 测完自动回 Suspend，无需中转）
	int err = ssi_reg_write_byte(
		SENSOR_INTERFACE_DEV_MAG,
		QMC5883P_CTRL_REG_1,
		OSR_MASK | ODR_MASK(ODR_200Hz) | MODE_SINGLE);
	last_state = 0xff;
	oneshot_failed = err != 0;
	oneshot_pending = true;
	oneshot_trigger_ms = k_uptime_get();
	if (err) {
		LOG_ERR("Communication error");
	}
}

bool qmc5883p_mag_read(float m[3])
{
	bool was_oneshot = oneshot_pending;
	if (oneshot_pending) {
		if (oneshot_failed) {
			oneshot_pending = false;
			oneshot_failed = false;
			return false;
		}

		// Oneshot mode: wait for DRDY with timeout
		uint8_t status = 0;
		int64_t deadline_ms = oneshot_trigger_ms + 10; // 10ms timeout
		while ((status & STAT_DATA_RDY_MASK) == 0) {
			int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &status);
			if (err) {
				LOG_ERR("Communication error");
				oneshot_pending = false;
				return false;
			}
			if (k_uptime_get() >= deadline_ms) {
				LOG_WRN("Data ready status timeout!");
				oneshot_pending = false;
				return false;
			}
		}
		oneshot_trigger_ms = 0;
		oneshot_pending = false;
		if (status & STAT_OVERFLOW_MASK) {
			if (!last_overflow) {
				LOG_INF("Magnetometer overflow");
			}
			last_overflow = true;
			return false;
		}
		last_overflow = false;
	}
	if (!was_oneshot && sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG) != SENSOR_INTERFACE_SPEC_EXT) {
		uint8_t status = 0;
		int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STAT_REG, &status);
		if (err) {
			LOG_ERR("Communication error");
			return false;
		}
		if (!(status & STAT_DATA_RDY_MASK) || (status & STAT_OVERFLOW_MASK)) {
			return false;
		}
	}
	uint8_t rawData[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, QMC5883P_OUTX_L_REG, rawData, 6);
	if (err) {
		LOG_ERR("Communication error");
		return false;
	}
	// 软件饱和判据（与 STATUS OVFL 位双保险）：任一轴 |raw|>=30000 拒收。
	// 报错丢弃、不自动扩量程——扩档会中途改灵敏度，令已完成的硬/软磁校准
	// （LS 拟合）与 VQF 磁场基准瞬间失效。
	for (int i = 0; i < 3; i++) {
		uint16_t raw = ((uint16_t)rawData[i * 2 + 1] << 8) | rawData[i * 2];
		if ((int16_t)raw >= MAG_SATURATION_LSB || (int16_t)raw <= -MAG_SATURATION_LSB) {
			if (!last_overflow) {
				LOG_INF("Magnetometer saturation (|raw| >= %d)", MAG_SATURATION_LSB);
			}
			last_overflow = true;
			return false;
		}
	}
	last_overflow = false;
	// Reject byte-identical readings until the cached period plus its integer
	// 10% margin expires（同 6309 启发式：有界重复抑制）。
	int64_t now = k_uptime_get();
	if (last_raw_sample_valid && memcmp(rawData, last_raw_sample, 6) == 0) {
		if ((now - last_mag_time_ms) < (mag_period_ms + mag_period_ms / 10)) {
			return false;
		}
		// Advance one cached period; resynchronize to now only when far behind.
		last_mag_time_ms += mag_period_ms;
		if (now - last_mag_time_ms > mag_period_ms * 2) {
			last_mag_time_ms = now;
		}
	} else {
		last_mag_time_ms = now;
	}
	memcpy(last_raw_sample, rawData, 6);
	last_raw_sample_valid = true;
	qmc5883p_mag_process(rawData, m);
	return true;
}

void qmc5883p_mag_process(uint8_t *raw_m, float m[3])
{
	// 数据 0x01-0x06 小端 XYZ（显式组装，防端序坑）
	for (int i = 0; i < 3; i++) // x, y, z
	{
		uint16_t raw = ((uint16_t)raw_m[i * 2 + 1] << 8) | raw_m[i * 2];
		m[i] = (int16_t)raw;
		m[i] *= sensitivity; // Gauss（8G 档 3750 LSB/G）
	}
}

const sensor_mag_t sensor_mag_qmc5883p
	= {qmc5883p_init,
	   qmc5883p_shutdown,

	   qmc5883p_update_odr,

	   qmc5883p_mag_oneshot,
	   qmc5883p_mag_read,
	   mag_none_temp_read, // P 无温度寄存器（不参与上游 tcal 磁温补；芯片内置自动温补）

	   qmc5883p_mag_process,
	   6,
	   6};
