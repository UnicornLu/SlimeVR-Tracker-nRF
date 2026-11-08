/* QMC5883P 三轴磁力计驱动 —— 按 QST 官方 datasheet（QMC5883P QST-PD-B002-22
   Rev E，2026-09 全文取证）新写；骨架克隆自本仓 QMC6309.c（P 与 6309 的寄存器
   接口风格同源：数据小端、CHIPID@0x00、STATUS 0x09、CTRL1/CTRL2、四模式 +
   Suspend、SET/RESET 位——是代码骨架复用的依据，不是产品血缘）。

   与 QMC6309 的关键差异（改自 6309 骨架的全部落点，逐条对应 datasheet）：
     ① I2C 地址 0x2C（6309 在 0x7C/0x0C 组）；CHIPID 0x00 = 0x80（0x2C 组空闲，
        QMC6310 同 ID 0x80 但在 0x1C/0x3C 组，不冲突）。
     ② ODR 位在 CTRL1(0x0A)[3:2]（6309 在 CTRL2[6:4]），2 位：00=10/01=50/
        10=100/11=200Hz。
     ③ 8G 档灵敏度 3750 LSB/G（6309 为 4000）；RNG_8G 编码 10 与 6309 相同。
     ④ CTRL2(0x0B) 初始化 = RNG_8G<<2 | SET_RESET_ON = 0x08；官方上电序列：
        0x29=0x06（符号寄存器）→ 0x0B=0x08 → 0x0A 配置。
     ⑤ 模式切换必须经 Suspend 中转（Normal/Single/Continuous 互切先写 MODE=00；
        P 的 Single 模式测完自动回 Suspend）。
     ⑥ 自测 = CTRL2 bit6，置 1 后等 5ms 读普通数据寄存器判差值（本驱动不在
        init 里跑自测——会扰动测量；仅保留寄存器定义供调试）。

   ⚠️ 严禁照搬 QMC5883L 驱动的两处（L 与 P 寄存器完全不兼容）：
     - CTRL_2=0x41：P 的 0x0B 位义不同，bit6=1 会误触发自测、RNG=00 变 30G 档；
     - 0x0B=0x01：P 的正确值是 0x08。
   L+P 同总线可共存（各 ACK 各地址：0x0D / 0x2C）。
*/
#ifndef QMC5883P_h
#define QMC5883P_h

#include "sensor/sensor.h"

int qmc5883p_init(float period_s, float *actual_period_s);
void qmc5883p_shutdown(void);

int qmc5883p_update_odr(float period_s, float *actual_period_s);

void qmc5883p_mag_oneshot(void);
bool qmc5883p_mag_read(float m[3]);

void qmc5883p_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_qmc5883p;

#endif
