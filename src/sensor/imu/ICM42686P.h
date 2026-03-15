#ifndef ICM42686P_h
#define ICM42686P_h

#include "sensor/sensor.h"

// https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf
// ICM42686P is similar to ICM42688 but with different WHO_AM_I value
// Register map is compatible with ICM42688

// User Bank 0
#define ICM42686P_DEVICE_CONFIG             0x11
#define ICM42686P_FIFO_CONFIG               0x16

#define ICM42686P_TEMP_DATA1                0x1D
#define ICM42686P_ACCEL_DATA_X1             0x1F
#define ICM42686P_GYRO_DATA_X1              0x25

#define ICM42686P_INT_STATUS                0x2D

#define ICM42686P_FIFO_COUNTH               0x2E
#define ICM42686P_FIFO_DATA                 0x30

#define ICM42686P_INTF_CONFIG0              0x4C
#define ICM42686P_INTF_CONFIG1              0x4D

#define ICM42686P_PWR_MGMT0                 0x4E

#define ICM42686P_GYRO_CONFIG0              0x4F
#define ICM42686P_ACCEL_CONFIG0             0x50
#define ICM42686P_GYRO_ACCEL_CONFIG0        0x52

#define ICM42686P_SMD_CONFIG                0x57

#define ICM42686P_FIFO_CONFIG1              0x5F
#define ICM42686P_FIFO_CONFIG2              0x60

#define ICM42686P_INT_SOURCE0               0x65
#define ICM42686P_INT_SOURCE1               0x66

#define ICM42686P_REG_BANK_SEL              0x76
#define ICM42686P_WHO_AM_I                  0x75

// User Bank 1
#define ICM42686P_INTF_CONFIG5              0x7B

// User Bank 4
#define ICM42686P_ACCEL_WOM_X_THR           0x4A
#define ICM42686P_ACCEL_WOM_Y_THR           0x4B
#define ICM42686P_ACCEL_WOM_Z_THR           0x4C

#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00 // default

#define GFS_2000DPS   0x00   // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_50DPS  0x05
#define GFS_31_25DPS  0x06
#define GFS_15_625DPS 0x07

// Low Noise mode
#define AODR_32kHz    0x01   
#define AODR_16kHz    0x02
#define AODR_8kHz     0x03
#define AODR_4kHz     0x04
#define AODR_2kHz     0x05
#define AODR_1kHz     0x06  // default
//Low Noise or Low Power modes
#define AODR_500Hz    0x0F
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
// Low Power mode
#define AODR_6_25Hz   0x0C  
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E

#define GODR_32kHz  0x01   
#define GODR_16kHz  0x02
#define GODR_8kHz   0x03
#define GODR_4kHz   0x04
#define GODR_2kHz   0x05
#define GODR_1kHz   0x06 // default
#define GODR_500Hz  0x0F
#define GODR_200Hz  0x07
#define GODR_100Hz  0x08
#define GODR_50Hz   0x09
#define GODR_25Hz   0x0A
#define GODR_12_5Hz 0x0B

#define aMode_OFF 0x01
#define aMode_LP  0x02
#define aMode_LN  0x03

#define gMode_OFF 0x00
#define gMode_SBY 0x01
#define gMode_LN  0x03

int icm42686p_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);
void icm42686p_shutdown(void);

void icm42686p_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range);
int icm42686p_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time);

uint16_t icm42686p_fifo_read(uint8_t *data, uint16_t len);
int icm42686p_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void icm42686p_accel_read(float a[3]);
void icm42686p_gyro_read(float g[3]);
float icm42686p_temp_read(void);

uint8_t icm42686p_setup_DRDY(uint16_t threshold);
uint8_t icm42686p_setup_WOM(void);

extern const sensor_imu_t sensor_imu_icm42686p;

#endif

