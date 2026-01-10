#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void cmd_sens_set(float x, float y, float z);
void cmd_sens_reset(void);

void cmd_reset_zro(void);
void cmd_reset_acc(void);
void cmd_reset_bat(void);
void cmd_reset_tcal(void);

void cmd_ping_start(void);

#ifdef __cplusplus
}
#endif
