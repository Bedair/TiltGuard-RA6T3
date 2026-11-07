/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_i3c.h"
#include "r_gpt.h"
#include "r_timer_api.h"
FSP_HEADER
/** I3C on I3C Instance. */
extern const i3c_instance_t g_i3c_imu;

/** Access the I3C instance using these structures when calling API functions directly (::p_api is not used). */
extern i3c_instance_ctrl_t g_i3c_imu_ctrl;
extern const i3c_cfg_t g_i3c_imu_cfg;

#ifndef mpu6050_i3c_callback
void mpu6050_i3c_callback(i3c_callback_args_t const *const p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer_control_tick;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer_control_tick_ctrl;
extern const timer_cfg_t g_timer_control_tick_cfg;

#ifndef control_tick_isr
void control_tick_isr(timer_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
