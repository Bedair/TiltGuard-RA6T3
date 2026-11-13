/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_adc.h"
#include "r_adc_api.h"
#include "r_i3c.h"
FSP_HEADER
/** Timer on GPT Instance. */
extern const timer_instance_t g_pwm_motor_right;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_pwm_motor_right_ctrl;
extern const timer_cfg_t g_pwm_motor_right_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_pwm_motor_left;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_pwm_motor_left_ctrl;
extern const timer_cfg_t g_pwm_motor_left_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** ADC on ADC Instance. */
extern const adc_instance_t g_adc_battery;

/** Access the ADC instance using these structures when calling API functions directly (::p_api is not used). */
extern adc_instance_ctrl_t g_adc_battery_ctrl;
extern const adc_cfg_t g_adc_battery_cfg;
extern const adc_channel_cfg_t g_adc_battery_channel_cfg;

#ifndef battery_adc_callback
void battery_adc_callback(adc_callback_args_t *p_args);
#endif

#ifndef NULL
#define ADC_DMAC_CHANNELS_PER_BLOCK_NULL  1
#endif
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
