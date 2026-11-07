/* generated common source file - do not edit */
#include "common_data.h"
icu_instance_ctrl_t g_irq_echo_ctrl;

/** External IRQ extended configuration for ICU HAL driver */
const icu_extended_cfg_t g_irq_echo_ext_cfg =
{ .filter_src = EXTERNAL_IRQ_DIGITAL_FILTER_PCLK_DIV, };

const external_irq_cfg_t g_irq_echo_cfg =
{ .channel = 5, .trigger = EXTERNAL_IRQ_TRIG_BOTH_EDGE, .filter_enable = false, .clock_source_div =
          EXTERNAL_IRQ_CLOCK_SOURCE_DIV_64,
  .p_callback = ultrasonic_echo_isr,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = (void*) &g_irq_echo_ext_cfg,
  .ipl = (4),
#if defined(VECTOR_NUMBER_ICU_IRQ5)
    .irq                 = VECTOR_NUMBER_ICU_IRQ5,
#else
  .irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const external_irq_instance_t g_irq_echo =
{ .p_ctrl = &g_irq_echo_ctrl, .p_cfg = &g_irq_echo_cfg, .p_api = &g_external_irq_on_icu };
icu_instance_ctrl_t g_irq_encoder_right_ctrl;

/** External IRQ extended configuration for ICU HAL driver */
const icu_extended_cfg_t g_irq_encoder_right_ext_cfg =
{ .filter_src = EXTERNAL_IRQ_DIGITAL_FILTER_PCLK_DIV, };

const external_irq_cfg_t g_irq_encoder_right_cfg =
{ .channel = 1, .trigger = EXTERNAL_IRQ_TRIG_BOTH_EDGE, .filter_enable = false, .clock_source_div =
          EXTERNAL_IRQ_CLOCK_SOURCE_DIV_64,
  .p_callback = encoder_right_isr,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = (void*) &g_irq_encoder_right_ext_cfg,
  .ipl = (3),
#if defined(VECTOR_NUMBER_ICU_IRQ1)
    .irq                 = VECTOR_NUMBER_ICU_IRQ1,
#else
  .irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const external_irq_instance_t g_irq_encoder_right =
{ .p_ctrl = &g_irq_encoder_right_ctrl, .p_cfg = &g_irq_encoder_right_cfg, .p_api = &g_external_irq_on_icu };
icu_instance_ctrl_t g_irq_encoder_left_ctrl;

/** External IRQ extended configuration for ICU HAL driver */
const icu_extended_cfg_t g_irq_encoder_left_ext_cfg =
{ .filter_src = EXTERNAL_IRQ_DIGITAL_FILTER_PCLK_DIV, };

const external_irq_cfg_t g_irq_encoder_left_cfg =
{ .channel = 7, .trigger = EXTERNAL_IRQ_TRIG_BOTH_EDGE, .filter_enable = false, .clock_source_div =
          EXTERNAL_IRQ_CLOCK_SOURCE_DIV_64,
  .p_callback = encoder_left_isr,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = (void*) &g_irq_encoder_left_ext_cfg,
  .ipl = (3),
#if defined(VECTOR_NUMBER_ICU_IRQ7)
    .irq                 = VECTOR_NUMBER_ICU_IRQ7,
#else
  .irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const external_irq_instance_t g_irq_encoder_left =
{ .p_ctrl = &g_irq_encoder_left_ctrl, .p_cfg = &g_irq_encoder_left_cfg, .p_api = &g_external_irq_on_icu };
ioport_instance_ctrl_t g_ioport_ctrl;
const ioport_instance_t g_ioport =
{ .p_api = &g_ioport_on_ioport, .p_ctrl = &g_ioport_ctrl, .p_cfg = &g_bsp_pin_cfg, };
void g_common_init(void)
{
}
