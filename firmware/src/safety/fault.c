/*
 * fault.c
 *
 * See fault.h. The handlers run in fault context: no FSP calls, no floats,
 * nothing that depends on interrupts. SEGGER RTT is plain memory writes into
 * the RTT control block, so it works here.
 */

#include <stdbool.h>
#include "fault.h"
#include "bsp_api.h"
#include "../telemetry/segger_rtt/SEGGER_RTT.h"

#define FAULT_STACK_WORDS          (256U)          /* 1 KB dedicated stack for the dump. */
#define FAULT_STACK_PAINT          (0xA5A5A5A5UL)
#define FAULT_PAINT_MARGIN_BYTES   (64U)           /* Left unpainted below the live SP at fault_init(). */
#define FAULT_STBY_PIN_MASK        (1UL << 6U)     /* P006 = STBY, low = motors off. */
#define FAULT_EXC_RETURN_SPSEL     (1UL << 2U)     /* EXC_RETURN bit 2: 1 = frame on PSP, 0 = on MSP. */
#define FAULT_BASIC_FRAME_BYTES    (32U)           /* R0-R3, R12, LR, PC, xPSR. */

/* Must match the "add r12, r12, #1024" in fault_entry(). */
#if (FAULT_STACK_WORDS * 4U) != 1024U
 #error "fault_entry() assumes a 1024-byte fault stack"
#endif

extern uint8_t g_main_stack[]; /* FSP startup.c, BSP_CFG_STACK_MAIN_BYTES long. */

/* Referenced by name from fault_entry()'s assembly, so not static. */
uint32_t g_fault_stack[FAULT_STACK_WORDS] __attribute__((aligned(8)));

void fault_entry(void) __attribute__((naked, noreturn));
void fault_dump(uint32_t msp, uint32_t psp, uint32_t exc_return, uint32_t msplim) __attribute__((noreturn, used));

/* Strong definitions override FSP's weak aliases to Default_Handler. */
void HardFault_Handler(void) __attribute__((alias("fault_entry"), noreturn));
void MemManage_Handler(void) __attribute__((alias("fault_entry"), noreturn));
void BusFault_Handler(void) __attribute__((alias("fault_entry"), noreturn));
void UsageFault_Handler(void) __attribute__((alias("fault_entry"), noreturn));

typedef struct st_fault_flag
{
    uint32_t     mask;
    const char * p_name;
} fault_flag_t;

static const fault_flag_t s_cfsr_flags[] =
{
    { (1UL << 0U),  "IACCVIOL"    },
    { (1UL << 1U),  "DACCVIOL"    },
    { (1UL << 3U),  "MUNSTKERR"   },
    { (1UL << 4U),  "MSTKERR"     },
    { (1UL << 5U),  "MLSPERR"     },
    { (1UL << 7U),  "MMARVALID"   },
    { (1UL << 8U),  "IBUSERR"     },
    { (1UL << 9U),  "PRECISERR"   },
    { (1UL << 10U), "IMPRECISERR" },
    { (1UL << 11U), "UNSTKERR"    },
    { (1UL << 12U), "STKERR"      },
    { (1UL << 13U), "LSPERR"      },
    { (1UL << 15U), "BFARVALID"   },
    { (1UL << 16U), "UNDEFINSTR"  },
    { (1UL << 17U), "INVSTATE"    },
    { (1UL << 18U), "INVPC"       },
    { (1UL << 19U), "NOCP"        },
    { (1UL << 20U), "STKOF"       },
    { (1UL << 24U), "UNALIGNED"   },
    { (1UL << 25U), "DIVBYZERO"   },
};

static const fault_flag_t s_hfsr_flags[] =
{
    { (1UL << 1U),  "VECTTBL"  },
    { (1UL << 30U), "FORCED"   },
    { (1UL << 31U), "DEBUGEVT" },
};

/*
 * Entry for all four fault vectors. Naked: no prologue, so nothing is pushed
 * before MSP is moved to the fault stack - essential after a stack overflow,
 * where MSP sits exactly at MSPLIM and any push would fault again.
 *   r0 = MSP, r1 = PSP, r2 = EXC_RETURN, r3 = MSPLIM (as at the fault).
 */
void fault_entry (void)
{
    __asm volatile (
        "mrs   r0, msp                              \n"
        "mrs   r1, psp                              \n"
        "mov   r2, lr                               \n"
        "mrs   r3, msplim                           \n"
        "mov   r12, #0                              \n"
        "msr   msplim, r12                          \n"
        "movw  r12, #:lower16:g_fault_stack         \n"
        "movt  r12, #:upper16:g_fault_stack         \n"
        "add   r12, r12, #1024                      \n"
        "msr   msp, r12                             \n"
        "isb                                        \n"
        "b     fault_dump                           \n"
        );
}

static void fault_print_flags (const char * p_label, uint32_t value, fault_flag_t const * p_flags, uint32_t count)
{
    SEGGER_RTT_printf(0U, "  %s:", p_label);

    for (uint32_t i = 0U; i < count; i++)
    {
        if (0U != (value & p_flags[i].mask))
        {
            SEGGER_RTT_printf(0U, " %s", p_flags[i].p_name);
        }
    }

    SEGGER_RTT_printf(0U, "\r\n");
}

static const char * fault_exception_name (uint32_t exception_number)
{
    switch (exception_number)
    {
        case 3U:
        {
            return "HardFault";
        }

        case 4U:
        {
            return "MemManage";
        }

        case 5U:
        {
            return "BusFault";
        }

        case 6U:
        {
            return "UsageFault";
        }

        default:
        {
            return "other";
        }
    }
}

void fault_dump (uint32_t msp, uint32_t psp, uint32_t exc_return, uint32_t msplim)
{
    /* Safe state first: STBY low (PCNTR3.PORR, a write-only reset register -
     * no read-modify-write, no FSP). The TB6612 turns every output off. */
    R_PORT0->PCNTR3 = FAULT_STBY_PIN_MASK << 16U;

    uint32_t const exception_number = __get_IPSR() & 0x1FFU;
    uint32_t const cfsr             = SCB->CFSR;
    uint32_t const hfsr             = SCB->HFSR;
    uint32_t const mmfar            = SCB->MMFAR;
    uint32_t const bfar             = SCB->BFAR;

    bool const     on_psp   = (0U != (exc_return & FAULT_EXC_RETURN_SPSEL));
    uint32_t const frame_sp = on_psp ? psp : msp;

    uint32_t const stack_base = (uint32_t) &g_main_stack[0];
    uint32_t const stack_top  = stack_base + (uint32_t) BSP_CFG_STACK_MAIN_BYTES;

    /* After a stack-limit violation on exception entry the frame was not
     * pushed (SP was clamped to MSPLIM), so the "stacked" values are garbage.
     * Only trust the frame if it lies fully inside the main stack. */
    bool const frame_in_stack = (!on_psp) && (frame_sp >= stack_base) &&
                                ((frame_sp + FAULT_BASIC_FRAME_BYTES) <= stack_top) &&
                                (0U == (frame_sp & 3U));

    /* Never drop the dump: wait for the host if channel 0 is full. We halt
     * afterwards anyway, with the motors already off. */
    (void) SEGGER_RTT_SetFlagsUpBuffer(0U, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_printf(0U, "\r\n*** FAULT: %s (exception %u) - motors off, halted ***\r\n",
                      fault_exception_name(exception_number), (unsigned) exception_number);
    SEGGER_RTT_printf(0U, "  CFSR=0x%08X HFSR=0x%08X MMFAR=0x%08X%s BFAR=0x%08X%s\r\n",
                      (unsigned) cfsr, (unsigned) hfsr,
                      (unsigned) mmfar, (0U != (cfsr & SCB_CFSR_MMARVALID_Msk)) ? "" : "(invalid)",
                      (unsigned) bfar, (0U != (cfsr & SCB_CFSR_BFARVALID_Msk)) ? "" : "(invalid)");
    fault_print_flags("CFSR flags", cfsr, s_cfsr_flags, sizeof(s_cfsr_flags) / sizeof(s_cfsr_flags[0]));
    fault_print_flags("HFSR flags", hfsr, s_hfsr_flags, sizeof(s_hfsr_flags) / sizeof(s_hfsr_flags[0]));
    SEGGER_RTT_printf(0U, "  EXC_RETURN=0x%08X -> frame on %s, SP=0x%08X\r\n",
                      (unsigned) exc_return, on_psp ? "PSP" : "MSP", (unsigned) frame_sp);
    SEGGER_RTT_printf(0U, "  MSP=0x%08X MSPLIM=0x%08X main stack=[0x%08X..0x%08X) (%u bytes)\r\n",
                      (unsigned) msp, (unsigned) msplim, (unsigned) stack_base, (unsigned) stack_top,
                      (unsigned) BSP_CFG_STACK_MAIN_BYTES);

    if (frame_in_stack)
    {
        uint32_t const * const p_frame = (uint32_t const *) frame_sp;

        SEGGER_RTT_printf(0U, "  R0=0x%08X R1=0x%08X R2=0x%08X R3=0x%08X R12=0x%08X\r\n",
                          (unsigned) p_frame[0], (unsigned) p_frame[1], (unsigned) p_frame[2],
                          (unsigned) p_frame[3], (unsigned) p_frame[4]);
        SEGGER_RTT_printf(0U, "  stacked LR=0x%08X PC=0x%08X xPSR=0x%08X\r\n",
                          (unsigned) p_frame[5], (unsigned) p_frame[6], (unsigned) p_frame[7]);
        SEGGER_RTT_printf(0U, "  -> arm-none-eabi-addr2line -e tiltguard.elf -f 0x%08X 0x%08X\r\n",
                          (unsigned) p_frame[6], (unsigned) p_frame[5]);
    }
    else
    {
        SEGGER_RTT_printf(0U, "  stacked frame not readable (SP outside the main stack%s) - "
                              "no PC/LR/xPSR available\r\n",
                          (0U != (cfsr & SCB_CFSR_STKOF_Msk)) ? ", STKOF: frame was never pushed" : "");
    }

    if (0U != (cfsr & SCB_CFSR_STKOF_Msk))
    {
        SEGGER_RTT_printf(0U, "  => main stack overflow (MSP hit MSPLIM). Raise BSP main stack size.\r\n");
    }

    while (1)
    {
        __NOP();
    }
}

void fault_init (void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;
    __DSB();
    __ISB();

    /* Paint the unused part of the main stack (below the live SP, minus a
     * margin) so fault_stack_free_bytes() can find the high-water mark. */
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();

    uint32_t * p_word      = (uint32_t *) &g_main_stack[0];
    uint32_t const paint_end = (__get_MSP() - FAULT_PAINT_MARGIN_BYTES) & ~3UL;

    while ((uint32_t) p_word < paint_end)
    {
        *p_word = FAULT_STACK_PAINT;
        p_word++;
    }

    __set_PRIMASK(primask);
}

uint32_t fault_stack_free_bytes (void)
{
    uint32_t const *       p_word = (uint32_t const *) &g_main_stack[0];
    uint32_t const * const p_end  = (uint32_t const *) &g_main_stack[BSP_CFG_STACK_MAIN_BYTES];
    uint32_t               free_bytes = 0U;

    while ((p_word < p_end) && (FAULT_STACK_PAINT == *p_word))
    {
        free_bytes += 4U;
        p_word++;
    }

    return free_bytes;
}

uint32_t fault_stack_size_bytes (void)
{
    return (uint32_t) BSP_CFG_STACK_MAIN_BYTES;
}

uint32_t fault_get_fpccr (void)
{
    return FPU->FPCCR;
}
