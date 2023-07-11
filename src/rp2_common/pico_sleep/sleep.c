/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"

#include "pico/stdlib.h"
#include "pico/sleep.h"

#include "hardware/rtc.h"
#include "hardware/pll.h"
#include "hardware/regs/clocks.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/xosc.h"
#include "hardware/rosc.h"
#include "hardware/regs/io_bank0.h"
// For __wfi
#include "hardware/sync.h"
// For scb_hw so we can enable deep sleep
#include "hardware/structs/scb.h"

// The difference between sleep and dormant is that ALL clocks are stopped in dormant mode,
// until the source (either xosc or rosc) is started again by an external event.
// In sleep mode some clocks can be left running controlled by the SLEEP_EN registers in the clocks
// block. For example you could keep clk_rtc running. Some destinations (proc0 and proc1 wakeup logic)
// can't be stopped in sleep mode otherwise there wouldn't be enough logic to wake up again.


// TODO: Optionally, memories can also be powered down.

static dormant_source_t _dormant_source;

bool dormant_source_valid(dormant_source_t dormant_source) {
    return (dormant_source == DORMANT_SOURCE_XOSC) || (dormant_source == DORMANT_SOURCE_ROSC);
}

// In order to go into dormant mode we need to be running from a stoppable clock source:
// either the xosc or rosc with no PLLs running. This means we disable the USB and ADC clocks
// and all PLLs
void sleep_run_from_dormant_source(dormant_source_t dormant_source) {
    assert(dormant_source_valid(dormant_source));
    _dormant_source = dormant_source;

    // FIXME: Just defining average rosc freq here.
    uint src_hz = (dormant_source == DORMANT_SOURCE_XOSC) ? XOSC_MHZ * MHZ : 6.5 * MHZ;
    uint clk_ref_src = (dormant_source == DORMANT_SOURCE_XOSC) ?
                       CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC :
                       CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH;

    // CLK_REF = XOSC or ROSC
    clock_configure(clk_ref,
                    clk_ref_src,
                    0, // No aux mux
                    src_hz,
                    src_hz);

    // CLK SYS = CLK_REF
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0, // Using glitchless mux
                    src_hz,
                    src_hz);

    // CLK ADC = 0MHz
    clock_stop(clk_adc);
    clock_stop(clk_usb);

    // CLK RTC = ideally XOSC (12MHz) / 256 = 46875Hz but could be rosc
    uint clk_rtc_src = (dormant_source == DORMANT_SOURCE_XOSC) ? 
                       CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC : 
                       CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH;

    clock_configure(clk_rtc,
                    0, // No GLMUX
                    clk_rtc_src,
                    src_hz,
                    46875);

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    src_hz,
                    src_hz);

    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    // Assuming both xosc and rosc are running at the moment
    if (dormant_source == DORMANT_SOURCE_XOSC) {
        // Can disable rosc
        rosc_disable();
    } else {
        // Can disable xosc
        xosc_disable();
    }

    // Reconfigure uart with new clocks
    setup_default_uart();
}

// Go to sleep until woken up by the RTC
void sleep_goto_sleep_until(datetime_t *t, rtc_callback_t callback) {
    // We should have already called the sleep_run_from_dormant_source function
    assert(dormant_source_valid(_dormant_source));

    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_RTC_RTC_BITS;
    clocks_hw->sleep_en1 = 0x0; 

    rtc_set_alarm(t, callback);

    uint save = scb_hw->scr;
    // Enable deep sleep at the proc
    scb_hw->scr = save | M0PLUS_SCR_SLEEPDEEP_BITS;

    // Go to sleep
    __wfi();
}

void sleep_goto_sleep_for(uint32_t delay_ms, irq_handler_t callback, int *alarm_id)
{
    // We should have already called the sleep_run_from_dormant_source function
    assert(dormant_source_valid(_dormant_source));

    //Turn off all clocks except for the timer
    clocks_hw->sleep_en0 = 0x0;
    clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS;

    int alarm_num = hardware_alarm_claim_unused(true);

    //Get the timer alarm irq number (0..3) so it can be disabled and freed from the main program
    *alarm_id = alarm_num;

    uart_default_tx_wait_blocking();

    //Enable the alarm interrupts
    hw_set_bits(&timer_hw->inte, 1u << alarm_num);
    irq_set_enabled(alarm_num, true);
    irq_set_exclusive_handler(alarm_num, callback);

    //Get the target time based on the current time 
    uint64_t target = timer_hw->timerawl + delay_ms*1000; //add to the lower 32 bits
    //Write to the alarm register
    timer_hw->alarm[alarm_num] = (uint32_t) target;

    uint save = scb_hw->scr;
    // Enable deep sleep at the proc
    scb_hw->scr = save | M0PLUS_SCR_SLEEPDEEP_BITS;

    // Go to sleep
    __wfi();

}

static void _go_dormant(void) {
    assert(dormant_source_valid(_dormant_source));

    if (_dormant_source == DORMANT_SOURCE_XOSC) {
        xosc_dormant();
    } else {
        rosc_set_dormant();
    }
}

void sleep_goto_dormant_until(datetime_t *t, rtc_callback_t callback, uint src_hz, uint gpio_pin)   {
    // We should have already called the sleep_run_from_dormant_source function 

    //The RTC must be run from an external source, since the dormant source will be inactive
    rtc_run_from_external_source(src_hz, gpio_pin);

    // Turn off all clocks when in sleep mode except for RTC
    clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_RTC_RTC_BITS;
    clocks_hw->sleep_en1 = 0x0;

    //Set the RTC interrupt to wake up the proc from dormant mode
    rtc_set_alarm(t, callback);

    uint save = scb_hw->scr;
    // Enable deep sleep at the proc
    scb_hw->scr = save | M0PLUS_SCR_SLEEPDEEP_BITS;

    //Go dormant
    _go_dormant();

}

void sleep_goto_dormant_until_pin(uint gpio_pin, bool edge, bool high) {
    bool low = !high;
    bool level = !edge;

    // Configure the appropriate IRQ at IO bank 0
    assert(gpio_pin < NUM_BANK0_GPIOS);

    uint32_t event = 0;

    if (level && low) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS;
    if (level && high) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_HIGH_BITS;
    if (edge && high) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_HIGH_BITS;
    if (edge && low) event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS;

    gpio_set_dormant_irq_enabled(gpio_pin, event, true);

    _go_dormant();
    // Execution stops here until woken up

    // Clear the irq so we can go back to dormant mode again if we want
    gpio_acknowledge_irq(gpio_pin, event);
}

//To be called after waking up from sleep/dormant mode to restore system clocks properly
void sleep_power_up(void)
{   
    //Re-enable the ring oscillator, which will essentially kickstart the proc
    rosc_restart();

    //Reset the sleep enable register so peripherals and other hardware can be used
    clocks_hw->sleep_en0 |= ~(0u);
    clocks_hw->sleep_en1 |= ~(0u);

    //Restore all clocks
    clocks_init();
    
    //UART needs to be reinitialised with the new clock frequencies for stable output
    setup_default_uart();
}