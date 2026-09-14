#include "PWM.h"

#if defined(PLATFORM_RP2350)
#include <hardware/clocks.h>
#include <hardware/pwm.h>

#include "logging.h"

#define PWM_SLICES 12

// Each slice is clocked at 1MHz so the counter value is microseconds
static struct
{
    uint32_t frequency;
    uint8_t users;
} slice_config[PWM_SLICES];

pwm_channel_t PWMController::allocate(uint8_t pin, uint32_t frequency)
{
    const uint8_t slice = pwm_gpio_to_slice_num(pin);
    if (slice >= PWM_SLICES || frequency < 16 || frequency > 1000000)
    {
        DBGLN("Cannot allocate PWM on pin %d at %dHz", pin, frequency);
        return -1;
    }
    // Both outputs of a slice share a counter, so they must share a frequency
    if (slice_config[slice].users > 0 && slice_config[slice].frequency != frequency)
    {
        DBGLN("PWM slice %d already running at %dHz", slice, slice_config[slice].frequency);
        return -1;
    }

    if (slice_config[slice].users == 0)
    {
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
        pwm_config_set_wrap(&cfg, (1000000U / frequency) - 1);
        pwm_init(slice, &cfg, true);
        slice_config[slice].frequency = frequency;
    }
    slice_config[slice].users++;

    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_gpio_level(pin, 0);
    DBGLN("allocate pwm slice %d on pin %d at %dHz", slice, pin, frequency);
    return pin;
}

void PWMController::release(pwm_channel_t channel)
{
    if (channel < 0)
    {
        ERRLN("Invalid PWM channel %x", channel);
        return;
    }
    const uint8_t slice = pwm_gpio_to_slice_num(channel);
    pwm_set_gpio_level(channel, 0);
    gpio_deinit(channel);
    if (slice_config[slice].users > 0 && --slice_config[slice].users == 0)
    {
        pwm_set_enabled(slice, false);
        slice_config[slice].frequency = 0;
    }
}

void PWMController::setDuty(pwm_channel_t channel, uint16_t duty)
{
    const uint8_t slice = pwm_gpio_to_slice_num(channel);
    const uint32_t period = 1000000U / slice_config[slice].frequency;
    pwm_set_gpio_level(channel, (uint32_t)duty * period / 1000U);
}

void PWMController::setMicroseconds(pwm_channel_t channel, uint16_t microseconds)
{
    pwm_set_gpio_level(channel, microseconds);
}

void PWMController::feedWatchdog()
{
}

#endif
