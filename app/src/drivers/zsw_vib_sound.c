/*
 * Copyright (c) 2022 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_work, LOG_LEVEL_DBG);

#include <drivers/zsw_vib_sound.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#include <stdlib.h>


#define FUNKYTOWN_NOTES 13
#define MARIO_NOTES 37
#define GOLIOTH_NOTES 21

/* Sensor device structs */

static const struct gpio_dt_spec enable_gpio = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(vib_pwr), enable_gpios, {});
const struct pwm_dt_spec sBuzzer = PWM_DT_SPEC_GET_OR(DT_ALIAS(vibrator_pwm), {});

enum song_choice
{
	beep,
	funkytown,
	mario,
	golioth
};

enum song_choice song = 3;

struct note_duration
{
	int note;	  // hz
	int duration; // msec
};

struct note_duration funkytown_song[FUNKYTOWN_NOTES] = {
	{.note = C5, .duration = quarter},
	{.note = REST, .duration = eigth},
	{.note = C5, .duration = quarter},
	{.note = Bb4, .duration = quarter},
	{.note = C5, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = G4, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = G4, .duration = quarter},
	{.note = C5, .duration = quarter},
	{.note = F5, .duration = quarter},
	{.note = E5, .duration = quarter},
	{.note = C5, .duration = quarter}};

struct note_duration mario_song[MARIO_NOTES] = {
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = eigth},
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = C6, .duration = quarter},
	{.note = E6, .duration = half},
	{.note = G6, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = G4, .duration = whole},
	{.note = REST, .duration = whole},
	// break in sound
	{.note = C6, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = G5, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = E5, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = A5, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = B5, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = Bb5, .duration = quarter},
	{.note = A5, .duration = half},
	{.note = G5, .duration = quarter},
	{.note = E6, .duration = quarter},
	{.note = G6, .duration = quarter},
	{.note = A6, .duration = half},
	{.note = F6, .duration = quarter},
	{.note = G6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = C6, .duration = quarter},
	{.note = D6, .duration = quarter},
	{.note = B5, .duration = quarter}};

struct note_duration golioth_song[] = {
	{.note = C6, .duration = quarter},
	{.note = REST, .duration = 100},
	{.note = G5, .duration = 100},
	{.note = A5, .duration = 100},
	{.note = Bb5, .duration = 100},
	{.note = REST, .duration = 100},
	{.note = Bb5, .duration = 100},
	{.note = REST, .duration = quarter},
	{.note = C5, .duration = half},
	{.note = REST, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = C6, .duration = quarter}
};

/* Thread plays song on buzzer */

K_SEM_DEFINE(buzzer_initialized_sem, 0, 1); /* Wait until buzzer is ready */

#define BUZZER_STACK 1024

static void play_song(enum song_choice song)
{
    switch (song)
    {
    case 0:
        LOG_DBG("beep");
        pwm_set_dt(&sBuzzer, PWM_HZ(1000), PWM_HZ(1000) / 2);
        gpio_pin_set_dt(&enable_gpio, 1);
        k_msleep(100);
        gpio_pin_set_dt(&enable_gpio, 0);
        break;
    case 1:
        LOG_DBG("funkytown");
        for (int i = 0; i < FUNKYTOWN_NOTES; i++)
        {
            if (funkytown_song[i].note < 10)
            {
                // Low frequency notes represent a 'pause'
                gpio_pin_set_dt(&enable_gpio, 0);
                pwm_set_pulse_dt(&sBuzzer, 0);
                k_msleep(funkytown_song[i].duration);
                gpio_pin_set_dt(&enable_gpio, 1);
            }
            else
            {
                pwm_set_dt(&sBuzzer, PWM_HZ(funkytown_song[i].note), PWM_HZ((funkytown_song[i].note)) / 2);
                gpio_pin_set_dt(&enable_gpio, 1);
                // LOG_DBG("note: %d, duration: %d", funkytown_song[i].note, funkytown_song[i].duration);
                k_msleep(funkytown_song[i].duration);
                gpio_pin_set_dt(&enable_gpio, 0);
            }
        }
        break;

    case 2:
        LOG_DBG("mario");
        for (int i = 0; i < MARIO_NOTES; i++)
        {
            if (mario_song[i].note < 10)
            {
                // Low frequency notes represent a 'pause'
                gpio_pin_set_dt(&enable_gpio, 0);
                pwm_set_pulse_dt(&sBuzzer, 0);
                k_msleep(mario_song[i].duration);
                gpio_pin_set_dt(&enable_gpio, 1);
            }
            else
            {
                pwm_set_dt(&sBuzzer, PWM_HZ(mario_song[i].note), PWM_HZ((mario_song[i].note)) / 2);
                gpio_pin_set_dt(&enable_gpio, 1);
                k_msleep(mario_song[i].duration);
                gpio_pin_set_dt(&enable_gpio, 0);
            }
        }
        break;
    case 3:
        LOG_DBG("golioth");
        for (int i = 0; i < (sizeof(golioth_song)/sizeof(golioth_song[1])); i++)
        {
            if (golioth_song[i].note < 10)
            {
                // Low frequency notes represent a 'pause'
                gpio_pin_set_dt(&enable_gpio, 0);
                pwm_set_pulse_dt(&sBuzzer, 0);
                k_msleep(golioth_song[i].duration);
                gpio_pin_set_dt(&enable_gpio, 1);
            }
            else
            {
                pwm_set_dt(&sBuzzer, PWM_HZ(golioth_song[i].note), PWM_HZ((golioth_song[i].note)) / 2);
                gpio_pin_set_dt(&enable_gpio, 1);
                k_msleep(golioth_song[i].duration);
                gpio_pin_set_dt(&enable_gpio, 0);
            }
        }
        break;
    default:
        LOG_WRN("invalid switch state");
        break;
    }

    // turn buzzer off (pulse duty to 0)
    pwm_set_pulse_dt(&sBuzzer, 0);
    gpio_pin_set_dt(&enable_gpio, 0);
}

int app_buzzer_init()
{
    int rc;

	if (!device_is_ready(sBuzzer.dev))
	{
		return -ENODEV;
	}
    rc = gpio_pin_configure_dt(&enable_gpio, GPIO_OUTPUT_LOW);
    if (rc != 0) {
        printk("Failed init vibration motor enable pin\n");
    }
    if (!device_is_ready(enable_gpio.port)) {
        LOG_WRN("Vibration motor enable/disable not supported");
        return -ENODEV;
    }
	k_sem_give(&buzzer_initialized_sem);
	return 0;
}

void play_beep_once(void)
{
	song = beep;
	play_song(song);
    k_msleep(100);
}

void play_funkytown_once(void)
{
	song = funkytown;
	play_song(song);
}

void play_mario_once(void)
{
	song = mario;
	play_song(song);
}

void play_golioth_once(void)
{
	song = golioth;
	play_song(song);
}
