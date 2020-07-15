/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_pwm.h"
#include "nrf_drv_saadc.h"
#include "nrf_temp.h"

#include "settings.h"

#include "thread_coap_utils.h"
#include "thread_utils.h"

#include <openthread/thread.h>

#define DIMMER_PWM_INSTANCE                  NRF_DRV_PWM_INSTANCE(0)
#define DIMMER_PWM_VALUE_MAX                 255
#define ADC_CHANNELS                         2

#define SCHED_QUEUE_SIZE      32
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE

APP_TIMER_DEF(m_voltage_timer_id);
APP_TIMER_DEF(m_internal_temperature_timer_id);

void pwm_set_brightness(char sensor_name, int64_t sensor_value);

sensor_subscription sensor_subscriptions[] = {
	{ .sensor_name = 'r', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = pwm_set_brightness, },
	{ .sensor_name = 'g', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = pwm_set_brightness, },
	{ .sensor_name = 'b', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = pwm_set_brightness, },
	{ .sensor_name = 'w', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = pwm_set_brightness, },
	{ .sensor_name = 'v', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 'V', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 't', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = SENSOR_SUBSCRIPTION_NAME_LAST, .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
};

static nrf_saadc_value_t adc_buf[ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL];

static nrf_drv_pwm_t m_led_pwm = DIMMER_PWM_INSTANCE;
static nrf_pwm_values_individual_t m_led_values;
static nrf_pwm_sequence_t const m_led_seq =
{
	.values.p_individual = &m_led_values,
	.length = NRF_PWM_VALUES_LENGTH(m_led_values),
	.repeats = 0,
	.end_delay = 0
};

int32_t dc_voltage_12 = 0;
int32_t dc_voltage_3v3 = 0;

void update_voltage_attributes_callback(void *p_event_data, uint16_t event_size)
{
	set_sensor_value('v', dc_voltage_3v3, false);
	set_sensor_value('V', dc_voltage_12, false);
}

void saadc_event_handler(nrf_drv_saadc_evt_t const *p_event)
{
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		uint32_t err_code;

		int32_t sums[ADC_CHANNELS];
		for (int i = 0; i < ADC_CHANNELS; i++) {
			sums[i] = 0;
			for (int j = 0; j < ADC_SAMPLES_PER_CHANNEL; j++) {
				sums[i] += p_event->data.done.p_buffer[j * 2 + i];
			}
			sums[i] = sums[i] / ADC_SAMPLES_PER_CHANNEL;
		}

		dc_voltage_12 = sums[1];

		if (sums[0] < 0)
			sums[0] = 0;

		dc_voltage_3v3 = sums[0];

		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL);
		APP_ERROR_CHECK(err_code);

		app_sched_event_put(NULL, 0, update_voltage_attributes_callback);
	}
	else
	{
		NRF_LOG_INFO("saadc unhandled event: %d", p_event->type);
	}
}

static void adc_configure(void)
{
	ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_calibrate_offset();
	APP_ERROR_CHECK(err_code);

	while (nrfx_saadc_is_busy());

	nrf_saadc_channel_config_t config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
	config0.acq_time = NRF_SAADC_ACQTIME_40US;
	err_code = nrf_drv_saadc_channel_init(0, &config0);
	APP_ERROR_CHECK(err_code);

	nrf_saadc_channel_config_t config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
	config1.acq_time = NRF_SAADC_ACQTIME_40US;
	err_code = nrf_drv_saadc_channel_init(1, &config1);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_buffer_convert(adc_buf, ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL);
	APP_ERROR_CHECK(err_code);
}

static void voltage_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	ret_code_t err_code = nrf_drv_saadc_sample();
	APP_ERROR_CHECK(err_code);
}

static void internal_temperature_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	NRF_TEMP->TASKS_START = 1;
	/* Busy wait while temperature measurement is not finished. */
	while (NRF_TEMP->EVENTS_DATARDY == 0) {
	}
	NRF_TEMP->EVENTS_DATARDY = 0;

	int32_t temp = nrf_temp_read();

	NRF_TEMP->TASKS_STOP = 1;

	set_sensor_value('t', temp, false);
}

void pwm_set_brightness(char sensor_name, int64_t sensor_value)
{
	uint16_t *p_channels = (uint16_t *)&m_led_values;

	if (sensor_value < 0)
		sensor_value = 0;
	if (sensor_value > DIMMER_PWM_VALUE_MAX)
		sensor_value = DIMMER_PWM_VALUE_MAX;

	uint8_t channel = 0;
	switch (sensor_name) {
		case 'r': { channel = 0; break; }
		case 'g': { channel = 1; break; }
		case 'b': { channel = 2; break; }
		case 'w': { channel = 3; break; }
		default:
			return;
	}

	p_channels[channel] = DIMMER_PWM_VALUE_MAX - (uint16_t)sensor_value;
}

static void pwm_init()
{
	nrf_drv_pwm_config_t const led_pwm_config =
		{
			.output_pins =
				{
					DIMMER_CHANNEL_PIN_R | DIMMER_PWM_INVERSION, // channel 0
					DIMMER_CHANNEL_PIN_G | DIMMER_PWM_INVERSION, // channel 1
					DIMMER_CHANNEL_PIN_B | DIMMER_PWM_INVERSION, // channel 2
					DIMMER_CHANNEL_PIN_W | DIMMER_PWM_INVERSION, // channel 3
				},
			.irq_priority = APP_IRQ_PRIORITY_LOWEST,
			.base_clock = NRF_PWM_CLK_1MHz,
			.count_mode = NRF_PWM_MODE_UP,
			.top_value = DIMMER_PWM_VALUE_MAX,
			.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
			.step_mode = NRF_PWM_STEP_AUTO
		};

	uint16_t *p_channels = (uint16_t *)&m_led_values;

	p_channels[0] = DIMMER_PWM_VALUE_MAX;
	p_channels[1] = DIMMER_PWM_VALUE_MAX;
	p_channels[2] = DIMMER_PWM_VALUE_MAX;
	p_channels[3] = DIMMER_PWM_VALUE_MAX;

	ret_code_t err_code = nrf_drv_pwm_init(&m_led_pwm, &led_pwm_config, NULL);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_pwm_simple_playback(&m_led_pwm, &m_led_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
	APP_ERROR_CHECK(err_code);
}

static void bsp_event_handler(bsp_event_t event)
{
	switch(event) {
		case BSP_EVENT_KEY_0:
			break;

		default:
			return;
	}
}

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
	if (flags & OT_CHANGED_THREAD_ROLE) {
		otDeviceRole device_role = otThreadGetDeviceRole(p_context);
		const char *szRole = "UNKNOWN ROLE";
		switch (device_role) {
			case OT_DEVICE_ROLE_CHILD:
				szRole = "OT_DEVICE_ROLE_CHILD";
				bsp_board_led_on(LED_CHILD_ROLE);
				bsp_board_led_off(LED_ROUTER_ROLE);
				break;
			case OT_DEVICE_ROLE_ROUTER:
				szRole = "OT_DEVICE_ROLE_ROUTER";
				bsp_board_led_on(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
				break;
			case OT_DEVICE_ROLE_LEADER:
				szRole = "OT_DEVICE_ROLE_LEADER";
				bsp_board_led_on(LED_CHILD_ROLE);
				bsp_board_led_on(LED_ROUTER_ROLE);
				break;

			case OT_DEVICE_ROLE_DISABLED:
				szRole = "OT_DEVICE_ROLE_DISABLED";
				bsp_board_led_off(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
				break;

			case OT_DEVICE_ROLE_DETACHED:
				szRole = "OT_DEVICE_ROLE_DETACHED";
				bsp_board_led_off(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
				break;
			default:
				break;
		}
		NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %s\r\n", flags, szRole);
	}
}

static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timer_init(void)
{
	uint32_t error_code = app_timer_init();
	APP_ERROR_CHECK(error_code);

	// Voltage timer
	error_code = app_timer_create(&m_voltage_timer_id, APP_TIMER_MODE_REPEATED, voltage_timeout_handler);
	APP_ERROR_CHECK(error_code);

	// Internal temperature timer
	error_code = app_timer_create(&m_internal_temperature_timer_id, APP_TIMER_MODE_REPEATED, internal_temperature_timeout_handler);
	APP_ERROR_CHECK(error_code);
}

static void thread_instance_init(void)
{
	thread_configuration_t thread_configuration =
	{
		.radio_mode            = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,
		.autocommissioning     = true,
		.default_child_timeout = 10,
		.wipe_settings         = false,
	};

	thread_init(&thread_configuration);
	thread_state_changed_callback_set(thread_state_changed_callback);
}

int main(int argc, char * argv[])
{
	log_init();
	APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	timer_init();
	pwm_init();
	adc_configure();
	nrf_temp_init();

	uint32_t error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
	APP_ERROR_CHECK(error_code);

	thread_instance_init();
	otPlatRadioSetTransmitPower(thread_ot_instance_get(), 8);
	thread_coap_utils_init();

	ret_code_t err_code = app_timer_start(m_voltage_timer_id, APP_TIMER_TICKS(VOLTAGE_TIMER_INTERVAL / ADC_SAMPLES_PER_CHANNEL), NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_internal_temperature_timer_id, APP_TIMER_TICKS(INTERNAL_TEMPERATURE_TIMER_INTERVAL), NULL);
	APP_ERROR_CHECK(err_code);

	while (true) {
		thread_process();
		app_sched_execute();

		if (NRF_LOG_PROCESS() == false) {
			thread_sleep();
		}
	}
}
