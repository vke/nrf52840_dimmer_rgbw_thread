/**
 * Copyright (c) 2017-2019 - 2020, Nordic Semiconductor ASA
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

#ifndef THREAD_COAP_UTILS_H__
#define THREAD_COAP_UTILS_H__

#include <stdbool.h>
#include <openthread/coap.h>

#include "thread_utils.h"

typedef void (*sensor_set_value_handler_t)(char sensor_name, int64_t sensor_value);

#define SENSOR_SUBSCRIPTION_NAME_LAST 255

typedef struct sensor_subscription
{
	char sensor_name;
	int64_t sent_value;
	int64_t current_value;
	int64_t reportable_change;
	bool read_only;
	bool disable_reporting;
	bool initialized;
	sensor_set_value_handler_t set_value_handler;
	uint32_t report_interval;
	uint32_t last_sent_at;
} sensor_subscription;

typedef struct subscription_settings_data
{
	otIp6Address subscription_address;
	uint32_t subscription_interval;
	uint32_t last_sent_at;
} subscription_settings_data;

void thread_coap_utils_init();

bool set_sensor_value(char sensor_name, int64_t sensor_value, bool external_request);
bool get_sensor_value(char sensor_name, int64_t *p_sensor_value);

#endif /* THREAD_COAP_UTILS_H__ */
