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

#include "thread_coap_utils.h"

#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_assert.h"
#include "sdk_config.h"
#include "thread_utils.h"

#include "settings.h"

#include "tinycbor/cbor.h"

#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/thread.h>
#include <openthread/icmp6.h>
#include <openthread/platform/alarm-milli.h>

APP_TIMER_DEF(m_led_send_timer);
APP_TIMER_DEF(m_led_recv_timer);
APP_TIMER_DEF(m_boot_timer);

static otIcmp6Handler m_icmp6_handler;

static void bsp_send_led_timer_handler(void * p_context)
{
	bsp_board_led_off(LED_SEND_NOTIFICATION);
}

static void bsp_recv_led_timer_handler(void * p_context)
{
	bsp_board_led_off(LED_RECV_NOTIFICATION);
}

void blink_recv_led()
{
	uint32_t err_code;
	bsp_board_led_on(LED_RECV_NOTIFICATION);
	err_code = app_timer_start(m_led_recv_timer, APP_TIMER_TICKS(25), NULL);
}

void blink_send_led()
{
	uint32_t err_code;
	bsp_board_led_on(LED_SEND_NOTIFICATION);
	err_code = app_timer_start(m_led_send_timer, APP_TIMER_TICKS(25), NULL);
}

static void icmp_receive_callback(void                * p_context,
								  otMessage           * p_message,
								  const otMessageInfo * p_message_info,
								  const otIcmp6Header * p_icmp_header)
{
	if (p_icmp_header->mType == OT_ICMP6_TYPE_ECHO_REQUEST) {
		blink_recv_led();
	}
}

static uint32_t m_poll_period;

static void boot_request_handler(void *, otMessage *, const otMessageInfo *);
static void info_request_handler(void *, otMessage *, const otMessageInfo *);
static void set_request_handler(void *, otMessage *, const otMessageInfo *);
static void get_request_handler(void *, otMessage *, const otMessageInfo *);
static void sub_request_handler(void *, otMessage *, const otMessageInfo *);

static otCoapResource m_boot_resource = { .mUriPath = "boot", .mHandler = boot_request_handler, .mContext = NULL, .mNext = NULL, };
static otCoapResource m_info_resource = { .mUriPath = "info", .mHandler = info_request_handler, .mContext = NULL, .mNext = NULL, };
static otCoapResource m_set_resource = { .mUriPath = "set", .mHandler = set_request_handler, .mContext = NULL, .mNext = NULL, };
static otCoapResource m_get_resource = { .mUriPath = "get", .mHandler = get_request_handler, .mContext = NULL, .mNext = NULL, };
static otCoapResource m_sub_resource = { .mUriPath = "sub", .mHandler = sub_request_handler, .mContext = NULL, .mNext = NULL, };

APP_TIMER_DEF(m_subscription_timer);

static subscription_settings_data subscription_settings = {
	.subscription_address = {0},
	.subscription_interval = 1000,
	.last_sent_at = 0,
};

extern sensor_subscription sensor_subscriptions[];

bool set_sensor_value(char sensor_name, int64_t sensor_value, bool external_request)
{
	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		if (sensor_subscriptions[i].sensor_name == sensor_name) {
			sensor_subscriptions[i].initialized = true;
			if (external_request) {
				sensor_subscriptions[i].sent_value = sensor_value;
				sensor_subscriptions[i].current_value = sensor_value;
				if (sensor_subscriptions[i].set_value_handler)
					sensor_subscriptions[i].set_value_handler(sensor_name, sensor_value);
			} else {
				sensor_subscriptions[i].current_value = sensor_value;
			}
			return true;
		}
	}
	return false;
}

bool get_sensor_value(char sensor_name, int64_t *p_sensor_value)
{
	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		if (sensor_subscriptions[i].sensor_name == sensor_name) {
			if (sensor_subscriptions[i].initialized == false)
				return false;
			*p_sensor_value = sensor_subscriptions[i].current_value;
			return true;
		}
	}
	return false;
}

bool is_sensor_readonly(char sensor_name)
{
	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		if (sensor_subscriptions[i].sensor_name == sensor_name) {
			return sensor_subscriptions[i].read_only;
		}
	}
	return true;
}

int16_t get_sensor_index(char sensor_name)
{
	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		if (sensor_subscriptions[i].sensor_name == sensor_name) {
			return i;
		}
	}
	return -1;
}

static void coap_default_handler(void * p_context, otMessage * p_message, const otMessageInfo * p_message_info)
{
	UNUSED_PARAMETER(p_context);
	UNUSED_PARAMETER(p_message);
	UNUSED_PARAMETER(p_message_info);
}

CborError cbor_encode_map_set_stringz(CborEncoder *p_encoder, const char *key, const char *value)
{
	CborError cborError = cbor_encode_text_stringz(p_encoder, key);
	if (cborError != CborNoError)
		return cborError;
	cborError = cbor_encode_text_stringz(p_encoder, value);
	return cborError;
}

CborError cbor_encode_map_set_int(CborEncoder *p_encoder, const char *key, int64_t value)
{
	CborError cborError = cbor_encode_text_stringz(p_encoder, key);
	if (cborError != CborNoError)
		return cborError;
	cborError = cbor_encode_int(p_encoder, value);
	return cborError;
}

static void boot_timer_handler(void * p_context)
{
	NRF_POWER->GPREGRET = 0xB1;
	NVIC_SystemReset();
}

static otError boot_response_send(otMessage * p_request_message, const otMessageInfo * p_message_info)
{
	otError error = OT_ERROR_NO_BUFS;
	otMessage * p_response;
	otInstance * p_instance = thread_ot_instance_get();

	do {
		p_response = otCoapNewMessage(p_instance, NULL);
		if (p_response == NULL)
			break;

		otCoapMessageInit(p_response, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);

		error = otCoapMessageSetToken(p_response, otCoapMessageGetToken(p_request_message), otCoapMessageGetTokenLength(p_request_message));
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapSendResponse(p_instance, p_response, p_message_info);

	} while (false);

	if (error != OT_ERROR_NONE && p_response != NULL)
		otMessageFree(p_response);

	return error;
}

static void boot_request_handler(void * p_context, otMessage * p_message, const otMessageInfo * p_message_info)
{
	UNUSED_PARAMETER(p_message);

	otError error;
	otMessageInfo message_info;

	if (otCoapMessageGetCode(p_message) == OT_COAP_CODE_POST) {
		message_info = *p_message_info;
		memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));

		error = boot_response_send(p_message, &message_info);
		if (error == OT_ERROR_NONE) {
		}

		ret_code_t err_code = app_timer_start(m_boot_timer, APP_TIMER_TICKS(1000), NULL);
	}

	blink_recv_led();
}

size_t fill_info_packet(uint8_t *pBuffer, size_t stBufferSize)
{
	CborEncoder encoder;
	CborError cborError = CborNoError;

	cbor_encoder_init(&encoder, pBuffer, stBufferSize, 0);

	CborEncoder encoderMap;

	cborError = cbor_encoder_create_map(&encoder, &encoderMap, CborIndefiniteLength);
	if (cborError != CborNoError)
		return 0;

	cborError = cbor_encode_map_set_stringz(&encoderMap, "t", INFO_FIRMWARE_TYPE);
	if (cborError != CborNoError)
		return 0;

	cborError = cbor_encode_map_set_stringz(&encoderMap, "v", INFO_FIRMWARE_VERSION);
	if (cborError != CborNoError)
		return 0;

	uint8_t macAddr[8];
	otPlatRadioGetIeeeEui64(thread_ot_instance_get(), macAddr);

	cborError = cbor_encode_text_stringz(&encoderMap, "m");
	if (cborError != CborNoError)
		return 0;
	cborError = cbor_encode_byte_string(&encoderMap, macAddr, sizeof(macAddr));
	if (cborError != CborNoError)
		return 0;

	cborError = cbor_encode_text_stringz(&encoderMap, "e");
	if (cborError != CborNoError)
		return 0;
	cborError = cbor_encode_byte_string(&encoderMap, (const uint8_t *)otLinkGetExtendedAddress(thread_ot_instance_get()), sizeof(otExtAddress));
	if (cborError != CborNoError)
		return 0;

	cborError = cbor_encode_text_stringz(&encoderMap, "a");
	if (cborError != CborNoError)
		return 0;
	cborError = cbor_encode_byte_string(&encoderMap, (const uint8_t *)otThreadGetMeshLocalEid(thread_ot_instance_get()), sizeof(otIp6Address));
	if (cborError != CborNoError)
		return 0;

	cborError = cbor_encode_text_stringz(&encoderMap, "s");
	CborEncoder encoderArr;

	cborError = cbor_encoder_create_array(&encoderMap, &encoderArr, CborIndefiniteLength);
	if (cborError != CborNoError)
		return 0;

	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		cbor_encode_text_string(&encoderArr, &sensor_subscriptions[i].sensor_name, 1);
	}

	cborError = cbor_encoder_close_container(&encoderMap, &encoderArr);
	if (cborError != CborNoError)
		return 0;

	cborError = cbor_encoder_close_container(&encoder, &encoderMap);
	if (cborError != CborNoError)
		return 0;

	return cbor_encoder_get_buffer_size(&encoder, pBuffer);
}

static otError info_response_send(otMessage * p_request_message, const otMessageInfo * p_message_info)
{
	otError error = OT_ERROR_NO_BUFS;
	otMessage * p_response;
	otInstance * p_instance = thread_ot_instance_get();

	do {
		p_response = otCoapNewMessage(p_instance, NULL);
		if (p_response == NULL)
			break;

		otCoapMessageInit(p_response, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);

		error = otCoapMessageSetToken(p_response, otCoapMessageGetToken(p_request_message), otCoapMessageGetTokenLength(p_request_message));
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageAppendContentFormatOption(p_response, OT_COAP_OPTION_CONTENT_FORMAT_CBOR);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageSetPayloadMarker(p_response);
		if (error != OT_ERROR_NONE)
			break;

		uint8_t buff[256];

		size_t packet_len = fill_info_packet(buff, sizeof(buff));
		if (packet_len == 0)
			break;

		error = otMessageAppend(p_response, buff, packet_len);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapSendResponse(p_instance, p_response, p_message_info);

	} while (false);

	if (error != OT_ERROR_NONE && p_response != NULL)
		otMessageFree(p_response);

	return error;
}

static void info_request_handler(void * p_context, otMessage * p_message, const otMessageInfo * p_message_info)
{
	UNUSED_PARAMETER(p_message);

	otError error;
	otMessageInfo message_info;

	if (otCoapMessageGetCode(p_message) == OT_COAP_CODE_GET) {
		message_info = *p_message_info;
		memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));

		error = info_response_send(p_message, &message_info);
		if (error == OT_ERROR_NONE) {
		}
	}

	blink_recv_led();
}

static void set_response_send(otMessage * p_request_message, const otMessageInfo * p_message_info, const uint8_t *p_buff, size_t buff_size)
{
	otError      error = OT_ERROR_NO_BUFS;
	otMessage  * p_response;
	otInstance * p_instance = thread_ot_instance_get();

	do {
		p_response = otCoapNewMessage(p_instance, NULL);
		if (p_response == NULL)
			break;

		otCoapMessageInit(p_response, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);

		error = otCoapMessageSetToken(p_response, otCoapMessageGetToken(p_request_message), otCoapMessageGetTokenLength(p_request_message));
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageAppendContentFormatOption(p_response, OT_COAP_OPTION_CONTENT_FORMAT_CBOR);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageSetPayloadMarker(p_response);
		if (error != OT_ERROR_NONE)
			break;

		error = otMessageAppend(p_response, p_buff, buff_size);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapSendResponse(p_instance, p_response, p_message_info);
	} while (false);

	if ((error != OT_ERROR_NONE) && (p_response != NULL))
		otMessageFree(p_response);
}

static void set_request_handler(void * p_context, otMessage * p_message, const otMessageInfo * p_message_info)
{
	do {
		if (otCoapMessageGetType(p_message) != OT_COAP_TYPE_CONFIRMABLE && otCoapMessageGetType(p_message) != OT_COAP_TYPE_NON_CONFIRMABLE)
			break;

		if (otCoapMessageGetCode(p_message) != OT_COAP_CODE_PUT)
			break;

		uint8_t buff[256];

		uint16_t body_len = otMessageGetLength(p_message) - otMessageGetOffset(p_message);

		if (body_len > sizeof(buff))
			break;

		if (otMessageRead(p_message, otMessageGetOffset(p_message), buff, body_len) != body_len)
			break;

		CborParser parser;
		CborValue it;
		CborError cborError = cbor_parser_init(buff, body_len, 0, &parser, &it);
		if (cborError != CborNoError)
			break;

		if (cbor_value_at_end(&it) || cbor_value_get_type(&it) != CborMapType)
			break;

		CborValue recursed;
		cborError = cbor_value_enter_container(&it, &recursed);
		if (cborError != CborNoError)
			break;

		uint8_t buff_resp[256];
		CborEncoder encoder;
		cbor_encoder_init(&encoder, buff_resp, sizeof(buff_resp), 0);

		CborEncoder encoderMap;
		cborError = cbor_encoder_create_map(&encoder, &encoderMap, CborIndefiniteLength);
		if (cborError != CborNoError)
			break;

		while (!cbor_value_at_end(&recursed)) {
			CborType type = cbor_value_get_type(&recursed);
			if (type != CborTextStringType)
				break;
			char key[256];
			size_t keyLen = sizeof(key);
			CborValue next;
			cborError = cbor_value_copy_text_string(&recursed, key, &keyLen, &next);
			if (key[1] != 0)
				break;
			recursed = next;
			type = cbor_value_get_type(&recursed);
			if (type != CborIntegerType)
				break;
			int64_t val;
			cborError = cbor_value_get_int64(&recursed, &val);
			if (cborError != CborNoError)
				break;

			if (!is_sensor_readonly(key[0])) {
				set_sensor_value(key[0], val, true);
				cbor_encode_map_set_int(&encoderMap, key, val);
			}

			cborError = cbor_value_advance(&recursed);
			if (cborError != CborNoError)
				break;
		}

		cborError = cbor_encoder_close_container(&encoder, &encoderMap);
		if (cborError != CborNoError)
			break;

		if (otCoapMessageGetType(p_message) == OT_COAP_TYPE_CONFIRMABLE) {
			size_t buff_size = cbor_encoder_get_buffer_size(&encoder, buff_resp);
			set_response_send(p_message, p_message_info, buff_resp, buff_size);
		}
	}
	while (false);

	blink_recv_led();
}

static otError get_response_send(otMessage *p_request_message, const otMessageInfo *p_message_info, const uint8_t *p_buff, size_t buff_size)
{
	otError error = OT_ERROR_NO_BUFS;
	otMessage *p_response;
	otInstance *p_instance = thread_ot_instance_get();

	do
	{
		p_response = otCoapNewMessage(p_instance, NULL);
		if (p_response == NULL)
			break;

		otCoapMessageInit(p_response, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);

		error = otCoapMessageSetToken(p_response, otCoapMessageGetToken(p_request_message), otCoapMessageGetTokenLength(p_request_message));
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageAppendContentFormatOption(p_response, OT_COAP_OPTION_CONTENT_FORMAT_CBOR);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageSetPayloadMarker(p_response);
		if (error != OT_ERROR_NONE)
			break;

		error = otMessageAppend(p_response, p_buff, buff_size);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapSendResponse(p_instance, p_response, p_message_info);

	} while (false);

	if (error != OT_ERROR_NONE && p_response != NULL)
		otMessageFree(p_response);

	return error;
}

static void get_request_handler(void *p_context, otMessage *p_message, const otMessageInfo *p_message_info)
{
	do
	{
		if (otCoapMessageGetType(p_message) != OT_COAP_TYPE_CONFIRMABLE)
			break;

		if (otCoapMessageGetCode(p_message) != OT_COAP_CODE_GET)
			break;

		uint8_t buff_req[256];

		uint16_t body_len = otMessageGetLength(p_message) - otMessageGetOffset(p_message);

		if (body_len > sizeof(buff_req))
			break;

		if (otMessageRead(p_message, otMessageGetOffset(p_message), buff_req, body_len) != body_len)
			break;

		CborParser parser;
		CborValue it;
		CborError cborError = cbor_parser_init(buff_req, body_len, 0, &parser, &it);
		if (cborError != CborNoError)
			break;

		if (cbor_value_at_end(&it) || cbor_value_get_type(&it) != CborArrayType)
			break;

		CborValue recursed;
		cborError = cbor_value_enter_container(&it, &recursed);
		if (cborError != CborNoError)
			break;

		uint8_t buff_resp[256];
		CborEncoder encoder;
		cbor_encoder_init(&encoder, buff_resp, sizeof(buff_resp), 0);

		CborEncoder encoderMap;
		cborError = cbor_encoder_create_map(&encoder, &encoderMap, CborIndefiniteLength);
		if (cborError != CborNoError)
			break;

		while (!cbor_value_at_end(&recursed))
		{
			CborType type = cbor_value_get_type(&recursed);
			if (type != CborTextStringType)
				break;

			char key[2];
			size_t keyLen = sizeof(key);
			CborValue next;
			cborError = cbor_value_copy_text_string(&recursed, key, &keyLen, &next);
			if (key[1] != 0)
				break;
			recursed = next;

			int64_t sensor_value;
			if (!get_sensor_value(key[0], &sensor_value))
				break;

			cbor_encode_map_set_int(&encoderMap, key, sensor_value);
		}

		cborError = cbor_encoder_close_container(&encoder, &encoderMap);
		if (cborError != CborNoError)
			break;

		if (otCoapMessageGetType(p_message) == OT_COAP_TYPE_CONFIRMABLE) {
			size_t buff_size = cbor_encoder_get_buffer_size(&encoder, buff_resp);
			get_response_send(p_message, p_message_info, buff_resp, buff_size);
		}
	} while (false);

	blink_recv_led();
}

static void sub_response_send(otMessage * p_request_message, const otMessageInfo * p_message_info, const uint8_t *p_buff, size_t buff_size)
{
	otError      error = OT_ERROR_NO_BUFS;
	otMessage  * p_response;
	otInstance * p_instance = thread_ot_instance_get();

	do {
		p_response = otCoapNewMessage(p_instance, NULL);
		if (p_response == NULL)
			break;

		otCoapMessageInit(p_response, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);

		error = otCoapMessageSetToken(p_response, otCoapMessageGetToken(p_request_message), otCoapMessageGetTokenLength(p_request_message));
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageAppendContentFormatOption(p_response, OT_COAP_OPTION_CONTENT_FORMAT_CBOR);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageSetPayloadMarker(p_response);
		if (error != OT_ERROR_NONE)
			break;

		error = otMessageAppend(p_response, p_buff, buff_size);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapSendResponse(p_instance, p_response, p_message_info);
	} while (false);

	if ((error != OT_ERROR_NONE) && (p_response != NULL))
		otMessageFree(p_response);
}

static size_t parse_subscriptions(uint8_t *p_request, size_t request_size, uint8_t *p_response, size_t response_size)
{
	uint32_t time_now = otPlatAlarmMilliGetNow();

	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		sensor_subscriptions[i].disable_reporting = true;
		sensor_subscriptions[i].last_sent_at = time_now;
		sensor_subscriptions[i].sent_value = sensor_subscriptions[i].current_value;
	}

	CborParser parser;
	CborValue it;
	CborError cborError = cbor_parser_init(p_request, request_size, 0, &parser, &it);
	if (cborError != CborNoError)
		return 0;

	if (cbor_value_at_end(&it) || cbor_value_get_type(&it) != CborMapType)
		return 0;

	CborValue recursed;
	cborError = cbor_value_enter_container(&it, &recursed);
	if (cborError != CborNoError)
		return 0;

	CborEncoder encoder;
	cbor_encoder_init(&encoder, p_response, response_size, 0);

	CborEncoder encoderMap;
	cborError = cbor_encoder_create_map(&encoder, &encoderMap, CborIndefiniteLength);
	if (cborError != CborNoError)
		return 0;

	while (!cbor_value_at_end(&recursed)) {
		CborType type = cbor_value_get_type(&recursed);
		if (type != CborTextStringType)
			return 0;

		char key[2];
		size_t keyLen = sizeof(key);
		CborValue next;

		cborError = cbor_value_copy_text_string(&recursed, key, &keyLen, &next);
		if (cborError != CborNoError)
			return 0;
		if (key[1] != 0)
			return 0;

		recursed = next;

		type = cbor_value_get_type(&recursed);
		switch (key[0]) {
			case 'a': {
				if (type != CborByteStringType)
					return 0;
				size_t addr_size = 0;
				cborError = cbor_value_calculate_string_length(&recursed, &addr_size);
				if (cborError != CborNoError)
					return 0;
				if (addr_size != sizeof(otIp6Address))
					return 0;
				addr_size = sizeof(otIp6Address);
				cborError = cbor_value_copy_byte_string(&recursed, subscription_settings.subscription_address.mFields.m8, &addr_size, &next);
				if (cborError != CborNoError)
					return 0;
				if (addr_size != sizeof(otIp6Address))
					return 0;
				recursed = next;
				break;
			}
			case 's': {
				if (type != CborMapType)
					return 0;
				
				CborValue recursedMapS;
				cborError = cbor_value_enter_container(&recursed, &recursedMapS);
				if (cborError != CborNoError)
					return 0;

				while (!cbor_value_at_end(&recursedMapS)) {
					CborType type = cbor_value_get_type(&recursedMapS);
					if (type != CborTextStringType)
						return 0;

					char keyS[2];
					size_t keySLen = sizeof(keyS);

					cborError = cbor_value_copy_text_string(&recursedMapS, keyS, &keySLen, &next);
					if (cborError != CborNoError)
						return 0;
					if (keyS[1] != 0)
						return 0;
					
					int16_t sensor_index = get_sensor_index(keyS[0]);
					if (sensor_index == -1)
						return 0;

					recursedMapS = next;

					if (cbor_value_get_type(&recursedMapS) != CborMapType)
						return 0;

					CborValue recursedMapSR;
					cborError = cbor_value_enter_container(&recursedMapS, &recursedMapSR);
					if (cborError != CborNoError)
						return 0;

					while (!cbor_value_at_end(&recursedMapSR)) {
						CborType type = cbor_value_get_type(&recursedMapSR);
						if (type != CborTextStringType)
							return 0;

						char keySR[2];
						size_t keySRLen = sizeof(keySR);

						cborError = cbor_value_copy_text_string(&recursedMapSR, keySR, &keySRLen, &next);
						if (cborError != CborNoError)
							return 0;

						if (keySR[0] != 'i' && keySR[0] != 'r')
							return 0;
						if (keySR[1] != 0)
							return 0;

						recursedMapSR = next;
						type = cbor_value_get_type(&recursedMapSR);
						if (type != CborIntegerType)
							return 0;
						int64_t val;
						cborError = cbor_value_get_int64(&recursedMapSR, &val);
						if (cborError != CborNoError)
							return 0;
						cborError = cbor_value_advance(&recursedMapSR);
						if (cborError != CborNoError)
							return 0;

						if (keySR[0] == 'i')
							sensor_subscriptions[sensor_index].report_interval = (uint32_t)val;
						else if (keySR[0] == 'r')
							sensor_subscriptions[sensor_index].reportable_change = val;
					}

					cborError = cbor_value_leave_container(&recursedMapS, &recursedMapSR);
					if (cborError != CborNoError)
						return 0;

					sensor_subscriptions[sensor_index].disable_reporting = false;
				}

				cborError = cbor_value_leave_container(&recursed, &recursedMapS);
				if (cborError != CborNoError)
					return 0;
				
				break;
			}
			default:
				return 0;
		}
	}

	cborError = cbor_encoder_close_container(&encoder, &encoderMap);
	if (cborError != CborNoError)
		return 0;

	return cbor_encoder_get_buffer_size(&encoder, p_response);
}

static void sub_request_handler(void * p_context, otMessage * p_message, const otMessageInfo * p_message_info)
{
	do {
		if (otCoapMessageGetType(p_message) != OT_COAP_TYPE_CONFIRMABLE && otCoapMessageGetType(p_message) != OT_COAP_TYPE_NON_CONFIRMABLE)
			break;

		if (otCoapMessageGetCode(p_message) != OT_COAP_CODE_PUT)
			break;

		uint8_t buff_request[256];

		uint16_t request_size = otMessageGetLength(p_message) - otMessageGetOffset(p_message);

		if (request_size > sizeof(buff_request))
			break;

		if (otMessageRead(p_message, otMessageGetOffset(p_message), buff_request, request_size) != request_size)
			break;

		uint8_t buff_response[256];

		size_t response_size = parse_subscriptions(buff_request, request_size, buff_response, sizeof(buff_response));

		if (response_size == 0)
			break;

		if (otCoapMessageGetType(p_message) == OT_COAP_TYPE_CONFIRMABLE)
			set_response_send(p_message, p_message_info, buff_response, response_size);
	}
	while (false);

	blink_recv_led();
}

static void send_subscription_broadcast()
{
	otError       error = OT_ERROR_NONE;
	otMessage   * p_request;
	otMessageInfo message_info;
	otInstance  * p_instance = thread_ot_instance_get();

	do
	{
		p_request = otCoapNewMessage(p_instance, NULL);
		if (p_request == NULL)
			break;

		otCoapMessageInit(p_request, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);

		error = otCoapMessageAppendUriPathOptions(p_request, "up");
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageAppendContentFormatOption(p_request, OT_COAP_OPTION_CONTENT_FORMAT_CBOR);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageSetPayloadMarker(p_request);
		if (error != OT_ERROR_NONE)
			break;

		uint8_t buff[256];

		size_t packet_len = fill_info_packet(buff, sizeof(buff));
		if (packet_len == 0)
			break;

		error = otMessageAppend(p_request, buff, packet_len);
		if (error != OT_ERROR_NONE)
			break;

		memset(&message_info, 0, sizeof(message_info));
		message_info.mPeerPort = OT_DEFAULT_COAP_PORT;

		error = otIp6AddressFromString("ff03::1", &message_info.mPeerAddr);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapSendRequest(p_instance, p_request, &message_info, NULL, NULL);
		if (error != OT_ERROR_NONE)
			break;

		blink_send_led();
	} while (false);

	if (error != OT_ERROR_NONE && p_request != NULL)
		otMessageFree(p_request);
}

size_t fill_subscriptions_packet(uint32_t time_now, uint8_t *pBuffer, size_t stBufferSize)
{
	CborEncoder encoder;
	CborError cborError = CborNoError;

	cbor_encoder_init(&encoder, pBuffer, stBufferSize, 0);

	CborEncoder encoderMap;

	cborError = cbor_encoder_create_map(&encoder, &encoderMap, CborIndefiniteLength);
	if (cborError != CborNoError)
		return 0;

	bool data_added = false;

	for (int i = 0; sensor_subscriptions[i].sensor_name != SENSOR_SUBSCRIPTION_NAME_LAST; i++) {
		if (sensor_subscriptions[i].disable_reporting)
			continue;

		if (sensor_subscriptions[i].initialized == false)
			continue;

		bool add = false;
		if (sensor_subscriptions[i].last_sent_at + sensor_subscriptions[i].report_interval < time_now)
			add = true;
		else if ((sensor_subscriptions[i].current_value > sensor_subscriptions[i].sent_value) &&
			((sensor_subscriptions[i].current_value - sensor_subscriptions[i].sent_value) > sensor_subscriptions[i].reportable_change))
			add = true;
		else if ((sensor_subscriptions[i].sent_value > sensor_subscriptions[i].current_value) &&
			(sensor_subscriptions[i].sent_value - sensor_subscriptions[i].current_value) > sensor_subscriptions[i].reportable_change)
			add = true;

		if (add) {
			data_added = true;

			sensor_subscriptions[i].last_sent_at = time_now;
			sensor_subscriptions[i].sent_value = sensor_subscriptions[i].current_value;

			char key[2] = {sensor_subscriptions[i].sensor_name, 0};

			cbor_encode_map_set_int(&encoderMap, key, sensor_subscriptions[i].current_value);
		}
	}

	if (!data_added)
		return 0;

	cborError = cbor_encoder_close_container(&encoder, &encoderMap);
	if (cborError != CborNoError)
		return 0;

	return cbor_encoder_get_buffer_size(&encoder, pBuffer);
}

static void subscription_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	otDeviceRole device_role = otThreadGetDeviceRole(thread_ot_instance_get());
	if (device_role != OT_DEVICE_ROLE_CHILD && device_role != OT_DEVICE_ROLE_ROUTER && device_role != OT_DEVICE_ROLE_LEADER)
		return;

	if (subscription_settings.subscription_address.mFields.m32[0] == 0xFFFFFFFF &&
		subscription_settings.subscription_address.mFields.m32[1] == 0xFFFFFFFF &&
		subscription_settings.subscription_address.mFields.m32[2] == 0xFFFFFFFF &&
		subscription_settings.subscription_address.mFields.m32[3] == 0xFFFFFFFF) {
			return;
	}

	uint32_t time_now = otPlatAlarmMilliGetNow();

	if (otIp6IsAddressUnspecified(&subscription_settings.subscription_address)) {
		if (subscription_settings.last_sent_at + subscription_settings.subscription_interval > time_now)
			return;
		subscription_settings.last_sent_at = time_now;
		if (subscription_settings.subscription_interval < 60000)
			subscription_settings.subscription_interval *= 2;
		send_subscription_broadcast();
		return;
	}

	uint8_t buff[256];

	size_t buff_size = fill_subscriptions_packet(time_now, buff, sizeof(buff));
	if (buff_size == 0)
		return;

	otError       error = OT_ERROR_NONE;
	otMessage   * p_request;
	otMessageInfo message_info;
	otInstance  * p_instance = thread_ot_instance_get();

	do
	{
		p_request = otCoapNewMessage(p_instance, NULL);
		if (p_request == NULL)
			break;

		otCoapMessageInit(p_request, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);

		error = otCoapMessageAppendUriPathOptions(p_request, "rep");
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageAppendContentFormatOption(p_request, OT_COAP_OPTION_CONTENT_FORMAT_CBOR);
		if (error != OT_ERROR_NONE)
			break;

		error = otCoapMessageSetPayloadMarker(p_request);
		if (error != OT_ERROR_NONE)
			break;

		error = otMessageAppend(p_request, buff, buff_size);
		if (error != OT_ERROR_NONE)
			break;

		memset(&message_info, 0, sizeof(message_info));
		message_info.mPeerPort = OT_DEFAULT_COAP_PORT;

		message_info.mPeerAddr.mFields.m32[0] = subscription_settings.subscription_address.mFields.m32[0];
		message_info.mPeerAddr.mFields.m32[1] = subscription_settings.subscription_address.mFields.m32[1];
		message_info.mPeerAddr.mFields.m32[2] = subscription_settings.subscription_address.mFields.m32[2];
		message_info.mPeerAddr.mFields.m32[3] = subscription_settings.subscription_address.mFields.m32[3];

		error = otCoapSendRequest(p_instance, p_request, &message_info, NULL, NULL);
		if (error != OT_ERROR_NONE)
			break;
		
		blink_send_led();
	} while (false);

	if (error != OT_ERROR_NONE && p_request != NULL)
		otMessageFree(p_request);
}

void thread_coap_utils_init()
{
	otInstance * p_instance = thread_ot_instance_get();

	otError error = otCoapStart(p_instance, OT_DEFAULT_COAP_PORT);
	ASSERT(error == OT_ERROR_NONE);

	otCoapSetDefaultHandler(p_instance, coap_default_handler, NULL);

	m_boot_resource.mContext = p_instance;
	m_info_resource.mContext = p_instance;
	m_set_resource.mContext = p_instance;
	m_get_resource.mContext = p_instance;
	m_sub_resource.mContext = p_instance;

	error = otCoapAddResource(p_instance, &m_boot_resource);
	ASSERT(error == OT_ERROR_NONE);

	error = otCoapAddResource(p_instance, &m_info_resource);
	ASSERT(error == OT_ERROR_NONE);

	error = otCoapAddResource(p_instance, &m_set_resource);
	ASSERT(error == OT_ERROR_NONE);

	error = otCoapAddResource(p_instance, &m_get_resource);
	ASSERT(error == OT_ERROR_NONE);

	error = otCoapAddResource(p_instance, &m_sub_resource);
	ASSERT(error == OT_ERROR_NONE);

	uint32_t error_code = app_timer_create(&m_subscription_timer, APP_TIMER_MODE_REPEATED, subscription_timeout_handler);
	APP_ERROR_CHECK(error_code);

	error_code = app_timer_start(m_subscription_timer, APP_TIMER_TICKS(SUBSCRIPTION_TIMER_INTERVAL), NULL);
	APP_ERROR_CHECK(error_code);

	error_code = app_timer_create(&m_led_recv_timer, APP_TIMER_MODE_SINGLE_SHOT, bsp_recv_led_timer_handler);
	APP_ERROR_CHECK(error_code);

	error_code = app_timer_create(&m_led_send_timer, APP_TIMER_MODE_SINGLE_SHOT, bsp_send_led_timer_handler);
	APP_ERROR_CHECK(error_code);

	error_code = app_timer_create(&m_boot_timer, APP_TIMER_MODE_SINGLE_SHOT, boot_timer_handler);
	APP_ERROR_CHECK(error_code);

	memset(&m_icmp6_handler, 0, sizeof(m_icmp6_handler));
	m_icmp6_handler.mReceiveCallback = icmp_receive_callback;

	error = otIcmp6RegisterHandler(thread_ot_instance_get(), &m_icmp6_handler);
	ASSERT((error == OT_ERROR_NONE) || (error == OT_ERROR_ALREADY));
}
