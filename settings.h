#ifndef __SETTINGS__H__
#define __SETTINGS__H__

#define INFO_FIRMWARE_TYPE                   "dimmer"
#define INFO_FIRMWARE_VERSION                "1.0.0"

#define SUBSCRIPTION_TIMER_INTERVAL          500
#define INTERNAL_TEMPERATURE_TIMER_INTERVAL  1000
#define VOLTAGE_TIMER_INTERVAL               1000

#define DEFAULT_POLL_PERIOD                  120000
#define DEFAULT_POLL_PERIOD_FAST             50
#define DEFAULT_POLL_PERIOD_FAST_TIMEOUT     500
#define DEFAULT_CHILD_TIMEOUT                240

#define ADC_SAMPLES_PER_CHANNEL              32

#define LED_SEND_NOTIFICATION                BSP_BOARD_LED_0
#define LED_RECV_NOTIFICATION                BSP_BOARD_LED_1
#define LED_ROUTER_ROLE                      BSP_BOARD_LED_2
#define LED_CHILD_ROLE                       BSP_BOARD_LED_3

#define DIMMER_CHANNEL_PIN_R                 LED2_DR // pwm channel 0
#define DIMMER_CHANNEL_PIN_G                 LED2_DG // pwm channel 1
#define DIMMER_CHANNEL_PIN_B                 LED2_DB // pwm channel 2
#define DIMMER_CHANNEL_PIN_W                 LED2_DW // pwm channel 3
#define DIMMER_PWM_INVERSION                 0 // NRF_DRV_PWM_PIN_INVERTED

#endif // __SETTINGS__H__
