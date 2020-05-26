#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define CHANNEL LEDC_CHANNEL_0

void app_main(void) {
  // ***************************************************************************
  // Apparently GPIO_NUM_34 does not need to be setup
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
  // Since the ADC has a range of 0V-1.1V we need to attenuate it by 11db
  // (divide it by 3.55) to read up to VDD_A (3.3V)
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL_0, ADC_ATTEN_11db));

  // ***************************************************************************
  ledc_timer_config_t timer_conf = {
      .speed_mode = SPEED_MODE,
      .duty_resolution = LEDC_TIMER_12_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 50,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  ledc_channel_config_t ledc_conf = {.gpio_num = 33,
                                     .speed_mode = SPEED_MODE,
                                     .channel = CHANNEL,
                                     .intr_type = LEDC_INTR_DISABLE,
                                     .timer_sel = LEDC_TIMER_0,
                                     .duty = 2048,
                                     .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_conf));

  for (;;) {
    int val = adc1_get_raw(ADC1_CHANNEL_0);
    // [0, 4096) -> [204.8, 409.6)
    uint32_t duty_12_bit = val / 20 + 205;

    printf("ADC1 on GPIO34 has read the value: %d, duty_12_bit: %d\n", val,
           duty_12_bit);

    // I can do this since both the ADC and the duty resolution are 12bits
    ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, CHANNEL, duty_12_bit));
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, CHANNEL));

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
