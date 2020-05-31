#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "sdkconfig.h"
#include <stdio.h>

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define CHANNEL LEDC_CHANNEL_0
#define ESC_SIGNAL_GPIO_NUM 33

extern "C" {
void app_main(void) {
  // ***************************************************************************
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)GPIO_NUM_14;
  conf.scl_io_num = (gpio_num_t)GPIO_NUM_26;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

  // ***************************************************************************
  MPU6050 mpu = MPU6050();
  mpu.initialize();
  mpu.dmpInitialize();

  // This need to be setup individually
  mpu.setXGyroOffset(120);
  mpu.setYGyroOffset(71);
  mpu.setZGyroOffset(-84);
  mpu.setZAccelOffset(1788);

  mpu.setDMPEnabled(true);

  // ***************************************************************************
  // Apparently GPIO_NUM_34 does not need to be setup
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
  // Since the ADC has a range of 0V-1.1V we need to attenuate it by 11db
  // (divide it by 3.55) to read up to VDD_A (3.3V)
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db));

  // ***************************************************************************
  ledc_timer_config_t timer_conf = {.speed_mode = SPEED_MODE,
                                    .duty_resolution = LEDC_TIMER_12_BIT,
                                    .timer_num = LEDC_TIMER_0,
                                    .freq_hz = 50,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  ledc_channel_config_t ledc_conf = {.gpio_num = ESC_SIGNAL_GPIO_NUM,
                                     .speed_mode = SPEED_MODE,
                                     .channel = CHANNEL,
                                     .intr_type = LEDC_INTR_DISABLE,
                                     .timer_sel = LEDC_TIMER_0,
                                     .duty = 2048,
                                     .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_conf));

  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorFloat gravity; // [x, y, z]            gravity vector
  float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity
                // vector
  uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;       // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64];   // FIFO storage buffer
  uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU

  // ***************************************************************************
  for (;;) {
    // *************************************************************************
    int val = adc1_get_raw(ADC1_CHANNEL_0);
    // [0, 4096) -> [204.8, 409.6)
    uint32_t duty_cycle = val / 20 + 205;

    printf("ADC1 on GPIO34 has read the value: %d, duty_cycle: %d\n", val,
           duty_cycle);

    // *************************************************************************
    ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, CHANNEL));

    // *************************************************************************
    // mpuIntStatus = mpu.getIntStatus();
    // // get current FIFO count
    // fifoCount = mpu.getFIFOCount();

    // if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    //   // reset so we can continue cleanly
    //   mpu.resetFIFO();

    //   // otherwise, check for DMP data ready interrupt frequently)
    // } else if (mpuIntStatus & 0x02) {
    //   // wait for correct available data length, should be a VERY short wait
    //   while (fifoCount < packetSize)
    //     fifoCount = mpu.getFIFOCount();

    //   // read a packet from FIFO

    //   mpu.getFIFOBytes(fifoBuffer, packetSize);
    //   mpu.dmpGetQuaternion(&q, fifoBuffer);
    //   mpu.dmpGetGravity(&gravity, &q);
    //   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //   printf("YAW: %3.1f, ", ypr[0] * 180 / M_PI);
    //   printf("PITCH: %3.1f, ", ypr[1] * 180 / M_PI);
    //   printf("ROLL: %3.1f \n", ypr[2] * 180 / M_PI);
    // }

    // *************************************************************************
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
}
