#include <stdio.h>

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TAG "blink"
#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define CHANNEL LEDC_CHANNEL_0
#define ESC_SIGNAL_GPIO_NUM 33
#define PERIOD 20 // 20ms => f = 1/T = 50Hz
#define UART_NUM UART_NUM_0
#define DUTY_RESOLUTION LEDC_TIMER_19_BIT

uint32_t ms_to_duty(float ms) { return (1 << DUTY_RESOLUTION) * (ms / 20.0f); }

float clamp(float a, float b, float c) {
  if (b < a) {
    return a;
  }
  if (b > c) {
    return c;
  }
  return b;
}

static const int BUF_SIZE = 1024;
static const int RD_BUF_SIZE = BUF_SIZE;
static QueueHandle_t uart0_queue;

extern "C" {
void app_main(void) {
  // ***************************************************************************
  uart_config_t uart_config = {};
  uart_config.baud_rate = 115200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  // Install the driver, and get the queue
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM,     // UART_NUM
                                      BUF_SIZE * 2, // rx_buffer_size
                                      BUF_SIZE * 2, // tx_buffer_size
                                      20,           // queue_size
                                      &uart0_queue, // uart_queue
                                      0             // intr_alloc_flags
                                      ));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

  // Set uart pin
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  // ***************************************************************************
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  // conf.sda_io_num = (gpio_num_t)GPIO_NUM_14;
  conf.sda_io_num = (gpio_num_t)14;
  // conf.scl_io_num = (gpio_num_t)GPIO_NUM_26;
  conf.scl_io_num = (gpio_num_t)26;
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
                                    .duty_resolution = DUTY_RESOLUTION,
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

  // **************************************************************************

  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorFloat gravity; // [x, y, z]            gravity vector
  float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity
                // vector
  uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;       // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64];   // FIFO storage buffer
  uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU

  // With this calibration
  float calibration_ms = 0.9075;
  uint32_t duty_cycle = ms_to_duty(calibration_ms);
  // **************************************************************************
  // Stay 5s at the minimum duty cycle to calibrate the ESC
  printf("Calibrating the ESC at %fms duty for the next 4 seconds\n",
         calibration_ms);
  ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, CHANNEL, duty_cycle));
  ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, CHANNEL));
  vTaskDelay(pdMS_TO_TICKS(4000));

  // ***************************************************************************
  uart_event_t event;
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);

  // Starting value
  float ms = 1.0f;

  float e_integral = 0.0f;
  float e_previous = 0.0f;

  // https: // en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
  float K_u = 1.0;
  float T_u = 1.0;

  // NO overshoot
  float K_p = K_u / 5;
  float T_i = T_u / 2;
  float T_d = T_u / 3;

  float r = 0.0f;

  // Normalize the angle between -1.0 and 1.0
  float K_roll = -1.0f / 25.0f;
  float y_des = clamp(-1.0f, K_roll * r, 1.0f);

  for (;;) {
    float roll = 0.0f;

    // *************************************************************************
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();

      // otherwise, check for DMP data ready interrupt frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
      }

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // printf("YAW: %3.1f, ", ypr[0] * 180 / M_PI);
      // printf("PITCH: %3.1f, ", ypr[1] * 180 / M_PI);
      roll = ypr[2] * 180 / M_PI;
    }

    // *************************************************************************
    // if (xQueueReceive(uart0_queue, (void *)&event,
    //                   (portTickType)pdMS_TO_TICKS(0))) {
    //   bzero(dtmp, RD_BUF_SIZE);
    //   switch (event.type) {
    //     case UART_DATA:
    //       uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
    //       if (dtmp[0] == 'w') {
    //         ms += 0.0005;
    //       }
    //       if (dtmp[0] == 's') {
    //         ms -= 0.0005;
    //       }
    //       break;
    //     default:
    //       ESP_LOGI(TAG, "uart event type: %d", event.type);
    //       break;
    //   }
    // }
    // ms = clamp(1.0f, ms, 1.08f);

    // *************************************************************************

    float y = clamp(-1.0f, K_roll * roll, 1.0f);

    float e = y_des - y;

    e_integral += (e * ((float)PERIOD / 1000.0f));

    float e_derivative = (e - e_previous) / ((float)PERIOD / 1000.0f);
    e_previous = e;

    float u = K_p * (e + (1 / T_i) * e_integral + T_d * e_derivative);

    ms = clamp(1.0f, 0.04f * u + 1.04f, 1.08f);

    uint32_t duty_cycle = ms_to_duty(ms);

    printf("ms: % 8.4f, e_i: % 8.4f, e: % 8.4f, e_d: % 8.4f\n", ms, e_integral,
           e, e_derivative);

    // *************************************************************************
    ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, CHANNEL));
    // *************************************************************************
    vTaskDelay(pdMS_TO_TICKS(PERIOD));
  }
}
}
