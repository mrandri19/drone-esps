// Create an access point, then read from the serial and broadcast an UDP packet
// with the data read from the serial port.
// TODO(Andrea): Use Wifi Long-Range
// TODO(Andrea): This polling is bad, maybe use UART events?
// TODO(Andrea): find a way to measure and minimise latency
// why the fuck it is so slow?

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/uart.h"

#include "esp_netif.h"
#include "freertos/event_groups.h"
#include <sys/param.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define EXAMPLE_ESP_WIFI_SSID "drone-wifi"
#define EXAMPLE_ESP_WIFI_PASS "password"
#define EXAMPLE_ESP_WIFI_CHANNEL 1
#define EXAMPLE_MAX_STA_CONN 1

static const char *TAG = "wifi softAP";

static const int UART_NUM = UART_NUM_0;
static const int BUF_SIZE = 1024;
static const int RD_BUF_SIZE = BUF_SIZE;
static QueueHandle_t uart0_queue;

TaskHandle_t udp_client_task_handle = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac),
             event->aid);

    // Start the upd client task
    xTaskNotifyGive(udp_client_task_handle);

  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac),
             event->aid);
  }
}

void wifi_init_softap() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .ap = {.ssid = EXAMPLE_ESP_WIFI_SSID,
             .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
             .channel = EXAMPLE_ESP_WIFI_CHANNEL,
             .password = EXAMPLE_ESP_WIFI_PASS,
             .max_connection = EXAMPLE_MAX_STA_CONN,
             .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  };
  if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
           EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS,
           EXAMPLE_ESP_WIFI_CHANNEL);
}

#define PORT 8765

static void udp_client_task(void *pvParameters) {
  ESP_LOGI(TAG, "udp_client_task started, waiting to be notified");
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  ESP_LOGI(TAG, "udp_client_task has been notified");

  int addr_family = 0;
  int ip_protocol = 0;

  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;

  uart_event_t event;
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);

  for (;;) {
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", "255.255.255.255", PORT);

    for (;;) {
      // Waiting for UART event.
      if (xQueueReceive(uart0_queue, (void *)&event,
                        (portTickType)portMAX_DELAY)) {
        bzero(dtmp, RD_BUF_SIZE);
        ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);
        switch (event.type) {
        // Event of UART receving data
        /*We'd better handler data event fast, there would be much more data
        events than other types of events. If we take too much time on data
        event, the queue might be full.*/
        case UART_DATA:
          ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
          uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
          ESP_LOGI(TAG, "[DATA EVT]:");
          uart_write_bytes(UART_NUM, (const char *)dtmp, event.size);
          break;
        // Event of HW FIFO overflow detected
        case UART_FIFO_OVF:
          ESP_LOGI(TAG, "hw fifo overflow");
          // If fifo overflow happened, you should consider adding flow
          // control for your application. The ISR has already reset the rx
          // FIFO, As an example, we directly flush the rx buffer here in
          // order to read more data.
          uart_flush_input(UART_NUM);
          xQueueReset(uart0_queue);
          break;
        // Event of UART ring buffer full
        case UART_BUFFER_FULL:
          ESP_LOGI(TAG, "ring buffer full");
          // If buffer full happened, you should consider encreasing your
          // buffer size As an example, we directly flush the rx buffer here
          // in order to read more data.
          uart_flush_input(UART_NUM);
          xQueueReset(uart0_queue);
          break;
        // Event of UART RX break detected
        case UART_BREAK:
          ESP_LOGI(TAG, "uart rx break");
          break;
        // Event of UART parity check error
        case UART_PARITY_ERR:
          ESP_LOGI(TAG, "uart parity error");
          break;
        // Event of UART frame error
        case UART_FRAME_ERR:
          ESP_LOGI(TAG, "uart frame error");
          break;
        // Others
        default:
          ESP_LOGI(TAG, "uart event type: %d", event.type);
          break;
        }

        int err = sendto(sock, dtmp, event.size, 0,
                         (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
          ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
          break;
        }
      }
    }

    if (sock != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(sock, 0);
      close(sock);
    }
  }

  free(dtmp);
  vTaskDelete(NULL);
}

void init_uart() {
  uart_config_t uart_config = {.baud_rate = 921600,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .source_clk = UART_SCLK_APB};
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
}

void app_main(void) {
  init_uart();

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  xTaskCreate(udp_client_task,        // task function
              "udp_client",           // task name
              4096,                   // task stack depth
              NULL,                   // task parameters
              5,                      // task priority
              &udp_client_task_handle // used to pass out the handle of the task
                                      // being created
  );

  ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
  wifi_init_softap();
}
