#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "mpu6050/eMPL/inv_mpu.h"
#include "mpu6050/eMPL/inv_mpu_dmp_motion_driver.h"
#include "esp_log.h"

#define MOITOR_A_PWM_PIN GPIO_NUM_32
#define MOITOR_A_IN1_PIN GPIO_NUM_33
#define MOITOR_A_IN2_PIN GPIO_NUM_25
#define MOITOR_A_E1_PIN GPIO_NUM_26
#define MOITOR_A_E2_PIN GPIO_NUM_27

#define MOITOR_B_PWM_PIN GPIO_NUM_14
#define MOITOR_B_IN1_PIN GPIO_NUM_13
#define MOITOR_B_IN2_PIN GPIO_NUM_19
#define MOITOR_A_E1_PIN GPIO_NUM_18
#define MOITOR_A_E2_PIN GPIO_NUM_5

#define MPU6050_INT_PIN GPIO_NUM_21

// mpu6050: 消息队列
static QueueHandle_t mpu6050_evt_queue = NULL;
// mpu6050: 数据读写信号量
SemaphoreHandle_t mpu6050_mutex_sem;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float temperature;
} mpu6050_data_t;

mpu6050_data_t mpu6050_data;

// 中断服务例程
static void IRAM_ATTR mpu6050_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    // 将中断引脚号发送到队列
    xQueueSendFromISR(mpu6050_evt_queue, &gpio_num, NULL);
}

// 任务：处理 MPU6050 数据
void mpu6050_read_task(void *arg)
{
    uint32_t io_num;
    while (pdTRUE)
    {
        // 等待中断信号
        if (xQueueReceive(mpu6050_evt_queue, &io_num, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(mpu6050_mutex_sem, portMAX_DELAY))
            {
                // 从 MPU6050 获取数据
                uint8_t res = mpu_dmp_get_data(&mpu6050_data.pitch, &mpu6050_data.roll, &mpu6050_data.yaw);
                mpu6050_data.temperature = MPU_Get_Temperature();
                if (res != 0)
                {
                    ESP_LOGE("main", "mpu read error: %d", res);
                }
                xSemaphoreGive(mpu6050_mutex_sem);
            }
        }
    }
}

// 任务：打印mpu6050 数据
void mpu6050_print_task(void *arg)
{
    mpu6050_data_t local_mpu6050_data;
    while (pdTRUE)
    {
        if (xSemaphoreTake(mpu6050_mutex_sem, portMAX_DELAY) == pdTRUE)
        {
            local_mpu6050_data = mpu6050_data;
            xSemaphoreGive(mpu6050_mutex_sem);
        }
        ESP_LOGI("main", "pitch: %.3f, roll: %.3f, yaw: %.3f , t: %.3f", local_mpu6050_data.pitch, local_mpu6050_data.roll, local_mpu6050_data.yaw, local_mpu6050_data.temperature);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    gpio_set_level(MOITOR_A_PWM_PIN, 0);
    gpio_set_level(MOITOR_A_IN1_PIN, 0);
    gpio_set_level(MOITOR_A_IN2_PIN, 0);

    gpio_set_level(MOITOR_B_PWM_PIN, 0);
    gpio_set_level(MOITOR_B_IN1_PIN, 0);
    gpio_set_level(MOITOR_B_IN2_PIN, 0);
    // 初始化 MPU6050
    mpu6050_config_t cfg = {
        .scl = GPIO_NUM_23,
        .sda = GPIO_NUM_22,
        .freq = 200 * 1000};
    mpu_dmp_init(&cfg);

    // 配置 GPIO
    gpio_config_t mpu6050_int_pin_cfg = {
        .pin_bit_mask = 1ULL << MPU6050_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE};
    gpio_config(&mpu6050_int_pin_cfg);

    // 创建队列
    mpu6050_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // 创建信号量
    mpu6050_mutex_sem = xSemaphoreCreateMutex();

    // 安装中断服务
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU6050_INT_PIN, mpu6050_handler, (void *)MPU6050_INT_PIN);

    // 创建任务处理 MPU6050 数据
    xTaskCreatePinnedToCore(mpu6050_read_task, "mpu6050_read_task", 2048, NULL, 10, NULL, 1);
    // 创建任务打印 MPU6050 数据
    xTaskCreatePinnedToCore(mpu6050_print_task, "mpu6050_print_task", 2048, NULL, 10, NULL, 1);
}
