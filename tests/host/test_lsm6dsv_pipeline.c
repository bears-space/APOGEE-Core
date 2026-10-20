/* Host integration test: real adapter, ST register/conversion code and
 * pipeline; only the ESP32 hardware and FreeRTOS scheduling/queues are
 * substituted. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../components/telemetry/telemetry.c"
#include "../../main/sensor/LSM6DSV_sensor.c"

static uint8_t registers[256];
static bool ready_accel, ready_gyro;
static esp_err_t read_error, accel_error, gyro_error, timer_error;
static int16_t accel_raw[3], gyro_raw[3];
static unsigned raw_reads;
static uint64_t timer_period;
static int64_t clock_us = 123456;
static bool initialized;
static bool task_fail;

static int32_t read_register(void* arg, uint8_t reg, uint8_t* data,
                             uint16_t len) {
    memcpy(data, registers + reg, len);
    return 0;
}
static int32_t write_register(void* arg, uint8_t reg, const uint8_t* data,
                              uint16_t len) {
    memcpy(registers + reg, data, len);
    return 0;
}
static const stmdev_ctx_t context = {.read_reg = read_register,
                                     .write_reg = write_register};
const stmdev_ctx_t* ve_lsm6dsv_get_ctx(void) { return &context; }
esp_err_t ve_lsm6dsv_init(const ve_lsm6dsv_config_t* cfg) {
    if (initialized) return ESP_ERR_INVALID_STATE;
    initialized = true;
    return ESP_OK;
}
esp_err_t ve_lsm6dsv_deinit(void) {
    initialized = false;
    return ESP_OK;
}
esp_err_t ve_lsm6dsv_data_ready(bool* hg, bool* accel, bool* gyro, bool* temp) {
    *accel = ready_accel;
    *gyro = ready_gyro;
    return read_error;
}
esp_err_t ve_lsm6dsv_read_accel_raw(int16_t* out) {
    ++raw_reads;
    memcpy(out, accel_raw, sizeof(accel_raw));
    ready_accel = false;
    return accel_error;
}
esp_err_t ve_lsm6dsv_read_gyro_raw(int16_t* out) {
    ++raw_reads;
    memcpy(out, gyro_raw, sizeof(gyro_raw));
    ready_gyro = false;
    return gyro_error;
}
int64_t esp_timer_get_time(void) { return clock_us; }
esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* out) {
    *out = (void*)1;
    return ESP_OK;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period) {
    timer_period = period;
    return timer_error;
}
esp_err_t esp_timer_delete(esp_timer_handle_t timer) { return ESP_OK; }
BaseType_t xTaskCreate(void (*fn)(void*), const char* name, unsigned stack,
                       void* arg, unsigned priority, TaskHandle_t* out) {
    if (task_fail) return pdFALSE;
    *out = (void*)1;
    return pdPASS;
}
void vTaskDelete(TaskHandle_t task) {}
void xTaskNotifyGive(TaskHandle_t task) {}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t timeout) { return 1; }
BaseType_t xTaskNotify(TaskHandle_t task, uint32_t bits, eNotifyAction action) {
    return pdPASS;
}
void host_log(const char* tag, const char* fmt, ...) {}
const char* esp_err_to_name(esp_err_t err) { return "mock error"; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void*)1; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout) {
    return pdTRUE;
}
void xSemaphoreGive(SemaphoreHandle_t sem) {}
void vSemaphoreDelete(SemaphoreHandle_t sem) {}

struct host_queue {
    unsigned size, depth, count, head, tail;
    unsigned char data[];
};
QueueHandle_t xQueueCreate(unsigned depth, unsigned size) {
    QueueHandle_t q = calloc(1, sizeof(*q) + depth * size);
    q->depth = depth;
    q->size = size;
    return q;
}
void vQueueDelete(QueueHandle_t q) { free(q); }
BaseType_t xQueueSend(QueueHandle_t q, const void* data, TickType_t timeout) {
    if (q->count == q->depth) return pdFALSE;
    memcpy(q->data + q->tail * q->size, data, q->size);
    q->tail = (q->tail + 1) % q->depth;
    ++q->count;
    return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void* data, TickType_t timeout) {
    if (!q->count) return pdFALSE;
    memcpy(data, q->data + q->head * q->size, q->size);
    q->head = (q->head + 1) % q->depth;
    --q->count;
    return pdTRUE;
}

static void set_config(lsm6dsv320x_xl_full_scale_t accel_fs,
                       lsm6dsv320x_gy_full_scale_t gyro_fs,
                       lsm6dsv320x_data_rate_t odr) {
    assert(lsm6dsv320x_xl_full_scale_set(&context, accel_fs) == 0);
    assert(lsm6dsv320x_gy_full_scale_set(&context, gyro_fs) == 0);
    assert(lsm6dsv320x_xl_setup(&context, odr,
                                LSM6DSV320X_XL_HIGH_PERFORMANCE_MD) == 0);
    assert(lsm6dsv320x_gy_setup(&context, odr,
                                LSM6DSV320X_GY_HIGH_PERFORMANCE_MD) == 0);
}

int main(void) {
    assert(pipeline_init() == ESP_OK);
    set_config(LSM6DSV320X_4g, LSM6DSV320X_2000dps, LSM6DSV320X_ODR_AT_120Hz);
    sensor_channel_t* channel = NULL;
    assert(lsm6dsv_sensor_init(&channel) == ESP_OK);
    assert(channel && channel->config->nominal_period_us == 8333);
    assert(timer_period == 4166);
    sensor_channel_t* duplicate = channel;
    assert(lsm6dsv_sensor_init(&duplicate) == ESP_ERR_INVALID_STATE);
    assert(duplicate == NULL);

    sensor_measurement_t m = {0}, received;
    raw_reads = 0;
    ready_accel = true;
    ready_gyro = false;
    assert(read_measurement(&m) == ESP_ERR_NOT_FOUND && raw_reads == 0);
    ready_gyro = true;
    accel_raw[2] = 8197;  // about 1 g at +/-4 g
    gyro_raw[0] = -4;
    gyro_raw[1] = 2;
    gyro_raw[2] = 6;
    assert(read_measurement(&m) == ESP_OK);
    assert(m.timestamp_us == (uint64_t)clock_us &&
           m.flags == MEASUREMENT_VALID);
    assert(fabsf(m.data.imu.acceleration[2] - 9.80665f) < 0.001f);
    assert(fabsf(m.data.imu.angular_velocity[0] - (-0.28f * 0.01745329252f)) <
           1e-7f);
    assert(fabsf(m.data.imu.angular_velocity[2] - (0.42f * 0.01745329252f)) <
           1e-7f);
    assert(pipeline_submit_measurement(channel, &m, 0) == ESP_OK);
    assert(xQueueReceive(channel->queue, &received, 0) == pdPASS);
    assert(received.sensor_id == 1 && received.type == MEASUREMENT_IMU &&
           received.sequence == 0);
    assert(received.timestamp_us == m.timestamp_us);
    assert(read_measurement(&m) == ESP_ERR_NOT_FOUND);  // no duplicate sample
    read_error = ESP_FAIL;
    assert(read_measurement(&m) == ESP_FAIL);
    read_error = ESP_OK;

    sensor_measurement_t before = m;
    ready_accel = ready_gyro = true;
    accel_error = ESP_FAIL;
    assert(read_measurement(&m) == ESP_FAIL);
    assert(memcmp(&m, &before, sizeof(m)) == 0);
    accel_error = ESP_OK;
    ready_accel = ready_gyro = true;
    gyro_error = ESP_FAIL;
    assert(read_measurement(&m) == ESP_FAIL);
    assert(memcmp(&m, &before, sizeof(m)) == 0);
    gyro_error = ESP_OK;

    // Every menuconfig scale must select the corresponding ST conversion.
    for (unsigned a = 0; a < 4; ++a) {
        for (unsigned g = 1; g <= 5; ++g) {
            set_config(a, g, LSM6DSV320X_ODR_AT_240Hz);
            assert(configure_sampling() == ESP_OK);
            assert(channel_config.nominal_period_us == 4167);
            accel_raw[0] = 1000;
            gyro_raw[0] = -1000;
            ready_accel = ready_gyro = true;
            assert(read_measurement(&m) == ESP_OK);
            const float expected_a = 1000 * 0.061f * (1U << a) * 0.00980665f;
            const float expected_g =
                -1000 * 8.75f * (1U << (g - 1)) * 0.00001745329252f;
            assert(fabsf(m.data.imu.acceleration[0] - expected_a) < 1e-5f);
            assert(fabsf(m.data.imu.angular_velocity[0] - expected_g) < 1e-5f);
        }
    }
    // Mismatched sensor rates cannot form the paired IMU channel.
    assert(lsm6dsv320x_gy_setup(&context, LSM6DSV320X_ODR_AT_120Hz,
                                LSM6DSV320X_GY_HIGH_PERFORMANCE_MD) == 0);
    assert(configure_sampling() == ESP_ERR_NOT_SUPPORTED);
    set_config(LSM6DSV320X_4g, LSM6DSV320X_2000dps, LSM6DSV320X_ODR_AT_240Hz);
    assert(configure_sampling() == ESP_OK);
    ready_accel = ready_gyro = true;
    accel_raw[0] = INT16_MAX;
    assert(read_measurement(&m) == ESP_OK);
    assert(m.flags == (MEASUREMENT_VALID | MEASUREMENT_SATURATED));
    for (unsigned i = 0; i < 10; ++i)
        assert(pipeline_submit_measurement(channel, &m, 0) == ESP_OK);
    assert(pipeline_submit_measurement(channel, &m, 0) == ESP_ERR_TIMEOUT);
    assert(channel->dropped_measurements == 1);
    for (unsigned i = 0; i < 10; ++i)
        assert(xQueueReceive(channel->queue, &received, 0) == pdPASS);
    assert(pipeline_submit_measurement(channel, &m, 0) == ESP_OK);
    assert(xQueueReceive(channel->queue, &received, 0) == pdPASS &&
           received.sequence == 12);
    vTaskDelete(sensor_task);
    sensor_task = NULL;
    esp_timer_delete(sample_timer);
    sample_timer = NULL;
    assert(pipeline_destroy_channel(1) == ESP_OK);
    ve_lsm6dsv_deinit();

    set_config(LSM6DSV320X_4g, LSM6DSV320X_2000dps, LSM6DSV320X_ODR_OFF);
    assert(lsm6dsv_sensor_init(&channel) == ESP_ERR_NOT_SUPPORTED);
    assert(!initialized && channel == NULL);
    set_config(LSM6DSV320X_4g, LSM6DSV320X_2000dps, LSM6DSV320X_ODR_AT_120Hz);
    task_fail = true;
    assert(lsm6dsv_sensor_init(&channel) == ESP_ERR_NO_MEM);
    assert(sensor_registry.count == 0 && !initialized && sample_timer == NULL);
    task_fail = false;
    timer_error = ESP_FAIL;
    assert(lsm6dsv_sensor_init(&channel) == ESP_FAIL);
    assert(sensor_registry.count == 0 && !initialized && sensor_task == NULL);
    timer_error = ESP_OK;
    assert(lsm6dsv_sensor_init(&channel) ==
           ESP_OK);  // retry after startup failures
    sensor_task = NULL;
    sample_timer = NULL;
    assert(pipeline_destroy_channel(1) == ESP_OK);
    assert(pipeline_deinit() == ESP_OK);
    puts("LSM6DSV pipeline host tests passed");
    return 0;
}
