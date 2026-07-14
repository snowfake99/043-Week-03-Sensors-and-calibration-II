#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
/* ================== ตั้งค่า ADC ================== */
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_6     // GPIO34 บน ESP32
#define ADC_ATTEN       ADC_ATTEN_DB_12   // รองรับแรงดันสูงสุด ~3.3V (เดิมชื่อ DB_11)
#define ADC_BITWIDTH    ADC_BITWIDTH_12   // 0-4095
 
static const char *TAG = "LDR_CAL";
 
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration = false;
 
/*
* ================== สมการ Calibration (ADC -> Lux) ==================
* ค่าที่ใส่ไว้นี้เป็นค่า "ตัวอย่าง" จากใบงาน (y = 0.42x - 50)
* ให้นักศึกษาแทนที่ด้วยค่า m (slope) และ b (intercept) ที่ได้จาก
* Trendline ในกราฟ Excel / Google Sheet ของตัวเองหลังเก็บข้อมูลจริง
*
* ถ้ากราฟเป็นเส้นตรง (Linear):      Lux = m * ADC + b
* ถ้ากราฟเป็น Polynomial ดีกว่า:    Lux = a*ADC^2 + b*ADC + c
*/
#define USE_POLYNOMIAL   0   // 0 = ใช้สมการ Linear, 1 = ใช้สมการ Polynomial
 
// ----- ค่าคงที่สำหรับสมการ Linear -----
#define CAL_M   0.42f
#define CAL_B  -50.0f
 
// ----- ค่าคงที่สำหรับสมการ Polynomial (ax^2 + bx + c) -----
#define CAL_A   0.0001f
#define CAL_B2  0.3f
#define CAL_C  -40.0f
 
/**
* @brief แปลงค่า ADC raw ให้เป็นค่าความสว่าง (Lux)
*        โดยใช้สมการ Calibration ที่ได้จากกราฟ
*/
float ldr_raw_to_lux(int raw)
{
    float lux;
 
#if USE_POLYNOMIAL
    lux = CAL_A * raw * raw + CAL_B2 * raw + CAL_C;
#else
    lux = CAL_M * raw + CAL_B;
#endif
 
    if (lux < 0) {
        lux = 0;
    }
 
    return lux;
}
 
/**
* @brief ตั้งค่า ADC calibration scheme (แปลง raw -> mV ให้แม่นยำขึ้น)
*        รองรับทั้งแบบ Curve Fitting และ Line Fitting ขึ้นกับชิป
*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                  adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
 
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif
 
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif
 
    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Success");
    } else {
        ESP_LOGW(TAG, "ADC Calibration ไม่รองรับหรือไม่สำเร็จ (จะใช้ raw ADC แทน)");
    }
 
    return calibrated;
}
 
void app_main(void)
{
    /* 1) สร้าง ADC oneshot unit */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
 
    /* 2) ตั้งค่า channel */
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &chan_config));
 
    /* 3) ทำ ADC calibration (raw -> mV) */
    do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc1_cali_handle);
 
    while (1) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw));
 
        int voltage_mv = 0;
        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage_mv));
        }
 
        float lux = ldr_raw_to_lux(raw);
 
        ESP_LOGI(TAG, "ADC Raw = %d | Voltage = %d mV | Lux (calibrated) = %.2f",
                 raw, voltage_mv, lux);
 
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
 