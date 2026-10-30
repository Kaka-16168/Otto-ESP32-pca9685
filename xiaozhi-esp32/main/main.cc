#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "application.h"
#include "pca9685.h"  // <-- 新增：引入 PCA9685 驱动头文件

#define TAG "main"

extern "C" void app_main(void)
{
    // 初始化 WiFi 配置所需的 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ==========================================
    // 新增：初始化 PCA9685 舵机控制板
    // ==========================================
    ESP_LOGI(TAG, "Initializing PCA9685...");
    // 使用 static 修饰，保证其生命周期与程序一致
    static Pca9685 pca;
    esp_err_t pca_ret = pca.Init();
    
    if (pca_ret == ESP_OK) {
        ESP_LOGI(TAG, "PCA9685 initialized successfully.");
        
        // 可选：这里可以加一个简单的测试动作，看看舵机有没有反应
        // 让 0 号通道的舵机转到 90 度位置（如果你的舵机接在 0 号口）
        // pca.SetServoAngle(0, 90);
        
    } else {
        ESP_LOGE(TAG, "Failed to initialize PCA9685! Error: %s", esp_err_to_name(pca_ret));
    }
    // ==========================================

    // 初始化并运行主应用程序（小智 AI）
    auto& app = Application::GetInstance();
    app.Initialize();
    app.Run();  // 这个函数会运行主事件循环，永不返回
}
