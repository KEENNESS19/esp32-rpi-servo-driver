
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "icm20948.h"
#include "esp_task_wdt.h"
#include "servo_driver.h"
#include "uart_comm.h"
#include "uart_config.h"
#include "servo_config.h"
#include "LED.h"
#include <stdlib.h>

#define PI_UART_NUM UART_NUM_1
icm20948_acce_value_t acce,acce1; //加速度
icm20948_gyro_value_t gyro,gyro1; //陀螺仪
icm20948_handle_t imu_handle = NULL;
static const char *TAG = "MAIN";
typedef enum {
    PI_COMMOND_NULL = 0x00,  
    PI_COMMOND_READ_SPEED = 0x01,    
    PI_COMMOND_READ_POSITION = 0x02,    
    PI_COMMOND_READ_ACCE = 0x03,   
    PI_COMMOND_READ_GYRO = 0x04,   
    PI_COMMOND_READ_TEMP = 0x05,
    PI_COMMOND_CALIBRATE = 0x06,
    PI_COMMOND_INIT_SERVOS = 0x07
} pi_cmd;

typedef struct {
    uint16_t position;      
    uint16_t speed;         
    bool speed_direction;   
    uint16_t load;          
    uint8_t voltage;        
    uint8_t temperature;    
    bool moving;           
    uint8_t mode;         
} servo_status_t;


esp_err_t esp_pi_command_handler(const uint8_t *data, size_t length);
esp_err_t esp_read_servo_status(pi_cmd cmd, const uint8_t* servo_ids, uint8_t id_count, uint32_t timeout_ms);
esp_err_t esp_handle_action_command(const uint8_t *data, size_t length);

// 从树莓派读取命令
int esp_pi_data_read(void)
{
    uint8_t buffer[128];
    int length = 0;

    size_t available = 0;
    esp_err_t err = uart_get_buffered_data_len(UART_NUM_1, &available);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART get buffered data failed: %d", err);
        return -1;
    }
    
    if (available > 0) {

        int to_read = (available > sizeof(buffer)) ? sizeof(buffer) : available;
        length = uart_read_bytes(UART_NUM_1, buffer, to_read, pdMS_TO_TICKS(20));
        
        if (length > 0) {
            #if SERVO_DEBUG_ENABLED
            ESP_LOGI(TAG, "Read %d bytes from PI (available was %d)", length, (int)available);
            
            char hex_str[300] = "";
            char temp[10];
            for (int i = 0; i < length && i < 50; i++) { 
                sprintf(temp, "%02X ", buffer[i]);
                strcat(hex_str, temp);
            }
            if (length > 50) {
                strcat(hex_str, "...");
            }
            ESP_LOGI(TAG, "Data: %s", hex_str);
            #endif
            
            esp_pi_command_handler(buffer, length);
            

            if (length == to_read && available > sizeof(buffer)) {
                ESP_LOGW(TAG, "Buffer overflow: had %d bytes, only read %d", 
                        (int)available, length);

                // uart_flush_input(UART_NUM_1);  // 直接清空
            }
        }
    }
    
    return length;
}

//发送给pi的数据包
esp_err_t esp_pi_data_send(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0) {
        ESP_LOGE(TAG, "Invalid data to send");
        return ESP_ERR_INVALID_ARG;
    }
    

    #if SERVO_DEBUG_ENABLED  
    ESP_LOGI(TAG, "数据长度: %d 字节", length);
    
    // 显示数据内容
    char hex_str[512] = "";
    for (int i = 0; i < length && i < 32; i++) {
        char temp[10];
        sprintf(temp, "%02X ", data[i]);
        strcat(hex_str, temp);
    }
    ESP_LOGI(TAG, "数据内容: %s%s", hex_str, (length > 32) ? "..." : "");
    #endif
    
    int tx_bytes = uart_write_bytes(UART_NUM_1, (const char *)data, length);
    
    if (tx_bytes != length) {
        ESP_LOGE(TAG, "UART write failed: sent %d/%d bytes", tx_bytes, length);
        return ESP_FAIL;
    }
    
    esp_err_t wait_ret = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(200));
    if (wait_ret != ESP_OK) {
        ESP_LOGE(TAG, "等待发送完成超时: %d", wait_ret);
        return wait_ret;
    }
    
    #if SERVO_DEBUG_ENABLED 
    ESP_LOGI(TAG, "成功发送 %d 字节给树莓派", tx_bytes);
    #endif
    
    return ESP_OK;
}

//自定义协议包
uint8_t eps_pi_build_packet(pi_cmd cmd, const uint8_t* parameters, uint8_t param_count, uint8_t* packet)
{
    if (packet == NULL) {
        ESP_LOGE(TAG, "Packet buffer is NULL");
        return 0;
    }
    
    if (param_count > 0 && parameters == NULL) {
        ESP_LOGE(TAG, "Parameters is NULL but param_count > 0");
        return 0;
    }
    
    bool has_cmd = (cmd != PI_COMMOND_NULL);
    
    uint8_t total_len = 3 + (has_cmd ? 1 : 0) + param_count; 
    
    if (total_len > 80) {
        ESP_LOGE(TAG, "Packet too long: %d > 80", total_len);
        return 0;
    }
    
    uint8_t idx = 0;
    
    packet[idx++] = 0xFF;
    
    if (has_cmd) {
        packet[idx++] = (uint8_t)cmd;
    }
    
    for (uint8_t i = 0; i < param_count; i++) {
        packet[idx++] = parameters[i];
    }
    
    uint8_t start_idx = 1;  
    uint8_t data_len = idx - 1; 
    uint8_t checksum = servo_calculate_checksum(packet, start_idx, data_len);

    packet[idx++] = checksum;
    packet[idx++] = 0xFE;
    
    return idx; 
}


//解析树莓派action命令
esp_err_t esp_handle_action_command(const uint8_t *data, size_t length)
{
    if (data == NULL || length != 24) {
        ESP_LOGE(TAG, "运动命令: 无效的数据长度 %d", length);
        return ESP_ERR_INVALID_SIZE;
    }
    
    const uint16_t DEFAULT_SPEED = 100;   
    const uint16_t DEFAULT_TORQUE = 500;  
    
    uint16_t angles[12];

    for (int i = 0; i < 12; i++) {
        if (servo_get_END() == 0) {  
            angles[i] = data[i * 2] | (data[i * 2 + 1] << 8);
        } else { 
            angles[i] = (data[i * 2] << 8) | data[i * 2 + 1];
        }
        
        #if SERVO_DEBUG_ENABLED
        ESP_LOGI(TAG, "舵机 %d 目标角度: %u", FULL_SERVO_IDS[i], angles[i]);
        #endif
    }
    
    uint16_t speeds[12];
    uint16_t torques[12];
    
    for (int i = 0; i < 12; i++) {
        speeds[i] = DEFAULT_SPEED;
        torques[i] = DEFAULT_TORQUE;
    }
    
    int status = servo_sync_set_position_speed_torque(FULL_SERVO_IDS, 12,angles, speeds, torques,100);  
    
    uint8_t response_packet[16];
    uint8_t response_data[1] = {(uint8_t)((status == SERVO_STATUS_OK) ? 12 : 0)};
    uint8_t response_len = eps_pi_build_packet(PI_COMMOND_NULL, response_data, 1, response_packet);
    
    if (response_len > 0) {
        esp_err_t send_ret = esp_pi_data_send(response_packet, response_len);
        if (send_ret != ESP_OK) {
            ESP_LOGE(TAG, "发送运动响应失败: %d", send_ret);
            return ESP_FAIL;
        }
    }
    return (status == SERVO_STATUS_OK) ? ESP_OK : ESP_FAIL;
}



// 读取舵机状态并返回pi

static int execute_sync_read_and_build_response(
    pi_cmd cmd, 
    const uint8_t* servo_ids, 
    uint8_t id_count,
    uint8_t addr, 
    uint8_t bytes_to_read,
    uint32_t timeout_ms)
{
    uint8_t** responses = (uint8_t**)malloc(id_count * sizeof(uint8_t*));
    uint8_t* response_sizes = (uint8_t*)malloc(id_count * sizeof(uint8_t));
    
    if (responses == NULL || response_sizes == NULL) {
        ESP_LOGE(TAG, "内存分配失败");
        if (responses) free(responses);
        if (response_sizes) free(response_sizes);
        return -1;
    }
    
    memset(responses, 0, id_count * sizeof(uint8_t*));
    memset(response_sizes, 0, id_count * sizeof(uint8_t));
    

    for (int i = 0; i < id_count; i++) {
        responses[i] = (uint8_t*)malloc(bytes_to_read);
        if (responses[i] == NULL) {
            ESP_LOGE(TAG, "舵机 %d 内存分配失败", i);
           
            for (int j = 0; j < i; j++) {
                free(responses[j]);
            }
            free(responses);
            free(response_sizes);
            return -1;
        }
    }
    
    int result = servo_sync_read(servo_ids, id_count, addr, bytes_to_read,
                               responses, response_sizes, timeout_ms);
    
    // 无论结果如何，都准备构建回复包
    uint8_t response_packet[128];
    uint8_t response_data[id_count * bytes_to_read];
    
    // 默认填充 0x00 或 0xFF 表示无效数据? 通常 0x00 更安全，或者根据协议定
    // 这里我们先保留原有的逻辑：如果是部分成功，就填数据，失败的填默认值
    // 如果完全失败(result <= 0)，则全填 0 或保持上次的值(但这很难)
    // 既然 Python 端解析时期望特定长度，我们必须填充完整长度
    
    memset(response_data, 0, sizeof(response_data)); // 初始化为 0
    
    int valid_count = 0;
    int data_idx = 0;
    
    if (result > 0) {
        for (int i = 0; i < id_count; i++) {
            if (response_sizes[i] == bytes_to_read) {
                memcpy(&response_data[data_idx], responses[i], bytes_to_read);
                valid_count++;
            }
            // 如果读取失败，response_data 中对应的位置已经是 0 了
            data_idx += bytes_to_read;
        }
    } else {
        // 如果完全失败，result <= 0，response_data 全为 0
        // valid_count = 0
        data_idx = id_count * bytes_to_read;
    }
    
    uint8_t response_len = eps_pi_build_packet(cmd, response_data, 
                                              data_idx, response_packet);
    
    if (response_len > 0) {
        esp_pi_data_send(response_packet, response_len);
    }
    
    // 释放内存
    for (int i = 0; i < id_count; i++) {
        free(responses[i]);
    }
    free(responses);
    free(response_sizes);
    
    return (result > 0) ? result : 0;
}

int esp_read_servo_status(pi_cmd cmd, const uint8_t* servo_ids, uint8_t id_count,
                         uint32_t timeout_ms)
{
    if (servo_ids == NULL || id_count == 0) {
        return -1;
    }
    
    switch (cmd) {
        case PI_COMMOND_READ_SPEED:
            return execute_sync_read_and_build_response(cmd, servo_ids, id_count,
                                                       SERVO_REG_CURRENT_SPEED_LOW, 2,
                                                       timeout_ms);
            
        case PI_COMMOND_READ_POSITION:
            return execute_sync_read_and_build_response(cmd, servo_ids, id_count,
                                                       SERVO_REG_CURRENT_POSITION_LOW, 2,
                                                       timeout_ms);
            
        case PI_COMMOND_READ_TEMP:
            return execute_sync_read_and_build_response(cmd, servo_ids, id_count,
                                                       SERVO_REG_CURRENT_TEMP, 1,
                                                       timeout_ms);
            
        case PI_COMMOND_READ_ACCE: {
            if (imu_handle == NULL) return 0;
            icm20948_raw_acce_value_t raw_acce;
            esp_err_t ret = icm20948_get_raw_acce(imu_handle, &raw_acce);
            
            if (ret == ESP_OK) {
                uint8_t response_data[6] = {
                    raw_acce.raw_acce_x & 0xFF,
                    (raw_acce.raw_acce_x >> 8) & 0xFF,
                    raw_acce.raw_acce_y & 0xFF,
                    (raw_acce.raw_acce_y >> 8) & 0xFF,
                    raw_acce.raw_acce_z & 0xFF,
                    (raw_acce.raw_acce_z >> 8) & 0xFF
                };
                
                uint8_t response_packet[128];
                uint8_t len = eps_pi_build_packet(cmd, response_data, 6, response_packet);
                if (len > 0) {
                    esp_pi_data_send(response_packet, len);
                }
                return 1;
            }
            return 0;
        }
            
        case PI_COMMOND_READ_GYRO: {
            if (imu_handle == NULL) return 0;
            icm20948_raw_gyro_value_t raw_gyro;
            esp_err_t ret = icm20948_get_raw_gyro(imu_handle, &raw_gyro);
            
            if (ret == ESP_OK) {
                uint8_t response_data[6] = {
                    raw_gyro.raw_gyro_x & 0xFF,
                    (raw_gyro.raw_gyro_x >> 8) & 0xFF,
                    raw_gyro.raw_gyro_y & 0xFF,
                    (raw_gyro.raw_gyro_y >> 8) & 0xFF,
                    raw_gyro.raw_gyro_z & 0xFF,
                    (raw_gyro.raw_gyro_z >> 8) & 0xFF
                };
                
                uint8_t response_packet[128];
                uint8_t len = eps_pi_build_packet(cmd, response_data, 6, response_packet);
                if (len > 0) {
                    esp_pi_data_send(response_packet, len);
                }
                return 1;
            }
            return 0;
        }

        case PI_COMMOND_CALIBRATE: {
             // 标定操作可能需要较长时间，建议先发送响应或在操作完成后发送
             // 这里我们在操作完成后发送结果，但注意Python端可能会超时
             int ret = servo_batch_calibrate_to_current_position(FULL_SERVO_IDS, 12, 50, 200, true);
             
             uint8_t response_data[1] = { (uint8_t)((ret > 0) ? 1 : 0) };
             uint8_t response_packet[16];
             uint8_t len = eps_pi_build_packet(cmd, response_data, 1, response_packet);
             if (len > 0) {
                 esp_pi_data_send(response_packet, len);
             }
             return 1;
        }

        case PI_COMMOND_INIT_SERVOS: {
             int ret = servo_initialize_12_servos(FULL_SERVO_IDS, 200);
             
             uint8_t response_data[1] = { (uint8_t)((ret > 0) ? 1 : 0) };
             uint8_t response_packet[16];
             uint8_t len = eps_pi_build_packet(cmd, response_data, 1, response_packet);
             if (len > 0) {
                 esp_pi_data_send(response_packet, len);
             }
             return 1;
        }

        default:
            ESP_LOGW(TAG, "未知命令: 0x%02X", cmd);
            return -1;
    }
}


// 根据命令类型调用相应的处理函数
esp_err_t esp_pi_command_handler(const uint8_t *data, size_t length)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (length < 3) {  
        return ESP_ERR_INVALID_SIZE;
    }
    
    if (data[length - 1] != 0xFE) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    uint8_t expected_checksum = servo_calculate_checksum(data, 1, length - 3);
    if (data[length - 2] != expected_checksum) {
        return ESP_ERR_INVALID_CRC;
    }
    
    switch (length) {
        case 4: {
            // 4字节数据包: FF CMD CHECK FE
            uint8_t cmd = data[1];
            return esp_read_servo_status(cmd,FULL_SERVO_IDS,12,200);
        }
        
        case 27: {
            // 27字节数据包: FF DATA(24字节) CHECK FE
            // data[1]-data[24]是24字节运动数据,数据按照FULL舵机顺序来，每舵机2byte
            return esp_handle_action_command(data + 1, 24);
        }
        
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}


void app_main(void)
{
  //初始化
  uart_comm_init(SERVO_UART_NUM,UART2_TXD_PIN,UART2_RXD_PIN,UART_BAUDRATE_2);
  uart_comm_init(PI_UART_NUM,UART1_TXD_PIN,UART1_RXD_PIN,UART_BAUDRATE_1);
  i2c_bus_init();
  led_init();
  // 注意：第一次运行时取消注释下面这行代码以初始化舵机参数（特别是设置最大角度限制）
  // 运行一次成功后，建议再次注释掉，避免每次启动都写EEPROM
  // servo_initialize_12_servos(FULL_SERVO_IDS, 200);
  
  // 确保所有舵机扭矩开启
  servo_batch_enable_torque(FULL_SERVO_IDS, 12, 0, 200);

  icm20948_configure(&imu_handle, ACCE_FS_2G, GYRO_FS_1000DPS);

  if (imu_handle == NULL) {
      ESP_LOGE(TAG, "IMU initialization failed!");
  } else {
      ESP_LOGI(TAG, "IMU initialized successfully, handle: %p", imu_handle);
  }

  //测试
  //舵机标定测试
  //servo_initialize_12_servos(FULL_SERVO_IDS,200);
  //servo_batch_calibrate_to_current_position(FULL_SERVO_IDS, 12, 50, 200, true);
  while (1) {
      esp_pi_data_read();
      vTaskDelay(pdMS_TO_TICKS(1));
  }  

}  
