#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "stdint.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "hd44780.h"
#include "ets_sys.h"
#include "esp_idf_lib_helpers.h"
#include "hd44780.c"

// --- DEFINITIONS ---
#define BUTTON_PIN GPIO_NUM_5
#define POT_ADC_CHANNEL ADC_CHANNEL_6  // GPIO1 on most ESP32 variants
#define POT_THRESHOLD_MIN 20            // Minimum adjustable threshold (%)
#define POT_THRESHOLD_MAX 80            // Maximum adjustable threshold (%)
#define PWR 255
#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 20
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define DHT20_ADDR 0x38

#define UART_WAIT_TIMEOUT_MS 300
#define UART_NUM UART_NUM_1
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_18
#define UART_BAUD_RATE 9600

#define HEADER 0x55
#define MODULE_ID_HIGH 0x31
#define MODULE_ID_LOW 0x03

#define READ_OUTPUT_DUTY 0x11
#define GET_ATOMIZE_TIME 0x12
#define GET_WATER_LEVEL_STATUS 0x13
#define SET_PWM_DUTY 0x01
#define SET_ATOMIZE_TIME 0x02

// --- SHARED STATE VARIABLES ---
volatile int button_value = 0; // 0: OFF, 1: ON, 2: AUTO
volatile float humidity = 0;
volatile int water_ok = 1;
volatile int humidity_threshold = 42; // Adjusted at runtime by potentiometer

// ADC handle (initialised in app_main, used in humidity_task)
static adc_oneshot_unit_handle_t adc_handle;

// Mutex for LCD access
SemaphoreHandle_t lcd_mutex;

hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_38,
        .e  = GPIO_NUM_37,
        .d4 = GPIO_NUM_36,
        .d5 = GPIO_NUM_35,
        .d6 = GPIO_NUM_48,
        .d7 = GPIO_NUM_47,
        .bl = HD44780_NOT_USED
    }
};

// ---- I2C INIT ----
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ---- UART ----
static esp_err_t uart_module_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_NUM, 256, 256, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    return uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
/**
 * Standard checksum: sum of all bytes in the frame before the checksum byte.
 */
static uint8_t checksum_frame(const uint8_t *frame, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) checksum += frame[i];
    return checksum;
}
// Builds and sends a message to CHANGE a setting (like mist power)
static void send_set_command(uint8_t cmd, uint8_t data) {
// Create the 7-byte packet: [Header] [IDs] [Command] [Length] [Value] [Checksum]   
    uint8_t frame[] = { HEADER, MODULE_ID_HIGH, MODULE_ID_LOW, cmd, 0x01, data, 0x00 };
    // Add up the first 6 bytes to create the safety checksum at the end
    frame[6] = checksum_frame(frame, 6);
    // Send the finished packet over the UART wires
    uart_write_bytes(UART_NUM, (const char*)frame, sizeof(frame));
}
// Builds and sends a message to ASK for info (like water level)
static void send_read_command(uint8_t cmd) {
    // Same as above, but we use 0x00 for the data since we're just asking a question
    uint8_t frame[] = { HEADER, MODULE_ID_HIGH, MODULE_ID_LOW, cmd, 0x01, 0x00, 0x00 };
    frame[6] = checksum_frame(frame, 6);
    uart_write_bytes(UART_NUM, (const char*)frame, sizeof(frame));
}
/**
 * Generic responder for UART module. Parses the incoming frame and verifies checksum.
 */
static int receive_module_response(uint8_t *data, uint8_t max_len, uint8_t *status) {
    uint8_t rxbuf[16];
    int len = uart_read_bytes(UART_NUM, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(UART_WAIT_TIMEOUT_MS));
    if (len < 7) return -1; // Frame too short
    if (rxbuf[0] != HEADER) return -2; // Wrong header
    if (rxbuf[1] != MODULE_ID_HIGH || rxbuf[2] != MODULE_ID_LOW) return -5;
    *status = rxbuf[3];
    uint8_t data_len = rxbuf[4];
    if (data_len > max_len) return -3;
    if (len < (6 + data_len)) return -1;
    // Checksum verification
    uint8_t checksum = checksum_frame(rxbuf, 5 + data_len);
    if (checksum != rxbuf[5 + data_len]) return -4;
    for (int i = 0; i < data_len; i++) data[i] = rxbuf[5 + i];
    return data_len;
}
// Helper: Wait for an "OK" or "Success" reply after we change a setting
static int receive_set_response(uint8_t *ack, uint8_t *status) {
    // We pass '1' as max_len because SET commands typically return a 1-byte ACK
    return receive_module_response(ack, 1, status);
}
// Helper: Wait for the actual data (like the water level) to come back
static int receive_read_response(uint8_t *value, uint8_t *status) {
    // We pass '1' as max_len because our sensor reads currently expect a 1-byte payload
    return receive_module_response(value, 1, status);
}
// The main function to turn the mist on/off
void set_pwm(uint8_t duty) {
    uint8_t status = 0;
    uint8_t ack = 0;
    // 1. Send the formatted UART frame to the module
    send_set_command(SET_PWM_DUTY, duty);
    // 2. Wait for the module to confirm it has updated the PWM output
    receive_set_response(&ack, &status);
}
// The main function to check the tank
int read_water_level(uint8_t *level) {
    // 1. Send the 'Get Status' command frame
    send_read_command(GET_WATER_LEVEL_STATUS);
    uint8_t status;
    // 2. Parse the module's response frame to extract the water level byte
    return receive_read_response(level, &status);
}

// ---- TASKS ----
void button_task(void *arg) {
    while (1) {
        // Active-low button: 0 = pressed
        if (gpio_get_level(BUTTON_PIN) == 0) {
            // Increment mode
            if (button_value + 1 > 2) button_value = 0;
            else button_value++;

            // Wait for release (debounce)
            while (gpio_get_level(BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
// Handles the DHT20 sensor over I2C
void humidity_task(void *arg) {
    uint8_t data[7];
    uint8_t trigger_cmd[] = { 0xAC, 0x33, 0x00 }; // Standard command to start a reading
    while (1) {
        // --- Read DHT20 humidity ---
        i2c_master_write_to_device(I2C_MASTER_NUM, DHT20_ADDR, trigger_cmd, 3, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(80));
        esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, DHT20_ADDR, data, 7, pdMS_TO_TICKS(100)); //Read the 7 bytes of data back from the sensor
        if (err == ESP_OK && (data[0] & 0x80) == 0) {
            // Conversion formula from DHT20 Datasheet
            // Humidity (%) = (S_rh / 2^20) * 100
            uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
            humidity = (float)raw_hum * 100.0 / 1048576.0;
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before checking again
    }
}
// Monitors the water level and shuts things down if the tank is empty
void water_guard_task(void *arg) {
    uint8_t water_level = 0;
    int result;
    int uart_error_count = 0;

    while (1) {
        result = read_water_level(&water_level); // Ask the module for the current water status
        
        // result > 0 means we successfully received a data payload
        if (result > 0) {
            uart_error_count = 0; // Reset the error counter on a good read
            
            // Evaluate physical sensor state (Assuming 0 means empty)
            if (water_level == 0) {
                water_ok = 0; // 0 = empty, 1 = ok
            } else {
                water_ok = 1; 
            }
        } 
        // result <= 0 means a UART error (timeout, bad checksum, or missing data)
        else {
            uart_error_count++;
            
            // Fail-safe: If we fail to read the sensor 3 seconds in a row, assume 
            // the sensor is disconnected/faulty and trigger the water shutoff.
            if (uart_error_count >= 3) {
                water_ok = 0; 
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// The "Brain" task: Decisions, Screen Updates, and Logic
void control_task(void *arg) {
    char line[17]; // 16 chars + null terminator
    
    while (1) {
        // --- MODE 0: OFF ---
        // Display handles OFF mode regardless of water level
        if (button_value == 0) {
            set_pwm(0);
            if (xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100))) {
                hd44780_gotoxy(&lcd, 0, 0);
                hd44780_puts(&lcd, "Mode: OFF       "); // Padded to overwrite old text
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "                "); // Clear second line
                xSemaphoreGive(lcd_mutex);
            }
        } 
        // --- WATER CHECK FOR ACTIVE MODES ---
        else if (!water_ok) {
            set_pwm(0);
            if (xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100))) {
                hd44780_gotoxy(&lcd, 0, 0);
                hd44780_puts(&lcd, "Water Too Low   "); // Padded
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "Please Refill   "); // Padded
                xSemaphoreGive(lcd_mutex);
            }
        } 
        // --- ACTIVE MODES (ON / AUTO) ---
        else {
            switch (button_value) {
                case 1: // ON
                    set_pwm(PWR);
                    if (xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100))) {
                        hd44780_gotoxy(&lcd, 0, 0);
                        hd44780_puts(&lcd, "Mode: ON        "); // Padded
                        hd44780_gotoxy(&lcd, 0, 1);
                        hd44780_puts(&lcd, "Running         "); // Padded
                        xSemaphoreGive(lcd_mutex);
                    }
                    break;

                case 2: // AUTO
                    // Read potentiometer and update threshold
                    int raw = 0;
                    adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &raw);
                    // Linear mapping of 12-bit ADC (0-4095) to Threshold Range (20-80)
                    humidity_threshold = POT_THRESHOLD_MIN +
                        (raw * (POT_THRESHOLD_MAX - POT_THRESHOLD_MIN)) / 4095;

                    if (humidity < humidity_threshold) set_pwm(PWR);
                    else set_pwm(0);
            
                    if (xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100))) {
                        hd44780_gotoxy(&lcd, 0, 0);
                        hd44780_puts(&lcd, "Mode: AUTO      "); // Padded
                        
                        // Show humidity and the current threshold set by the pot
                        snprintf(line, sizeof(line), "H:%-4.1f T:%2d%%   ", humidity, humidity_threshold);
                        hd44780_gotoxy(&lcd, 0, 1);
                        hd44780_puts(&lcd, line);
                        xSemaphoreGive(lcd_mutex);
                    }
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---- MAIN ----
void app_main(void) {
    // Hardware Init
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    // ADC init for potentiometer
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&adc_cfg, &adc_handle);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,   // 0–3.3V range
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &chan_cfg);
    
    i2c_master_init();
    hd44780_init(&lcd);
    uart_module_init();

    // Create LCD mutex
    lcd_mutex = xSemaphoreCreateMutex();

    // Display initializing
    if (xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100))) {
        hd44780_clear(&lcd);
        hd44780_puts(&lcd, "Initializing...");
        xSemaphoreGive(lcd_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Brief delay to let user read it

    // Create Tasks
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(humidity_task, "humidity_task", 4096, NULL, 5, NULL);
    xTaskCreate(water_guard_task, "water_guard", 4096, NULL, 6, NULL); // Higher priority
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
}
