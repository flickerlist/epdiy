#include <stdint.h>
#include "epd_board.h"
#include "epdiy.h"

#include "../output_common/render_method.h"
#include "../output_lcd/lcd_driver.h"
#include "esp_log.h"
#include "pca9555.h"
#include "tps65185.h"
#include "esp_timer.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Make this compile von the ESP32 without ifdefing the whole file
#ifndef CONFIG_IDF_TARGET_ESP32S3
#define GPIO_NUM_40 -1
#define GPIO_NUM_41 -1
#define GPIO_NUM_42 -1
#define GPIO_NUM_43 -1
#define GPIO_NUM_44 -1
#define GPIO_NUM_45 -1
#define GPIO_NUM_46 -1
#define GPIO_NUM_47 -1
#define GPIO_NUM_48 -1
#endif

#define CFG_SCL GPIO_NUM_2
#define CFG_SDA GPIO_NUM_14
#define CFG_INTR GPIO_NUM_38
#define EPDIY_I2C_PORT I2C_NUM_1

#define CFG_PIN_OE (PCA_PIN_PC10 >> 8)
#define CFG_PIN_MODE (PCA_PIN_PC11 >> 8)
#define __CFG_PIN_STV (PCA_PIN_PC12 >> 8)
#define CFG_PIN_PWRUP (PCA_PIN_PC13 >> 8)
#define CFG_PIN_VCOM_CTRL (PCA_PIN_PC14 >> 8)
#define CFG_PIN_WAKEUP (PCA_PIN_PC15 >> 8)
#define CFG_PIN_PWRGOOD (PCA_PIN_PC16 >> 8)
#define CFG_PIN_INT (PCA_PIN_PC17 >> 8)

#define D15 GPIO_NUM_47
#define D14 GPIO_NUM_21
#define D13 GPIO_NUM_14
#define D12 GPIO_NUM_13
#define D11 GPIO_NUM_12
#define D10 GPIO_NUM_11
#define D9 GPIO_NUM_10
#define D8 GPIO_NUM_9

#define D7 GPIO_NUM_8
#define D6 GPIO_NUM_18
#define D5 GPIO_NUM_17
#define D4 GPIO_NUM_16
#define D3 GPIO_NUM_15
#define D2 GPIO_NUM_7
#define D1 GPIO_NUM_6
#define D0 GPIO_NUM_5

/* Control Lines */
#define CKV GPIO_NUM_46
#define STH GPIO_NUM_41
#define LEH GPIO_NUM_42
#define STV GPIO_NUM_45

/* Edges */
#define CKH GPIO_NUM_4

typedef struct {
    i2c_port_t port;
    bool pwrup;
    bool vcom_ctrl;
    bool wakeup;
    bool others[8];
} epd_config_register_t;

/** The VCOM voltage to use. */
static int vcom = 1600;

static epd_config_register_t config_reg;
// Cleared after any expander communication failure. The next power-on attempt
// then restores a known-safe output latch and pin-direction configuration.
static bool expander_initialized = false;

static bool interrupt_done = false;

static void IRAM_ATTR interrupt_handler(void* arg) {
    interrupt_done = true;
}

static lcd_bus_config_t lcd_config = {
    .clock = CKH,
    .ckv = CKV,
    .leh = LEH,
    .start_pulse = STH,
    .stv = STV,
    .data[0] = D0,
    .data[1] = D1,
    .data[2] = D2,
    .data[3] = D3,
    .data[4] = D4,
    .data[5] = D5,
    .data[6] = D6,
    .data[7] = D7,
    .data[8] = D8,
    .data[9] = D9,
    .data[10] = D10,
    .data[11] = D11,
    .data[12] = D12,
    .data[13] = D13,
    .data[14] = D14,
    .data[15] = D15,
};

/**
 * Initialize the TCA9535 without exposing its power-on output-latch values.
 *
 * After POR all pins are inputs, but the output registers default to 0xFF.
 * Writing the safe inactive levels before changing pin direction prevents
 * WAKEUP, PWRUP, or VCOM from pulsing high during initialization or recovery.
 */
static esp_err_t epd_board_init_expander() {
    expander_initialized = false;
    config_reg.pwrup = false;
    config_reg.vcom_ctrl = false;
    config_reg.wakeup = false;

    // Preload a safe low level while all pins are still inputs. This prevents the
    // TCA9535 power-on output latch (0xFF) from briefly enabling the TPS65185.
    esp_err_t err = pca9555_set_value(config_reg.port, 0x00, 1);
    if (err != ESP_OK) {
        return err;
    }

    // Set all EPD control lines to outputs except TPS interrupt and PWRGOOD.
    err = pca9555_set_config(config_reg.port, CFG_PIN_PWRGOOD | CFG_PIN_INT, 1);
    if (err == ESP_OK) {
        expander_initialized = true;
    }
    return err;
}

static void epd_board_init(uint32_t epd_row_width) {
    gpio_hold_dis(CKH);  // free CKH after wakeup

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CFG_SDA;
    conf.scl_io_num = CFG_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0;
    // These calls configure the local ESP-IDF controller. Device transactions
    // below propagate errors instead of aborting through ESP_ERROR_CHECK.
    ESP_ERROR_CHECK(i2c_param_config(EPDIY_I2C_PORT, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(EPDIY_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    config_reg.port = EPDIY_I2C_PORT;
    config_reg.pwrup = false;
    config_reg.vcom_ctrl = false;
    config_reg.wakeup = false;
    for (int i = 0; i < 8; i++) {
        config_reg.others[i] = false;
    }

    gpio_set_direction(CFG_INTR, GPIO_MODE_INPUT);
    gpio_set_intr_type(CFG_INTR, GPIO_INTR_NEGEDGE);

    // ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_EDGE));

    ESP_ERROR_CHECK(gpio_isr_handler_add(CFG_INTR, interrupt_handler, (void*)CFG_INTR));

    esp_err_t err = epd_board_init_expander();
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "TCA9535 initialization failed: %s", esp_err_to_name(err));
    }

    const EpdDisplay_t* display = epd_get_display();

    LcdEpdConfig_t config = {
        .pixel_clock = display->bus_speed * 1000 * 1000,
        .ckv_high_time = 60,
        .line_front_porch = 4,
        .le_high_time = 4,
        .bus_width = display->bus_width,
        .bus = lcd_config,
    };
    epd_lcd_init(&config, display->width, display->height);
}

static void epd_board_deinit() {
    epd_lcd_deinit();

    esp_err_t err = pca9555_set_config(
        config_reg.port, CFG_PIN_PWRGOOD | CFG_PIN_INT | CFG_PIN_VCOM_CTRL | CFG_PIN_PWRUP, 1
    );
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "TCA9535 deinit configuration failed: %s", esp_err_to_name(err));
    }

    int tries = 0;
    uint8_t input = 0;
    while (true) {
        err = pca9555_read_input_checked(config_reg.port, 1, &input);
        if (err != ESP_OK) {
            ESP_LOGE("epdiy", "TCA9535 deinit status read failed: %s", esp_err_to_name(err));
            break;
        }
        if ((input & 0xC0) == 0x80) {
            break;
        }
        if (tries >= 50) {
            ESP_LOGE("epdiy", "failed to shut down TPS65185!");
            break;
        }
        tries++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Not sure why we need this delay, but the TPS65185 seems to generate an interrupt after some
    // time that needs to be cleared.
    vTaskDelay(pdMS_TO_TICKS(500));
    err = pca9555_read_input_checked(config_reg.port, 0, &input);
    if (err == ESP_OK) {
        err = pca9555_read_input_checked(config_reg.port, 1, &input);
    }
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "TCA9535 interrupt clear failed: %s", esp_err_to_name(err));
    }
    i2c_driver_delete(EPDIY_I2C_PORT);
    expander_initialized = false;

    gpio_uninstall_isr_service();
}

/** Checked internal form of the board callback used by power sequencing. */
static esp_err_t epd_board_write_ctrl(
    epd_ctrl_state_t* state, const epd_ctrl_state_t* const mask
) {
    uint8_t value = 0x00;
    if (mask->ep_output_enable || mask->ep_mode || mask->ep_stv) {
        if (state->ep_output_enable)
            value |= CFG_PIN_OE;
        if (state->ep_mode)
            value |= CFG_PIN_MODE;
        // if (state->ep_stv) value |= CFG_PIN_STV;
        if (config_reg.pwrup)
            value |= CFG_PIN_PWRUP;
        if (config_reg.vcom_ctrl)
            value |= CFG_PIN_VCOM_CTRL;
        if (config_reg.wakeup)
            value |= CFG_PIN_WAKEUP;

        esp_err_t err = pca9555_set_value(config_reg.port, value, 1);
        if (err != ESP_OK) {
            expander_initialized = false;
            ESP_LOGE("epdiy", "TCA9535 control write failed: %s", esp_err_to_name(err));
        }
        return err;
    }
    return ESP_OK;
}

static void epd_board_set_ctrl(epd_ctrl_state_t* state, const epd_ctrl_state_t* const mask) {
    // The public board callback has no error return. Log and remember failures in
    // epd_board_write_ctrl(); checked power paths call that function directly.
    (void)epd_board_write_ctrl(state, mask);
}

/** Disable VCOM and the power rails before taking the PMIC out of wake state. */
static esp_err_t epd_board_poweroff_checked(epd_ctrl_state_t* state) {
    epd_ctrl_state_t mask = {
        .ep_stv = true,
        .ep_output_enable = true,
        .ep_mode = true,
    };
    config_reg.vcom_ctrl = false;
    config_reg.pwrup = false;
    state->ep_stv = false;
    state->ep_output_enable = false;
    state->ep_mode = false;

    esp_err_t first_err = epd_board_write_ctrl(state, &mask);
    vTaskDelay(pdMS_TO_TICKS(10));

    config_reg.wakeup = false;
    esp_err_t second_err = epd_board_write_ctrl(state, &mask);
    return first_err != ESP_OK ? first_err : second_err;
}

/** Apply the required WAKEUP -> PWRUP -> VCOM sequence with checked writes. */
static esp_err_t epd_board_start_power_sequence(
    epd_ctrl_state_t* state, const epd_ctrl_state_t* const mask
) {
    state->ep_stv = true;
    state->ep_mode = false;
    state->ep_output_enable = true;

    config_reg.wakeup = true;
    esp_err_t err = epd_board_write_ctrl(state, mask);
    if (err != ESP_OK) {
        return err;
    }

    const EpdDisplay_t* display = epd_get_display();
    if (display->display_type & DISPLAY_UPSEQ_MC2) {
        vTaskDelay(pdMS_TO_TICKS(30));
        err = tps_set_upseq_carta1300(config_reg.port);
        if (err != ESP_OK) {
            return err;
        }
        ESP_LOGI("epdiy", "Setting UPSEQ for DISPLAY_UPSEQ_MC2");
    }

    config_reg.pwrup = true;
    err = epd_board_write_ctrl(state, mask);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    config_reg.vcom_ctrl = true;
    err = epd_board_write_ctrl(state, mask);
    if (err != ESP_OK) {
        return err;
    }

    // Give the PMIC time to start its power-up sequence.
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static void epd_board_poweroff(epd_ctrl_state_t* state);
static bool epd_board_poweron(epd_ctrl_state_t* state) {
    epd_ctrl_state_t mask = {
        .ep_output_enable = true,
        .ep_mode = true,
        .ep_stv = true,
    };

    esp_err_t err = ESP_OK;
    if (!expander_initialized) {
        err = epd_board_init_expander();
        if (err != ESP_OK) {
            ESP_LOGE("epdiy", "TCA9535 recovery initialization failed: %s", esp_err_to_name(err));
            return false;
        }
    }

    err = epd_board_start_power_sequence(state, &mask);
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "display power sequence write failed: %s", esp_err_to_name(err));
        return false;
    }

    int64_t start = esp_timer_get_time();
    int failed_count = 0;
    while (true) {
        uint8_t input = 0;
        err = pca9555_read_input_checked(config_reg.port, 1, &input);
        if (err != ESP_OK) {
            // A failed transaction is not a valid low PWRGOOD sample. Return the
            // communication error path and force safe expander reinitialization.
            expander_initialized = false;
            ESP_LOGE("epdiy", "TCA9535 PWRGOOD read failed: %s", esp_err_to_name(err));
            return false;
        }
        if (input & CFG_PIN_PWRGOOD) {
            break;
        }

        // Only a successfully read low PWRGOOD value enters the timed PMIC retry
        // path. This keeps electrical startup failures separate from I2C faults.
        int64_t _cur = esp_timer_get_time();
        if (_cur - start > 700 * 1000) { // 700ms
            start  = _cur;
            failed_count++;

            if (failed_count >= 3) {
                // poweron failed
                esp_rom_printf("\nepdiy epd_board_poweron failed [finally] !!!\n");
                return false;
            }
            esp_rom_printf("\nepdiy epd_board_poweron failed [once], core: %d retry...\n", xPortGetCoreID());

            err = epd_board_poweroff_checked(state);
            if (err != ESP_OK) {
                ESP_LOGE("epdiy", "display poweroff retry failed: %s", esp_err_to_name(err));
                return false;
            }
            err = epd_board_start_power_sequence(state, &mask);
            if (err != ESP_OK) {
                ESP_LOGE("epdiy", "display poweron retry failed: %s", esp_err_to_name(err));
                return false;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    err = tps_write_register(config_reg.port, TPS_REG_ENABLE, 0x3F);
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "TPS65185 enable write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = tps_set_vcom(config_reg.port, vcom);
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "TPS65185 VCOM write failed: %s", esp_err_to_name(err));
        return false;
    }

    state->ep_sth = true;
    mask = (const epd_ctrl_state_t){
        .ep_sth = true,
    };
    epd_board_set_ctrl(state, &mask);

    int tries = 0;
    uint8_t pg_status = 0;
    while (true) {
        err = tps_read_register_checked(config_reg.port, TPS_REG_PG, &pg_status);
        if (err != ESP_OK) {
            // Do not treat an unreadable PG register as power rails being low.
            ESP_LOGE("epdiy", "TPS65185 PG read failed: %s", esp_err_to_name(err));
            return false;
        }
        if ((pg_status & 0xFA) == 0xFA) {
            break;
        }
        if (tries >= 500) {
            ESP_LOGE("epdiy", "Power enable failed! PG status: %X", pg_status);
            return false;
        }
        tries++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

static void epd_board_measure_vcom(epd_ctrl_state_t* state) {
    epd_ctrl_state_t mask = {
        .ep_output_enable = true,
        .ep_mode = true,
        .ep_stv = true,
    };
    state->ep_stv = true;
    state->ep_mode = false;
    state->ep_output_enable = true;

    esp_err_t err = ESP_OK;
    if (!expander_initialized) {
        err = epd_board_init_expander();
        if (err != ESP_OK) {
            ESP_LOGE("epdiy", "TCA9535 measurement initialization failed: %s", esp_err_to_name(err));
            return;
        }
    }

    config_reg.wakeup = true;
    err = epd_board_write_ctrl(state, &mask);
    if (err != ESP_OK) {
        return;
    }
    config_reg.pwrup = true;
    err = epd_board_write_ctrl(state, &mask);
    if (err != ESP_OK) {
        return;
    }

    // give the IC time to powerup and set lines
    vTaskDelay(pdMS_TO_TICKS(10));
    state->ep_sth = true;
    mask = (const epd_ctrl_state_t){
        .ep_sth = true,
    };
    if (epd_board_write_ctrl(state, &mask) != ESP_OK) {
        return;
    }

    int tries = 0;
    uint8_t input = 0;
    while (true) {
        err = pca9555_read_input_checked(config_reg.port, 1, &input);
        if (err != ESP_OK) {
            expander_initialized = false;
            ESP_LOGE("epdiy", "TCA9535 measurement PWRGOOD read failed: %s", esp_err_to_name(err));
            return;
        }
        if (input & CFG_PIN_PWRGOOD) {
            break;
        }
        if (tries++ >= 500) {
            ESP_LOGE("epdiy", "measurement PWRGOOD timeout");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI("epdiy", "Power rails enabled");

    state->ep_sth = true;
    mask = (const epd_ctrl_state_t){
        .ep_sth = true,
    };
    if (epd_board_write_ctrl(state, &mask) != ESP_OK) {
        return;
    }

    tries = 0;
    uint8_t pg_status = 0;
    while (true) {
        err = tps_read_register_checked(config_reg.port, TPS_REG_PG, &pg_status);
        if (err != ESP_OK) {
            ESP_LOGE("epdiy", "TPS65185 measurement PG read failed: %s", esp_err_to_name(err));
            return;
        }
        if ((pg_status & 0xFA) == 0xFA) {
            break;
        }
        if (tries >= 500) {
            ESP_LOGE("epdiy", "Power enable failed! PG status: %X", pg_status);
            return;
        }
        tries++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void epd_board_poweroff(epd_ctrl_state_t* state) {
    esp_err_t err = epd_board_poweroff_checked(state);
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "display poweroff failed: %s", esp_err_to_name(err));
    }
}

static float epd_board_ambient_temperature() {
    return 20;
}

static void set_vcom(int value) {
    vcom = value;
}

const EpdBoardDefinition epd_board_v7 = {
    .init = epd_board_init,
    .deinit = epd_board_deinit,
    .set_ctrl = epd_board_set_ctrl,
    .poweron = epd_board_poweron,
    .poweroff = epd_board_poweroff,

    .measure_vcom = epd_board_measure_vcom,
    .get_temperature = epd_board_ambient_temperature,
    .set_vcom = set_vcom,

    // unimplemented for now, but shares v6 implementation
    .gpio_set_direction = NULL,
    .gpio_read = NULL,
    .gpio_write = NULL,
};
