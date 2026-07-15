
#include "tps65185.h"
#include "pca9555.h"
#include "epd_board.h"
#include "esp_err.h"
#include "esp_log.h"

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

static const int EPDIY_TPS_ADDR = 0x68;

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, int reg, uint8_t* value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Phase 1 selects the register. Delete the command link before checking the
    // transaction result so failed I2C operations cannot leak heap objects.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE("epdiy", "insufficient memory for TPS65185 I2C transaction");
        return ESP_ERR_NO_MEM;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EPDIY_TPS_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // Phase 2 reads the selected register and follows the same ownership rule.
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE("epdiy", "insufficient memory for TPS65185 I2C transaction");
        return ESP_ERR_NO_MEM;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EPDIY_TPS_ADDR << 1) | I2C_MASTER_READ, true);
    /*
        if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    */
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave(
    i2c_port_t i2c_num, uint8_t ctrl, uint8_t* data_wr, size_t size
) {
    if (size > 0 && data_wr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE("epdiy", "insufficient memory for TPS65185 I2C transaction");
        return ESP_ERR_NO_MEM;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EPDIY_TPS_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ctrl, true);

    i2c_master_write(cmd, data_wr, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t tps_write_register(i2c_port_t port, int reg, uint8_t value) {
    uint8_t w_data[1];
    esp_err_t err;

    w_data[0] = value;

    err = i2c_master_write_slave(port, reg, w_data, 1);
    return err;
}

uint8_t tps_read_register(i2c_port_t i2c_num, int reg) {
    // Keep the legacy API for existing callers. Power sequencing uses the
    // checked API so a communication failure is never interpreted as value 0.
    uint8_t value = 0;
    esp_err_t err = tps_read_register_checked(i2c_num, reg, &value);
    if (err != ESP_OK) {
        ESP_LOGE("TPS65185", "%s failed: %s", __func__, esp_err_to_name(err));
    }
    return value;
}

esp_err_t tps_read_register_checked(i2c_port_t i2c_num, int reg, uint8_t* value) {
    return i2c_master_read_slave(i2c_num, reg, value);
}

esp_err_t tps_set_vcom(i2c_port_t i2c_num, unsigned vcom_mV) {
    unsigned val = vcom_mV / 10;
    esp_err_t err = tps_write_register(i2c_num, TPS_REG_VCOM2, (val & 0x100) >> 8);
    if (err != ESP_OK) {
        return err;
    }
    return tps_write_register(i2c_num, TPS_REG_VCOM1, val & 0xFF);
}

int8_t tps_read_thermistor(i2c_port_t i2c_num) {
    esp_err_t err = tps_write_register(i2c_num, TPS_REG_TMST1, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "thermistor start failed: %s", esp_err_to_name(err));
        return 0;
    }
    int tries = 0;
    while (true) {
        uint8_t val = 0;
        err = tps_read_register_checked(i2c_num, TPS_REG_TMST1, &val);
        if (err != ESP_OK) {
            ESP_LOGE("epdiy", "thermistor status read failed: %s", esp_err_to_name(err));
            return 0;
        }
        // temperature conversion done
        if (val & 0x20) {
            break;
        }
        tries++;

        if (tries >= 100) {
            ESP_LOGE("epdiy", "thermistor read timeout!");
            break;
        }
    }
    uint8_t value = 0;
    err = tps_read_register_checked(i2c_num, TPS_REG_TMST_VALUE, &value);
    if (err != ESP_OK) {
        ESP_LOGE("epdiy", "thermistor value read failed: %s", esp_err_to_name(err));
        return 0;
    }
    return (int8_t)value;
}

void tps_vcom_kickback() {
    printf("VCOM Kickback test\n");
    // pull the WAKEUP pin and the PWRUP pin high to enable all output rails.
    epd_current_board()->measure_vcom(epd_ctrl_state());
    // set the HiZ bit in the VCOM2 register (BIT 5) 0x20
    // this puts the VCOM pin in a high-impedance state.
    // bit 3 & 4 Number of acquisitions that is averaged to a single kick-back V. measurement
    tps_write_register(I2C_NUM_0, 4, 0x38);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t int1reg = tps_read_register(I2C_NUM_0, TPS_REG_INT1);
    uint8_t vcomreg = tps_read_register(I2C_NUM_0, TPS_REG_VCOM2);
}

void tps_vcom_kickback_start() {
    uint8_t int1reg = tps_read_register(I2C_NUM_0, TPS_REG_INT1);
    // set the ACQ bit in the VCOM2 register to 1 (BIT 7)
    tps_write_register(I2C_NUM_0, TPS_REG_VCOM2, 0xA0);
}

unsigned tps_vcom_kickback_rdy() {
    uint8_t int1reg = tps_read_register(I2C_NUM_0, TPS_REG_INT1);

    if (int1reg == 0x02) {
        uint8_t lsb = tps_read_register(I2C_NUM_0, 3);
        uint8_t msb = tps_read_register(I2C_NUM_0, 4);
        int u16Value = (lsb | (msb << 8)) & 0x1ff;
        ESP_LOGI("vcom", "raw value:%d temperature:%d C", u16Value, tps_read_thermistor(I2C_NUM_0));
        return u16Value * 10;
    } else {
        return 0;
    }
}

esp_err_t tps_set_upseq_carta1300(i2c_port_t i2c_num) {
    esp_err_t err = tps_write_register(i2c_num, TPS_REG_UPSEQ0, 0xE1);
    if (err != ESP_OK) {
        return err;
    }
    return tps_write_register(i2c_num, TPS_REG_UPSEQ1, 0xAA);
}
