#ifndef TPS65185_H
#define TPS65185_H

#include <driver/i2c.h>
#include <esp_err.h>

#define TPS_REG_TMST_VALUE 0x00
#define TPS_REG_ENABLE 0x01
#define TPS_REG_VADJ 0x02
#define TPS_REG_VCOM1 0x03
#define TPS_REG_VCOM2 0x04
#define TPS_REG_INT_EN1 0x05
#define TPS_REG_INT_EN2 0x06
#define TPS_REG_INT1 0x07
#define TPS_REG_INT2 0x08
#define TPS_REG_UPSEQ0 0x09
#define TPS_REG_UPSEQ1 0x0A
#define TPS_REG_DWNSEQ0 0x0B
#define TPS_REG_DWNSEQ1 0x0C
#define TPS_REG_TMST1 0x0D
#define TPS_REG_TMST2 0x0E
#define TPS_REG_PG 0x0F
#define TPS_REG_REVID 0x10

/** Write a TPS65185 register and return the underlying I2C transaction status. */
esp_err_t tps_write_register(i2c_port_t port, int reg, uint8_t value);

/**
 * Read a register using the legacy value-only API.
 *
 * This function logs an I2C error and returns zero. Use
 * tps_read_register_checked() when zero is a meaningful register value.
 */
uint8_t tps_read_register(i2c_port_t i2c_num, int reg);

/**
 * Read a TPS65185 register without collapsing an I2C failure into value zero.
 *
 * @param i2c_num I2C controller connected to the TPS65185.
 * @param reg Register address to read.
 * @param value Receives the register value when the transaction succeeds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null output pointer, or
 *         the error returned by the ESP-IDF I2C driver.
 */
esp_err_t tps_read_register_checked(i2c_port_t i2c_num, int reg, uint8_t* value);

/**
 * Set the VCOM voltage in positive millivolts: 1600 means -1.6 V.
 *
 * @return ESP_OK when both VCOM registers are written, otherwise the first I2C
 *         error encountered.
 */
esp_err_t tps_set_vcom(i2c_port_t i2c_num, unsigned vcom_mV);

/**
 * @brief Please read datasheet section 8.3.7.1 Kick-Back Voltage Measurement
 *  1 Device enters ACTIVE mode
 *  2 All power rails are up except VCOM
 *    VCOM pin is in HiZ state
 */
void tps_vcom_kickback();

/**
 * @brief start VCOM kick-back voltage measurements
 */
void tps_vcom_kickback_start();

/**
 * VCOM kick-back ACQC (Acquisition Complete) bit in the INT1 register is set
 * @return unsigned: 0 is not read
 */
unsigned tps_vcom_kickback_rdy();

/**
 * Set the power-up voltage sequence required by Carta 1300 panels.
 *
 * The caller supplies the board's I2C controller; v7 uses I2C_NUM_1 rather than
 * the I2C_NUM_0 value used by older boards.
 *
 * @return ESP_OK when both sequence registers are written, otherwise the first
 *         I2C error encountered.
 */
esp_err_t tps_set_upseq_carta1300(i2c_port_t i2c_num);
/**
 * Read the temperature via the on-board thermistor.
 */
int8_t tps_read_thermistor(i2c_port_t i2c_num);

#endif  //  TPS65185_H
