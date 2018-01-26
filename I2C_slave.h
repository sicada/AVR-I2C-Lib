#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

volatile uint8_t buffer_address;
volatile uint8_t i2c_tx_buffer[0xFF];
volatile uint8_t i2c_rx_buffer[0xFF];

void I2C_init_slave(uint8_t address);
void I2C_stop(void);

ISR(TWI_vect);

#endif // I2C_SLAVE_H
