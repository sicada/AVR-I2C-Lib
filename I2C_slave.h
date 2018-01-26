#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#define I2C_MASTER_RECEIVER_MODE    0x01
#define I2C_MASTER_TRANSMITTER_MODE 0x02
#define I2C_SLAVE_TRANSMITTER_MODE  0x12
#define I2C_SLAVE_RECEIVER_MODE     0x11

volatile uint8_t buffer_address;
volatile uint8_t i2c_tx_buffer[0xFF];
volatile uint8_t i2c_rx_buffer[0xFF];


typedef void (*addressed_callback)(unsigned char slave_mode);
addressed_callback i2c_address_matched;

typedef void (*received_callback)(unsigned char datain);
received_callback i2c_register_address_received;
received_callback i2c_data_value_received;


void I2C_init_slave(uint8_t address);
void I2C_stop(void);

ISR(TWI_vect);

#endif // I2C_SLAVE_H
