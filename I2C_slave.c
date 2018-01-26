#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "I2C_slave.h"

void I2C_init_slave (uint8_t address) {
	// Load slave address into TWI address register
	TWAR = (address << 1);

	// Set the TWCR to enable address matching and
    // enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}


void I2C_stop (void) {
	// Clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}


ISR (TWI_vect) {
	// Temporarily stores the received data
	uint8_t data;

    // -- Called in Slave Receiver Mode -------------------------------------
	// This MCU's own address has been called by a master on the bus,
    // on the next interrupt cycle data will presumably be received
	if ( (TWSR & 0xF8) == TW_SR_SLA_ACK ) {
		buffer_address = 0xFF;
		// Clear TWI interrupt flag,
        // prepare to receive next byte and acknowledge
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	}

    // -- Received data in Slave Receiver Mode ------------------------------
    // The MCU is in slave receiver mode and a data byte
    // has been received from a master on the bus
	else if ( (TWSR & 0xF8) == TW_SR_DATA_ACK ) {
		data = TWDR;

		// Check if a register has been set/selected.
        // If not, assume this received byte is the register address.
		if (buffer_address == 0xFF) {
			buffer_address = data;

			// Clear TWI interrupt flag,
            // prepare to receive next byte and acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		}

        // A register was previously set, so assume this data byte
        // is a value to store into the previously selected register
		else {

			// store the data at the current address
			i2c_rx_buffer[buffer_address] = data;

			buffer_address++;

			// If there is still enough space inside the buffer
			if (buffer_address < 0xFF) {
				// Clear TWI interrupt flag, prepare to receive
                // next byte and acknowledge
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			}

            // Don't acknowledge; the buffer is full
			else {
				TWCR &= ~(1<<TWEA);
				// Clear TWI interrupt flag, prepare to receive last byte.
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN);
			}
		}
	}

    // -- Called in Slave Transmitter Mode ----------------------------------
	// This MCU's own address has been called by a master on the bus,
    // on the next interrupt cycle data will presumably be transmitted
    else if ( (TWSR & 0xF8) == TW_ST_SLA_ACK) {
		TWDR = i2c_tx_buffer[buffer_address];

		// Clear the TWI interrupt flag,
        // prepare to receive next byte and acknowledge
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	}

    // -- Transmitter data in Slave Transmitter Mode ------------------------
    // The MCU is in slave transmitter mode and a data byte
    // has been transmitted back to a master on the bus
	else if ( (TWSR & 0xF8) == TW_ST_DATA_ACK ) {
		// Copy the specified buffer address into the
        // TWDR register for transmission
		TWDR = i2c_tx_buffer[buffer_address];

		buffer_address++;

		// If there is another buffer address that can be sent
		if (buffer_address < 0xFF) {
			// clear TWI interrupt flag, prepare to send next byte and receive acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		}

        // Don't acknowledge; end of buffer was reached,
        // No more data can be sent
		else {
			TWCR &= ~(1<<TWEA);
			// clear TWI interrupt flag, prepare to receive last byte.
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN);
		}

	}

	// If none of the above apply prepare the TWI to
    // listen on the bus and respond to future requests
	else {
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	}
}
