/*
 * i2c_comm.h
 *
 *  Created on: 09-Feb-2020
 *      Author: ASUS
 */

#ifndef I2C_COMM_H_
#define I2C_COMM_H_

void I2C_LowLevel_Init(uint32_t ClockSpeed, uint8_t OwnAddress);
uint8_t i2c_register_read(uint8_t slaveAddr, uint8_t registerAddr);
int8_t i2c_slave_mem_read(uint8_t slaveAddr, uint8_t registerAddr, uint8_t* writeBuffer, uint8_t bytesNum);
int8_t i2c_slave_mem_write(uint8_t slaveAddr, uint8_t registerAddr, uint8_t writedata);


#endif /* I2C_COMM_H_ */
