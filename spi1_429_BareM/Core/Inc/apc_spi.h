/*
 * apc_spi.h
 *
 *  Created on: Nov 10, 2022
 *      Author: rajdeep
 */

#ifndef INC_APC_SPI_H_
#define INC_APC_SPI_H_

#include "stm32f429xx.h"

void spi_gpio_init(void);
void spi1_config(void);

void spi1_tranmit(uint8_t *data,uint32_t size);
void spi1_receive(uint8_t *data,uint32_t size);
void cs_enable(void);
void cs_disable(void);



#endif /* INC_APC_SPI_H_ */
