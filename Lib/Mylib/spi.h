#ifndef SPI_H
#define SPI_H
#include "common_defs.h"

//Для доступа по 8бит
#define SPI1_TXDR_8b          (*(__IO uint8_t *)((uint32_t)SPI1+0x020))
//Для доступа по 16бит
#define SPI1_TXDR_16b         (*(__IO uint16_t *)((uint32_t)SPI1+0x020))

#define SPI1_RXDR_8b          (*(__IO uint8_t *)((uint32_t)SPI1+0x030))
//Для доступа по 16бит
#define SPI1_RXDR_16b         (*(__IO uint16_t *)((uint32_t)SPI1+0x030))


#define PIN_UP    GPIOA->BSRR = GPIO_BSRR_BS_3
#define PIN_DOWN  GPIOA->BSRR = GPIO_BSRR_BR_3

//#define CS_UP    GPIOA->BSRR = GPIO_BSRR_BS_4
//#define CS_DOWN  GPIOA->BSRR = GPIO_BSRR_BR_4

#define LIS_WRITE 0
#define LIS_READ  1

#define LIS_STATUS_27  0x27
#define LIS_CTRL4_20  0x20


#define LIS_ODR_OFF  		0
#define LIS_ODR_3				1
#define LIS_ODR_6				(1 << 1)
#define LIS_ODR_12			((1 << 1) | 1)
#define LIS_ODR_25			(1 << 2)
#define LIS_ODR_50			((1 << 2) | 1)
#define LIS_ODR_100			((1 << 2) | (1 << 1))
#define LIS_ODR_400			((1 << 2) | (1 << 1) | 1)
#define LIS_ODR_800			((1 << 3))
#define LIS_ODR_1600    ((1 << 3) | 1)

#define LIS_REG4_XEN 		1
#define LIS_REG4_YEN 		(1 << 1)
#define LIS_REG4_ZEN 		(1 << 2)
#define LIS_REG4_BDU		(1 << 3)


#define STATUS_X_AV		1
#define STATUS_Y_AV		(1 << 1)
#define STATUS_Z_AV		(1 << 2)



void spi_gpio_init(void);
void spi_init(SPI_TypeDef* SPI);
uint8_t spi_8breg(SPI_TypeDef* SPI,uint8_t reg, uint8_t data);
uint16_t spi_16breg(SPI_TypeDef* SPI,uint8_t reg);

#endif