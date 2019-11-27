#ifndef LIS3DSH_H
#define LIS3DSH_H

#include "common_defs.h"
#include "spi.h"

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


struct accel_out
{    
	int16_t x;
	int16_t y;
	int16_t z;
};

struct accel_shift
{    
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t count;
	uint8_t ready;
};

extern volatile struct accel_out acc;
//extern volatile struct accel_shift acc_shift;

void lis3dsh_init(void);
uint8_t lis_send(uint8_t reg, uint8_t data, uint8_t mode);

void lis3dh_show(void);

void dma_tx_init(void);
void dma_rx_init(void);
void lis_dma_start(void);


#endif