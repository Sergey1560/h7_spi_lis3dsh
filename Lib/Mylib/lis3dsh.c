#include "lis3dsh.h"

volatile struct accel_out acc;

/* LIS3DH cmd 0x28 + bit7=1 for reading, bit6=1 for multireading */
uint8_t RAM_D2 ALGN4 LIS3DH_CMD_LIST[]  = {0xE8,0x0,0x0,0x0,0x0,0x0,0x0};

/* LIS3DSH cmd 0x28 + bit7=1 for reading*/
uint8_t RAM_D2 ALGN4 LIS3DSH_CMD_LIST[] = {0xA8,0x0,0x0,0x0,0x0,0x0,0x0};

volatile uint8_t RAM_D2 ALGN4 LIS_RESPONSE[sizeof(LIS3DH_CMD_LIST)];

volatile uint8_t *cmd_list;

void lis3dsh_init(void){
    uint8_t cmd;

	/* SPI wihout DMA for setup LIS3DH */
    spi_init(SPI1,0);

    cmd=lis_send(LIS_WHO_I_AM,0x00,LIS_READ);
	if(cmd == LIS3DSH_ID){
		INFO("LIS3DSH, ID: %0X",cmd);
		cmd_list = LIS3DSH_CMD_LIST;
	}else if(cmd == LIS3DH_ID){
		INFO("LIS3DH, ID: %0X",cmd);
		cmd_list = LIS3DH_CMD_LIST;
	}else{
		ERROR("UNKNOWN ID: %d",cmd);
		return;
	}

    cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    INFO("REG4 read %0X",cmd);

	cmd=(LIS_ODR_1600 << 4) | LIS_REG4_BDU | LIS_REG4_XEN | LIS_REG4_YEN | LIS_REG4_ZEN;
    lis_send(LIS_CTRL4_20,cmd,LIS_WRITE);
	INFO("REG4 write %0X",cmd);

	for(uint32_t i=0; i<0x10000; i++){__NOP();}

    /* Check value */
	cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    INFO("REG4 read %0X",cmd);

	/* Poll data */
	spi_transfer(SPI1,(uint8_t *)cmd_list, (uint8_t *)LIS_RESPONSE, sizeof(LIS3DH_CMD_LIST));
	acc.y=(LIS_RESPONSE[4] << 8 ) | LIS_RESPONSE[3];
	acc.z=(LIS_RESPONSE[6] << 8 ) | LIS_RESPONSE[5];
	acc.x=(LIS_RESPONSE[2] << 8 ) | LIS_RESPONSE[1];
	INFO("X: %d Y: %d Z %d",acc.x,acc.y,acc.z);

	/* SPI with DMA, need to reset */
	spi_init(SPI1,1);

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;   
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
};



uint8_t lis_send(uint8_t reg, uint8_t data, uint8_t mode){
	uint8_t in[2];
	uint8_t out[2];	

	if(mode == LIS_READ){
		reg |= (1 << 7);
	}else{
		reg &= ~(1 << 7);
	};

 	out[0] = reg;
	out[1] = data;
	
	spi_transfer(SPI1,out, in, 2); 
    
	return in[1];	
}


void lis3_start_dma(void){

	/* Reset SPI */
	spi_init(SPI1,1);

	if(DMA2_Stream6->CR & DMA_SxCR_EN){
		ERROR("TX dma still running");
	}
	DMA2->HIFCR = DMA_CLEAR_S6;
   	DMA2_Stream6->CR = 0;
	DMA2_Stream6->PAR = (uint32_t) &(SPI1->TXDR);
    DMA2_Stream6->M0AR = (uint32_t )cmd_list;
    DMA2_Stream6->CR = (1u << DMA_SxCR_DIR_Pos)|DMA_SxCR_MINC;
    DMA2_Stream6->NDTR = sizeof(LIS3DH_CMD_LIST); 
	DMAMUX1_Channel14->CCR = 38; //SPI1_TX
	DMA2_Stream6 -> CR |= DMA_SxCR_EN;


	if(DMA2_Stream5->CR & DMA_SxCR_EN){
		ERROR("RX dma still running");
	}
	DMA2->HIFCR = DMA_CLEAR_S5;
	DMA2_Stream5->CR = 0;
    DMA2_Stream5->PAR = (uint32_t) &(SPI1->RXDR);
    DMA2_Stream5->M0AR = (uint32_t )LIS_RESPONSE;
    DMA2_Stream5->CR =  DMA_SxCR_MINC|DMA_SxCR_TCIE;
    DMA2_Stream5->NDTR = sizeof(LIS3DH_CMD_LIST); //DMA transfer length
	DMAMUX1_Channel13->CCR = 37; //SPI1_RX
	DMA2_Stream5 -> CR |= DMA_SxCR_EN;

	SPI1->CR2 = sizeof(LIS3DH_CMD_LIST);
	SPI1->CR1 |= SPI_CR1_CSTART;	
}


void DMA2_Stream5_IRQHandler(void){
	DMA2->HIFCR = DMA_CLEAR_S5;
	acc.y=(LIS_RESPONSE[4] << 8 ) | LIS_RESPONSE[3];
	acc.z=(LIS_RESPONSE[6] << 8 ) | LIS_RESPONSE[5];
	acc.x=(LIS_RESPONSE[2] << 8 ) | LIS_RESPONSE[1];
	INFO("X: %d Y: %d Z %d",acc.x,acc.y,acc.z);
}

