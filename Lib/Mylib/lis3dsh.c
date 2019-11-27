#include "lis3dsh.h"

volatile struct accel_out acc;

uint8_t RAM_D2 ALGN4 LIS3DH_CMD_LIST[] = {0xE8,0x0,0x0,0x0,0x0,0x0,0x0};
volatile uint8_t RAM_D2 ALGN4 response[sizeof(LIS3DH_CMD_LIST)];

void lis3dsh_init(void){
    uint8_t cmd;

    spi_init(SPI1);
  	
    cmd=lis_send(0x0F,0x00,LIS_READ);
	INFO("LIS3DH ID: %0X",cmd);

    cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    INFO("REG4 read %0X",cmd);

	cmd=(LIS_ODR_1600 << 4) | LIS_REG4_BDU | LIS_REG4_XEN | LIS_REG4_YEN | LIS_REG4_ZEN;
    lis_send(LIS_CTRL4_20,cmd,LIS_WRITE);
	INFO("REG4 write %0X",cmd);
	
	for(uint32_t i=0; i<0x10000; i++){__NOP();}

    cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    INFO("REG4 read %0X",cmd);

	dma_rx_init();
	dma_tx_init();
		
	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR2 = 7;
	SPI1->CFG1 |= SPI_CFG1_RXDMAEN;// | (7u << SPI_CFG1_FTHLV_Pos);
	//RX
	DMA2_Stream5 -> CR |= DMA_SxCR_EN;
	//TX
	DMA2_Stream6 -> CR |= DMA_SxCR_EN;

	SPI1->CFG1 |= SPI_CFG1_TXDMAEN;
	SPI1->CR1 |= SPI_CR1_SPE;

	SPI1->CR1 |= SPI_CR1_CSTART;	
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

void lis3dh_show(void){

	spi_transfer(SPI1,(uint8_t *)LIS3DH_CMD_LIST, (uint8_t *)response, 7);
	acc.x=(response[2] << 8 ) | response[1];
	acc.y=(response[4] << 8 ) | response[3];
	acc.z=(response[6] << 8 ) | response[5];
	INFO("X: %d Y: %d Z %d",acc.x,acc.y,acc.z);

};



// DMA2 TX: Stream4  RX: Stream5
void dma_tx_init(void){

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;   // DMA2 clock enable;

    // Set the peripheral and memory addresses:
    DMA2_Stream6->PAR = (uint32_t) &(SPI1->TXDR);
    DMA2_Stream6->M0AR = (uint32_t )LIS3DH_CMD_LIST;
    DMA2_Stream6->CR = 0;
    DMA2_Stream6->CR |= (1u << DMA_SxCR_DIR_Pos)|\
    					 DMA_SxCR_MINC;
    					 

    DMA2_Stream6->NDTR = sizeof(LIS3DH_CMD_LIST); //DMA transfer length
    DMAMUX1_Channel14->CCR = 38; //SPI1_TX
};

void dma_rx_init(void){

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;   // DMA2 clock enable;
    DMA2_Stream5->PAR = (uint32_t) &(SPI1->RXDR);
    DMA2_Stream5->M0AR = (uint32_t )response;
    DMA2_Stream5->CR = 0;
    DMA2_Stream5->CR |=  DMA_SxCR_MINC;
    					 

    DMA2_Stream5->NDTR = sizeof(LIS3DH_CMD_LIST); //DMA transfer length
    DMAMUX1_Channel13->CCR = 37; //SPI1_RX
};

void lis_dma_start(void){
		
	SPI1->CR1 |= SPI_CR1_CSTART;
	DMA2->HIFCR = (DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CFEIF5|DMA_HIFCR_CDMEIF5)|\
				  (DMA_HIFCR_CTCIF4|DMA_HIFCR_CHTIF4|DMA_HIFCR_CTEIF4|DMA_HIFCR_CFEIF4|DMA_HIFCR_CDMEIF4);
	//RX
	DMA2_Stream5 -> CR |= DMA_SxCR_EN;
	//TX
	DMA2_Stream6 -> CR |= DMA_SxCR_EN;



};
