#include "lis3dsh.h"

volatile struct accel_out acc;

const uint8_t LIS3DH_CMD_LIST[] = {0xE8,0x0,0x0,0x0,0x0,0x0,0x0};


void lis3dsh_init(void){
    uint8_t cmd;
	//uint8_t msb,lsb;

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

/*
	cmd=lis_send(LIS_STATUS_27,0xFF,LIS_READ);   //считывание регистра статуса

    INFO("Status %0X",cmd);

      if(cmd & STATUS_X_AV){
		msb = lis_send(0x29,0xFF,LIS_READ);   
		lsb = lis_send(0x28,0xFF,LIS_READ);     
        acc.x = (msb << 8) | (lsb); 
      };
      if(cmd & STATUS_Y_AV){
         msb = lis_send(0x2B,0xFF,LIS_READ);     
         lsb = lis_send(0x2A,0xFF,LIS_READ);     
         acc.y = (msb << 8) | (lsb); 
      };
      if(cmd & STATUS_Z_AV){
         msb = lis_send(0x2D,0xFF,LIS_READ);     
         lsb = lis_send(0x2C,0xFF,LIS_READ);     
         acc.z = (msb << 8) | (lsb); 
      };
    INFO("X: %d Y: %d Z %d",acc.x,acc.y,acc.z);

	spi_transfer(SPI1,LIS3DH_CMD_LIST, response, 7);
	acc.x=(response[2] << 8 ) | response[1];
	acc.y=(response[4] << 8 ) | response[3];
	acc.z=(response[6] << 8 ) | response[5];

	INFO("X: %d Y: %d Z %d",acc.x,acc.y,acc.z);
*/
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
	uint8_t response[sizeof(LIS3DH_CMD_LIST)];

	spi_transfer(SPI1,(uint8_t *)LIS3DH_CMD_LIST, response, 7);
	acc.x=(response[2] << 8 ) | response[1];
	acc.y=(response[4] << 8 ) | response[3];
	acc.z=(response[6] << 8 ) | response[5];
	INFO("X: %d Y: %d Z %d",acc.x,acc.y,acc.z);

};


/*
// DMA2 Chanel3   TX: Stream3,Stream5  RX: Stream0,Stream2
void dma_tx_init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
	DMA2->HIFCR = (DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTCIF5|DMA_HIFCR_CTEIF5);
	while(DMA2_Stream5->CR & DMA_SxCR_EN){asm volatile("nop");}

	DMA2_Stream5 -> CR = 0;
	DMA2_Stream5 -> NDTR = (uint16_t)sizeof(LIS_CMD_LIST); 
	DMA2_Stream5 -> PAR = (uint32_t)&(SPI1 -> DR);
	DMA2_Stream5 -> M0AR = (uint32_t)LIS_CMD_LIST; 
	DMA2_Stream5 -> CR = (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;  
	SPI1->CR2 |= SPI_CR2_TXDMAEN;
};

void dma_rx_init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
	while(DMA2_Stream0->CR & DMA_SxCR_EN){asm volatile("nop");}
	DMA2->LIFCR = (DMA_LIFCR_CDMEIF0|DMA_LIFCR_CFEIF0|DMA_LIFCR_CHTIF0|DMA_LIFCR_CTCIF0|DMA_LIFCR_CTEIF0);

	DMA2_Stream0 -> CR = 0;
	DMA2_Stream0 -> NDTR = (uint16_t)sizeof(LIS_DATA_OUT); 
	DMA2_Stream0 -> PAR = (uint32_t)&(SPI1 -> DR);
	DMA2_Stream0 -> M0AR = (uint32_t)LIS_DATA_OUT; 
	DMA2_Stream0 -> CR = (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE;
	SPI1->CR2 |= SPI_CR2_RXDMAEN;
	
	NVIC_EnableIRQ (DMA2_Stream0_IRQn);
	NVIC_SetPriority (DMA2_Stream0_IRQn, 5);
};

void DMA2_Stream0_IRQHandler(void) {  
		int32_t tmp_value;

	#ifdef DEBUG_SYSVIEW
	SEGGER_SYSVIEW_RecordEnterISR();
	#endif
		

		if(DMA2 -> LISR & (DMA_LISR_TCIF0)) {  
        
			DMA2->LIFCR = (DMA_LIFCR_CDMEIF0|DMA_LIFCR_CFEIF0|DMA_LIFCR_CHTIF0|DMA_LIFCR_CTCIF0|DMA_LIFCR_CTEIF0);
			DMA2->HIFCR = (DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTCIF5|DMA_HIFCR_CTEIF5);
			
			while((SPI1->SR & SPI_SR_BSY) != 0){asm volatile("nop");};
				
			CS_HIGH();

			tmp_value=((int16_t)( (int16_t)LIS_DATA_OUT[2]<<8 | (int16_t)LIS_DATA_OUT[1]))*100 / 16384;
			if(acc_shift.ready == 1){
				acc.x=(int16_t)tmp_value - acc_shift.x;
			}else{
				acc.x=(int16_t)tmp_value;
			};
				
			tmp_value=((int16_t)( (int16_t)LIS_DATA_OUT[4]<<8 | (int16_t)LIS_DATA_OUT[3]))*100 / 16384;
			if(acc_shift.ready == 1){
				acc.y=(int16_t)tmp_value - acc_shift.y;
			}else{
				acc.y=(int16_t)tmp_value;
			};
				
			tmp_value=((int16_t)( (int16_t)LIS_DATA_OUT[6]<<8 | (int16_t)LIS_DATA_OUT[5]))*100 / 16384;
			if(acc_shift.ready == 1){
				acc.z=(int16_t)tmp_value - acc_shift.z;
			}else{
				acc.z=(int16_t)tmp_value;
			};
				
			DMA2_Stream5 -> NDTR = (uint16_t)sizeof(LIS_CMD_LIST); 
			DMA2_Stream5 -> M0AR = (uint32_t)LIS_CMD_LIST; 
				
			CS_LOW();
			//DMA2_Stream5 -> CR |= DMA_SxCR_EN;
    }

	#ifdef DEBUG_SYSVIEW
		SEGGER_SYSVIEW_RecordExitISR();
	#endif

}


void lis_dma_start(void){
		
	while((SPI1->SR & SPI_SR_BSY) != 0){asm volatile("nop");};

	dma_rx_init();
	dma_tx_init();
		
	CS_LOW();
	DMA2_Stream0 -> CR |= DMA_SxCR_EN;
	DMA2_Stream5 -> CR |= DMA_SxCR_EN;
	
};
*/