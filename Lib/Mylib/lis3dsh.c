#include "lis3dsh.h"

volatile struct accel_out acc;

uint8_t LIS_CMD_LIST[] = {0xA8,0x0,0xA9,0x0,0xAA,0x0,0xAB,0x0,0xAC,0x0,0xAD,0x0};
uint8_t response[12];

void lis3dsh_init(void){
    uint8_t cmd;
	uint8_t msb,lsb;

    spi_init(SPI1);
  	
    cmd=lis_send(0x0F,0x00,LIS_READ);
	INFO("LIS3DH ID: %0X",cmd);

    cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    INFO("REG4 read %0X",cmd);

    // cmd=lis_send(0x0F,0x00,LIS_READ);
	// INFO("LIS3DH ID: %0X",cmd);

    // cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    // INFO("REG4 read %0X",cmd);

	cmd=(LIS_ODR_1600 << 4) | LIS_REG4_BDU | LIS_REG4_XEN | LIS_REG4_YEN | LIS_REG4_ZEN;
	INFO("REG4 write %0X",cmd);
    lis_send(LIS_CTRL4_20,cmd,LIS_WRITE);
	
	for(uint32_t i=0; i<0x10000; i++){__NOP();}

    cmd=lis_send(LIS_CTRL4_20,0x0,LIS_READ);
    INFO("REG4 read %0X",cmd);


	cmd=lis_send(LIS_STATUS_27,0xFF,LIS_READ);   //считывание регистра статуса

    INFO("Status %0X",cmd);

      if(cmd & STATUS_X_AV){
		msb = lis_send(0x29,0xFF,LIS_READ);   
		lsb = lis_send(0x28,0xFF,LIS_READ);     
        acc.x = (msb << 8) | (lsb); 
//		 acc.x = lis_read_16b(0x28); 
	    INFO("X: %d",acc.x);

      };
         
      if(cmd & STATUS_Y_AV){
         msb = lis_send(0x2B,0xFF,LIS_READ);     
         lsb = lis_send(0x2A,0xFF,LIS_READ);     
         acc.y = (msb << 8) | (lsb); 
		 INFO("Y: %d",acc.y);
      };

      if(cmd & STATUS_Z_AV){
         msb = lis_send(0x2C,0xFF,LIS_READ);     
         lsb = lis_send(0x2D,0xFF,LIS_READ);     
         acc.z = (msb << 8) | (lsb); 
		 INFO("Z: %d",acc.z);
      };


};


uint16_t lis_read_16b(uint8_t reg){
	uint16_t reply;

	reg |= (1 << 7);
	reply=spi_16breg(SPI1,reg);
	return reply;
}


uint8_t lis_send(uint8_t reg, uint8_t data, uint8_t mode){
	uint8_t reply=0;

	if(mode == LIS_READ){
		reg |= (1 << 7);
	}else{
		reg &= ~(1 << 7);
	};

 	reply=spi_8breg(SPI1,reg,data);
    
	return reply;	
}


int16_t lis_get_x(void){
	int8_t lsb,msb;

	lsb = lis_send(0x28,0xFF,LIS_READ); 
	msb = lis_send(0x29,0xFF,LIS_READ); 
	return (msb * 256 + lsb);
};

int16_t lis_get_y(void){
	int8_t lsb,msb;

	lsb = lis_send(0x2A,0xFF,LIS_READ); 
	msb = lis_send(0x2B,0xFF,LIS_READ); 
	return (msb * 256 + lsb);
};

int16_t lis_get_z(void){
	int8_t lsb,msb;

	lsb = lis_send(0x2C,0xFF,LIS_READ); 
	msb = lis_send(0x2D,0xFF,LIS_READ); 
	return (msb * 256 + lsb);
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