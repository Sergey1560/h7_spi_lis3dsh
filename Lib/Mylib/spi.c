#include "spi.h"


void spi_init(SPI_TypeDef* SPI){
    
    spi_gpio_init();
    //PLL1Q, 48Mhz
    
    RCC->D2CCIP1R |= (2 << RCC_D2CCIP1R_SPI123SEL_Pos);
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    for(uint32_t i=0; i<0x1000; i++){__NOP();}
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;


    //SPI->CFG1 |= (7 << SPI_CFG1_MBR_Pos);//|(4 << SPI_CFG1_FTHLV_Pos);
	SPI1->CFG2 |= SPI_CFG2_SSOE|\
                  SPI_CFG2_CPOL|\
                  SPI_CFG2_CPHA|\
                  SPI_CFG2_AFCNTR|\
                  SPI_CFG2_MASTER;

    SPI1->CR1 |= SPI_CR1_SPE;
}


uint8_t spi_8breg(SPI_TypeDef* SPI,uint8_t reg, uint8_t data){
    uint8_t reply;
    
	SPI->CR2 = 2;
    SPI->CR1 |= SPI_CR1_CSTART;

	while((SPI->SR & SPI_SR_TXP) == 0){__NOP();};
    SPI1_TXDR_8b = reg;

    while((SPI->SR & SPI_SR_RXP) == 0){__NOP();};
    (void)SPI1_RXDR_8b;	


    while((SPI->SR & SPI_SR_TXP) == 0){__NOP();};
    SPI1_TXDR_8b = data;


    while((SPI->SR & SPI_SR_RXP) == 0){__NOP();};
    reply=SPI1_RXDR_8b;	

    while(!(SPI->SR & SPI_SR_EOT)){__NOP();}
    SPI->IFCR = SPI_IFCR_EOTC;

    return reply;
}


uint16_t spi_16breg(SPI_TypeDef* SPI,uint8_t reg){
    uint8_t b;
    uint16_t reply;
    
	SPI->CR2 = 3;
    SPI->CR1 |= SPI_CR1_CSTART;

	while((SPI->SR & SPI_SR_TXP) == 0){__NOP();};
    SPI1_TXDR_8b = reg;

    while((SPI->SR & SPI_SR_RXP) == 0){__NOP();};
    (void)SPI1_RXDR_8b;	
    
    while((SPI->SR & SPI_SR_TXP) == 0){__NOP();};
    SPI1_TXDR_8b = reg+1;

    while((SPI->SR & SPI_SR_RXP) == 0){__NOP();};
    b=SPI1_RXDR_8b;	
    reply = (b << 8);

    while((SPI->SR & SPI_SR_TXP) == 0){__NOP();};
    SPI1_TXDR_8b = 0xFF;

    while((SPI->SR & SPI_SR_RXP) == 0){__NOP();};
    b=SPI1_RXDR_8b;	
    reply |= b;

    SPI->IFCR = SPI_IFCR_EOTC;
    return reply;
}





void spi_gpio_init(void){
    /* SPI1
    PA4 - CS
    PA5 - SCK
    PA6 - MISO
    PA7 - MOSI
    */

   	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN; 
	
    //MODE - 10 AF
	GPIOA->MODER &= ~(GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER4_1|GPIO_MODER_MODER5_1|GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1);

	//Push-pull mode 0
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4|GPIO_OTYPER_OT_5|GPIO_OTYPER_OT_6|GPIO_OTYPER_OT_7);
	//Very High speed 11
	
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5|GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7);
    GPIOA->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR4_Pos)|\
					  (S_VH << GPIO_OSPEEDER_OSPEEDR5_Pos)|\
					  (S_VH << GPIO_OSPEEDER_OSPEEDR6_Pos)|\
					  (S_VH << GPIO_OSPEEDER_OSPEEDR7_Pos);
	
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4|GPIO_AFRL_AFRL5|GPIO_AFRL_AFRL6|GPIO_AFRL_AFRL7);
    
    //AF5
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFRL4_Pos)|\
					 (5 << GPIO_AFRL_AFRL5_Pos)|\
					 (5 << GPIO_AFRL_AFRL6_Pos)|\
					 (5 << GPIO_AFRL_AFRL7_Pos);

 	GPIOA->PUPDR |=  GPIO_PUPDR_PUPDR4_0;  
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;   // Ensure all pull up pull down resistors are disabled
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR6;   // Ensure all pull up pull down resistors are disabled
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR7;

/*
    GPIOA->PUPDR |=  GPIO_PUPDR_PUPDR3_0;  
    GPIOA->MODER &= ~(GPIO_MODER_MODER3);
    GPIOA->MODER |= (GPIO_MODER_MODER3_0);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_3);
    GPIOA->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR3_Pos);

    PIN_UP;
    */
}
