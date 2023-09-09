#include "TMC5160.h"


//  Это все,  что   я добавил!

HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){
	  //starting translation, set down
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  // reset SPI "message count" counter
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  // sending buffer
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);

	  //reg_addr - т.к. это на write, первый бит -> 1
	  reg_addr |= 0b10000000;
	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  buff[1] = data[0];
	  buff[2] = data[1];
	  buff[3] = data[2];
	  buff[4] = data[3];


	  //reg_addr - как выполнить сложение его с буфером, чтобы за один заход вместе отпрваить?
	  HAL_StatusTypeDef result = HAL_SPI_Transmit(htmc->_hspi, buff, 5, 100);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
	  return result;
}

HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  // sending buffer
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);

	  reg_addr &= 0b01111111;
	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  buff[1] = data[0];
	  buff[2] = data[1];
	  buff[3] = data[2];
	  buff[4] = data[3];

	  HAL_StatusTypeDef result = HAL_SPI_TransmitRecieve(htmc->_hspi, buff, data, 5, 100);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  return result;
}

//  Configuration

uint8_t cmd_enable[8][5]= {
		{0xA4, 0x00, 0x00, 0x03, 0xE8},
		{0xA5, 0x00, 0x00, 0xC3, 0x50},
		{0xA6, 0x00, 0x00, 0x01, 0xF4},
		{0xA7, 0x00, 0x03, 0x0D, 0x40},
		{0xA8, 0x00, 0x00, 0x02, 0xBC},
		{0xAA, 0x00, 0x00, 0x05, 0x78},
		{0xAB, 0x00, 0x00, 0x00, 0x0A},
		{0xA0, 0x00, 0x00, 0x00, 0x00},

};


HAL_StatusTypeDef TMC5160_Configuration(TMC5160_HandleTypeDef *htmc,){

	uint8_t cmd_init[5][5]= {
			{0, 0x00, 0x01, 0x00, 0xC3},
			{0,0x00,0x06,0x1F,0x0A},
			{0, 0x00, 0x00, 0x00, 0x0A},
			{0, 0x00, 0x00, 0x00, 0x04},
			{0, 0x00, 0x00, 0x01, 0xF4},

	};

	TMC5160_WriteRegister(&htmc, CHOPCONF, cmd_init[0]);
	TMC5160_WriteRegister(&htmc, IHOLD_IRUN, cmd_init[1]);
	TMC5160_WriteRegister(&htmc, TPOWERDOWN, cmd_init[2]);
	TMC5160_WriteRegister(&htmc, GCONF, cmd_init[3]);
	TMC5160_WriteRegister(&htmc,  TPWMTHRS, cmd_init[4]);

}
