#include "TMC5160.h"


//  Это все,  что   я добавил!

HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){
	  //starting translation, set down
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_Delay(10);
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
	  HAL_Delay(1);
	  return result;
}

HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_Delay(10);
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

	  HAL_SPI_Transmit(htmc->_hspi, buff, 5, 100);
	  HAL_StatusTypeDef result = HAL_SPI_TransmitRecieve(htmc->_hspi, buff,data, 5, 100);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  HAL_Delay(1);
	  return result;
}


