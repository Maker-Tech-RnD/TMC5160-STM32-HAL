#include "TMC5160.h"


HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){
	  //reg_addr - т.к. это на write, первый бит -> 1
	  reg_addr |= 0b10000000;
	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  buff[1] = data[0];
	  buff[2] = data[1];
	  buff[3] = data[2];
	  buff[4] = data[3];

	  //Receive
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_StatusTypeDef result = HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
	  return result;
}

HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){

	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  buff[1] = data[0];
	  buff[2] = data[1];
	  buff[3] = data[2];
	  buff[4] = data[3];
	  // sending buffer
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(htmc->spi, buff, data, 5, 100);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  return result;
}

