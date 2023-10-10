#include "TMC5160.h"

// Transform data
void  divide_uint32_t_and_pacckage_in_array(uint32_t value, uint8_t *data){
    data[1]  = (uint8_t)(value >> 24);
    data[2]  = (uint8_t)(value >> 16);
    data[3]  = (uint8_t)(value >> 8);
    data[4]  = (uint8_t)value;
}

uint32_t parsing_data( uint8_t data[]) {

    // Init parts of the message
    uint32_t third_byte = (uint32_t)data[1] << 24;
    uint32_t second_byte = (uint32_t)data[2] << 16;
    uint32_t first_byte = (uint32_t)data[3] << 8;
    uint32_t zero_byte = (uint32_t)data[4];
    // Return
    return (zero_byte | first_byte | second_byte | third_byte);
}

// WRITE/READ
HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint32_t data_of_register){
	  //reg_addr - т.к. это на write, первый бит -> 1
	  reg_addr |= 0b10000000;
	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  divide_uint32_t_and_pacckage_in_array(data_of_register, buff);
	  //Receive
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_StatusTypeDef result = HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  return result;
}

HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint32_t data){
	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  // sending buffer
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(htmc->spi, buff, buff, 5, 100);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  data = parsing_data(buff);
	  return result;
}

//  Configuration


HAL_StatusTypeDef TMC5160_Configuration(TMC5160_HandleTypeDef *htmc){

	uint32_t cmd_init[5] = {0x100C3,0x61F0A,0x0A,0x04,0x01F4};
	// Костыль, который отправляет первую посылку данных, так как первая  посылка с одного раза не отправляется
	HAL_StatusTypeDef result = TMC5160_WriteRegister(htmc, CHOPCONF, cmd_init[0]);

	if(result != HAL_OK){
				return result;
		}

	result = TMC5160_WriteRegister(htmc, CHOPCONF, cmd_init[0]);

	if(result != HAL_OK){
			return result;
	}

	result = TMC5160_WriteRegister(htmc, IHOLD_IRUN, cmd_init[1]);

	if(result != HAL_OK){
				return result;
	}

	result = TMC5160_WriteRegister(htmc, TPOWERDOWN, cmd_init[2]);

	if(result != HAL_OK){
		return result;
	}

	result = TMC5160_WriteRegister(htmc, GCONF, cmd_init[3]);

	if(result != HAL_OK){
				return result;
	}

	result = TMC5160_WriteRegister(htmc,  TPWMTHRS, cmd_init[4]);

	if(result != HAL_OK){
				return result;
	}
	return result ;
}




// Drive functions
HAL_StatusTypeDef TMC5160_setFirstAcceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
if((value > TMC5160_FIRST_ACCELERATION_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
    htmc->configuration_velosity.first_acceleration = value;
	return HAL_OK;
}


HAL_StatusTypeDef TMC5160_setMaxAcceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_MAX_ACCELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
    htmc->configuration_velosity.max_acceleration = value;
	return HAL_OK;
}
//
HAL_StatusTypeDef TMC5160_setMaxDeceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_MAX_DECELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
    htmc->configuration_velosity.max_deceleration = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setSecondDeceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_SECOND_DECELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
    htmc->configuration_velosity.second_deceleration = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setStopVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_STOP_VELOCITY_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
    htmc->configuration_velosity.stop_speed = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setFirstVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_FIRST_VELOCITY_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
  	htmc->configuration_velosity.first_speed = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setMaxVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_MAX_VELOCITY_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
    htmc->configuration_velosity.max_speed = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setRampMode(TMC5160_HandleTypeDef *htmc, RampModes mode){
	htmc->configuration_velosity.ramp_mode = mode;
	return HAL_OK;
}

// Configuration Drive
HAL_StatusTypeDef TMC5160_Configuration_Drive(TMC5160_HandleTypeDef *htmc){
	TMC5160_setFirstAcceleration(htmc,1000);
	TMC5160_setFirstVelocity(htmc,50000);
	TMC5160_setMaxAcceleration(htmc,500);
	TMC5160_setMaxVelocity(htmc,200000);
	TMC5160_setMaxDeceleration(htmc,700);
	TMC5160_setSecondDeceleration(htmc,1400);
	TMC5160_setStopVelocity(htmc,10);
	TMC5160_setRampMode(htmc, Positioning);
	return HAL_OK;
}

// Default Configurations
HAL_StatusTypeDef TMC5160_default_init(TMC5160_HandleTypeDef *htmc){

	HAL_StatusTypeDef result  =   TMC5160_Configuration_Drive(htmc);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_Configuration(htmc);
	if(result != HAL_OK){
			return result;
		    }
	result = TMC5160_WriteRegister(htmc, A1, htmc->configuration_velosity.first_acceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, V1, htmc->configuration_velosity.first_speed);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, AMAX, htmc->configuration_velosity.max_acceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, VMAX, htmc->configuration_velosity.max_speed);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, DMAX, htmc->configuration_velosity.max_deceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, D1, htmc->configuration_velosity.second_deceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, VSTOP, htmc->configuration_velosity.stop_speed);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, RAMPMODE, htmc->configuration_velosity.ramp_mode);
	if(result != HAL_OK){
				return result;
			}
	return HAL_OK;
}


// New config function
// Drive functions
HAL_StatusTypeDef TMC5160_Conguration(TMC5160_HandleTypeDef *htmc){

	uint32_t cmd_init[5] = {0x100C2,0x61F0A,0x0A,0x04,0x01F4};
	// Костыль, который отправляет первую посылку данных, так как первая  посылка с одного раза не отправляется
	HAL_StatusTypeDef result = TMC5160_WriteRegister(htmc, CHOPCONF, cmd_init[0]);

	if(result != HAL_OK){
				return result;
		}

	result = TMC5160_WriteRegister(htmc, CHOPCONF, cmd_init[0]);

	if(result != HAL_OK){
			return result;
	}

	result = TMC5160_WriteRegister(htmc, IHOLD_IRUN, cmd_init[1]);

	if(result != HAL_OK){
				return result;
	}

	result = TMC5160_WriteRegister(htmc, TPOWERDOWN, cmd_init[2]);

	if(result != HAL_OK){
		return result;
	}

	result = TMC5160_WriteRegister(htmc, GCONF, cmd_init[3]);

	if(result != HAL_OK){
				return result;
	}

	result = TMC5160_WriteRegister(htmc,  TPWMTHRS, cmd_init[4]);

	if(result != HAL_OK){
				return result;
	}
	return result ;
}


















// CHOPCONF
HAL_StatusTypeDef TMC5160_Slow_decay(TMC5160_HandleTypeDef *htmc,uint32_t frequency, uint32_t frequency_clock, uint32_t persent_of_chopperCycle){
	// Sets the slow decay time (off time). This setting also limits the maximum chopper frequency.
	//Setting this parameter to zero completely disables all  driver transistors and the motor can free-wheel.
	uint32_t time = ((1 / frequency) * (persent_of_chopperCycle / 100) * 0.5);
	//Здесь необходмо округлть  значение
	uint32_t TOFF = (time*frequency_clock - 12) / 32;
	if(TOFF > 15 ){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = TOFF;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Comparator_Blank_Time(TMC5160_HandleTypeDef *htmc, uint32_t TBL){
//Set comparator blank time to 16, 24, 36 or 54 clocks
//Hint: %01 or %10 is recommended for most applications
	if((TBL != 16) || (TBL != 24) || (TBL != 36) || (TBL != 54)){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + (TBL << 15);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Selection_Of_ChopperMode(TMC5160_HandleTypeDef *htmc, uint32_t chm){
	//0 SpreadCycle
	//1 classic const. off time
	if((chm != 0) || (chm != 1)){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + (chm << 14);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Increase_Passive_Decay(TMC5160_HandleTypeDef *htmc, uint32_t TPFD){
//TPFD allows dampening of motor mid-range resonances.
//Passive fast decay time setting controls duration of the
//fast decay phase inserted after bridge polarity change NCLK= 128 * TPFD;
	if(TPFD > 15){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + ( TPFD << 20);
	return HAL_OK;
}
// Вопрос с chm - второй режим не понятен
HAL_StatusTypeDef TMC5160_Hysteresis_Start_Setting(TMC5160_HandleTypeDef *htmc, uint32_t HSTRT){
	uint32_t chm = htmc->configuration.CHOPCONF & 0x4000;
	if(HSTRT > 7){
		return HAL_ERROR;
	}
	if(chm == 0){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + (HSTRT << 4);
		return HAL_OK;
	}
	if(chm == 1){

		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Hysteresis_End_Setting(TMC5160_HandleTypeDef *htmc, uint32_t HEND){
//HSTRT+HEND must be ≤16.
	uint32_t chm = htmc->configuration.CHOPCONF & 0x4000;
	if(HEND > 15){
		return HAL_ERROR;
	}
	if(chm == 0){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + (HEND << 7);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Fast_Decay_Time_Setting_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t TFD){
//Fast decay time setting. With CHM=1, these bits  control the portion of fast decay for each chopper cycle.
	uint32_t chm = htmc->configuration.CHOPCONF & 0x4000;
	if(TFD > 15){
		return HAL_ERROR;
	}
	if(chm == 1){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + ( (TFD & 0b00000111) << 4);
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + ( (TFD & 0b00001000) << 11);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Sine_Wave_Offset_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t OFFSET){
//With CHM=1, these bits control the sine wave offset. A positive offset corrects for zero crossing error.
	uint32_t chm = htmc->configuration.CHOPCONF & 0x4000;
	if(OFFSET > 15){
		return HAL_ERROR;
	}
	if(chm != 1){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + (OFFSET << 7);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Fast_Decay_Mode_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t disfdcc){
//With CHM=1, these bits control the sine wave offset. A positive offset corrects for zero crossing error.
	uint32_t chm = htmc->configuration.CHOPCONF & 0x4000;
	if(disfdcc > 1){
		return HAL_ERROR;
	}
	if(chm != 1){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF + ( disfdcc << 12);
	return HAL_OK;
}






// IHOLD_IRUN

HAL_StatusTypeDef TMC5160_Standstill_Current(TMC5160_HandleTypeDef *htmc, uint32_t IHOLD){
	if(IHOLD > 31){
		return HAL_ERROR;
	}
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN + (IHOLD);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Motor_Run_Current(TMC5160_HandleTypeDef *htmc, uint32_t IRUN){
	if( IRUN > 31){
		return HAL_ERROR;
	}
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN + (IRUN << 8);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Smooth_Current_Reduction(TMC5160_HandleTypeDef *htmc, uint32_t IHOLDDELAY){
	if(IHOLDDELAY > 1){
		return HAL_ERROR;
	}
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN + (IHOLDDELAY << 16);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Global_Scaling_Of_Motor_Current(TMC5160_HandleTypeDef *htmc, uint32_t GLOBALSCALER){
	if(GLOBALSCALER > 255){
		return HAL_ERROR;
	}
	htmc->configuration.GLOBALSCALER = htmc->configuration.GLOBALSCALER + (GLOBALSCALER);
	return HAL_OK;
}


//Velocity Based Mode Control





HAL_StatusTypeDef TMC5160_Flag_Indicates_Motor_Stand_Still (TMC5160_HandleTypeDef *htmc, uint32_t stst){
	if(stst > 1){
		return HAL_ERROR;
	}
	htmc->configuration.DRV_STATUS = htmc->configuration.DRV_STATUS + (stst << 31);
	return HAL_OK;
}




HAL_StatusTypeDef TMC5160_Delay_Time_After_Standstill(TMC5160_HandleTypeDef *htmc, uint32_t TPOWER_DOWN){
	//TPOWERDOWN sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to4 seconds
	if(TPOWER_DOWN > 4){
		return HAL_ERROR;
	}
	htmc->configuration.TPOWER_DOWN = htmc->configuration.TPOWER_DOWN + (TPOWER_DOWN);
	return HAL_OK;
}


HAL_StatusTypeDef TMC5160_Upper_Velocity_For_StealthChop_Voltage_PWM_Mode(TMC5160_HandleTypeDef *htmc, uint32_t TPWMTHRS){
	//This is the upper velocity for StealthChop voltage PWM mode.
	//TSTEP ≥ TPWMTHRS
	//- StealthChop PWM mode is enabled, if configured
	//- DcStep is disabled
	uint32_t check =  htmc->configuration.TSTEP;
	if((TPWMTHRS > check) || (TPWMTHRS > 1048575)){
		return HAL_ERROR;
	}
	htmc->configuration.TPWMTHRS = TPWMTHRS;
	return HAL_OK;
}


HAL_StatusTypeDef TMC5160_Control_The_Lower_Velocity_Threshold_For_Operation_With_CoolStep_And_StallGuard(TMC5160_HandleTypeDef *htmc, uint32_t TCOOLTHRS){
	//Setting to control the lower velocity threshold for operation with CoolStep and StallGuard
	//TCOOLTHRS ≥ TSTEP
	uint32_t check =  htmc->configuration.TSTEP;
	if((TCOOLTHRS > 1048575) || (TCOOLTHRS < check)){
		return HAL_ERROR;
	}
	// Функция недописана =Ю следует добавить ерорные условия для stealhchop
	htmc->configuration.TCOOLTHRS = TCOOLTHRS;
	return HAL_OK;
}



HAL_StatusTypeDef TMC5160_Control_The_upper_threshold_for_operation_with_CoolStep_and_StallGuard(TMC5160_HandleTypeDef *htmc, uint32_t THIGH){
	if(GLOBALSCALER > 255){
		return HAL_ERROR;
	}
	htmc->configuration.THIGH = THIGH;
	return HAL_OK;
}
