#include "TMC5160.h"

// Transform data for REAd\WRITE_REGISTER function
void  divide_uint32_t_and_pacckage_in_array(uint32_t value, uint8_t *data){
	data[1]  = (uint8_t)(value >> 24);
	data[2]  = (uint8_t)(value >> 16);
	data[3]  = (uint8_t)(value >> 8);
	data[4]  = (uint8_t)value;
}

uint32_t parsing_data( uint8_t data[]) {
	// Initialization parts of the message
	uint32_t third_byte = (uint32_t)data[1] << 24;
	uint32_t second_byte = (uint32_t)data[2] << 16;
	uint32_t first_byte = (uint32_t)data[3] << 8;
	uint32_t zero_byte = (uint32_t)data[4];
	// Return the uint32_t variable
	return (zero_byte | first_byte | second_byte | third_byte);
}

// WRITE/READ
HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint32_t data_of_register){
	//reg_addr + bit_of_flow - first bit show, that it's write_function or read_function -> In read case it's number: 1
	reg_addr |= 0b10000000;
	uint8_t buff[5];
	buff[0] = reg_addr;
	//Transform in uint32_t variable and receive buff
	divide_uint32_t_and_pacckage_in_array(data_of_register, buff);
	HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	HAL_StatusTypeDef result = HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	return result;
}

HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint32_t data_of_register){
	//reg_addr + bit_of_flow - first bit show, that it's write_function or read_function -> In read case it's number: 0. We didn't have to do anything, just left.
	uint8_t buff[5];
	buff[0] = reg_addr;
	// sending buffer and fill it, and after that transform in uint32_t variable
	HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(htmc->spi, buff, buff, 5, 100);
	HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	data_of_register = parsing_data(buff);
	return result;
}



// Drive functions(various velosities and accelerations)
HAL_StatusTypeDef TMC5160_setFirstAcceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_FIRST_ACCELERATION_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.first_acceleration = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setMaxAcceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_MAX_ACCELERATION_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.max_acceleration = value;
	return HAL_OK;
}
//
HAL_StatusTypeDef TMC5160_setMaxDeceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_MAX_DECELERATION_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.max_deceleration = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setSecondDeceleration(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_SECOND_DECELERATION_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.second_deceleration = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setStopVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_STOP_VELOCITY_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.stop_speed = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setFirstVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_FIRST_VELOCITY_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.first_speed = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setMaxVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
	if((value > TMC5160_MAX_VELOCITY_LIMIT) || (value < 0)){
		return HAL_ERROR;
	}
	htmc->configuration_velosity.max_speed = value;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setRampMode(TMC5160_HandleTypeDef *htmc, RampModes mode){
	htmc->configuration_velosity.ramp_mode = mode;
	return HAL_OK;
}


// New configuration bits functions

// CHOPCONF
HAL_StatusTypeDef TMC5160_Slow_decay(TMC5160_HandleTypeDef *htmc, uint32_t TOFF){
	// Sets the slow decay time (off time). This setting also limits the maximum chopper frequency.
	//Setting this parameter to zero completely disables all  driver transistors and the motor can free-wheel.
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~TOFF_Mask);
	if(TOFF > 15 ){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | TOFF;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Comparator_Blank_Time(TMC5160_HandleTypeDef *htmc, uint32_t TBL){
	//Set comparator blank time to 16, 24, 36 or 54 clocks
	//Hint: %01 or %10 is recommended for most applications
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~TBL_Mask);
	if(TBL >3){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (TBL << 15);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Selection_Of_ChopperMode(TMC5160_HandleTypeDef *htmc, uint32_t chm){
	//0 SpreadCycle
	//1 classic const. off time
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~chm_Mask);
	if((chm == 0) || (chm == 1)){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (chm << 14);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Increase_Passive_Decay(TMC5160_HandleTypeDef *htmc, uint32_t TPFD){
	//TPFD allows dampening of motor mid-range resonances.
	//Passive fast decay time setting controls duration of the
	//fast decay phase inserted after bridge polarity change NCLK= 128 * TPFD;
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~TPFD_Mask);
	if(TPFD > 15){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | ( TPFD << 20);
	return HAL_OK;
}
// Вопрос с chm - второй режим не понятен
HAL_StatusTypeDef TMC5160_Hysteresis_Start_Setting(TMC5160_HandleTypeDef *htmc, uint32_t HSTRT){
	uint32_t chm = htmc->configuration.CHOPCONF & chm_Mask;
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF &(~HSTRT_Mask);
	if(HSTRT > 7){
		return HAL_ERROR;
	}
	if(chm == 0){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (HSTRT << 4);
		return HAL_OK;
	}
	if(chm == 1){
		return HAL_ERROR;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Hysteresis_End_Setting(TMC5160_HandleTypeDef *htmc, uint32_t HEND){
	//HSTRT+HEND must be ≤16.
	uint32_t chm = htmc->configuration.CHOPCONF & chm_Mask;
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~HEND_Mask);
	if(HEND > 15){
		return HAL_ERROR;
	}
	if(chm == 0){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (HEND << 7);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Fast_Decay_Time_Setting_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t TFD){
	//Fast decay time setting. With CHM=1, these bits  control the portion of fast decay for each chopper cycle.
	uint32_t chm = htmc->configuration.CHOPCONF & chm_Mask;
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~(TFD_Mask|fd3_Mask));
	if(TFD > 15){
		return HAL_ERROR;
	}
	if(chm == 0x4000){
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | ( (TFD & 0x8) << 8);
		htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | ( (TFD & 0x7) << 4);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMC5160_Sine_Wave_Offset_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t OFFSET){
	//With CHM=1, these bits control the sine wave offset. A positive offset corrects for zero crossing error.
	uint32_t chm = htmc->configuration.CHOPCONF & chm_Mask;
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~OFFSET_Mask);

	if(OFFSET > 15){
		return HAL_ERROR;
	}
	if(chm != 0x4000){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (OFFSET << 7);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Fast_Decay_Mode_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t disfdcc){
	//With CHM=1, these bits control the sine wave offset. A positive offset corrects for zero crossing error.
	uint32_t chm = htmc->configuration.CHOPCONF & chm_Mask;
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~disfdcc_Mask);
	if(disfdcc > 1){
		return HAL_ERROR;
	}
	if(chm != 0x4000){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | ( disfdcc << 12);
	return HAL_OK;
}





// IHOLD_IRUN
HAL_StatusTypeDef TMC5160_Standstill_Current(TMC5160_HandleTypeDef *htmc, uint32_t IHOLD){\
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN & (~IHOLD_Mask);
	if(IHOLD > 31){
		return HAL_ERROR;
	}
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN | (IHOLD);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Motor_Run_Current(TMC5160_HandleTypeDef *htmc, uint32_t IRUN){
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN & (~IRUN_Mask);
	if( IRUN > 31){
		return HAL_ERROR;
	}
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN | (IRUN << 8);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Smooth_Current_Reduction(TMC5160_HandleTypeDef *htmc, uint32_t IHOLDDELAY){
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN & (~IHOLDDELAY_Mask);
	if(IHOLDDELAY > 15){
		return HAL_ERROR;
	}
	htmc->configuration.IHOLD_IRUN = htmc->configuration.IHOLD_IRUN | (IHOLDDELAY << 16);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Global_Scaling_Of_Motor_Current(TMC5160_HandleTypeDef *htmc, uint32_t GLOBALSCALER){
	if(GLOBALSCALER > 255){
		return HAL_ERROR;
	}
	htmc->configuration.GLOBALSCALER = GLOBALSCALER;
	return HAL_OK;
}






//Velocity Based Mode Control
HAL_StatusTypeDef TMC5160_Flag_Indicates_Motor_Stand_Still (TMC5160_HandleTypeDef *htmc, uint32_t stst){
	htmc->configuration.DRV_STATUS = htmc->configuration.DRV_STATUS & (~stst_Mask);
	if(stst > 1){
		return HAL_ERROR;
	}
	htmc->configuration.DRV_STATUS = htmc->configuration.DRV_STATUS | (stst << 31);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Delay_Time_After_Standstill(TMC5160_HandleTypeDef *htmc, uint32_t TPOWER_DOWN){
	//TPOWERDOWN sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to4 seconds
	if((TPOWER_DOWN > 0x41FF) || (TPOWER_DOWN < 2)){
		return HAL_ERROR;
	}
	htmc->configuration.TPOWER_DOWN = TPOWER_DOWN;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Upper_Velocity_For_StealthChop_Voltage_PWM_Mode(TMC5160_HandleTypeDef *htmc, uint32_t TPWMTHRS){
	//This is the upper velocity for StealthChop voltage PWM mode.
	//TSTEP ≥ TPWMTHRS
	//- StealthChop PWM mode is enabled, if configured
	//- DcStep is disabled
	/*uint32_t check = htmc->configuration.TSTEP;*/
	if((TPWMTHRS > 1048575)){
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
	if((GLOBALSCALER > 255) || (THIGH > 1048575)){
		return HAL_ERROR;
	}
	htmc->configuration.THIGH = THIGH;
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_Hysteresis_For_Step_Frequency_Comparison(TMC5160_HandleTypeDef *htmc, uint32_t small_hysteresis){
	htmc->configuration.GCONF = htmc->configuration.GCONF & (~small_hysteresis_Mask);
	if( small_hysteresis > 1){
		return HAL_ERROR;
	}
	htmc->configuration.GCONF = htmc->configuration.GCONF | (small_hysteresis << 14);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_High_Velocity_Fullstep_Selection(TMC5160_HandleTypeDef *htmc, uint32_t vhighfs){
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~vhighfs);
	if( vhighfs > 1){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (vhighfs << 18);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_High_Velocity_Chopper_Mode(TMC5160_HandleTypeDef *htmc, uint32_t vhighchm){
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF & (~vhighchm_Mask);
	if( vhighchm > 1){
		return HAL_ERROR;
	}
	htmc->configuration.CHOPCONF = htmc->configuration.CHOPCONF | (vhighchm << 19);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_StealthChop_Voltage_PWM_Enable_Flag(TMC5160_HandleTypeDef *htmc, uint32_t en_pwm_mode){
	htmc->configuration.GCONF = htmc->configuration.GCONF & (~en_pwm_mode_Mask);
	if( en_pwm_mode > 1){
		return HAL_ERROR;
	}
	htmc->configuration.GCONF = htmc->configuration.GCONF | (en_pwm_mode << 2);
	return HAL_OK;
}




// Configuration Drive: fill Example from the DataSheet's variable
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

// Default Configurations, that initializes all characteristics and variables for TMC5160
HAL_StatusTypeDef TMC5160_default_init(TMC5160_HandleTypeDef *htmc){
	HAL_StatusTypeDef result;
	result = TMC5160_Configuration_Drive(htmc);
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


// New all configuration
HAL_StatusTypeDef TMC5160_Bits_Configuration(TMC5160_HandleTypeDef *htmc){
	TMC5160_Slow_decay(htmc,3);
	TMC5160_Comparator_Blank_Time(htmc,2);
	TMC5160_Selection_Of_ChopperMode(htmc, 0);
	TMC5160_Increase_Passive_Decay(htmc, 0);
	TMC5160_Hysteresis_Start_Setting(htmc, 4);
	TMC5160_Hysteresis_End_Setting(htmc, 1);
	// IHOLD_IRUN
	TMC5160_Standstill_Current(htmc, 10);
	TMC5160_Motor_Run_Current(htmc,31);
	TMC5160_Smooth_Current_Reduction(htmc, 6);
	//Velocity Based Mode Control
	TMC5160_Delay_Time_After_Standstill(htmc, 10);
	TMC5160_StealthChop_Voltage_PWM_Enable_Flag(htmc, 1);
	TMC5160_Upper_Velocity_For_StealthChop_Voltage_PWM_Mode(htmc, 500);
	return HAL_OK;
}
//  Configuration example from the DataSheet
HAL_StatusTypeDef TMC5160_Configuration_Mode(TMC5160_HandleTypeDef *htmc){
	// Example from the DataSheet's variable
	HAL_StatusTypeDef result;
	// The crutch that sends the first parcel of data, since the first parcel is not sent once (Alert: IT'S a BAG!!!)
	result = TMC5160_Bits_Configuration(htmc);
	if(result != HAL_OK){
	return result;
	}
	result = TMC5160_WriteRegister(htmc, CHOPCONF, htmc->configuration.CHOPCONF);
	if(result != HAL_OK){
		return result;
	}
	result = TMC5160_WriteRegister(htmc, CHOPCONF, htmc->configuration.CHOPCONF);
	if(result != HAL_OK){
		return result;
	}
	result = TMC5160_WriteRegister(htmc, IHOLD_IRUN, htmc->configuration.IHOLD_IRUN);
	if(result != HAL_OK){
		return result;
	}
	result = TMC5160_WriteRegister(htmc, TPOWERDOWN, htmc->configuration.TPOWER_DOWN);
	if(result != HAL_OK){
		return result;
	}
	result = TMC5160_WriteRegister(htmc, GCONF, htmc->configuration.GCONF);
	if(result != HAL_OK){
		return result;
	}
	result = TMC5160_WriteRegister(htmc, TPWMTHRS, htmc->configuration.TPWMTHRS);
	if(result != HAL_OK){
		return result;
	}

	return result;
}


HAL_StatusTypeDef TMC5160_default_init_Bits_time(TMC5160_HandleTypeDef *htmc){
	HAL_StatusTypeDef result;
	result = TMC5160_Configuration_Drive(htmc);
	if(result != HAL_OK){
		return result;
	}
	result = TMC5160_Configuration_Mode(htmc);
	if(result != HAL_OK){
		return result;
	}
	// VELOCITY & ACCELARATION
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


//Coнfiguratioн
HAL_StatusTypeDef TMC5160_SpreadCycle_Configuration(TMC5160_HandleTypeDef *htmc){
	TMC5160_StealthChop_Voltage_PWM_Enable_Flag(htmc, 0);
	TMC5160_Slow_decay(htmc,5);//TOFF=5,TL=20=;HSTART=0=HED
	// IHOLD_IRUN
	TMC5160_Standstill_Current(htmc, 10);
	TMC5160_Motor_Run_Current(htmc,31);
	TMC5160_Smooth_Current_Reduction(htmc, 6);
	//Velocity Based Mode Control
	TMC5160_Delay_Time_After_Standstill(htmc, 10);
	TMC5160_StealthChop_Voltage_PWM_Enable_Flag(htmc, 1);
	TMC5160_Upper_Velocity_For_StealthChop_Voltage_PWM_Mode(htmc, 500);
	return HAL_OK;
}

