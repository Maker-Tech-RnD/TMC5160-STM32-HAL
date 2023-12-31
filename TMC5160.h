/*
 * TMC5160_STM32_HAL.h
 *
 *  Created on: Jul 1, 2023
 *      Author: sam
 */

#ifndef LIBS_TMC5160_STM32_HAL_TMC5160_STM32_HAL_H_
#define LIBS_TMC5160_STM32_HAL_TMC5160_STM32_HAL_H_
#include "main.h"

//Filter setting macros
#define TMC5160_FIRST_ACCELERATION_LIMIT     0xFFFF // 2^16 - 1
#define TMC5160_FIRST_VELOCITY_LIMIT         0xFFFFF  // 2^20 - 1
#define TMC5160_MAX_ACCELERATION_LIMIT       0xFFFF // 2^16 - 1
/*Attention for BELOW: Do not set 0 in positioning mode, even if V1=0!*/
#define TMC5160_SECOND_DECELERATION_LIMIT    0xFFFF  // 2^16 - 1
#define TMC5160_MAX_DECELERATION_LIMIT       0xFFFF  // 2^16 - 1
/*Attention BELOW: Do not set 0 in positioning mode, minimum 10 recommend!*/
#define TMC5160_STOP_VELOCITY_LIMIT          0x3FFFF  // 2^18 - 1
#define TMC5160_MAX_VELOCITY_LIMIT           8388096  // 2^23 - 512



//Registers
typedef enum{
	//GENERAL CONFIGURATION REGISTERS (0X00…0X0F)
	GCONF			= 0x00,	// RW
	GSTAT			= 0x01,	// R + WC
	IFCNT			= 0x02, // R
	SLAVECONF		= 0x03, // W
	IOIN			= 0x04, // R
	OUTPUT			= 0x04, // W --- In UART mode, SDO_CFG0 is an output.
	X_COMPARE		= 0x05, // W
	OTP_PROG		= 0x06, // W
	OTP_READ		= 0x07, // R
	FACTORY_CONF	= 0x08, // RW
	SHORT_CONF		= 0x09, // W
	DRV_CONF		= 0x0A, // W
	GLOBALSCALER	= 0x0B, // W
	OFFSET_READ		= 0x0C, // R
	//VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10…0X1F)
	IHOLD_IRUN		= 0x10, // W
	TPOWERDOWN		= 0x11, // W
	TSTEP			= 0x12, // R
	TPWMTHRS		= 0x13, // W
	TCOOLTHRS		= 0x14, // W
	THIGH			= 0x15, // W
	//RAMP GENERATOR MOTION CONTROL REGISTER SET (0X20…0X2D)
	RAMPMODE		= 0x20, // RW
	XACTUAL			= 0x21, // RW
	VACTUAL			= 0x22, // R
	VSTART			= 0x23, // W
	A1				= 0x24, // W
	V1				= 0x25, // W
	AMAX			= 0x26, // W
	VMAX			= 0x27, // W
	DMAX			= 0x28, // W
	D1				= 0x2A, // W
	VSTOP			= 0x2B, // W
	TZEROWAIT		= 0x2C, // W
	XTARGET			= 0x2D, // RW
	//RAMP GENERATOR DRIVER FEATURE CONTROL REGISTER SET (0X30…0X36)
	VDCMIN			= 0x33, // W
	SW_MODE			= 0x34, // RW
	RAMP_STAT		= 0x35, // R + WC
	XLATCH			= 0x36, // R
	//ENCODER REGISTER SET (0X38…0X3C)
	ENCMODE			= 0x38, // RW
	X_ENC			= 0x39, // RW
	ENC_CONST		= 0x3A, // W
	ENC_STATUS		= 0x3B, // R + WC
	ENC_LATCH		= 0x3C, // R
	ENC_DEVIATION	= 0x3D, // W
	//DRIVER REGISTER SET (0X6C…0X7F)
	CHOPCONF		= 0x6C, // RW
	COOLCONF		= 0x6D, // W
	DCCTRL			= 0x6E, // W
	DRV_STATUS		= 0x6F, // R
	PWMCONF			= 0x70, // W
	PWM_SCALE		= 0x71, // R
	PWM_AUTO		= 0x72, // R
	LOST_STEPS		= 0x73, // R
}TMC5160_Regs;

//RampMode
typedef enum  {
	/* using all A, D and V parameters */
	Positioning = 0,
	/* Velocity mode to positive VMAX (using AMAX acceleration) */
	VelocityPositive = 1,
	/* Velocity mode to negative VMAX (using AMAX acceleration) */
	VelocityNegative = 2,
	/* velocity remains unchanged, unless stop event occurs */
	Hold = 3,
}RampModes;

// Structs for TMC5160
typedef struct{
	//Configuration Acceleration and velocity
	uint32_t CHOPCONF;
	uint32_t IHOLD_IRUN;
	uint32_t DRV_STATUS;
	uint32_t TPOWER_DOWN;
	uint32_t TSTEP;
	uint32_t TPWMTHRS;
	uint32_t TCOOLTHRS;
	uint32_t THIGH;
	uint32_t GCONF;
	uint32_t GLOBALSCALER;
}TMC5160_RegisterOfConfiguration_HandleTypeDef;

typedef struct{
	//Configuration Acceleration and velocity
	uint32_t max_speed;
	uint32_t stop_speed;
	uint32_t first_speed;
	uint32_t first_acceleration;
	uint32_t max_deceleration;
	uint32_t second_deceleration;
	uint32_t max_acceleration;
	uint32_t ramp_mode;
}TMC5160_ConfigurationOfVelosity_HandleTypeDef;

typedef struct{
	//HAL configuration of the MCU
	SPI_HandleTypeDef        *spi;
	GPIO_TypeDef           *GPIOx;
	uint16_t				   CS;
	//Configuration Mode
	TMC5160_RegisterOfConfiguration_HandleTypeDef configuration;
	//Configuration Acceleration and velocity
	TMC5160_ConfigurationOfVelosity_HandleTypeDef configuration_velosity;
}TMC5160_HandleTypeDef;

//Prototypes Functions
HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint32_t data_of_register);
HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint32_t data_of_register);
HAL_StatusTypeDef TMC5160_Configuration(TMC5160_HandleTypeDef *htmc);

//Configuration  function of the Velocity and Acceleration
HAL_StatusTypeDef TMC5160_setFirstAcceleration(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setMaxAcceleration(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setMaxDeceleration(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setSecondDeceleration(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setStopVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setFirstVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setMaxVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value);
HAL_StatusTypeDef TMC5160_setRampMode(TMC5160_HandleTypeDef *htmc, RampModes mode);

//Configuration Drive of all function of the Velocity and Acceleration
HAL_StatusTypeDef TMC5160_Configuration_Drive(TMC5160_HandleTypeDef *htmc);

//All configuration of the example from DataSheet
HAL_StatusTypeDef TMC5160_default_init(TMC5160_HandleTypeDef *htmc);

// New configuration bits functions
	// CHOPCONF
HAL_StatusTypeDef TMC5160_Slow_decay(TMC5160_HandleTypeDef *htmc,uint32_t TOFF);
HAL_StatusTypeDef TMC5160_Comparator_Blank_Time(TMC5160_HandleTypeDef *htmc, uint32_t TBL);
HAL_StatusTypeDef TMC5160_Selection_Of_ChopperMode(TMC5160_HandleTypeDef *htmc, uint32_t chm);
HAL_StatusTypeDef TMC5160_Increase_Passive_Decay(TMC5160_HandleTypeDef *htmc, uint32_t TPFD);
HAL_StatusTypeDef TMC5160_Hysteresis_Start_Setting(TMC5160_HandleTypeDef *htmc, uint32_t HSTRT);
HAL_StatusTypeDef TMC5160_Hysteresis_End_Setting(TMC5160_HandleTypeDef *htmc, uint32_t HEND);
	// Вопрос с chm - второй режим не понятен
HAL_StatusTypeDef TMC5160_Fast_Decay_Time_Setting_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t TFD);
HAL_StatusTypeDef TMC5160_Sine_Wave_Offset_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t OFFSET);
HAL_StatusTypeDef TMC5160_Fast_Decay_Mode_CHM1(TMC5160_HandleTypeDef *htmc, uint32_t disfdcc);
	// IHOLD_IRUN
HAL_StatusTypeDef TMC5160_Standstill_Current(TMC5160_HandleTypeDef *htmc, uint32_t IHOLD);
HAL_StatusTypeDef TMC5160_Motor_Run_Current(TMC5160_HandleTypeDef *htmc, uint32_t IRUN);
HAL_StatusTypeDef TMC5160_Smooth_Current_Reduction(TMC5160_HandleTypeDef *htmc, uint32_t IHOLDDELAY);
HAL_StatusTypeDef TMC5160_Global_Scaling_Of_Motor_Current(TMC5160_HandleTypeDef *htmc, uint32_t GLOBALSCALER);
	//Velocity Based Mode Control
HAL_StatusTypeDef TMC5160_Flag_Indicates_Motor_Stand_Still (TMC5160_HandleTypeDef *htmc, uint32_t stst);
HAL_StatusTypeDef TMC5160_Delay_Time_After_Standstill(TMC5160_HandleTypeDef *htmc, uint32_t TPOWER_DOWN);
HAL_StatusTypeDef TMC5160_Upper_Velocity_For_StealthChop_Voltage_PWM_Mode(TMC5160_HandleTypeDef *htmc, uint32_t TPWMTHRS);
HAL_StatusTypeDef TMC5160_Control_The_Lower_Velocity_Threshold_For_Operation_With_CoolStep_And_StallGuard(TMC5160_HandleTypeDef *htmc, uint32_t TCOOLTHRS);
HAL_StatusTypeDef TMC5160_Control_The_upper_threshold_for_operation_with_CoolStep_and_StallGuard(TMC5160_HandleTypeDef *htmc, uint32_t THIGH);
HAL_StatusTypeDef TMC5160_Hysteresis_For_Step_Frequency_Comparison(TMC5160_HandleTypeDef *htmc, uint32_t small_hysteresis);
HAL_StatusTypeDef TMC5160_High_Velocity_Fullstep_Selection(TMC5160_HandleTypeDef *htmc, uint32_t vhighfs);
HAL_StatusTypeDef TMC5160_High_Velocity_Chopper_Mode(TMC5160_HandleTypeDef *htmc, uint32_t vhighchm);
HAL_StatusTypeDef TMC5160_StealthChop_Voltage_PWM_Enable_Flag(TMC5160_HandleTypeDef *htmc, uint32_t en_pwm_mod);

// New all configuration BITS TIME
HAL_StatusTypeDef TMC5160_Bits_Configuration(TMC5160_HandleTypeDef *htmc);
HAL_StatusTypeDef TMC5160_Configuration_Mode(TMC5160_HandleTypeDef *htmc);
HAL_StatusTypeDef TMC5160_default_init_Bits_time(TMC5160_HandleTypeDef *htmc);

#endif /* LIBS_TMC5160_STM32_HAL_TMC5160_STM32_HAL_H_ */
