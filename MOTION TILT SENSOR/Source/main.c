/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"
#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90
#define TASK_MOTION_SENSOR_FREQ_HZ (50) 
#define TASK_I2C_SERVER_FREQ_HZ (50) 

void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG1_POS);

	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);
	
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PE(1);
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PS(1);
}

void Task_Motion_Sensor_B(void) {

	
	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1
	
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	read_full_xyz(&acc_X, &acc_Y, &acc_Z);

	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		

	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
	
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
	
}

void Task_Motion_Sensor(void) {
	
	
	
	uint8_t dataw[1];
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	static uint8_t rf, gf, bf;
	int16_t tempr[3];
	int i;
	// enum state for FSM
	//static enum {INIT, RACCEL, ACCEL, LED} next_state = INIT;
	static enum {INIT, RACCEL, ACCEL, LED1, LED2, LED3} next_state = INIT;
	
	if(g_I2C_Msg.Status == WRITE_COMPLETE || g_I2C_Msg.Status == READ_COMPLETE){
		g_I2C_Msg.Status = IDLE;
	}
	if(g_I2C_Msg.Status == IDLE)
	{
	switch(next_state)
	{
		case INIT:
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1

			//check for device
			// This piece of code can be removed as per Prof Dean. Hence didn't perform FSM for same.
			/*
			i2c_read_bytes(MMA_ADDR, REG_WHOAMI, data, 1); 
			if (data[0] != WHOAMI)	{
			return 0; // error code
			}
	
			Delay(1); // for accelerometer 
			*/
	
			//set active mode, 14 bit samples, 2g full scale, low noise and 800 Hz ODR 
			dataw[0] = 0x05;
		
			// update data structure
			g_I2C_Msg.Dev_adx = MMA_ADDR;
			g_I2C_Msg.Reg_adx = REG_CTRL1;
			*g_I2C_Msg.Data = *dataw;
			g_I2C_Msg.Data_count = 1;
			g_I2C_Msg.Command = WRITE;
			RTCS_Release_Task_i(1); // Release client = I2C Server
			
			next_state = RACCEL;
			
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
			break;
		case RACCEL:
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1
			SET_BIT(DEBUG_TASK_MOTION_SENSOR_READ); // debug bit PTB10 = 1
			
			g_I2C_Msg.Dev_adx = MMA_ADDR;
			g_I2C_Msg.Reg_adx = REG_XHI;
			g_I2C_Msg.Data_count = 6;
			g_I2C_Msg.Command = READ;
			next_state = ACCEL;
			RTCS_Release_Task_i(1); // Release client = I2C Server
		
		    CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR_READ); // debug bit PTB10 = 0
			break;
		case ACCEL:
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1
		
			for ( i=0; i<3; i++ ) {
			tempr[i] = (int16_t) ((g_I2C_Msg.Data[2*i]<<8) | g_I2C_Msg.Data[2*i+1]);
			}

			// Align for 14 bits
			acc_X = tempr[0]/4;
			acc_Y = tempr[1]/4;
			acc_Z = tempr[2]/4;

			rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
			gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
			bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;
			
			prev_acc_X = acc_X;
			prev_acc_Y = acc_Y;
			prev_acc_Z = acc_Z;
			
			next_state = LED1;
			g_I2C_Msg.Command = NONE;
			
			RTCS_Release_Task_i(0); // Release client = Motion Sensor
			
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
			break;

		case LED1:
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1
			//SET_BIT(DEBUG_TASK_MOTION_SENSOR_LED); // debug bit PTB11 = 1
		
		
			Control_RGB_LEDs(rf, gf, bf);
			RTCS_Set_Task_Period_i(0,2,0); // set period of the task to 2 ticks = 4 ms
			RTCS_Release_Task_i(1); // Release client = I2C Server
			next_state = LED2;
		
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
			//CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR_LED); // debug bit PTB11 = 0
			break;
		
		case LED2:
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1
			//SET_BIT(DEBUG_TASK_MOTION_SENSOR_LED); // debug bit PTB11 = 1
			Control_RGB_LEDs(0, 0, 0);
			RTCS_Set_Task_Period_i(0,4,0); // set period of the task to 4 ticks = 4 ms
			//RTCS_Release_Task_i(1); // Release client = I2C Server
			next_state = LED3;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
			//CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR_LED); // debug bit PTB11 = 0
			break;
		
		case LED3:
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 1
			RTCS_Set_Task_Period_i(0,TICKS(TASK_MOTION_SENSOR_FREQ_HZ),0);
			RTCS_Release_Task_i(0);
			next_state = RACCEL;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); // debug bit PTB2 = 0
			break;	
	}
    }
}

	/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	Control_RGB_LEDs(0, 0, 0);
	
	if(PTE->PDIR & MASK(CONFIG1_POS))
	{
		
		if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
		;
	}
	//Control_RGB_LEDs(0, 0, 0);							
	
	RTCS_Init(SCHED_FREQ_HZ);
	RTCS_Add_Task(Task_Motion_Sensor_B, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	RTCS_Run_Scheduler();
		
	}
	else
	{
	RTCS_Init(SCHED_FREQ_HZ);
	RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	RTCS_Add_Task(Task_I2C_Server, 1, TICKS(TASK_I2C_SERVER_FREQ_HZ)); // Run periodically
	RTCS_Run_Scheduler();
	}
		
							
	
	
	
}

