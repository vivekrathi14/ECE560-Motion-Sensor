#include	 <MKL25Z4.H>
#include	 "i2c.h"
#include 	"gpio_defs.h"
#include 	"RTCS.h"

int lock_detect = 0;
int i2c_lock	= 0;

// #define ENABLE_LOCK_DETECT // don't include lock detect, simplifying FSM creation

volatile I2C_MESSAGE_T g_I2C_Msg = {0, NONE, IDLE, 0,0};

//init i2c0
void i2c_init( void )
{
 //clock i2c peripheral and port E
	SIM->SCGC4		 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5		 |= SIM_SCGC5_PORTE_MASK;

	//set pins to I2C function
	PORTE->PCR[ 24 ] |= PORT_PCR_MUX( 5 );
	PORTE->PCR[ 25 ] |= PORT_PCR_MUX( 5 );

	//set baud rate
	//baud = bus freq/(scl_div+mul)
	I2C0->F				= ( I2C_F_ICR( 0x11 ) | I2C_F_MULT( 0 ) );

	//enable i2c and set to master mode
	I2C0->C1		 |= ( I2C_C1_IICEN_MASK );

	// Select high drive mode ? what is high drive??
	I2C0->C2		 |= ( I2C_C2_HDRS_MASK );
}

void i2c_busy( void )
{
 // Start Signal
	lock_detect	= 0;
	I2C0->C1	&= ~I2C_C1_IICEN_MASK;
	I2C_TRAN;
	I2C_M_START;
	I2C0->C1 |= I2C_C1_IICEN_MASK;
	// Write to clear line
	I2C0->C1 |= I2C_C1_MST_MASK;			// set MASTER mode								
	I2C0->C1 |= I2C_C1_TX_MASK;				// set transmit (TX) mode							
	I2C0->D	 = 0xFF;
	while( ( I2C0->S & I2C_S_IICIF_MASK ) == 0U ) {// await interrupt									
	}														

	I2C0->S		|= I2C_S_IICIF_MASK;		// Clear interrupt bit							
	I2C0->S		|= I2C_S_ARBL_MASK;			// Clear arbitration error flag						

																		// Send start										
	I2C0->C1	&= ~I2C_C1_IICEN_MASK;
	I2C0->C1	|= I2C_C1_TX_MASK;			// Set transmit (TX) mode							
	I2C0->C1	|= I2C_C1_MST_MASK;			// START signal generated							

	I2C0->C1	|= I2C_C1_IICEN_MASK;		// Wait until start is sent						

																		// Send stop										
	I2C0->C1	&= ~I2C_C1_IICEN_MASK;
	I2C0->C1	|= I2C_C1_MST_MASK;
	I2C0->C1	&= ~I2C_C1_MST_MASK;		// Set SLAVE mode									
	I2C0->C1	&= ~I2C_C1_TX_MASK;			// Set Rx											
	I2C0->C1	|= I2C_C1_IICEN_MASK;
																		
	I2C0->S		|= I2C_S_IICIF_MASK; 		// Clear arbitration error & interrupt flag			
	I2C0->S		|= I2C_S_ARBL_MASK;
	lock_detect	= 0;
	i2c_lock	 = 1;
}

void i2c_wait( void )
{
	SET_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 1
	
	lock_detect = 0;
	while( ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) & ( lock_detect < LOCK_DETECT_THRESHOLD )) {
		lock_detect++;
	}
/*	
#ifdef ENABLE_LOCK_DETECT
	if( lock_detect >= LOCK_DETECT_THRESHOLD )
		i2c_busy( );
#endif
	*/
	I2C0->S |= I2C_S_IICIF_MASK;
	
	CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 0
}

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
	
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	I2C_TRAN;													//	set to transmit mode	

	SET_BIT(DEBUG_MSG_ON_BUS); 								// debug bit PTB8 = 1
	
	I2C_M_START;											//	send start	
	

	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();													//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();													//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	i2c_wait();													//	wait for completion								

	I2C_REC;													//	set to receive mode								
	while (num_bytes_read < data_count) {
		is_last_read = (num_bytes_read == data_count-1)? 1: 0;
		if (is_last_read){
			NACK;													// tell HW to send NACK after read							
		} else {
			ACK;													// tell HW to send ACK after read								
		}
		
		

		dummy = I2C0->D;								//	dummy read										
		i2c_wait();												//	wait for completion								

		if (is_last_read){
			I2C_M_STOP;										//	send stop
			
			CLEAR_BIT(DEBUG_MSG_ON_BUS); 				// debug bit PTB8 = 0			
		}
		data[num_bytes_read++] = I2C0->D; //	read data										
	}
	
	
	CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
	return 1;
}

int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	SET_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 1
	
	uint8_t num_bytes_written=0;
	I2C_TRAN;													//	set to transmit mode

	SET_BIT(DEBUG_MSG_ON_BUS); // debug bit PTB8 = 1
	
	I2C_M_START;											//	send start	
	
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();													//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();													//	wait for completion								

	while (num_bytes_written < data_count) {
		I2C0->D = data[num_bytes_written++]; //	write data										
		i2c_wait();												//	wait for completion								
	}
	I2C_M_STOP;												//		send stop										
    CLEAR_BIT(DEBUG_MSG_ON_BUS); // debug bit PTB8 = 0
	
	CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
	return 1;
}


void Task_I2C_Server(void)
{
	uint8_t dummy;
	static uint8_t is_last_read=0, num_bytes_written=0;
	static uint8_t num_bytes_read=0;
	
	// enum for switch in READ
	static enum {R_DEV,R_REG,R_RST,R_REC,R_RECA,R_RECB,WAITR,RARBL} r_n,r_nn,r_c;
	//enum for switch in WRITE
	static enum {W_DEV,W_REG,W_DATA,W_STOP,WAITW,WARBL} w_n,w_nn,w_c;
	
	
	switch(g_I2C_Msg.Command)
	{
		case NONE:
			g_I2C_Msg.Status = IDLE;
			break;
		
		case READ:
			//g_I2C_Msg.Client = Task_I2C_Server;
			switch(r_n)
			{
				case R_DEV:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					g_I2C_Msg.Status = READING;
					num_bytes_read = 0;
					I2C_TRAN;													//	set to transmit mode
					I2C_M_START;											//	send start
					SET_BIT(DEBUG_MSG_ON_BUS); 								// debug bit PTB8 = 1
					I2C0->D = g_I2C_Msg.Dev_adx;								//	send dev address (write)
					if (I2C0->S & I2C_S_ARBL_MASK){
						r_n = RARBL; // next state
						r_c = R_DEV; // current state
						RTCS_Release_Task_i(1); // Release client = I2C Server
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						break;
					}
					r_n = WAITR; // next state
					r_nn = R_REG; // next next state, after WAITR
					RTCS_Release_Task_i(1); // Release client = I2C Server
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
		
				case R_REG:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					I2C0->D = g_I2C_Msg.Reg_adx;								//	send register address
					if (I2C0->S & I2C_S_ARBL_MASK){
						r_n = RARBL;
						r_c = R_REG;
						if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						break;
					}
					r_n = WAITR;
					r_nn = R_RST;
					if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
				
				case R_RST:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					I2C_M_RSTART;											//	repeated start									
					I2C0->D = g_I2C_Msg.Dev_adx | 0x01 ;				//	send dev address (read)
					if (I2C0->S & I2C_S_ARBL_MASK){
						r_n = RARBL;
						r_c = R_RST;
						if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						break;
					}
					r_n = WAITR;
					r_nn = R_REC;
					if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
				case R_REC:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					I2C_REC;													//	set to receive mode	
					r_n = R_RECA;
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
				
				case R_RECA:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					if (num_bytes_read < g_I2C_Msg.Data_count)
					{
					is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
						if (is_last_read){
							NACK;													// tell HW to send NACK after read							
						} else {
						ACK;													// tell HW to send ACK after read								
						}
						dummy = I2C0->D;								//	dummy read										
						r_n = WAITR;												//	wait for completion	
						r_nn = R_RECB;
						//break;
						if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
						}
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
					}
				
				case R_RECB:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					if (is_last_read){
						I2C_M_STOP;										//	send stop
						CLEAR_BIT(DEBUG_MSG_ON_BUS); 				// debug bit PTB8 = 0
						g_I2C_Msg.Status = READ_COMPLETE;
						RTCS_Release_Task_i(0);	// Release client = Motion Sensor
						r_n = R_DEV;
						g_I2C_Msg.Command = NONE;
						//I2C0->S |= I2C_S_IICIF_MASK;
						}
						g_I2C_Msg.Data[num_bytes_read++] = I2C0->D; //	read data	
						if(!is_last_read){
						r_n = R_RECA;
						RTCS_Release_Task_i(1);	// Release client = I2C Server
						}
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						break;
			
		
				case WAITR:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					SET_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 1
					if(( I2C0->S & I2C_S_IICIF_MASK ) == 0 )
					{
						if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 0
						break;

					}
					else
					{
						I2C0->S |= I2C_S_IICIF_MASK;
						r_n = r_nn;
						if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 0
						break;
					}
					
				case RARBL:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					i2c_busy();
					r_n = r_c;
					if(g_I2C_Msg.Status == READING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
					
				default:
					break;
				}
			 break;
			
		case WRITE:
			switch(w_n)
			{
				case W_DEV:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					g_I2C_Msg.Status = WRITING;
					I2C_TRAN;													//	set to transmit mode	
					I2C_M_START;											//	send start	
					SET_BIT(DEBUG_MSG_ON_BUS); 								// debug bit PTB8 = 1
					I2C0->D = g_I2C_Msg.Dev_adx;								//	send dev address (write)
					if (I2C0->S & I2C_S_ARBL_MASK){
						w_n = WARBL;
						w_c = W_DEV;
						RTCS_Release_Task_i(1); // Release client = I2C Server
						CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
						break;
					}
					w_n = WAITW;
					w_nn = W_REG;
					w_c = W_DEV;
					RTCS_Release_Task_i(1); // Release client = I2C Server
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
				    break;
				case W_REG:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					I2C0->D = g_I2C_Msg.Reg_adx;								//	send register address
					if (I2C0->S & I2C_S_ARBL_MASK){
						w_n = WARBL;
						w_c = W_REG;
						if(g_I2C_Msg.Status == WRITING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0	
					break;
					}
					w_n = WAITW;
					w_nn = W_DATA;
					w_c = W_REG;
					if(g_I2C_Msg.Status == WRITING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
				
				case W_DATA:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					while (num_bytes_written < 1) {
						I2C0->D = g_I2C_Msg.Data[num_bytes_written++]; //	write data										
						w_n = WAITW;												//	wait for completion		
					}
					w_nn = W_STOP;
					num_bytes_written = 0;
					if(g_I2C_Msg.Status == WRITING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
					
				case W_STOP:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					I2C_M_STOP;												//		send stop	
					CLEAR_BIT(DEBUG_MSG_ON_BUS); 				// debug bit PTB8 = 0
					g_I2C_Msg.Status = WRITE_COMPLETE;
					RTCS_Release_Task_i(0); // Release client = Motion Sensor
				    w_n = W_DEV;
					g_I2C_Msg.Command = NONE;
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					//I2C0->S |= I2C_S_IICIF_MASK;
					break;
				case WAITW:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					SET_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 1
					if(( I2C0->S & I2C_S_IICIF_MASK ) == 0 )
					{
						if(g_I2C_Msg.Status == WRITING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0	
					CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 0
					break;
						
					}
					
					else
					{

						I2C0->S |= I2C_S_IICIF_MASK;
						w_n = w_nn;
						if(g_I2C_Msg.Status == WRITING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
						
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // debug bit PTB9 = 0
					break;
					}
					
				case WARBL:
					SET_BIT(DEBUG_I2C_CODE); 								// debug bit PTB3 = 1
					i2c_busy();
					w_n = w_c;
					if(g_I2C_Msg.Status == WRITING){
						RTCS_Release_Task_i(1); // Release client = I2C Server
					}
					CLEAR_BIT(DEBUG_I2C_CODE); // debug bit PTB3 = 0
					break;
				default:
					break;
			}
			break;
	
		default:
			g_I2C_Msg.Status = IDLE;
			//RTCS_Release_Task_i(0); // Release client = Motion Sensor
			break;
		}

}