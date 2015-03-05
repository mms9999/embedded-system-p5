#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "Adafruit_GFX.h"

#define XOUT6 0x00
#define YOUT6 0x01
#define ZOUT6 0x02
volatile int send_code[3] = {0,0,0};
const int ORIG_X_SPEED = 2;
const int ORIG_Y_SPEED = 1;
signed int xval;
signed int yval;
signed int zval;
volatile int count = 0;
volatile int leds;
volatile int temp ;
volatile int xpc = 64,ypc =64;
	volatile int xDiff, yDiff;
		volatile int xc, yc;


volatile int * Tx_ptr;
volatile bool Tx_done;

signed int ReadData = 0;
#define NUM_SSI_DATA            3
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
//*****************************************************************************
//
//! \addtogroup i2c_examples_list
//! <h1>I2C Master Loopback (i2c_master_slave_loopback)</h1>
//!
//! This example shows how to configure the I2C0 module for loopback mode.
//! This includes setting up the master and slave module.  Loopback mode
//! internally connects the master and slave data and clock lines together.
//! The address of the slave module is set in order to read data from the
//! master.  Then the data is checked to make sure the received data matches
//! the data that was transmitted.  This example uses a polling method for
//! sending and receiving data.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - I2C0 peripheral
//! - GPIO Port B peripheral (for I2C0 pins)
//! - I2C0SCL - PB2
//! - I2C0SDA - PB3
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of I2C.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.
//
//*****************************************************************************

//*****************************************************************************
//
// Number of I2C data packets to send.
//
//*****************************************************************************
#define NUM_I2C_DATA 3

//*****************************************************************************
//
// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
//
//*****************************************************************************
#define SLAVE_ADDRESS 0x4C

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, 16000000);
}

void ConfigureUART1(void) {
	
		// Enable the GPIO Peripheral used by the UART1 and enable UART1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    // Initialize UART1
    //
		ROM_UARTConfigSetExpClk(UART1_BASE, 16000000,9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}


int I2CReceive(int reg,int data)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);
 
    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);
 
    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
    //return data pulled from the specified register
    ReadData = I2CMasterDataGet(I2C0_BASE);
		return ReadData;
}
void SendStr( int * Tx_buf) {

		Tx_done = false;		// global flag used by ISR
		Tx_ptr = Tx_buf;		// global pointer used by ISR
		ROM_UARTIntDisable(UART1_BASE, UART_INT_TX);	// avoid critical section
		while(ROM_UARTSpaceAvail(UART1_BASE))
		{
			if (*Tx_ptr) {
	//			UARTprintf("hi 2 ");
				ROM_UARTCharPutNonBlocking(UART1_BASE, *Tx_ptr++);
			} else {
				break;
			}
			
		}
		
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}
void IntHandler(void)  
{

	//			int i = 0;
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_INT_PIN_6);
	// Clear interrupt request
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_6);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,GPIO_PIN_2);
	//ROM_SysCtlDelay(ROM_SysCtlClockGet()/3);
	  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,0);
	//ROM_SysCtlDelay(ROM_SysCtlClockGet()/3);

	  //
    // Set PE1 high to indicate entry to this interrupt handler.
    //
   // ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);

//	leds = ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
///  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ~leds);
			xval = I2CReceive(XOUT6,0);		
			yval = I2CReceive(YOUT6,0);
			if(xval >= 33 && xval <= 63)
					xval = xval - 64;
			if(yval >= 33 && yval <= 63)
					yval = yval - 64;
			
			xDiff = (xval/3);
			yDiff = -(yval/3);
		//	moveball(xDiff, yDiff);
		//	ROM_SysCtlDelay(ROM_SysCtlClockGet()/100);
			xc = xpc + xDiff;
			yc = ypc + yDiff;
	if(xc >= 10 && xc <= 110 && yc >= 10 && yc <=110){
		fillCircle(xpc, ypc, 4, BLACK);
		xpc = xc;
		ypc = yc;
		fillCircle(xpc,ypc, 4,BLUE);
	}
				
			send_code[0] = 6;  // check for conditions
				
					send_code[1] = xc;  // send the ball x axis
					
					send_code[2] = yc;
					SendStr(send_code);
				
  	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6 );

	
}
void UART1IntHandler(void) {
    uint32_t ui32Status;
		
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);

    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
	
    ROM_UARTIntClear(UART1_BASE, ui32Status);

	// send
		while(ROM_UARTSpaceAvail(UART1_BASE))
		{
			if (*Tx_ptr) {

				// We can use NonBlocking Put since we know space is available.
		//	UARTprintf("send1\n");
				ROM_UARTCharPutNonBlocking(UART1_BASE, *Tx_ptr++);
			} else {
			//	UARTprintf("send2\n");
				Tx_done = true;
		    ROM_UARTIntDisable(UART1_BASE, UART_INT_TX);
				break;
			}
		}
		UARTprintf("between\n");
	// receive
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        
				
    }
	
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);

}


int
main(void)
{
    uint32_t pui32DataTx[NUM_I2C_DATA];
    uint32_t pui32DataRx;
    uint32_t ui32Index;
		ROM_FPULazyStackingEnable();
	    ConfigureUART();
			ConfigureUART1();
    //
    // Set the clocking to run directly from the external crystal/oscillator.
    //
   ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);
		//  ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);		


    //
    // The I2C0 peripheral must be enabled before use.
    //

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

 
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
				GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6 |GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);


    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
		   ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);


    ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    ROM_SSIEnable(SSI0_BASE);
		    while(ROM_SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx))
    {
    }

		ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
		ROM_IntEnable(INT_UART1);
		IntEnable(INT_GPIOB);
		begin();
		UARTprintf("HI  ");
		fillScreen(BLACK);
		fillCircle(xpc,ypc,4,WHITE);
	
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);	
//	  ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
		GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6 );

		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	

			leds = ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	//	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ~leds);

          //       HWREG(I2C0_BASE + I2C_O_MCR) |= 0x01;

    ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
		//HWREG(NVIC_SW_TRIG) = INT_GPIOB;

    I2CSlaveEnable(I2C0_BASE);

 //  I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);


		UARTprintf("starting here\n");
		UARTprintf("hell    1\n");
//****************************************************************************

    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);
	  
		
		I2CMasterDataPut(I2C0_BASE, 0x08);          
	  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);	
    while(I2CMasterBusy(I2C0_BASE));
    I2CMasterDataPut(I2C0_BASE, 0x01);    
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	//	UARTprintf("hi  \n");
    while(I2CMasterBusy(I2C0_BASE));
		//UARTprintf("hell    7\n");	
   


		
		
	
//****************************************************************************
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

			I2CMasterDataPut(I2C0_BASE, 0x06);        
   	  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		    			
			while(I2CMasterBusy(I2C0_BASE));
			I2CMasterDataPut(I2C0_BASE, 0x10);          
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
			//UARTprintf("hell    7\n");
			while(I2CMasterBusy(I2C0_BASE)){;}
			//UARTprintf("hell    7\n");
		//		temp = I2CReceive(0x06, 0x10);
			//				 UARTprintf("\n this is temp %i \n", temp);


//****************************************************************************
		  
				I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

			I2CMasterDataPut(I2C0_BASE, 0x07);  
	    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		    			
				while(I2CMasterBusy(I2C0_BASE));
				I2CMasterDataPut(I2C0_BASE, 0x41);          
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
       while(I2CMasterBusy(I2C0_BASE));


	//	temp = I2CReceive(0x07, 0x41);
		//					 UARTprintf("\n this is temp %i \n", temp);
		UARTprintf("\n 1  2 3 \n");
						ROM_IntEnable(INT_GPIOD);

		ROM_IntMasterEnable();
		while(1)
    {
			ROM_SysCtlSleep();
					UARTprintf("x value: %i      ", xval);
					UARTprintf("y value: %i      ", yval);
					UARTprintf("z value: %i    \n", zval);
						ROM_SysCtlDelay(ROM_SysCtlClockGet()/2);


			// Clear pending interrupt (switch bounce) and re-enable interrupts			
			GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_6);
			GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_6);
    }

			 

			UARTprintf("\nhell    2\n");

			
     UARTprintf("hell   4");
		
		UARTprintf("check\n");

		


		
    //
    // Display the example setup on the console.
    //
    UARTprintf("I2C Loopback Example ->");
    UARTprintf("\n   Module = I2C0");
    UARTprintf("\n   Mode = Single Send/Receive");
    UARTprintf("\n   Rate = 100kbps\n\n");

    //
    // Initalize the data to send.
    //
    pui32DataTx[0] = 'I';
    pui32DataTx[1] = '2';
    pui32DataTx[2] = 'C';

    //
    // Initalize the receive buffer.
    //
  /*  for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        pui32DataRx[ui32Index] = 0;
    }

    //
    // Indicate the direction of the data.
    //
    UARTprintf("Tranferring from: Master -> Slave\n");

    //
    // Send 3 pieces of I2C data from the master to the slave.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        //
        // Display the data that the I2C0 master is transferring.
        //
        UARTprintf("  Sending: '%c'  . . .  ", pui32DataTx[ui32Index]);

        //
        // Place the data to be sent in the data register
        //
        ROM_I2CMasterDataPut(I2C0_BASE, pui32DataTx[ui32Index]);

        //
        // Initiate send of data from the master.  Since the loopback
        // mode is enabled, the master and slave units are connected
        // allowing us to receive the same data that we sent out.
        //
        ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        //
        // Wait until the slave has received and acknowledged the data.
        //
        while(!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ))
        {
        }

        //
        // Read the data from the slave.
        //
        pui32DataRx[ui32Index] = I2CSlaveDataGet(I2C0_BASE);

        //
        // Wait until master module is done transferring.
        //
        while(ROM_I2CMasterBusy(I2C0_BASE))
        {
        }

        //
        // Display the data that the slave has received.
        //
        UARTprintf("Received: '%c'\n", pui32DataRx[ui32Index]);
    }

    //
    // Reset receive buffer.
    //
    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        pui32DataRx[ui32Index] = 0;
    }

    //
    // Indicate the direction of the data.
    //
    UARTprintf("\n\nTranferring from: Slave -> Master\n");

    //
    // Modifiy the data direction to true, so that seeing the address will
    // indicate that the I2C Master is initiating a read from the slave.
    //
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);

    //
    // Do a dummy receive to make sure you don't get junk on the first receive.
    //
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //
    // Dummy acknowledge and wait for the receive request from the master.
    // This is done to clear any flags that should not be set.
    //
    while(!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ))
    {
    }

    for(ui32Index = 0; ui32Index < NUM_I2C_DATA; ui32Index++)
    {
        //
        // Display the data that I2C0 slave module is transferring.
        //
        UARTprintf("  Sending: '%c'  . . .  ", pui32DataTx[ui32Index]);

        //
        // Place the data to be sent in the data register
        //
        I2CSlaveDataPut(I2C0_BASE, pui32DataTx[ui32Index]);

        //
        // Tell the master to read data.
        //
        ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //
        // Wait until the slave is done sending data.
        //
        while(!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ))
        {
        }

        //
        // Read the data from the master.
        //
        pui32DataRx[ui32Index] = ROM_I2CMasterDataGet(I2C0_BASE);

        //
        // Display the data that the slave has received.
        //
        UARTprintf("Received: '%c'\n", pui32DataRx[ui32Index]);
    }

    //
    // Tell the user that the test is done.
    //
    UARTprintf("\nDone.\n\n");
		
			GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_6);
			GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_6);
    //
    // Return no errors
    //*/
    return(0);
}
