/*
 * Copyright (c) 2009-2012 Xilinx, Inc.  All rights reserved.
 *
 * Xilinx, Inc.
 * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
 * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include <xparameters.h>

void print(char *str);

void delay(unsigned int microseconds) {
	unsigned int i,j;
    for (i = 0; i < microseconds; i++) {
        // Adjust this loop based on the actual clock speed
        for (j = 0; j < 1; j++) {
            // This loop contributes to the delay
        }
    }
}


int main()
{
    init_platform();
    uint * my_cordic_ip_dev =(uint * )XPAR_MY_CORDIC_IP_0_BASEADDR;

    my_cordic_ip_dev[0]=0;  		//reset
    my_cordic_ip_dev[1]=1;  		//enable
    my_cordic_ip_dev[2]=36400;  	//input angle


    delay(1);

    if(my_cordic_ip_dev[4]==1){
     xil_printf("sin value and sign : - %f\n\r",(my_cordic_ip_dev[3]/32768));  	//sin_vlaue= sin_val*(1/(2^15))
    }
    else {
    xil_printf("sin value and sign : + %f\n\r",(my_cordic_ip_dev[3]/32768));	//sin_vlaue= sin_val*(1/(2^15))
    }

    if(my_cordic_ip_dev[6]==1){
    xil_printf("cos value and sign : - %f\n\r",(my_cordic_ip_dev[5]/32768));	//cos_vlaue= cos_val*(1/(2^15))
    }
    else {
    xil_printf("cos value and sign : + %f\n\r",(my_cordic_ip_dev[5]/32768));	//cos_vlaue= cos_val*(1/(2^15))
    }


    return 0;
}
