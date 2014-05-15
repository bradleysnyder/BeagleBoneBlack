//#include <msp430.h>
//#include <legacymsp430.h>

#include <stdlib.h>
#include <string.h>
//#include <sys/wait.h> //guess we're not using the system call

#include "uart.h"
//#include "tprintf.h"
#include "i2c.h"
#include "ov7670.h"
#include "pins.h"

void wait();
void board_init();

int main()
{
    uint16_t i;
    uint8_t rcvbuf[16], rcvbufpos, c;

    //not sure what those three addition functions actually do
    //WDTCTL = WDTPW + WDTHOLD;

    //not sure what these do
    //BCSCTL1 = CALBC1_16MHZ; // Set range
    //DCOCTL = CALDCO_16MHZ; // Set DCO step + modulation */

    board_init();

    uart_init();
    wait();

    //tprintf("Hello world!\r\n");
    //wait();
    
    //tprintf("Initializing i2c...");
    printf("Initializing i2c...");
    i2c_init();
    wait();
    //tprintf("done.\r\n");

    printf("Initializing ov7670...");
    for (i = 0; i < 10; i ++) {
        if (ov7670_init()) {
            break;
        } else if (i == 5) {
            printf("\r\nPANIC! ov7670 init keeps failing!\r\n");
            //while (1);
	    return -1;
        } else {
            printf("retrying...");
            wait();
        }
    }
    wait();
    //tprintf("done.\r\n");

    //tprintf("READY\r\n");

    rcvbufpos = 0;
    while (1) {
        if (!uart_getc(&c)) 
	    continue;
        //tprintf("Got character: [%c]\r\n", c);
        //uart_putc(c);

        if ((c >= 32) && (c <= 126)) {
            rcvbuf[rcvbufpos++] = c;
        } else if (c == 13) {
            rcvbuf[rcvbufpos++] = 0;
            rcvbufpos = 0;
            if (strcmp((char *) rcvbuf, "hello") == 0) {
                printf("Hello to you too!\r\n");
            } else if (strcmp((char *) rcvbuf, "cap") == 0) {
                ov7670_capture();
                printf("OK\r\n");
            } else if (strcmp((char *) rcvbuf, "rrst") == 0) {
                ov7670_rrst();
                printf("OK\r\n");
            } else if (strlen((char *) rcvbuf) > 5 &&
                    strncmp((char *) rcvbuf, "read ", 5) == 0) {
                for (i = 0; i < atoi((char *) (rcvbuf + 5)); i ++) {
                    uart_putc(ov7670_read());
                }
            } else if (strlen((char *) rcvbuf) > 6 &&
                    strncmp((char *) rcvbuf, "hread ", 6) == 0) {
                for (i = 0; i < atoi((char *) (rcvbuf + 6)); i ++) {
                    printf("Data: [0x%x]\r\n", ov7670_read());
                }
            }
        }
    }
}

void board_init(void)
{
    //P1DIR |= WEN | RRST | RCLK;
    //P1DIR &= ~VSYNC;

    //P1OUT |= RRST;
    //P2OUT &= ~RCLK;

    //P2DIR = 0x00;

    /* XIN (P2.6) / XOUT (P2.7) pins to I/O function */
    //P2SEL &= ~(BIT6|BIT7);

    __enable_interrupt();
}

/* vim: set sw=4 et: */
