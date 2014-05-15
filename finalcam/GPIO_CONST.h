//GPIO base addresses
#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000

//GPIO0
//#define BIT_OE 10
#define BIT_WE 15 //changed from 11
#define BIT_RCK 14 //changed from 9
//#define BIT_PWDN 8 //taking this out temporarily

//GPIO1
//NONE
#define BIT_VSYNC 17

//GPIO2
//#define BIT_RRST 23
//#define BIT_WRST 25
//#define BIT_STR 17
//#define BIT_RST 16
#define BIT_D00 7 //14
#define BIT_D01 6 //15
#define BIT_D02 9 //12
#define BIT_D03 8 //13
#define BIT_D04 11 //10
#define BIT_D05 10 //11
#define BIT_D06 13 //8
#define BIT_D07 12 //9
#define BIT_HREF 21 //6
//#define BIT_VSYNC 7

//GPIO3
#define BIT_OE 19
#define BIT_RRST 17


//GPIO size thing
#define GPIO_SIZE  0x00000FFF

// OE: 0 is output, 1 is input
#define GPIO_OE 0x14d
#define GPIO_IN 0x14e
#define GPIO_OUT 0x14f

// Direction constants
#define GPIO_DIR_IN 0
#define GPIO_DIR_OUT 1

