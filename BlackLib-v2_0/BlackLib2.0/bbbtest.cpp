#include "BlackLib.h"
#include "BlackErr.h"
#include "ov7670.h"


//using namespace BlackLib;

/*OV7670 camera (
	I2C_1, 0x42, //bus and address
	GPIO_49, GPIO_117, //vsync, href
	GPIO_15, //wen
	GPIO_71, GPIO_70, GPIO_73, GPIO_72, GPIO_75, GPIO_74, GPIO_77, GPIO_76, //d0 - d7
	GPIO_113, GPIO_115, GPIO_14, 0 //rrst, oe, rclk, data init value

); */


int main(){

	//BlackPWM* pwm;
	//BlackGPIO*  myGpio2 = new BlackGPIO(GPIO_60, input);
	//myGpio2.doExport();


	//int16_t i;
	//uint8_t rcvbuf[16], rcvbufpos, c;
	//BlackI2C camera(I2C_1, 0x42);
	//WDTCTL = WDTPW + WDTHOLD; doesn't compile in BBB	
	
	//unsigned int LEDGPIO = 60;   // GPIO1_28 = (1x32) + 28 = 60
	//unsigned int ButtonGPIO = 15;   // GPIO0_15 = (0x32) + 15 = 15

	/*gpio_export(LEDGPIO);    // The LED
	//gpio_export(ButtonGPIO);   // The push button switch
        //gpio_set_dir(LEDGPIO, OUTPUT_PIN);   // The LED is an output
	//gpio_set_dir(ButtonGPIO, INPUT_PIN);   // The push button input
	
	// Flash the LED 5 times
	//for(int i=0; i<5; i++){
		//cout << "Setting the LED on" << endl;
                //gpio_set_value(LEDGPIO, HIGH);*/
	//sample code above for working with functions
	
	printf("Got an I2C device declared\n");
	//camera.open(ReadWrite | NonBlock); //this is example from blacklib.h


	//export all pins to user space using some helper functions	
	gpio_export(49); //vsync
	gpio_export(117); //href
	gpio_export(15); //wen
	gpio_export(71); //d0
	gpio_export(70); //d1
	gpio_export(72); //d3
	gpio_export(73); //d2
	gpio_export(74); //d5
	gpio_export(75); //d4
	gpio_export(76); //d7
	gpio_export(77); //d6
	gpio_export(113); //rrst
	gpio_export(115); //oe
	gpio_export(14); //rclk

	//OV7670 camera(I2C_1, 0x42, GPIO_49, GPIO_117, GPIO_15, GPIO_71, GPIO_70, GPIO_73, GPIO_72, GPIO_75,
			//GPIO_74, GPIO_77, GPIO_76, GPIO_113, GPIO_115, GPIO_14, 0);

	//had to declare this structure here. Declaring before main caused a seg fault

	//now we declare the direction for the pins
	gpio_set_dir(49, INPUT_PIN);
	gpio_set_dir(117, INPUT_PIN);
	gpio_set_dir(15, OUTPUT_PIN);
	
	gpio_set_dir(71, INPUT_PIN);
	gpio_set_dir(70, INPUT_PIN);
	gpio_set_dir(72, INPUT_PIN);
	gpio_set_dir(73, INPUT_PIN);
	gpio_set_dir(74, INPUT_PIN);
	gpio_set_dir(75, INPUT_PIN);
	gpio_set_dir(76, INPUT_PIN);
	gpio_set_dir(77, INPUT_PIN);

	gpio_set_dir(113, OUTPUT_PIN);
	gpio_set_dir(115, OUTPUT_PIN);
	gpio_set_dir(14, OUTPUT_PIN);	
	

	//and now on to edge of the vsync and href pins
	gpio_set_edge(49, "falling");
	gpio_set_edge(117, "rising"); //changed from rise/fall


	//and finally we set pin mode
	//gpio_omap_mux_setup("49", "7");
	//gpio_omap_mux_setup(117, "07");
	//gpio_omap_mux_setup(15, "07");
	//gpio_omap_mux_setup(71, "07");
	//gpio_omap_mux_setup(70, "07");
	//gpio_omap_muc_setup(72, "07");
	//gpio_omap_mux_setup(73, "07");	
	//gpio_omap_mux_setup(74, "07";
	//gpio_omap_mux_setup(75, "07");
	//gpio_omap_mux_setup(76, "07");
	//gpio_omap_mux_setup(77, "07");
	//gpio_omap_mux_setup(113, "07");
	//gpio_omap_mux_setup(115, "07");
	//gpio_omap_mux_setup(14, "07");
		


	OV7670 camera(I2C_1, 0x42, GPIO_49, GPIO_117, GPIO_15, GPIO_71, GPIO_70, GPIO_73, GPIO_72, GPIO_75,
			GPIO_74, GPIO_77, GPIO_76, GPIO_113, GPIO_115, GPIO_14, 0);

	

	camera.Reset();	
	



	return 0;
}
