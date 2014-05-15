#include "BlackLib-v2_0/BlackLib.h"
#include "BlackLib-v2_0/BlackGPIO.h"

using namespace BlackLib;

int main(){


	//BlackPWM* pwm;
	//BlackPWM* pwm = new BlackPWM(GPIO_40);
	//BlackGPIO* gpio = new BlackGPIO(GPIO_40, input, FastMode);
	//BlackGPIO* gpio;
	//BlackGPIO* myGpioPtr;
	//myGpioPtr = new BlackGPIO(GPIO_40, input, FastMode);
	//BlackGPIO::BlackGPIO* myptr = new BlackGPIO::BlackGPIO(GPIO_40, input, FastMode);
	BlackGPIO*  myGpio2 = new BlackGPIO(GPIO_60, output, FastMode);
	return 0;
}
