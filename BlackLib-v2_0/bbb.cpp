#include "BlackLib.h"
#include "BlackGPIO.h"

using namespace BlackLib;

int main(){

	BlackPWM* pwm;
	//BlackGPIO* myGpio = new BlackCoreGPIO::BlackGPIO(GPIO_30, input);
	BlackGPIO thing(GPIO_40, input, FastMode);
	return 0;
}
