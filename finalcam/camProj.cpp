#include "camProj.h"

char buf1[1];
char bufResponse[MAX_SERIAL_BUFFER_SIZE];
int fd1, fd2, res1, rightDuty, leftDuty;

Ov7670 camera;
Image image(&camera);
ofstream file;
uint8_t camera_init_success = false;
volatile uint16_t snapCounter = 0;

volatile uint8_t vsyncCounter = 0;
volatile uint32_t runtime = 0;
volatile bool oneSecondPassed = false;
volatile bool takePictures = false;


/****************************************************************
 * Handler for verticle(y direction)
 ****************************************************************/
void vsync_handler() {
	//printf("main: handling vsync\n");
	//camera.vsync_handler();
	//this line looks like it just fiddles with write reset which we don't need
	vsyncCounter++;
	if (vsyncCounter > 15) { // with qqvga fps = 15
		vsyncCounter = 0;
		runtime++;
		oneSecondPassed = true;
	}
}

/****************************************************************
 * Handler for horizontal(x direction)
 ****************************************************************/
void href_handler() {
	camera.href_handler();
}


//trying to get a simple main file to work first


int main()
{
	
	printf("Made it to main\n");	
	camera.init();
	camera.reset(MODE_RGB444);
	camera.brightness(2);
	camera.capture_image();




	return 0;
}
