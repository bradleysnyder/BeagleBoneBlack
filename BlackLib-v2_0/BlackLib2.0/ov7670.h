//#include "mbed.h"
//#include "ov7670reg.h"
 
#include "BlackLib.h"

#define OV7670_WRITE (0x42)
#define OV7670_READ  (0x43)
#define OV7670_WRITEWAIT (20)
#define OV7670_NOACK (0)
#define OV7670_REGMAX (201)
//#define OV7670_I2CFREQ (50000)
 
//
// OV7670 + FIFO AL422B camera board test
//
class OV7670 //: public Base
{
public:
    BlackI2C camera;

    //
    //InterruptIn vsync,href;
    //DigitalOut wen ;
    ////BusIn data ;
    //PortIn data;
    //DigitalOut rrst,oe,rclk ;
    //

    //BlackGPIO vsync(GPIO_49, input); //doesn't like initialization like this
    //BlackGPIO href(GPIO_117, input);

    //BlackGPIO* test = new BlackGPIO(GPIO_60, input);
    BlackGPIO vsync, href, wen, rrst, oe, rclk;

    volatile int LineCounter ;
    volatile int LastLines ;
    volatile bool CaptureReq ;
    volatile bool Busy ;
    volatile bool Done;    
       
    //BlackGPIO data[8]; //not sure size wise
    gpioName pinNames[8]; 
    int data;


    OV7670(
        //PinName sda,// Camera I2C port
        //PinName scl,// Camera I2C port
        //PinName vs, // VSYNC
        //PinName hr, // HREF
        //PinName we, // WEN

	i2cName busNum,
	unsigned int devAddr,

	//gpioName "P9_20",
	//gpioName "P9_19",
	//gpioName "P9_23", //vsync
	//gpioName "P9_25", //href
	//gpioName "P9_24", //wen
	gpioName vs,
	gpioName hr,
	gpioName we,
	

	//BlackGPIO sda,
	//BlackGPIO scl,
	//BlackGPIO vs,
	//BlackGPIO hr,
	//BlackGPIO we,
	//BlackGPIO sda(GPIO_12, input | output);
	//BlackGPIO scl(GPIO_13, input | output);
        
        
        //BlackGPIO d7, // D7
        //BlackGPIO d6, // D6
        //BlackGPIO d5, // D5
        //BlackGPIO d4, // D4
        //BlackGPIO d3, // D3
        //BlackGPIO d2, // D2
        //BlackGPIO d1, // D1
        //BlackGPIO d0, // D0


	//gpioName "P8_39", //D7
	//gpioName "P8_40",
	//gpioName "P8_41",
	//gpioName "P8_42",
	//gpioName "P8_43",
	//gpioName "P8_44",
	//gpioName "P8_45",
	//gpioName "P8_46", //D0
	gpioName d7,
	gpioName d6,
	gpioName d5,
	gpioName d4,
	gpioName d3,
	gpioName d2,
	gpioName d1,
	gpioName d0,
        
        //PortName port, // 8bit bus port
        //int mask, // 0b0000_0M65_4000_0321_L000_0000_0000_0000 = 0x07878000
        
        //PinName rt, // /RRST
        //PinName o,  // /OE
        //PinName rc  // RCLK 
	//BlackGPIO rt,
	//BlackGPIO o,
	//BlackGPIO rc   
	//gpioName "P9_28", //rrst
	//gpioName "P9_27", //oe
	//gpioName "P9_26" //rclk
	gpioName rt,
	gpioName o,
	gpioName rc,
	int dataInit	
        ) : camera(busNum, devAddr),vsync(vs, input),href(hr, input),wen(we, output),pinNames{d0, d1, d2, d3, d4, d5, d6, d7},rrst(rt,output ),oe(o, output),rclk(rc, output), data(dataInit)
        //) : camera(sda,scl),vsync(vs),href(hr),wen(we),data(port,mask),rrst(rt),oe(o),rclk(rc)
    {
        camera.close() ;
        //camera.frequency(OV7670_I2CFREQ) ;
        //vsync.fall(this,&OV7670::VsyncHandler) ;
        //href.rise(this,&OV7670::HrefHandler) ;
	//need to change those
        CaptureReq = false ;
        Busy = false ;
        Done = false ;
        LineCounter = 0 ;
	rrst.setValue(high);
        //rrst = 1 ;
	oe.setValue(high);
        //oe = 1 ;
	rclk.setValue(high);
        //rclk = 1 ;
	wen.setValue(high);
        //wen = 0 ;

	BlackGPIO data0(d0, input);
	BlackGPIO data1(d1, input);
	BlackGPIO data2(d2, input);
	BlackGPIO data3(d3, input);
	BlackGPIO data4(d4, input);
	BlackGPIO data5(d5, input);
	BlackGPIO data6(d6, input);
	BlackGPIO data7(d7, input);
	int bit0;
	int bit1;
	int bit2, bit3, bit4, bit5, bit6, bit7;
		if(data0.isHigh() )
        {
        	bit0 = 1;
        }
	else
		bit0 = 0;
	if(data1.isHigh())
		bit1 = 1;
	else
		bit1 = 0;
	if (data2.isHigh())
		bit2 = 1;
	else
		bit2 = 0;
	if (data3.isHigh())
		bit3 = 1;
	else
		bit3 = 0;
	if (data4.isHigh())
		bit4 = 1;
	else
		bit4 = 0;
	if (data5.isHigh())
		bit5 = 1;
	else
		bit5= 0;
 	if (data6.isHigh())
		bit6 = 1;
	else
		bit6 = 0;
	if (data7.isHigh())
		bit7 = 1;
	else
		bit7 = 0;

	//not sure about this
	//data = bit7 & bit6 & bit5 & bit4 & bit3 & bit2 & bit1 & bit0;
	data = bit7 << 7 | bit6 << 6 | bit5 << 5 | bit4 << 4 | bit3 << 3 | bit2 <<2 | bit1 << 1 | bit0;
	

	}	
 
    // capture request
    void CaptureNext(void)
    {
        CaptureReq = true ;
        Busy = true ;
    }
    
    // capture done? (with clear)
    bool CaptureDone(void)
    {
        bool result ;
        if (Busy) {
            result = false ;
        } else {
            result = Done ;
            Done = false ;
        }
        return result ;
    }
 
    // write to camera
    void WriteReg(int addr,int data)
    {
        // WRITE 0x42,ADDR,DATA
        camera.open(ReadWrite | NonBlock) ;
        camera.writeByte(OV7670_WRITE, data) ; //might need to change to addr instead
        //wait_us(OV7670_WRITEWAIT);
        //camera.write(addr) ;
        //wait_us(OV7670_WRITEWAIT);
        //camera.write(data) ;
	usleep(20);
        camera.close() ;
    }
 
    // read from camera
    int ReadReg(int addr)
    {
        int data ;
 
        // WRITE 0x42,ADDR
        camera.open(ReadWrite | NonBlock) ;
        //camera.write(OV7670_WRITE) ;
        //wait_us(OV7670_WRITEWAIT);
        //camera.write(addr) ;
        //camera.stop() ;
        //wait_us(OV7670_WRITEWAIT);
	usleep(20);    
 
        // WRITE 0x43,READ
        //camera.start() ;
        //camera.write(OV7670_READ) ;
        //wait_us(OV7670_WRITEWAIT);
        data = camera.readByte(addr) ;
	usleep(20);
        camera.close() ;
    
        return data ;
    }
 
    void Reset(void) {    
        WriteReg(0x12,0x80) ; // RESET CAMERA
        //wait_ms(200) ;
	usleep(200000);
    }
    
    void InitQQVGA() {
        // QQVGA RGB444
        WriteReg(REG_CLKRC,0x80);
        WriteReg(REG_COM11,0x0A) ;
        WriteReg(REG_TSLB,0x04);
        WriteReg(REG_COM7,0x04) ;
        
        //WriteReg(REG_RGB444, 0x02);
        //WriteReg(REG_COM15, 0xd0);
        WriteReg(REG_RGB444, 0x00);     // Disable RGB 444?
        WriteReg(REG_COM15, 0xD0);      // Set RGB 565?
        
        WriteReg(REG_HSTART,0x16) ;
        WriteReg(REG_HSTOP,0x04) ;
        WriteReg(REG_HREF,0x24) ;
        WriteReg(REG_VSTART,0x02) ;
        WriteReg(REG_VSTOP,0x7a) ;
        WriteReg(REG_VREF,0x0a) ;
        WriteReg(REG_COM10,0x02) ;
        WriteReg(REG_COM3, 0x04);
        WriteReg(REG_COM14, 0x1a);
        WriteReg(REG_MVFP,0x27) ;
        WriteReg(0x72, 0x22);
        WriteReg(0x73, 0xf2);
 
        // COLOR SETTING
        WriteReg(0x4f,0x80);
        WriteReg(0x50,0x80);
        WriteReg(0x51,0x00);
        WriteReg(0x52,0x22);
        WriteReg(0x53,0x5e);
        WriteReg(0x54,0x80);
        WriteReg(0x56,0x40);
        WriteReg(0x58,0x9e);
        WriteReg(0x59,0x88);
        WriteReg(0x5a,0x88);
        WriteReg(0x5b,0x44);
        WriteReg(0x5c,0x67);
        WriteReg(0x5d,0x49);
        WriteReg(0x5e,0x0e);
        WriteReg(0x69,0x00);
        WriteReg(0x6a,0x40);
        WriteReg(0x6b,0x0a);
        WriteReg(0x6c,0x0a);
        WriteReg(0x6d,0x55);
        WriteReg(0x6e,0x11);
        WriteReg(0x6f,0x9f);
 
        WriteReg(0xb0,0x84);
    }    
 
 
 
    // vsync handler
    void VsyncHandler(void)
    {
        // Capture Enable
        if (CaptureReq) {
            wen.setValue(high);
            Done = false ;
            CaptureReq = false ;
        } else {
            //wen = 0 ;
	    wen.setValue(low);
            if (Busy) {
                Busy = false ;
                Done = true ;
            }
        }
 
        // Hline Counter
        LastLines = LineCounter ;
        LineCounter = 0 ;
    }
    
    // href handler
    void HrefHandler(void)
    {
        LineCounter++ ;
    }
    
    // Data Read
    int ReadOneByte(void)
    {
        int result;
        //rclk = 1;
	rclk.setValue(high);


	//need to get data for this
        //result = data;

 
        // Shift the bits around to form the byte
        int top = result >> 19;         // Isolate the top nibble
        int middle = result >> 2;       // Isolate bits 2 & 3
        result = result & 0x00000003;   // Isolate bits 0 & 1
        
        result += middle;
        result += top;
       
        //rclk = 0;
	rclk.setValue(low);
        return result;
    }
    
    // Data Start
    void ReadStart(void)
    {        
	rrst.setValue(low);
        //rrst = 0 ;
	oe.setValue(low);
        //oe = 0 ;
        usleep(1) ;
	rclk.setValue(low);
        //rclk = 0 ;
        usleep(1) ;
	rclk.setValue(high);
        //rclk = 1 ;
        usleep(1) ;        
	rrst.setValue(high);
        //rrst = 1 ;
    }
    
    // Data Stop
    void ReadStop(void)
    {
	oe.setValue(high);
        //oe = 1 ;
        ReadOneByte() ;
	rclk.setValue(high);
        //rclk = 1 ;
    }
};
 
