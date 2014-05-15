/*
 ####################################################################################
 #  BlackLib Library controls Beaglebone Black's inputs and outputs.                #
 #  Copyright (C) 2013-2014 by Yigit YUCE                                           #
 ####################################################################################
 #                                                                                  #
 #  This file is part of BlackLib library.                                          #
 #                                                                                  #
 #  BlackLib library is free software: you can redistribute it and/or modify        #
 #  it under the terms of the GNU Lesser General Public License as published by     #
 #  the Free Software Foundation, either version 3 of the License, or               #
 #  (at your option) any later version.                                             #
 #                                                                                  #
 #  BlackLib library is distributed in the hope that it will be useful,             #
 #  but WITHOUT ANY WARRANTY; without even the implied warranty of                  #
 #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                   #
 #  GNU Lesser General Public License for more details.                             #
 #                                                                                  #
 #  You should have received a copy of the GNU Lesser General Public License        #
 #  along with this program.  If not, see <http://www.gnu.org/licenses/>.           #
 #                                                                                  #
 #  For any comment or suggestion please contact the creator of BlackLib Library    #
 #  at ygtyce@gmail.com                                                             #
 #                                                                                  #
 ####################################################################################
 */

#ifndef BLACKLIB_H_
#define BLACKLIB_H_

#include "BlackErr.h"
#include "BlackDef.h"

#include <cmath>           // need for round() function in BlackADC::getParsedValue()
#include <string>
#include <fstream>
#include <cstring>
#include <sstream>
#include <cstdio>
#include <dirent.h>
#include <cstdint>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <termios.h>
#include <unistd.h>
//#include <InterruptIn.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <poll.h>


//
// OV7670REG.H FILE
//

#define REG_GAIN        0x00    /* Gain lower 8 bits (rest in vref) */
#define REG_BLUE        0x01    /* blue gain */
#define REG_RED         0x02    /* red gain */
#define REG_VREF        0x03    /* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1        0x04    /* Control 1 */
#define COM1_CCIR656    0x40    /* CCIR656 enable */
#define REG_BAVE        0x05    /* U/B Average level */
#define REG_GbAVE       0x06    /* Y/Gb Average level */
#define REG_AECHH       0x07    /* AEC MS 5 bits */
#define REG_RAVE        0x08    /* V/R Average level */
#define REG_COM2        0x09    /* Control 2 */
#define COM2_SSLEEP     0x10    /* Soft sleep mode */
#define REG_PID         0x0a    /* Product ID MSB */
#define REG_VER         0x0b    /* Product ID LSB */
#define REG_COM3        0x0c    /* Control 3 */
#define COM3_SWAP       0x40    /* Byte swap */
#define COM3_SCALEEN    0x08    /* Enable scaling */
#define COM3_DCWEN      0x04    /* Enable downsamp/crop/window */
#define REG_COM4        0x0d    /* Control 4 */
#define REG_COM5        0x0e    /* All "reserved" */
#define REG_COM6        0x0f    /* Control 6 */
#define REG_AECH        0x10    /* More bits of AEC value */
#define REG_CLKRC       0x11    /* Clocl control */
#define CLK_EXT         0x40    /* Use external clock directly */
#define CLK_SCALE       0x3f    /* Mask for internal clock scale */
#define REG_COM7        0x12    /* Control 7 */
#define COM7_RESET      0x80    /* Register reset */
#define COM7_FMT_MASK   0x38
#define COM7_FMT_VGA    0x00
#define COM7_FMT_CIF    0x20    /* CIF format */
#define COM7_FMT_QVGA   0x10    /* QVGA format */
#define COM7_FMT_QCIF   0x08    /* QCIF format */
#define COM7_RGB        0x04    /* bits 0 and 2 - RGB format */
#define COM7_YUV        0x00    /* YUV */
#define COM7_BAYER      0x01    /* Bayer format */
#define COM7_PBAYER     0x05    /* "Processed bayer" */
#define REG_COM8        0x13    /* Control 8 */
#define COM8_FASTAEC    0x80    /* Enable fast AGC/AEC */
#define COM8_AECSTEP    0x40    /* Unlimited AEC step size */
#define COM8_BFILT      0x20    /* Band filter enable */
#define COM8_AGC        0x04    /* Auto gain enable */
#define COM8_AWB        0x02    /* White balance enable */
#define COM8_AEC        0x01    /* Auto exposure enable */
#define REG_COM9        0x14    /* Control 9  - gain ceiling */
#define REG_COM10       0x15    /* Control 10 */
#define COM10_HSYNC     0x40    /* HSYNC instead of HREF */
#define COM10_PCLK_HB   0x20    /* Suppress PCLK on horiz blank */
#define COM10_HREF_REV  0x08    /* Reverse HREF */
#define COM10_VS_LEAD   0x04    /* VSYNC on clock leading edge */
#define COM10_VS_NEG    0x02    /* VSYNC negative */
#define COM10_HS_NEG    0x01    /* HSYNC negative */
#define REG_HSTART      0x17    /* Horiz start high bits */
#define REG_HSTOP       0x18    /* Horiz stop high bits */
#define REG_VSTART      0x19    /* Vert start high bits */
#define REG_VSTOP       0x1a    /* Vert stop high bits */
#define REG_PSHFT       0x1b    /* Pixel delay after HREF */
#define REG_MIDH        0x1c    /* Manuf. ID high */
#define REG_MIDL        0x1d    /* Manuf. ID low */
#define REG_MVFP        0x1e    /* Mirror / vflip */
#define MVFP_MIRROR     0x20    /* Mirror image */
#define MVFP_FLIP       0x10    /* Vertical flip */
#define REG_AEW         0x24    /* AGC upper limit */
#define REG_AEB         0x25    /* AGC lower limit */
#define REG_VPT         0x26    /* AGC/AEC fast mode op region */
#define REG_HSYST       0x30    /* HSYNC rising edge delay */
#define REG_HSYEN       0x31    /* HSYNC falling edge delay */
#define REG_HREF        0x32    /* HREF pieces */
#define REG_TSLB        0x3a    /* lots of stuff */
#define TSLB_YLAST      0x04    /* UYVY or VYUY - see com13 */
#define REG_COM11       0x3b    /* Control 11 */
#define COM11_NIGHT     0x80    /* NIght mode enable */
#define COM11_NMFR      0x60    /* Two bit NM frame rate */
#define COM11_HZAUTO    0x10    /* Auto detect 50/60 Hz */
#define COM11_50HZ      0x08    /* Manual 50Hz select */
#define COM11_EXP       0x02
#define REG_COM12       0x3c    /* Control 12 */
#define COM12_HREF      0x80    /* HREF always */
#define REG_COM13       0x3d    /* Control 13 */
#define COM13_GAMMA     0x80    /* Gamma enable */
#define COM13_UVSAT     0x40    /* UV saturation auto adjustment */
#define COM13_UVSWAP    0x01    /* V before U - w/TSLB */
#define REG_COM14       0x3e    /* Control 14 */
#define COM14_DCWEN     0x10    /* DCW/PCLK-scale enable */
#define REG_EDGE        0x3f    /* Edge enhancement factor */
#define REG_COM15       0x40    /* Control 15 */
#define COM15_R10F0     0x00    /* Data range 10 to F0 */
#define COM15_R01FE     0x80    /*            01 to FE */
#define COM15_R00FF     0xc0    /*            00 to FF */
#define COM15_RGB565    0x10    /* RGB565 output */
#define COM15_RGB555    0x30    /* RGB555 output */
#define REG_COM16       0x41    /* Control 16 */
#define COM16_AWBGAIN   0x08    /* AWB gain enable */
#define REG_COM17       0x42    /* Control 17 */
#define COM17_AECWIN    0xc0    /* AEC window - must match COM4 */
#define COM17_CBAR      0x08    /* DSP Color bar */
#define REG_CMATRIX_BASE 0x4f
#define CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58
#define REG_BRIGHT      0x55    /* Brightness */
#define REG_CONTRAS     0x56    /* Contrast control */
#define REG_GFIX        0x69    /* Fix gain control */
#define REG_REG76       0x76    /* OV's name */
#define R76_BLKPCOR     0x80    /* Black pixel correction enable */
#define R76_WHTPCOR     0x40    /* White pixel correction enable */
#define REG_RGB444      0x8c    /* RGB 444 control */
#define R444_ENABLE     0x02    /* Turn on RGB444, overrides 5x5 */
#define R444_RGBX       0x01    /* Empty nibble at end */
#define REG_HAECC1      0x9f    /* Hist AEC/AGC control 1 */
#define REG_HAECC2      0xa0    /* Hist AEC/AGC control 2 */
#define REG_BD50MAX     0xa5    /* 50hz banding step limit */
#define REG_HAECC3      0xa6    /* Hist AEC/AGC control 3 */
#define REG_HAECC4      0xa7    /* Hist AEC/AGC control 4 */
#define REG_HAECC5      0xa8    /* Hist AEC/AGC control 5 */
#define REG_HAECC6      0xa9    /* Hist AEC/AGC control 6 */
#define REG_HAECC7      0xaa    /* Hist AEC/AGC control 7 */
#define REG_BD60MAX     0xab    /* 60hz banding step limit */
 

//
//defining ov signals and registers
//

#define OV7670_WRITE (0x42)
#define OV7670_READ  (0x43)
#define OV7670_WRITEWAIT (20)
#define OV7670_NOACK (0)
#define OV7670_REGMAX (201)
#define OV7670_I2CFREQ (50000            


using std::string;
//namespace BlackLib
//{

    /*! @brief Uses for converting the different types variable to string type.
    *
    * @tparam T variable which will convert.
    * @return String type value of T type variable.
    */
    template <typename T>
    inline std::string tostr(const T& t)
    {
        std::ostringstream os;
        os << t;
        return os.str();
    }





    // ########################################### BLACKCORE DECLARATION STARTS ########################################### //

    /*! @brief Base class of the other classes.
     *
     *    This class is core of the @b BlackLib Library.
     *    It includes private functions like communicating with operating system and their limited versions for
     *    using from derived class.
     */
    class BlackCore
    {
        private:
            errorCore       *coreErrors;            /*!< @brief is used to hold the errors of BlackCore class */
            std::string     capeMgrName;            /*!< @brief is used to hold the capemgr name */
            std::string     ocpName;                /*!< @brief is used to hold the ocp name */
            std::string     slotsFilePath;          /*!< @brief is used to hold the slots file path */

            /*! @brief Finds full name of capemgr directory.
            *
            *  This function searches @b "/sys/devices/" directory,
            *  to find directory which starts with @b "bone_capemgr."
            *  @return True if successful, else false.
            *  @sa BlackCore::searchDirectory()
            */
            bool            findCapeMgrName();

            /*! @brief Finds full name of ocp directory.
            *
            *  This function searches @b "/sys/devices/" directory,
            *  to find directory which starts with @b "ocp."
            *  @return True if successful, else false.
            *  @sa BlackCore::searchDirectory()
            */
            bool            findOcpName();

            /*! @brief Executes system call.
            *
            *  This function executes system commands with using popen() function.
            *  This example executes "ls" command with argument "la" and saves
            *  output to returnValue variable. @n @n
            *  <b> string returnValue = executeCommand("ls -la"); </b>
            */
            std::string     executeCommand(std::string command);

            /*! @brief Searches specified directory to find specified file/directory.
            *
            *  @param[in] searchIn searching directory
            *  @param[in] searchThis search file/directory
            *  @return Full name of searching file/directory.
            */
            std::string     searchDirectory(std::string searchIn, std::string searchThis);

            /*! @brief First declaration of this function.
            */
            virtual bool    loadDeviceTree() = 0;



        protected:

            /*!
             * This enum is used for defining ADC, PWM and SPI device driver names.
             */
            enum ocpSearch  {   ADC_helper  =   0,         /*!< enumeration for @a adc device driver name for using at BlackCore::searchDirectoryOcp() function parameter*/
                                PWM_P8_13   =   1,         /*!< enumeration for @a P8_13 pwm device driver name */
                                PWM_P8_19   =   2,         /*!< enumeration for @a P8_19 pwm device driver name */
                                PWM_P9_14   =   3,         /*!< enumeration for @a P9_14 pwm device driver name */
                                PWM_P9_16   =   4,         /*!< enumeration for @a P9_16 pwm device driver name */
                                PWM_P9_21   =   5,         /*!< enumeration for @a P9_21 pwm device driver name */
                                PWM_P9_22   =   6,         /*!< enumeration for @a P9_22 pwm device driver name */
                                PWM_P9_42   =   7,         /*!< enumeration for @a P9_42 pwm device driver name */
                                SPI0        =   8,         /*!< enumeration for @a SPI0 spi device driver name */
                                SPI1        =   9          /*!< enumeration for @a SPI1 spi device driver name */
                            };

            /*! @brief Searches ocp directory to find specified file/directory.
            *
            * This function searches ocp directory only. It can be used from derived class.
            *  @param[in] searchThis takes BlackCore::ocpSearch type variable(enum)
            *  @return Full name of searching file/directory.
            */
            std::string     searchDirectoryOcp(BlackCore::ocpSearch searchThis);

            /*! @brief Exports errorCore struct to derived class.
            *
            *  @return errorCore struct pointer.
            */
            errorCore       *getErrorsFromCore();

            /*! @brief Exports capemgr name to derived class.
            *
            *  @return BlackCore::capeMgrName variable.
            */
            std::string     getCapeMgrName();

            /*! @brief Exports ocp name to derived class.
            *
            *  @return BlackCore::ocpName variable.
            */
            std::string     getOcpName();

            /*! @brief Exports slots file path to derived class.
            *
            *  @return BlackCore::slotsFilePath variable.
            */
            std::string     getSlotsFilePath();



        public:
            /*! @brief Constructor of BlackCore class.
            *
            * This function initializes errorCore struct and runs these functions:
            * @li findCapeMgrName()
            * @li findOcpName()
            */
            BlackCore();

            /*! @brief Destructor of BlackCore class.
            *
            * This function deletes errorCore struct pointer.
           */
            virtual ~BlackCore();

    };
    // ############################################ BLACKCORE DECLARATION ENDS ############################################ //




    // ######################################### BLACKCOREADC DECLARATION STARTS ########################################## //

    /*! @brief Preparation phase of Beaglebone Black, to use ADC.
     *
     *    This class is core of the BlackADC class. It includes private functions which are doing base processes
     *    for using analog input feature.
     */
    class BlackCoreADC : virtual private BlackCore
    {
        private:
            errorCoreADC    *adcCoreErrors;         /*!< @brief is used to hold the errors of BlackCoreADC class */
            std::string     helperName;             /*!< @brief is used to hold the helper(analog input device driver) name */

            /*! @brief Loads ADC overlay to device tree.
            *
            *  This function loads @b "cape-bone-iio" overlay to device tree.
            *  This overlay performs pinmuxing and generates device driver.
            *  @return True if successful, else false.
            */
            bool            loadDeviceTree();

            /*! @brief Finds full name of helper.
            *
            *  This function searches @b "ocp.X" directory, to find directory which starts with
            *  @b "helper." by using searchDirectoryOcp() protected function at BlackCore class.
            *  @return True if successful, else false.
            *  @sa BlackCore::searchDirectoryOcp()
            */
            bool            findHelperName();



        protected:

            /*! @brief Exports helper path to derived class.
            *
            *  @return Full path of helper.
            */
            std::string     getHelperPath();

            /*! @brief Exports errorCoreADC struct pointer to derived class.
            *
            *  @return errorCoreADC struct pointer.
            */
            errorCoreADC    *getErrorsFromCoreADC();



        public:

            /*! @brief Constructor of BlackCoreADC class.
            *
            * This function initializes errorCoreADC struct and calls device tree loading and
            * helper name finding functions.
            * @sa BlackCoreADC::loadDeviceTree()
            * @sa BlackCoreADC::findHelperName()
            * @sa adcName
            */
                            BlackCoreADC();

            /*! @brief Destructor of BlackCoreADC class.
            *
            * This function deletes errorCoreADC struct pointer.
            */
            virtual         ~BlackCoreADC();


            /*! @brief First declaration of this function.
            */
            virtual std::string getValue() = 0;
    };
    // ########################################## BLACKCOREADC DECLARATION ENDS ########################################### //





    // ########################################### BLACKADC DECLARATION STARTS ############################################ //

    /*! @brief Interacts with end user, to use ADC.
     *
     *    This class is end node to use analog inputs. End users interact with
     *    analog inputs from this class. It includes public functions for reading analog
     *    values.
     *
     * @par Example
      @verbatim
      EXAMPLE PROJECT FILE TREE:
         myAdcProject
         |-> src
             |-> BlackLib
                 |-> BlackADC.cpp
                 |-> BlackADC.h
                 |-> BlackCore.cpp
                 |-> BlackCore.h
                 |-> BlackDef.h
                 |-> BlackErr.h
                 |-> BlackGPIO.cpp
                 |-> BlackGPIO.h
                 |-> BlackI2C.cpp
                 |-> BlackI2C.h
                 |-> BlackLib.h
                 |-> BlackPWM.cpp
                 |-> BlackPWM.h
                 |-> BlackSPI.cpp
                 |-> BlackSPI.h
                 |-> BlackUART.cpp
                 |-> BlackUART.h
             |-> myAdcProject.cpp
      @endverbatim
     *  @n@n If BlackLib source files are located in your project like above example project file tree, you have to
     *  include BlackADC.h or another source files with adding this line to your project file (myAdcProject.cpp at
     *  the example):
     *  @code{.cpp}
     *      #include "BlackLib/BlackADC.h"
     *  @endcode
     *  @n@n If BlackLib source files are located at same level with your project file (myAdcProject.cpp at the
     *  example), you have to include BlackADC.h or another source files with adding this line to your project file:
     *  @code{.cpp}
     *      #include "BlackADC.h"
     *  @endcode
     *  @n @n
     *  @code{.cpp}
     *  // Filename: myAdcProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackADC.h"
     *
     *  int main()
     *  {
     *      BlackLib::BlackADC  myAdc(BlackLib::AIN0);
     *      std::cout << myAdc.getValue();
     *
     *      return 0;
     *  }
     * @endcode
     * @n @n
     * You can use "using namespace BlackLib" also. You can get rid of writing "BlackLib::", with using this method.
     * @code{.cpp}
     *  // Filename: myAdcProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackADC.h"
     *  using namespace BlackLib;
     *
     *  int main()
     *  {
     *      BlackADC  myAdc(AIN0);
     *      std::cout << myAdc.getValue();
     *
     *      return 0;
     *  }
     * @endcode
     *
     */
    class BlackADC : virtual private BlackCoreADC
    {
        private:
            errorADC        *adcErrors;             /*!< @brief is used to hold the errors of BlackADC class */
            std::string     ainPath;                /*!< @brief is used to hold the AINx file path */
            adcName         ainName;                /*!< @brief is used to hold the selected adc name */


        public:

            /*!
             * This enum is used to define ADC debugging flags.
             */
            enum flags      {   cpmgrErr    = 0,    /*!< enumeration for @a errorCore::capeMgrError status */
                                ocpErr      = 1,    /*!< enumeration for @a errorCore::ocpError status */
                                helperErr   = 2,    /*!< enumeration for @a errorCoreADC::helperError status */
                                dtErr       = 3,    /*!< enumeration for @a errorCoreADC::dtError status */
                                readErr     = 4     /*!< enumeration for @a errorADC::readError status */
                            };

            /*! @brief Constructor of BlackADC class.
            *
            * This function initializes errorADC struct and sets value path for reading analog values.
            *
            * @param [in] adc    name of adc (enum),(AINx)
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC  myAdc(BlackLib::AIN0);
            *   BlackLib::BlackADC *myAdcPtr = new BlackLib::BlackADC(BlackLib::AIN1);
            *
            *   std::cout << myAdc.getValue() << std::endl;
            *   std::cout << myAdcPtr->getValue();
            * @endcode
            *
            * @sa getHelperPath()
            * @sa adcName
            */
                            BlackADC(adcName adc);

            /*! @brief Destructor of BlackADC class.
            *
            * This function deletes errorADC struct pointer.
            */
            virtual         ~BlackADC();

            /*! @brief Reads analog input DC value(mV).
            *
            *  This function reads specified file from path, where defined
            *  at BlackADC::ainPath variable. This file holds analog input voltage at milivolt level.
            *  @return @a String type analog input value. If file opening fails, it returns
            *  BlackLib::FILE_COULD_NOT_OPEN_STRING.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   std::string val = myAdc.getValue();
            *   std::cout << "Analog mV Value: " << val << " mV";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Analog mV Value: 1085 mV
            * @endcode
            */
            std::string     getValue();

            /*! @brief Reads analog input DC value(mV).
            *
            *  This function reads specified file from path, where defined at BlackADC::ainPath
            *  variable. This file holds analog input voltage at milivolt level.
            *  @return @a integer type analog input value. If file opening fails, it returns
            *  BlackLib::FILE_COULD_NOT_OPEN_INT.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   int val = myAdc.getNumericValue();
            *   std::cout << "Analog mV Value: " << val << " mV";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Analog mV Value: 1085 mV
            * @endcode
            */
            int             getNumericValue();

            /*! @brief Reads converted analog input DC value(Volt).
            *
            * This function reads specified file from path, where defined at BlackADC::ainPath
            * variable. This file holds analog input voltage at milivolt level. Then converts this
            * value to volt level, according to input parameter.
            * @param [in] dap    convertion level (enum),(dapX) @a Expansion: digit after point
            * @return @a converted analog input value as float. If file opening fails, it returns
            *  BlackLib::FILE_COULD_NOT_OPEN_FLOAT.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   float converted_dap1 = myAdc.getConvertedValue(BlackLib::dap1);
            *   std::cout << "Analog Volt Value (in a.b form): " << converted_dap1 << " Volt" << std::endl;
            *
            *   usleep(1000);       // give some time to device
            *
            *   float converted_dap3 = myAdc.getConvertedValue(BlackLib::dap3);
            *   std::cout << "Analog Volt Value (in a.bcd form): " << converted_dap3 << " Volt" << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Analog Volt Value (in a.b form): 1.1 Volt
            *   // Analog Volt Value (in a.bcd form): 1.085 Volt
            * @endcode
            *
            * @sa digitAfterPoint
            */
            float           getConvertedValue(digitAfterPoint dap);

            /*! @brief Exports analog input pin name.
            *
            *  @return BlackADC::adcName variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   std::cout << "My adc name: AIN" << static_cast<int>(myAdc.getName()) << std::endl;
            *   std::cout << "My adc name: AIN" << (int)myAdc.getName() << std::endl;
            *   std::cout << "My adc name: AIN" << myAdc.getName() << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // My adc name: AIN0
            *   // My adc name: AIN0
            *   // My adc name: AIN0
            * @endcode
            */
            adcName         getName();

            /*! @brief Is used for general debugging.
            *
            * @return True if any error occured, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   myAdc.getNumericValue();
            *
            *   if( myAdc.fail() )
            *   {
            *       std::cout << "ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "EVERYTHING IS OK" << std::endl;
            *   }
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // EVERYTHING IS OK
            * @endcode
            *
            * @sa errorADC
            */
            bool            fail();

            /*! @brief Is used for specific debugging.
            *
            * You can use this function, after call BlackADC member functions in your code. The
            * input parameter is used for finding out status of selected error.
            * @param [in] f specific error type (enum)
            * @return Value of @a selected error.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   if( myAdc.fail(BlackLib::BlackADC::helperErr) or
            *       myAdc.fail(BlackLib::BlackADC::cpmgrErr) or
            *       myAdc.fail(BlackLib::BlackADC::ocpErr) or
            *       myAdc.fail(BlackLib::BlackADC::dtErr) )
            *   {
            *       std::cout << "BlackADC INITIALIZATION FAILED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "BlackADC INITIALIZATION IS OK" << std::endl;
            *   }
            *
            *
            *   myAdc.getValue();
            *
            *   if( myAdc.fail(BlackLib::BlackADC::readErr) )
            *   {
            *       std::cout << "READING ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "READING IS OK" << std::endl;
            *   }
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // BlackADC INITIALIZATION IS OK
            *   // READING IS OK
            * @endcode
            *
            * @sa errorADC
            */
            bool            fail(BlackADC::flags f);

            /*! @brief Reads analog input DC value(mV) with ">>" operator.
            *
            *  This function reads specified file from path, where defined at BlackADC::ainPath
            *  variable, with ">>" operator. This file holds analog input voltage at milivolt level.
            *  @param [in] &readToThis read value and save this value to this variable. If file opening
            *  fails, this functions sets BlackLib::FILE_COULD_NOT_OPEN_STRING to variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   std::string val;
            *   myAdc >> val;
            *   std::cout << "Analog mV Value: " << val << " mV";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Analog mV Value: 1085 mV
            * @endcode
            */
            BlackADC&       operator>>(std::string &readToThis);

            /*! @brief Reads analog input DC value(mV) with ">>" operator.
            *
            *  This function reads specified file from path, where defined at BlackADC::ainPath
            *  variable, with ">>" operator. This file holds analog input voltage at milivolt level.
            *  @param [in] &readToThis read value and save this value to this variable. If file opening
            *  fails, this functions sets BlackLib::FILE_COULD_NOT_OPEN_INT to variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   int val;
            *   myAdc >> val;
            *   std::cout << "Analog mV Value: " << val << " mV";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Analog mV Value: 1085 mV
            * @endcode
            */
            BlackADC&       operator>>(int &readToThis);

            /*! @brief Reads converted analog input DC value(volt) with ">>" operator.
            *
            *  This function reads specified file from path, where defined at BlackADC::ainPath
            *  variable, with ">>" operator. This file holds analog input voltage at milivolt level.
            *  Then converts this value to volt level at BlackLib::dap3 mode.
            *  @param [in] &readToThis read value and save this value to this variable. If file opening
            *  fails, this functions sets BlackLib::FILE_COULD_NOT_OPEN_FLOAT to variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackADC myAdc(BlackLib::AIN0);
            *
            *   float val;
            *   myAdc >> val;
            *   std::cout << "Analog Volt Value (in a.bcd form): " << val << " Volt" << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Analog Volt Value (in a.bcd form): 1.085 Volt
            * @endcode
            *
            *  @sa getConvertedValue()
            */
            BlackADC&       operator>>(float &readToThis);
    };
    // ############################################ BLACKADC DECLARATION ENDS ############################################# //



// ######################################### BLACKCOREGPIO DECLARATION STARTS ######################################### //

    /*! @brief Preparation phase of Beaglebone Black, to use GPIO.
     *
     *    This class is core of the BlackGPIO class. It includes private functions which are doing base processes
     *    for using gpio pins, and protected functions which are using for exporting private variables to
     *    derived class(es).
     */
    class BlackCoreGPIO : virtual private BlackCore
    {
        private:
            errorCoreGPIO   *gpioCoreError;         /*!< @brief is used to hold the errors of BlackCoreGPIO class */
            int             pinNumericName;         /*!< @brief is used to hold the selected pin number */
            int             pinNumericType;         /*!< @brief is used to hold the selected pin direction */
            std::string     expPath;                /*!< @brief is used to hold the @a export file path */
            std::string     directionPath;          /*!< @brief is used to hold the @a direction file path */
            std::string     unExpPath;              /*!< @brief is used to hold the @a unexport file path */


            /*! @brief Device tree loading is not necessary for using GPIO feature.
            *
            *  But this function have to declare in this class again. Because it is a pure virtual
            *  function. It does nothing.
            *  @return false all time.
            */
            bool            loadDeviceTree();

            /*! @brief Exports pin.
            *
            *  This function exports pin. This step is necessary.
            *  @return True if exporting is successful, else false.
            */
            bool            doExport();

            /*! @brief Sets pin direction.
            *
            *  This function sets pin direction to input or output. Pin directions are input at default.
            *  @return True if setting direction is successful, else false.
            */
            bool            setDirection();

            /*! @brief Unexports pin to release it.
            *
            *  This function is reverse of pin exporting. But this step is not necessary.
            *  @return True if unexporting is successful, else false.
            */
            bool            doUnexport();

        protected:

            /*! @brief Exports direction file path to derived class.
            *
            *  @return BlackCoreGPIO::directionPath variable.
            */
            std::string     getDirectionFilePath();

            /*! @brief Exports value file path to derived class.
            *
            *  @return BlackCoreGPIO::valuePath variable.
            */
            std::string     getValueFilePath();

            /*! @brief Exports errorCoreGPIO struct to derived class.
            *
            *  @return errorCoreGPIO struct pointer.
            */
            errorCoreGPIO   *getErrorsFromCoreGPIO();

        public:

            /*! @brief Constructor of BlackCoreGPIO class.
            *
            * This function initializes errorCoreGPIO struct, sets file path variables
            * and calls exporting and setting direction functions.
            *
            * @sa BlackCoreGPIO::doExport()
            * @sa BlackCoreGPIO::setDirection()
            * @sa gpioName
            * @sa direction
            */
                            BlackCoreGPIO(gpioName pin, direction dir);


            /*! @brief Destructor of BlackCoreGPIO class.
            *
            * This function unexports pin and deletes errorCoreGPIO struct pointer.
            */
            virtual         ~BlackCoreGPIO();

            /*! @brief First declaration of this function.
            */
            virtual std::string getValue() = 0;

    };
    // ########################################## BLACKCOREGPIO DECLARATION ENDS ########################################## //




    // ########################################### BLACKGPIO DECLARATION STARTS ########################################### //

    /*! @brief Interacts with end user, to use GPIO.
     *
     *    This class is end node to use GPIO. End users interact with gpio pins from this class.
     *    It includes public functions to set and get properties of GPIO.
     * @par Example
      @verbatim
      EXAMPLE PROJECT FILE TREE:
         myGpioProject
         |-> src
             |-> BlackLib
                 |-> BlackADC.cpp
                 |-> BlackADC.h
                 |-> BlackCore.cpp
                 |-> BlackCore.h
                 |-> BlackDef.h
                 |-> BlackErr.h
                 |-> BlackGPIO.cpp
                 |-> BlackGPIO.h
                 |-> BlackI2C.cpp
                 |-> BlackI2C.h
                 |-> BlackLib.h
                 |-> BlackPWM.cpp
                 |-> BlackPWM.h
                 |-> BlackSPI.cpp
                 |-> BlackSPI.h
                 |-> BlackUART.cpp
                 |-> BlackUART.h
             |-> myGpioProject.cpp
      @endverbatim
     *  @n@n If BlackLib source files are located in your project like above example project file tree, you have to
     *  include BlackGPIO.h or another source files with adding this line to your project file (myGpioProject.cpp at
     *  the example):
     *  @code{.cpp}
     *      #include "BlackLib/BlackGPIO.h"
     *  @endcode
     *  @n@n If BlackLib source files are located at same level with your project file (myGpioProject.cpp at the
     *  example), you have to include BlackGPIO.h or another source files with adding this line to your project file:
     *  @code{.cpp}
     *      #include "BlackGPIO.h"
     *  @endcode
     *  @n @n
     *  @code{.cpp}
     *  // Filename: myGpioProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackGPIO.h"
     *
     *  int main()
     *  {
     *      BlackLib::BlackGPIO  myGpio(BlackLib::GPIO_30, BlackLib::input);
     *      std::cout << myGpio.getValue();
     *
     *      return 0;
     *  }
     * @endcode
     * @n @n
     * You can use "using namespace BlackLib" also. You can get rid of writing "BlackLib::", with using this method.
     * @code{.cpp}
     *  // Filename: myGpioProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackGPIO.h"
     *  using namespace BlackLib;
     *
     *  int main()
     *  {
     *      BlackGPIO  myGpio(GPIO_30, input);
     *      std::cout << myGpio.getValue();
     *
     *      return 0;
     *  }
     * @endcode
     *
     */
    class BlackGPIO : virtual private BlackCoreGPIO
    {
        private:
            errorGPIO       *gpioErrors;                    /*!< @brief is used to hold the errors of BlackGPIO class */
            gpioName        pinName;                        /*!< @brief is used to hold the selected GPIO pin name */
            direction       pinDirection;                   /*!< @brief is used to hold the selected GPIO pin direction */
            workingMode     workMode;                       /*!< @brief is used to hold the selected working mode */
            std::string     valuePath;                      /*!< @brief is used to hold the value file path */

            /*! @brief Checks the export state of GPIO pin.
            *
            * This function reads specified file from path, where defined at BlackGPIO::valuePath variable.
            * If this file could open successfully, this means pin is exported successfully.
            * @return False if file opening fails, else true.
            */
            bool            isExported();

            /*! @brief Checks direction of GPIO pin.
            *
            * This function reads specified file from path, where defined at BlackGPIO::directionPath variable.
            * @return True if direction file can open successfully and its value is equal to defined direction when
            * class initialization, else false.
            */
            bool            isDirectionSet();

            /*! @brief Checks ready state of GPIO pin.
            *
            * This function calls isExported() and isDirectionSet() functions and then evaluates return
            * values of these functions.
            * @return True if both functions return true, else false.
            * @sa isExported()
            * @sa isDirectionSet()
            */
            bool            isReady();


        public:

            /*!
            * This enum is used to define GPIO debugging flags.
            */
            enum flags      {   exportFileErr       = 0,    /*!< enumeration for @a errorCoreGPIO::exportFileError status */
                                exportErr           = 1,    /*!< enumeration for @a errorGPIO::exportError status */
                                directionFileErr    = 2,    /*!< enumeration for @a errorCoreGPIO::directionFileError status */
                                directionErr        = 3,    /*!< enumeration for @a errorGPIO::directionError status */
                                readErr             = 4,    /*!< enumeration for @a errorGPIO::readError status */
                                writeErr            = 5,    /*!< enumeration for @a errorGPIO::writeError status */
                                forcingErr          = 6,    /*!< enumeration for @a errorGPIO::forcingError status */
                            };

            /*! @brief Constructor of BlackGPIO class.
            *
            * This function initializes BlackCoreGPIO class with entered parameters and errorGPIO struct.
            * Then it sets value file path variable.
            * @param [in] pn        gpio pin name(enum)
            * @param [in] pd        gpio pin direction(enum)
            * @param [in] wm        working mode(enum), default value is SecureMode
            *
            * @par Example
            *  @code{.cpp}
            *   // Pin:30 - Direction:Out - Working Mode:SecureMode
            *   BlackLib::BlackGPIO  myGpio(BlackLib::GPIO_30, BlackLib::output);
            *
            *   // Pin:60 - Direction:Out - Working Mode:FastMode
            *   BlackLib::BlackGPIO  myGpio2(BlackLib::GPIO_60, BlackLib::output, BlackLib::FastMode);
            *
            *   // Pin:40 - Direction:In - Working Mode:SecureMode
            *   BlackLib::BlackGPIO *myGpioPtr = new BlackLib::BlackGPIO(BlackLib::GPIO_40, BlackLib::input);
            *
            *   myGpio.getValue();
            *   myGpio2.getValue();
            *   myGpioPtr->getValue();
            *
            * @endcode
            *
            * @sa gpioName
            * @sa direction
            * @sa workingMode
            */
                            BlackGPIO(gpioName pn, direction pd, workingMode wm = SecureMode);

            /*! @brief Destructor of BlackGPIO class.
            *
            * This function deletes errorGPIO struct pointer.
            */
            virtual         ~BlackGPIO();

            /*! @brief Reads value of gpio pin as string type.
            *
            * If working mode is selected SecureMode, this function checks pin ready state by calling isReady() function.
            * If pin is not ready, function returns with BlackLib::GPIO_PIN_NOT_READY_STRING value. If working mode is
            * selected FastMode, ready state checking will skip. Then it reads specified file from path, where defined at
            * BlackGPIO::valuePath variable. This file holds gpio pin value.
            * @return @a string type GPIO pin value. If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_STRING
            * or if pin isn't ready, it returns BlackLib::GPIO_PIN_NOT_READY_STRING.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   std::string val = myGpio.getValue();
            *   std::cout << "Gpio30 Value: " << val;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Gpio30 Value: 1
            * @endcode
            */
            std::string     getValue();

            /*! @brief Reads value of gpio pin as int type.
            *
            * If working mode is selected SecureMode, this function checks pin ready state by calling isReady() function.
            * If pin is not ready, function returns with BlackLib::GPIO_PIN_NOT_READY_INT value. If working mode is
            * selected FastMode, ready state checking will skip. Then it reads specified file from path, where defined at
            * BlackGPIO::valuePath variable. This file holds gpio pin value.
            * @return @a int type GPIO pin value. If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_INT
            * or if pin isn't ready, it returns BlackLib::GPIO_PIN_NOT_READY_INT.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   int val = myGpio.getNumericValue();
            *   std::cout << "Gpio30 Value: " << val;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Gpio30 Value: 1
            * @endcode
            */
            int             getNumericValue();

            /*! @brief Exports name of gpio pin.
            *
            *  @return BlackGPIO::pinName variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   std::cout << "My gpio name: GPIO_" << static_cast<int>(myGpio.getName()) << std::endl;
            *   std::cout << "My gpio name: GPIO_" << (int)myGpio.getName() << std::endl;
            *   std::cout << "My gpio name: GPIO_" << myGpio.getName() << std::endl;
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // My gpio name: GPIO_30
            *   // My gpio name: GPIO_30
            *   // My gpio name: GPIO_30
            *  @endcode
            */
            gpioName        getName();

            /*! @brief Exports direction of gpio pin.
            *
            *  @return BlackGPIO::pinDirection variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   std::cout << "Values: input=1, output=2, bothDirection=3" << std::endl << std::endl;
            *
            *   std::cout << "My gpio direction: " << static_cast<int>(myGpio.getDirection()) << std::endl;
            *   std::cout << "My gpio direction: " << (int)myGpio.getDirection() << std::endl;
            *   std::cout << "My gpio direction: " << myGpio.getDirection() << std::endl;
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Values: input=1, output=2, bothDirection=3
            *   //
            *   // My gpio direction: 2
            *   // My gpio direction: 2
            *   // My gpio direction: 2
            *  @endcode
            */
            direction       getDirection();

            /*! @brief Sets value of GPIO pin.
            *
            * If pin direction is not output, function returns with false value. If working mode is selected SecureMode,
            * this function checks pin ready state by calling isReady() function. If pin is not ready, function returns
            * with false value. If working mode is selected FastMode, ready state checking will skip. Then the input
            * parameter is converted to 1 or 0 and this value is saved to value file.
            * @param [in] v new pin value(enum)
            * @return True if setting new value is successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *   BlackLib::BlackGPIO myGpio2(BlackLib::GPIO_60, BlackLib::input, BlackLib::SecureMode);
            *
            *   if( myGpio.setValue(BlackLib::high) )
            *   {
            *       std::cout << "Gpio 30 set high successfully." << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "Gpio 30 couldn't set." << std::endl;
            *   }
            *
            *   if( myGpio2.setValue(BlackLib::low) )
            *   {
            *       std::cout << "Gpio 60 set low successfully (This is not possible)." << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << std::endl << "Gpio 60 couldn't set. Because this pin's direction is input."
            *                 << std::endl << "You can't write anything to input type pin." << std::endl;
            *   }
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Gpio 30 set high successfully.
            *   //
            *   // Gpio 60 couldn't set. Because this pin's direction is input.
            *   // You can't write anything to input type pin.
            *  @endcode
            *
            * @sa digitalValue
            */
            bool            setValue(digitalValue v);

            /*! @brief Checks value of GPIO pin.
            *
            * This function calls getNumericValue() function and evaluates return value.
            * @return True if return value equals 1, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio.setValue(BlackLib::high);
            *   std::cout << std::boolalpha << myGpio.isHigh() << std::endl;
            *
            *   if( myGpio.isHigh() )
            *   {
            *       std::cout << "Do something with me, i am high." << std::endl;
            *   }
            *
            *   myGpio.setValue(BlackLib::low);
            *   std::cout << ( myGpio.isHigh() ? "yeap" : "nope") << std::endl;
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // true
            *   // Do something with me, i am high.
            *   // nope
            *  @endcode
            *
            * @sa getNumericValue()
            */
            bool            isHigh();

            /*! @brief Toggles value of GPIO pin.
            *
            * If pin direction is output, this function sets pin value to 1 or 0, by value of current state.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio.setValue(BlackLib::high);
            *   std::cout << "Before toggle: " << std::boolalpha << myGpio.isHigh() << std::endl;
            *
            *   myGpio.toggleValue();
            *   std::cout << "After toggle: " << std::boolalpha << myGpio.isHigh() << std::endl;
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Before toggle: true
            *   // After toggle: false
            *  @endcode
            */
            void            toggleValue();

            /*! @brief Changes working mode.
            *
            * This function sets new working mode value to BlackGPIO::workingMode variable.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio.setWorkingMode(BlackLib::FastMode);
            *   std::cout << "Now you can be more faster than the previous one, but MAYBE." << std::endl;
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Now you can be more faster than the previous one, but MAYBE.
            *  @endcode
            */
            void            setWorkingMode(workingMode newWM);

            /*! @brief Exports working mode value.
            *
            *  @return BlackLib::workingMode variable.
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   std::cout << "Your current working mode is "
            *             << ( (myGpio.getWorkingMode() == BlackLib::SecureMode) ? "Secure Mode" : "Fast Mode" ) << std::endl;
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Your current working mode is Secure Mode
            *  @endcode
            */
            workingMode     getWorkingMode();


            /*! @brief Is used for general debugging.
            *
            * @return True if any error occured, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio.getNumericValue();
            *
            *   if( myGpio.fail() )
            *   {
            *       std::cout << "ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "EVERYTHING IS OK" << std::endl;
            *   }
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // EVERYTHING IS OK
            * @endcode
            *
            * @sa errorGPIO
            */
            bool            fail();

            /*! @brief Is used for specific debugging.
            *
            * You can use this function, after call BlackGPIO member functions in your code. The
            * input parameter is used for finding out status of selected error.
            * @param [in] f specific error type (enum)
            * @return Value of @a selected error.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   if( myGpio.fail(BlackLib::BlackGPIO::directionFileErr) or
            *       myGpio.fail(BlackLib::BlackGPIO::exportFileErr) )
            *   {
            *        std::cout << "GPIO30: BlackGPIO INITIALIZATION FAILED" << std::endl;
            *   }
            *
            *   myGpio.setValue(BlackLib::high);
            *   if( myGpio.fail(BlackLib::BlackGPIO::directionErr) or
            *       myGpio.fail(BlackLib::BlackGPIO::exportErr) )
            *   {
            *        std::cout << "GPIO30: You are in secure mode and isReady() function failed." << std::endl;
            *   }
            *
            *   if( myGpio.fail(BlackLib::BlackGPIO::forcingErr) )
            *   {
            *        std::cout << "GPIO30: You are tried to set some value to input type pin."
            *                  << " This operation is not valid." << std::endl;
            *   }
            *
            *   if( myGpio.fail(BlackLib::BlackGPIO::writeErr) )
            *   {
            *        std::cout << "GPIO30: You could not write anything." << std::endl;
            *   }
            *
            *
            *
            *   BlackLib::BlackGPIO myGpio2(BlackLib::GPIO_60, BlackLib::input, BlackLib::FastMode);
            *
            *   if( myGpio2.fail(BlackLib::BlackGPIO::directionFileErr) or
            *       myGpio2.fail(BlackLib::BlackGPIO::exportFileErr) )
            *   {
            *        std::cout << "GPIO60: BlackGPIO INITIALIZATION FAILED" << std::endl;
            *   }
            *
            *   myGpio2.setValue(BlackLib::high);
            *   if( myGpio2.fail(BlackLib::BlackGPIO::directionErr) or
            *       myGpio2.fail(BlackLib::BlackGPIO::exportErr) )
            *   {
            *        std::cout << "GPIO60: This is not meaningful. Because you are in fast mode and these flags" << std::endl
            *                  << "could not change in fast mode." << std::endl;
            *   }
            *
            *   if( myGpio2.fail(BlackLib::BlackGPIO::forcingErr) )
            *   {
            *       std::cout << "GPIO60: You are tried to set some value to input type pin."
            *                 << " This operation is not valid." << std::endl;
            *   }
            *
            *   if( myGpio2.fail(BlackLib::BlackGPIO::writeErr) )
            *   {
            *        std::cout << "GPIO60: You could not write anything." << std::endl;
            *   }
            *
            *   myGpio2.getValue();
            *   if( ! myGpio2.fail(BlackLib::BlackGPIO::readErr) )
            *   {
            *        std::cout << "GPIO60: You are read some value from GPIO60 successfully." << std::endl;
            *   }
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // GPIO60: You are tried to set some value to input type pin. This operation is not valid.
            *   // GPIO60: You could not write anything.
            *   // GPIO60: You are read some value from GPIO60 successfully.
            *  @endcode
            *
            * @sa errorGPIO
            */
            bool            fail(BlackGPIO::flags f);

            /*! @brief Reads value of gpio pin as string type with ">>" operator.
            *
            * If working mode is selected SecureMode, this function checks pin ready state by calling isReady() function.
            * If working mode is selected FastMode, ready state checking will skip. Then it reads specified file from path,
            * where defined at BlackGPIO::valuePath variable, with ">>" operator. This file holds gpio pin value.
            * @param [in] &readToThis read value and save this value to this variable. If file opening fails, this
            * functions sets BlackLib::FILE_COULD_NOT_OPEN_STRING to variable or if pin isn't ready, it sets
            * BlackLib::GPIO_PIN_NOT_READY_STRING.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio << BlackLib::high;
            *
            *   std::string val;
            *   myGpio >> val;
            *
            *   std::cout << "Current value: " << val << std::endl;
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Current value: 1
            *  @endcode
            */
            BlackGPIO&      operator>>(std::string &readToThis);

            /*! @brief Reads value of gpio pin as int type with ">>" operator.
            *
            * If working mode is selected SecureMode, this function checks pin ready state by calling isReady() function.
            * If working mode is selected FastMode, ready state checking will skip. Then it reads specified file from path,
            * where defined at BlackGPIO::valuePath variable, with ">>" operator. This file holds gpio pin value.
            * @param [in] &readToThis read value and save this value to this variable. If file opening fails, this
            * functions sets BlackLib::FILE_COULD_NOT_OPEN_INT to variable or if pin isn't ready, it sets
            * BlackLib::GPIO_PIN_NOT_READY_INT.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio << BlackLib::high;
            *
            *   int val;
            *   myGpio >> val;
            *
            *   std::cout << "Current value: " << val << std::endl;
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Current value: 1
            *  @endcode
            */
            BlackGPIO&      operator>>(int &readToThis);

            /*! @brief Sets value of GPIO pin with "<<" operator.
            *
            *  If pin direction is not output, function does nothing. If working mode is selected SecureMode, this function
            *  checks pin ready state by calling isReady() function. If pin is not ready, function does nothing. If working
            *  mode is selected FastMode, ready state checking will skip. Then the input parameter is saved to value file.
            *  @param [in] &value new pin value(enum)
            *
            *  @par Example
            *  @code{.cpp}
            *   BlackLib::BlackGPIO myGpio(BlackLib::GPIO_30, BlackLib::output, BlackLib::SecureMode);
            *
            *   myGpio << BlackLib::high ;
            *   std::cout << std::boolalpha << myGpio.isHigh() << std::endl;
            *
            *   myGpio << BlackLib::low ;
            *   std::cout << std::boolalpha << myGpio.isHigh() << std::endl;
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // true
            *   // false
            *  @endcode
            *
            *  @sa digitalValue
            */
            BlackGPIO&      operator<<(digitalValue value);
    };
    // ############################################ BLACKGPIO DECLARATION ENDS ############################################ //



// ########################################### BLACKI2C DECLARATION STARTS ############################################ //

    /*! @brief Interacts with end user, to use I2C.
     *
     *    This class is end node to use i2c. End users interact with i2c from this class.
     *    It includes public functions to read and write byte, word or block datas.
     *
     *    @warning BlackI2C uses SMBUS. Device and register addresses vary by slave device.
     *
     *    @par Example
      @verbatim
      EXAMPLE PROJECT FILE TREE:
         myI2cProject
         |-> src
             |-> BlackLib
                 |-> BlackADC.cpp
                 |-> BlackADC.h
                 |-> BlackCore.cpp
                 |-> BlackCore.h
                 |-> BlackDef.h
                 |-> BlackErr.h
                 |-> BlackGPIO.cpp
                 |-> BlackGPIO.h
                 |-> BlackI2C.cpp
                 |-> BlackI2C.h
                 |-> BlackLib.h
                 |-> BlackPWM.cpp
                 |-> BlackPWM.h
                 |-> BlackSPI.cpp
                 |-> BlackSPI.h
                 |-> BlackUART.cpp
                 |-> BlackUART.h
             |-> myI2cProject.cpp
      @endverbatim
     *  @n@n If BlackLib source files are located in your project like above example project file tree, you have to
     *  include BlackI2C.h or another source files with adding this line to your project file (myI2cProject.cpp at
     *  the example):
     *  @code{.cpp}
     *      #include "BlackLib/BlackI2C.h"
     *  @endcode
     *  @n@n If BlackLib source files are located at same level with your project file (myI2cProject.cpp at the
     *  example), you have to include BlackI2C.h or another source files with adding this line to your project file:
     *  @code{.cpp}
     *      #include "BlackI2C.h"
     *  @endcode
     *  @n @n
     *  @code{.cpp}
     *  // Filename: myI2cProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackI2C.h"
     *
     *  int main()
     *  {
     *      BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
     *
     *      myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock);
     *
     *      uint8_t who_am_i = myI2c.readByte(0x28);
     *      std::cout << "0x28: " << std::hex << (int)who_am_i << std::endl;
     *
     *      return 0;
     *  }
     * @endcode
     * @n @n
     * You can use "using namespace BlackLib" also. You can get rid of writing "BlackLib::", with using this method.
     * @code{.cpp}
     *  // Filename: myI2cProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackI2C.h"
     *  using namespace BlackLib;
     *
     *  int main()
     *  {
     *      BlackI2C  myI2c(I2C_1, 0x53);
     *
     *      myI2c.open( ReadWrite|NonBlock );
     *
     *      uint8_t who_am_i = myI2c.readByte(0x28);
     *      std::cout << "0x28: " << std::hex << (int)who_am_i << std::endl;
     *
     *      return 0;
     *  }
     * @endcode
     */
    class BlackI2C : virtual private BlackCore
    {
        private:
            errorI2C        *i2cErrors;                 /*!< @brief is used to hold the errors of BlackI2C class */

            unsigned int    i2cDevAddress;              /*!< @brief is used to hold the i2c's device address */
            int             i2cFD;                      /*!< @brief is used to hold the i2c's tty file's file descriptor */
            std::string     i2cPortPath;                /*!< @brief is used to hold the i2c's tty port path */
            bool            isOpenFlag;                 /*!< @brief is used to hold the i2c's tty file's state */



            /*! @brief Device tree loading is not necessary for using I2C feature.
            *
            *  But this function have to declare in this class again. Because it is a pure virtual
            *  function. It does nothing.
            *  @return false all time.
            */
            bool        loadDeviceTree();

            /*! @brief Does SMBUS kernel requests.
            *
            * This function generates <i><b>I2C SMBUS IOCTL DATA PACKAGE</b></i> with read/write mode selection, register
            * address, transaction type and <i><b>I2C SMBUS DATA</b></i>. Then this function does ioctl kernel request with
            * using this smbus package.
            *
            * @param [in] rwMode            write or read mode selection (enum)
            * @param [in] registerAddr      register address which will access
            * @param [in] smbusTransaction  transaction type (enum)
            * @param [in] data              data
            * @return If kernel request is finished successfully, this function returns true, else false.
            *
            */
            inline bool useSmbusIOCTL(direction rwMode, uint8_t registerAddr, transactionType smbusTransaction, i2c_smbus_data &data);

            /*! @brief Sets slave to device.
            *
            * This function does ioctl kernel request with "I2C_SLAVE" command.
            *
            * @return If kernel request is finished successfully, this function returns true, else false.
            */
            inline bool setSlave();



        public:
            /*!
            * This enum is used to define I2C debugging flags.
            */
            enum flags      {   openErr     = 0,    /*!< enumeration for @a errorI2C::openError status */
                                closeErr    = 1,    /*!< enumeration for @a errorI2C::closeError status */
                                setSlaveErr = 2,    /*!< enumeration for @a errorI2C::setSlaveError status */
                                readErr     = 3,    /*!< enumeration for @a errorI2C::readError status */
                                writeErr    = 4     /*!< enumeration for @a errorI2C::writeError status */
                            };

            /*! @brief Constructor of BlackI2C class.
            *
            * This function initializes errorI2C struct and sets local variables.
            *
            * @param [in] i2c               name of i2c (enum),(I2C_x)
            * @param [in] i2cDeviceAddress  address of device
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *   BlackLib::BlackI2C  *myI2cPtr(BlackLib::I2C_1, 0x69);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock);
            *   myI2cPtr->open( BlackLib::ReadWrite | BlackLib::NonBlock);
            *
            * @endcode
            *
            * @sa i2cName
            */
            BlackI2C(i2cName i2c, unsigned int i2cDeviceAddress);

            /*! @brief Destructor of BlackI2C class.
            *
            * This function closes TTY file and deletes errorI2C struct pointer.
            */
            virtual ~BlackI2C();


            /*! @brief Opens TTY file of i2c.
            *
            * This function opens i2c's TTY file with selected open mode. Users can send "or"ed
            * BlackLib::openMode enums as parameter to this function.
            * @warning After initialization of BlackI2C class, this function must call. Otherwise users
            * could not use any of data transfering functions.
            *
            * @param [in] openMode          file opening mode
            * @return True if tty file opening successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            * @endcode
            *
            * @sa openMode
            */
            bool        open(uint openMode);

            /*! @brief Closes TTY file of i2c.
            *
            * This function closes i2c's TTY file and changes isOpenFlag's value.
            * @return True if tty file closing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myI2c.close();
            *
            * @endcode
            *
            * @sa BlackSPI::isOpenFlag
            */
            bool        close();

            /*! @brief Writes byte value to i2c smbus.
            *
            * This function writes byte value to i2c smbus. Register address of device and values sent
            * to this function as uint8_t type.
            *
            * @param [in] registerAddr      register address
            * @param [in] value             byte data
            * @return true if writing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *                                       //  ___ ___ ___ ___|___ ___ ___ ___
            *   uint8_t powerCtl_Addr       = 0x2D; // |_0_|_0_|_1_|_0_|_1_|_1_|_0_|_1_|
            *   uint8_t measureMode         = 0x08; // |_0_|_0_|_0_|_0_|_1_|_0_|_0_|_0_|
            *
            *   uint8_t powerCtlReg         = myI2c.readByte(powerCtl_Addr);
            *   std::cout << "Power Ctrl's current value: " << std::hex << (int)powerCtlReg << std::dec << std::endl;
            *
            *   powerCtlReg |= (measureMode);       //                      ___ ___ ___ ___|___ ___ ___ ___
            *                                       // powerCtlReg:        |_x_|_x_|_x_|_x_|_x_|_x_|_x_|_x_|
            *                                       // measureMode:        |_0_|_0_|_0_|_0_|_1_|_0_|_0_|_0_|
            *                                       //                      ___ ___ ___ ___|___ ___ ___ ___ or these
            *                                       // result:             |_x_|_x_|_x_|_x_|_1_|_x_|_x_|_x_|
            *
            *   bool resultOfWrite          = myI2c.writeByte(powerCtl_Addr, powerCtlReg);
            *   std::cout << "Power Ctrl's new value is wrote?: " << std::boolalpha << resultOfWrite << std::dec << std::endl;
            *
            *   powerCtlReg                 = myI2c.readByte(powerCtl_Addr);
            *   std::cout << "Power Ctrl's new value: " << std::hex << (int)powerCtlReg << std::dec << std::endl;
            *
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Power Ctrl's current value: 00
            *   // Power Ctrl's new value is wrote?: true
            *   // Power Ctrl's new value: 08
            * @endcode
            */
            bool        writeByte(uint8_t registerAddr, uint8_t value);

            /*! @brief Writes word value to i2c smbus.
            *
            * This function writes word value to i2c smbus. Values sent to this function as uint16_t type.
            *
            * @param [in] registerAddr      register address
            * @param [in] value             word data
            *
            * @warning Word data operations use little-endian system. This means, it stores the most significant byte
            * of a word in the largest address and the least significant byte is stored in the smallest address.
            *
            * @return true if writing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *                                       //  ___ ___ ___ ___|___ ___ ___ ___
            *   uint8_t offsetX_Addr        = 0x1E; // |_0_|_0_|_0_|_1_|_1_|_1_|_1_|_0_|
            *   uint8_t offsetY_Addr        = 0x1F; // |_0_|_0_|_0_|_1_|_1_|_1_|_1_|_1_|
            *
            *   // Example: Register Map of slave device(0x53), Little-Endian System
            *   // ----------------------------------
            *   // |   | 0 | 1 | .......... | e | f |
            *   // ----------------------------------
            *   // |00 |...|...| .......... |...|...|
            *   // ----------------------------------
            *   // |10 |...|...| .......... | 1 | 4 |   ==> 0x1E = 0x01 , 0x1F = 0x04
            *   // ----------------------------------
            *   // | : | : | : | .......... | : | : |
            *   // | : | : | : | .......... | : | : |
            *   // | : | : | : | .......... | : | : |
            *   // ----------------------------------
            *   // |f0 |...|...| .......... |...|...|
            *   // ----------------------------------
            *   //
            *   // When users read word data from 0x1E address, word value is 0x0401.
            *   // This means smallest address(0x1E) will be least significant byte(LSB, 0x01)
            *   // and largest address(0x1F) will be most significant byte(MSB, 0x04)
            *
            *
            *   uint16_t tempRead           = myI2c.readWord(offsetX_Addr);
            *   uint8_t offsetX             = static_cast<uint8_t>(tempRead & 0xFF);
            *   uint8_t offsetY             = static_cast<uint8_t>(tempRead >> 8);
            *   std::cout << "Offset X's current value: " << std::hex << (int)offsetX << std::dec << std::endl;
            *   std::cout << "Offset Y's current value: " << std::hex << (int)offsetY << std::dec << std::endl;
            *
            *   offsetX                     = 0xA0;
            *   offsetY                     = 0x0B;
            *   uint16_t tempWrite          = ((static_cast<uint16_t>(offsetY) << 8) | static_cast<uint16_t>(offsetX));
            *
            *   bool resultOfWrite          = myI2c.writeWord(offsetX_Addr, tempWrite);
            *   std::cout << "Offset X&Y's new value is wrote?: " << std::boolalpha << resultOfWrite << std::dec << std::endl;
            *
            *   tempRead                    = myI2c.readWord(offsetX_Addr);
            *   offsetX                     = static_cast<uint8_t>(tempRead & 0xFF);
            *   offsetY                     = static_cast<uint8_t>(tempRead >> 8);
            *   std::cout << "Offset X's new value: " << std::hex << (int)offsetX << std::dec << std::endl;
            *   std::cout << "Offset Y's new value: " << std::hex << (int)offsetY << std::dec << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Offset X's current value: 00
            *   // Offset Y's current value: 00
            *   // Offset X&Y's new value is wrote?: true
            *   // Offset X's new value: A0
            *   // Offset Y's new value: 0B
            * @endcode
            */
            bool        writeWord(uint8_t registerAddr, uint16_t value);

            /*! @brief Writes block of data to i2c smbus.
            *
            * This function writes block of data to i2c smbus. Data block sent to this function as pointer
            * of uint8_t type.
            *
            * @param [in] registerAddr      register address
            * @param [in] writeBuffer       buffer pointer
            * @param [in] bufferSize        buffer size
            *
            * @warning Maximum block size is 32.
            *
            * @return true if writing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *                                       //  ___ ___ ___ ___|___ ___ ___ ___
            *   uint8_t offsetX_Addr        = 0x1E; // |_0_|_0_|_0_|_1_|_1_|_1_|_1_|_0_|
            *   uint8_t offsetY_Addr        = 0x1F; // |_0_|_0_|_0_|_1_|_1_|_1_|_1_|_1_|
            *   uint8_t offsetZ_Addr        = 0x20; // |_0_|_0_|_1_|_0_|_0_|_0_|_0_|_0_|
            *
            *   uint8_t offsetValues[3]     = { 0x00, 0x00, 0x00 };
            *   uint8_t readBlockSize       = myI2c.readBlock(offsetX_Addr, offsetValues, sizeof(offsetValues) );
            *
            *   std::cout << "Total read block size: " << (int)readBlockSize << std::endl;
            *   std::cout << "Offset X's current value: " << std::hex << (int)offsetValues[0] << std::dec << std::endl;
            *   std::cout << "Offset Y's current value: " << std::hex << (int)offsetValues[1] << std::dec << std::endl;
            *   std::cout << "Offset Z's current value: " << std::hex << (int)offsetValues[2] << std::dec << std::endl;
            *
            *
            *   offsetValues[0]             = 0x1A;
            *   offsetValues[1]             = 0x2B;
            *   offsetValues[2]             = 0x3C;
            *
            *   bool resultOfWrite          = myI2c.writeBlock(offsetX_Addr, offsetValues, sizeof(offsetValues) );
            *   std::cout << "Offsets' new values are wrote?: " << std::boolalpha << resultOfWrite << std::dec << std::endl;
            *
            *   memset(offsetValues, 0, sizeof(offsetValues));      // clear buffer
            *
            *   readBlockSize               = myI2c.readBlock(offsetX_Addr, offsetValues, sizeof(offsetValues) );
            *
            *   std::cout << "Total read block size: " << (int)readBlockSize << std::endl;
            *   std::cout << "Offset X's current value: " << std::hex << (int)offsetValues[0] << std::dec << std::endl;
            *   std::cout << "Offset Y's current value: " << std::hex << (int)offsetValues[1] << std::dec << std::endl;
            *   std::cout << "Offset Z's current value: " << std::hex << (int)offsetValues[2] << std::dec << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Total read block size: 3
            *   // Offset X's current value: 00
            *   // Offset Y's current value: 00
            *   // Offset Z's current value: 00
            *   // Offsets' new values are wrote?: true
            *   // Total read block size: 3
            *   // Offset X's current value: 1A
            *   // Offset Y's current value: 2B
            *   // Offset Z's current value: 3C
            * @endcode
            */
            bool        writeBlock(uint8_t registerAddr, uint8_t *writeBuffer, size_t bufferSize);

            /*! @brief Writes data block to i2c line.
            *
            * This function writes data block to i2c line directly. Data block sent to this function as pointer
            * of uint8_t type.
            *
            * @param [in] writeBuffer       buffer pointer
            * @param [in] bufferSize        buffer size
            *
            * @warning Data block's first element must be the address of register which will write datas.
            *
            * @return true if writing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *                                       //  ___ ___ ___ ___|___ ___ ___ ___
            *   uint8_t offsetX_Addr        = 0x1E; // |_0_|_0_|_0_|_1_|_1_|_1_|_1_|_0_|
            *   uint8_t offsetY_Addr        = 0x1F; // |_0_|_0_|_0_|_1_|_1_|_1_|_1_|_1_|
            *   uint8_t offsetZ_Addr        = 0x20; // |_0_|_0_|_1_|_0_|_0_|_0_|_0_|_0_|
            *
            *   uint8_t offsetValues[3]     = { 0x00, 0x00, 0x00 };
            *   myI2c.writeLine(&offsetX_Addr, 1);                      // address of register which will read datas
            *   myI2c.readLine( offsetValues, sizeof(offsetValues) );   // read three data from "offsetX_Addr"
            *
            *   std::cout << "Offset X's current value: " << std::hex << (int)offsetValues[0] << std::dec << std::endl;
            *   std::cout << "Offset Y's current value: " << std::hex << (int)offsetValues[1] << std::dec << std::endl;
            *   std::cout << "Offset Z's current value: " << std::hex << (int)offsetValues[2] << std::dec << std::endl;
            *
            *
            *   offsetValues[0]             = 0xD4;
            *   offsetValues[1]             = 0xE5;
            *   offsetValues[2]             = 0xF6;
            *   uint8_t tempBufWithAddr[4]  = { offsetX_Addr, offsetValues[0], offsetValues[1], offsetValues[2] };
            *
            *   bool resultOfWrite          = myI2c.writeLine(tempBufWithAddr, sizeof(tempBufWithAddr) );
            *   std::cout << "Offsets' new values are wrote?: " << std::boolalpha << resultOfWrite << std::dec << std::endl;
            *
            *   memset(offsetValues, 0, sizeof(offsetValues));          // clear buffer
            *   myI2c.writeLine(&offsetX_Addr, 1);                      // address of register which will read datas
            *   myI2c.readLine( offsetValues, sizeof(offsetValues) );   // read three data from "offsetX_Addr"
            *
            *   std::cout << "Offset X's current value: " << std::hex << (int)offsetValues[0] << std::dec << std::endl;
            *   std::cout << "Offset Y's current value: " << std::hex << (int)offsetValues[1] << std::dec << std::endl;
            *   std::cout << "Offset Z's current value: " << std::hex << (int)offsetValues[2] << std::dec << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Offset X's current value: 00
            *   // Offset Y's current value: 00
            *   // Offset Z's current value: 00
            *   // Offsets' new values are wrote?: true
            *   // Offset X's current value: D4
            *   // Offset Y's current value: E5
            *   // Offset Z's current value: F6
            * @endcode
            */
            bool        writeLine(uint8_t *writeBuffer, size_t bufferSize);

            /*! @brief Read byte value from i2c smbus.
            *
            * This function reads byte value from i2c smbus. Register address of device sent
            * to this function as uint8_t type.
            *
            * @param [in] registerAddr      register address
            * @return read value if reading successfull, else 0x00.
            *
            * @par Example
            *   Example usage is shown in BlackI2C::writeByte() function's example.
            */
            uint8_t     readByte(uint8_t registerAddr);

            /*! @brief Read word value from i2c smbus.
            *
            * This function reads word value from i2c smbus. Register address of device sent
            * to this function as uint8_t type.
            *
            * @param [in] registerAddr      register address
            * @return read value if reading successfull, else 0x0000.
            *
            * @par Example
            *   Example usage is shown in BlackI2C::writeWord() function's example.
            */
            uint16_t    readWord(uint8_t registerAddr);

            /*! @brief Read data block from i2c smbus.
            *
            * This function reads data block from i2c smbus. Register address of device sent
            * to this function as uint8_t type.
            *
            * @param [in] registerAddr      register address
            * @param [out] readBuffer       buffer pointer
            * @param [in] bufferSize        buffer size
            * @return size of read data block if reading successfull, else 0x00.
            *
            * @par Example
            *   Example usage is shown in BlackI2C::writeBlock() function's example.
            */
            uint8_t     readBlock(uint8_t registerAddr, uint8_t *readBuffer, size_t bufferSize);

            /*! @brief Read data block from i2c line.
            *
            * This function reads data block from i2c line directly.
            *
            * @param [out] readBuffer       buffer pointer
            * @param [in] bufferSize        buffer size
            *
            * @warning For reading something from i2c line, firstly users must write register address
            * data to line.
            * @return true reading successfull, else false.
            *
            * @par Example
            *   Example usage is shown in BlackI2C::writeLine() function's example.
            */
            bool        readLine(uint8_t *readBuffer, size_t bufferSize);

            /*! @brief Changes device address of slave device.
            *
            * This function changes device address of slave device and sets this device to slave.
            *
            * @param [in] newDeviceAddr  new slave device address
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Current slave address: " << std::hex << myI2c.getDeviceAddress() << std::endl;
            *
            *   myI2c.setDeviceAddress(0x69);
            *   std::cout << "Current slave address: " << std::hex << myI2c.getDeviceAddress() << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current slave address: 53
            *   // Current slave address: 69
            * @endcode
            *
            */
            void        setDeviceAddress(unsigned int newDeviceAddr);

            /*! @brief Exports device address of slave device.
            *
            * @return address of current slave device.
            *
            * @par Example
            *   Example usage is shown in BlackI2C::setDeviceAddress() function's example.
            *
            */
            int         getDeviceAddress();

            /*! @brief Exports i2c's port path.
            *
            * @return i2c's port path as string.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Port path: " << myI2c.getPortName();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Port path: /dev/i2c-1
            * @endcode
            *
            * @sa i2cPortPath
            */
            std::string getPortName();

            /*! @brief Checks i2c's tty file's open state.
            *
            * @return true if tty file is open, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   std::cout << "Is open?: " << std::boolalpha << myI2c.isOpen() << std::endl;
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Is open?: " << std::boolalpha << myI2c.isOpen();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Is open?: false
            *   // Is open?: true
            * @endcode
            *
            * @sa BlackI2C::isOpenFlag
            */
            bool        isOpen();

            /*! @brief Checks i2c's tty file's close state.
            *
            * @return true if tty file is close, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   std::cout << "Is close?: " << std::boolalpha << myI2c.isClose() << std::endl;
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Is close?: " << std::boolalpha << myI2c.isClose();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Is close?: true
            *   // Is close?: false
            * @endcode
            *
            * @sa BlackI2C::isOpenFlag
            */
            bool        isClose();

            /*! @brief Is used for general debugging.
            *
            * @return True if any error occured, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   if( myI2c.fail() )
            *   {
            *       std::cout << "ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "EVERYTHING IS OK" << std::endl;
            *   }
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // EVERYTHING IS OK
            * @endcode
            *
            * @sa errorUART
            */
            bool        fail();

            /*! @brief Is used for specific debugging.
            *
            * You can use this function, after call BlackI2C member functions in your code. The
            * input parameter is used for finding out status of selected error.
            * @param [in] f specific error type (enum)
            * @return Value of @a selected error.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackI2C  myI2c(BlackLib::I2C_1, 0x53);
            *
            *   myI2c.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   if( myI2c.fail(BlackLib::BlackI2C::openErr) )
            *   {
            *       std::cout << "OPENNING ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "OPENNING IS OK" << std::endl;
            *
            *       if( myI2c.fail(BlackLib::BlackI2C::setSlaveErr) )
            *       {
            *           std::cout << "SETTING SLAVE ERROR OCCURED" << std::endl;
            *       }
            *       else
            *       {
            *           std::cout << "SETTING SLAVE IS OK" << std::endl;
            *       }
            *   }
            *
            *
            *   uint8_t powerCtl_Addr       = 0x2D;
            *   uint8_t measureMode         = 0x08;
            *   uint8_t powerCtlReg         = myI2c.readByte(powerCtl_Addr);
            *
            *   if( myI2c.fail(BlackLib::BlackI2C::readErr) )
            *   {
            *       std::cout << "READ DATA ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "READ DATA IS OK" << std::endl;
            *   }
            *
            *
            *   powerCtlReg |= (measureMode);
            *   bool resultOfWrite          = myI2c.writeByte(powerCtl_Addr, powerCtlReg);
            *
            *   if( myI2c.fail(BlackLib::BlackI2C::writeErr) )
            *   {
            *       std::cout << "WRITE DATA ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "WRITE DATA IS OK" << std::endl;
            *   }
            *
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // OPENNING IS OK
            *   // SETTING SLAVE IS OK
            *   // READ DATA IS OK
            *   // WRITE DATA IS OK
            * @endcode
            *
            * @sa errorI2C
            */
            bool        fail(BlackI2C::flags f);
    };

    // ########################################### BLACKI2C DECLARATION ENDS ############################################## //


// ######################################### BLACKCOREPWM DECLARATION STARTS ########################################## //

    /*! @brief Preparation phase of Beaglebone Black, to use PWM.
     *
     *    This class is core of the BlackPWM class. It includes private functions which are doing base processes
     *    for using pwms and protected functions which are using for exporting private variables to
     *    derived class(es).
     */
    class BlackCorePWM : virtual private BlackCore
    {
        private:
            errorCorePWM    *pwmCoreErrors;             /*!< @brief is used to hold the errors of BlackCorePWM class */
            std::string     pwmTestPath;                /*!< @brief is used to hold the pwm_test (pwm device driver) path */
            pwmName         pwmPinName;                 /*!< @brief is used to hold the selected pwm @b pin name */

            /*! @brief Loads PWM overlays to device tree.
            *
            *  This function loads @b "am33xx_pwm" and @b "bone_pwm_P?_?" overlay to device tree.
            *  Question marks at the second overlay, represents port and pin numbers of selected PWM
            *  output. This overlays perform pinmuxing and generate device drivers.
            *  @return True if successful, else false.
            */
            bool            loadDeviceTree();

            /*! @brief Finds full name of pwm_test.
            *
            *  This function searches @b "ocp.X" directory to find directory starts with
            *  @b "pwm_test_?_?." by using searchDirectoryOcp() protected function at BlackCore class.
            *  @return Full name of pwm_test directory if successfull, else BlackLib::PWM_TEST_NAME_NOT_FOUND string.
            *  @sa BlackCore::searchDirectoryOcp()
            */
            std::string     findPwmTestName(pwmName pwm);


        protected:
            /*! @brief Exports pwm period file name to derived class.
            *
            *  @return Pwm period file name.
            */
            std::string     getPeriodFilePath();

            /*! @brief Exports pwm duty file name to derived class.
            *
            *  @return Pwm duty file name.
            */
            std::string     getDutyFilePath();

            /*! @brief Exports pwm run file name to derived class.
            *
            *  @return Pwm run file name.
            */
            std::string     getRunFilePath();

            /*! @brief Exports pwm polarity file name to derived class.
            *
            *  @return Pwm polarity file name.
            */
            std::string     getPolarityFilePath();

            /*! @brief Exports errorCorePWM struct to derived class.
            *
            *  @return errorCorePWM struct pointer.
            */
            errorCorePWM    *getErrorsFromCorePWM();

        public:

            /*! @brief Constructor of BlackCorePWM class.
            *
            *  This function initializes errorCorePWM struct and calls device tree loading and
            *  pwm test name finding functions.
            *
            *  @sa BlackCorePWM::loadDeviceTree()
            *  @sa BlackCorePWM::findPwmTestName()
            *  @sa pwmName
            */
                            BlackCorePWM(pwmName pwm);

            /*! @brief Destructor of BlackCorePWM class.
            *
            * This function deletes errorCorePWM struct pointer.
            */
            virtual         ~BlackCorePWM();

            /*! @brief First declaration of this function.
            */
            virtual std::string getValue() = 0;
    };
    // ########################################## BLACKCOREPWM DECLARATION ENDS ########################################### //










    // ########################################### BLACKPWM DECLARATION STARTS ############################################ //

    /*! @brief Interacts with end user, to use PWM.
     *
     *    This class is end node to use PWM. End users interact with PWM signal from this class.
     *    It includes public functions to set and get properties of PWM.
     *
     * @par Example
      @verbatim
      EXAMPLE PROJECT FILE TREE:
         myPwmProject
         |-> src
             |-> BlackLib
                 |-> BlackADC.cpp
                 |-> BlackADC.h
                 |-> BlackCore.cpp
                 |-> BlackCore.h
                 |-> BlackDef.h
                 |-> BlackErr.h
                 |-> BlackGPIO.cpp
                 |-> BlackGPIO.h
                 |-> BlackI2C.cpp
                 |-> BlackI2C.h
                 |-> BlackLib.h
                 |-> BlackPWM.cpp
                 |-> BlackPWM.h
                 |-> BlackSPI.cpp
                 |-> BlackSPI.h
                 |-> BlackUART.cpp
                 |-> BlackUART.h
             |-> myPwmProject.cpp
      @endverbatim
     *  @n@n If BlackLib source files are located in your project like above example project file tree, you have to
     *  include BlackPWM.h or another source files with adding this line to your project file (myPwmProject.cpp at
     *  the example):
     *  @code{.cpp}
     *      #include "BlackLib/BlackPWM.h"
     *  @endcode
     *  @n@n If BlackLib source files are located at same level with your project file (myPwmProject.cpp at the
     *  example), you have to include BlackPWM.h or another source files with adding this line to your project file:
     *  @code{.cpp}
     *      #include "BlackPWM.h"
     *  @endcode
     *  @n @n
     *  @code{.cpp}
     *  // Filename: myPwmProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackPWM.h"
     *
     *  int main()
     *  {
     *      BlackLib::BlackPWM  myPwm(BlackLib::P8_19);
     *      std::cout << myPwm.getValue();
     *
     *      return 0;
     *  }
     * @endcode
     * @n @n
     * You can use "using namespace BlackLib" also. You can get rid of writing "BlackLib::", with using this method.
     * @code{.cpp}
     *  // Filename: myPwmProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackPWM.h"
     *  using namespace BlackLib;
     *
     *  int main()
     *  {
     *      BlackPWM  myPwm(EHRPWM2A);
     *      std::cout << myPwm.getValue();
     *
     *      return 0;
     *  }
     *
     * @endcode
     *
     * @n @n
     * @par Pwm Definition
       @verbatim
        1 ...............................________............................________
             :        :        :        |        |        :        :        |        |
             :   (1)  :   (2)  :   (3)  |   (4)  |        :        :        |        |
        0 ...:________:________:________|        |________:________:________|        |
             :        :        :        :        :        :        :        :        :
             <-----------t1------------> <--t2--> <-----------t1-----------> <--t2-->
       @endverbatim
     * @n @n @b t1 represents "space time" ==> @b 3 units at the figure
     * @n @b t2 represents "load time"     ==> @b 1 unit at the figure
     * @n <b> (t1+t2) </b> represents "period time" ==> @b 4 units at the figure
     * @n <b> (t2/(t1+t2)) </b> represents "duty ratio" ==> @b 0.25 at the figure
     */
    class BlackPWM : virtual private BlackCorePWM
    {
        private:
            errorPWM        *pwmErrors;                 /*!< @brief is used to hold the errors of BlackPWM class */
            std::string     periodPath;                 /*!< @brief is used to hold the @a period file path */
            std::string     dutyPath;                   /*!< @brief is used to hold the @a duty file path */
            std::string     runPath;                    /*!< @brief is used to hold the @a run file path */
            std::string     polarityPath;               /*!< @brief is used to hold the @a polarity file path */


        public:
            /*!
            * This enum is used to define PWM debugging flags.
            */
            enum flags      {   periodFileErr   = 0,    /*!< enumeration for @a errorPWM::periodFileError status */
                                dutyFileErr     = 1,    /*!< enumeration for @a errorPWM::dutyFileError status */
                                runFileErr      = 2,    /*!< enumeration for @a errorPWM::runFileError status */
                                polarityFileErr = 3,    /*!< enumeration for @a errorPWM::polarityFileError status */
                                outOfRangeErr   = 4,    /*!< enumeration for @a errorPWM::outOfRange status */
                                dtErr           = 5,    /*!< enumeration for @a errorCorePWM::dtError status */
                                dtSubSystemErr  = 6,    /*!< enumeration for @a errorCorePWM::dtSsError status */
                                pwmTestErr      = 7,    /*!< enumeration for @a errorCorePWM::pwmTestError status */
                                cpmgrErr        = 9,    /*!< enumeration for @a errorCore::capeMgrError status */
                                ocpErr          = 10    /*!< enumeration for @a errorCore::ocpError status */
                            };

            /*! @brief Constructor of BlackPWM class.
            *
            * This function initializes BlackCorePWM class with entered parameter and errorPWM struct.
            * Then it sets file paths of period, duty, polarity and run files.
            * @param [in] pwm        pwm name (enum)
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackPWM  myPwm(BlackLib::P8_19);
            *   BlackLib::BlackPWM *myPwmPtr = new BlackLib::BlackPWM(BlackLib::EHRPWM2B);
            *
            *   myPwm.getValue();
            *   myPwmPtr->getValue();
            *
            * @endcode
            *
            * @sa pwmName
            */
                            BlackPWM(pwmName pwm);

            /*! @brief Destructor of BlackPWM class.
            *
            * This function deletes errorPWM struct pointer.
            */
            virtual         ~BlackPWM();

            /*! @brief Reads percentage value of duty cycle.
            *
            * This function calls getNumericPeriodValue() and getNumericDutyValue() inline functions for
            * finding out period and duty values of pwm. After do that, it calculates percentage value of
            * time that stays at "1".
            *  @return @a String type percentage value.
            *
            *  @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setDutyPercent(21.0);
            *   std::string val = myPwm.getValue();
            *   std::cout << "Pwm duty ratio: " << val << "%";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm duty ratio: 21%
            * @endcode
            *
            *  @sa getNumericPeriodValue()
            *  @sa getNumericDutyValue()
            */
            std::string     getValue();

            /*! @brief Reads period value of pwm signal.
            *
            * This function reads specified file from path, where defined at BlackPWM::periodPath variable.
            * This file holds pwm period value at nanosecond (ns) level.
            *  @return @a string type period value. If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_STRING.
            *
            *  @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   // if new period value is less than the current duty value, the new period value setting
            *   // operation couldn't execute. So firstly duty value is set to zero for safe steps.
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(214000);
            *
            *   std::string val = myPwm.getPeriodValue();
            *   std::cout << "Pwm period time: " << val << " nanoseconds";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 214000 nanoseconds
            * @endcode
            */
            std::string     getPeriodValue();

            /*! @brief Reads duty value of pwm signal.
            *
            * This function reads specified file from path, where defined at BlackPWM::dutyPath variable.
            * This file holds pwm duty value at nanosecond (ns) level.
            *  @return @a string type duty value. If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_STRING.
            *
            *  @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   // if new period value is less than the current duty value, the new period value setting
            *   // operation couldn't execute. So firstly duty value is set to zero for safe steps.
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(900000);
            *   myPwm.setDutyPercent(50.0);
            *
            *   std::string val = myPwm.getDutyValue();
            *   std::cout << "Pwm duty time: " << val << " nanoseconds";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm duty time: 450000 nanoseconds
            * @endcode
            */
            std::string     getDutyValue();

            /*! @brief Reads run value of pwm signal.
            *
            * This function reads specified file from path, where defined at BlackPWM::runPath variable.
            * This file holds pwm run value.
            *  @return @a string type run value. If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_STRING.
            *
            *  @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setRunState(BlackLib::run);
            *   std::string val1 = myPwm.getRunValue();
            *
            *   myPwm.setRunState(BlackLib::stop);
            *   std::string val2 = myPwm.getRunValue();
            *
            *   std::cout << "Run value 1: " << val1 << std::endl;
            *   std::cout << "Run value 2: " << val2 << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Run value 1: 1
            *   // Run value 2: 0
            * @endcode
            */
            std::string     getRunValue();

            /*! @brief Reads polarity value of pwm signal.
            *
            * This function reads specified file from path, where defined at BlackPWM::polarityPath variable.
            * This file holds pwm polarity value.
            *  @return @a String type polarity value. If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_STRING.
            *
            *  @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setPolarity(BlackLib::straight);
            *   std::string val1 = myPwm.getPolarityValue();
            *
            *   myPwm.setPolarity(BlackLib::reverse);
            *   std::string val2 = myPwm.getPolarityValue();
            *
            *   std::cout << "Polarity value 1: " << val1 << std::endl;
            *   std::cout << "Polarity value 2: " << val2 << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Polarity value 1: 0
            *   // Polarity value 2: 1
            * @endcode
            */
            std::string     getPolarityValue();

            /*! @brief Reads numeric percentage value of duty cycle.
            *
            * This function calls getNumericPeriodValue() and getNumericDutyValue() inline functions, for finding
            * out the period and duty values of pwm. After do that, it calculates percentage value of
            * time that stays at "1".
            *  @return @a Float type percentage value.
            *
            *  @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setDutyPercent(21.0);
            *   float val = myPwm.getNumericValue();
            *   std::cout << "Pwm duty ratio: " << val << "%";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm duty ratio: 21.0%
            * @endcode
            *
            *  @sa getNumericPeriodValue()
            *  @sa getNumericDutyValue()
            */
            float           getNumericValue();

            /*! @brief Reads numeric period value of pwm signal.
            *
            * This function reads specified file from path, where defined at BlackPWM::periodPath variable.
            * This file holds pwm period value at nanosecond (ns) level.
            * @return @a int64_t (long int) type period value.  If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_INT.
            * @warning Any alphabetic character existence in period file can crash your application.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   // if new period value is less than the current duty value, the new period value setting
            *   // operation couldn't execute. So firstly duty value is set to zero for safe steps.
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(214000);
            *
            *   long int val = myPwm.getNumericPeriodValue();
            *   std::cout << "Pwm period time: " << val << " nanoseconds";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 214000 nanoseconds
            * @endcode
            */
            int64_t         getNumericPeriodValue();

            /*! @brief Reads numeric duty value of pwm signal.
            *
            * This function reads specified file from path, where defined at BlackPWM::dutyPath variable.
            * This file holds pwm duty value at nanosecond (ns) level.
            * @return @a int64_t (long int) type duty value.  If file opening fails, it returns BlackLib::FILE_COULD_NOT_OPEN_INT.
            * @warning Any alphabetic character existence in period file can crash your application.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   // if new period value is less than the current duty value, the new period value setting
            *   // operation couldn't execute. So firstly duty value is set to zero for safe steps.
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(214000);
            *
            *   // space ratio time = duty value, also duty value must be less than current period time
            *   myPwm.setSpaceRatioTime(200000, BlackLib::nanosecond);
            *
            *   long int val = myPwm.getNumericDutyValue();
            *   std::cout << "Pwm duty time: " << val << " nanoseconds";
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 200000 nanoseconds
            * @endcode
            */
            int64_t         getNumericDutyValue();


            /*! @brief Sets percentage value of duty cycle.
            *
            * If input parameter is in range (from 0.0 to 100.0), this function changes duty value
            * without changing period value. For calculating new duty value, the current period
            * multiplies by (1 - entered percentage/100) value. After do that, this calculated value
            * is saved to duty file.
            * @param [in] percentage new percantage value(float)
            * @return True if setting new value is successful, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(500000, BlackLib::nanosecond);
            *   myPwm.setDutyPercent(20.0);
            *
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *   std::cout << "Pwm duty time  : " << myPwm.getDutyValue() << " nanoseconds \n";
            *   std::cout << "Pwm duty ratio : " << myPwm.getValue() << "%";
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 500000 nanoseconds
            *   // Pwm duty time  : 400000 nanoseconds
            *   // Pwm duty ratio : 20.0%
            * @endcode
            *
            * @sa getNumericPeriodValue()
            * @note This function can do very little rounding error at new duty value. For example if
            * your current duty value is \f$ 10^6 \f$, this rounding error equals maximum \f$ 0.5 / 10^6 = 5*10^-7 \f$
            */
            bool            setDutyPercent(float percentage);

            /*! @brief Sets period value of pwm signal.
            *
            * If input parameter's nanosecond equivalent is in range (from 0 to 10^9), this function changes period value
            * by saving calculated input parameter to period file. Users can select time type of entered period
            * value like picosecond, nanosecond, microsecond, milisecond and second. This parameter's
            * default value is nanosecond.
            * @param [in] period new period value
            * @param [in] tType time type of your new period value(enum)
            * @return True if setting new period value is successful, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setDutyPercent(100.0);
            *
            *   myPwm.setPeriodTime(300000000, BlackLib::picosecond);
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *
            *   myPwm.setPeriodTime(500000, BlackLib::nanosecond);
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *
            *   myPwm.setPeriodTime(700, BlackLib::microsecond);
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *
            *   myPwm.setPeriodTime(1, BlackLib::milisecond);
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 300000 nanoseconds
            *   // Pwm period time: 500000 nanoseconds
            *   // Pwm period time: 700000 nanoseconds
            *   // Pwm period time: 1000000 nanoseconds
            * @endcode
            *
            * @sa BlackLib::timeType
            */
            bool            setPeriodTime(uint64_t period, timeType tType = nanosecond);

            /*! @brief Sets space time value of pwm signal.
            *
            * If input parameter's nanosecond equivalent is in range (from 0 to 10^9), this function changes duty value
            * by saving calculated input parameter to duty file. Users can select time type of entered space time value
            * like picosecond, nanosecond, microsecond, milisecond and second. This parameter's default value is nanosecond.
            * @param [in] space new space time
            * @param [in] tType time type of your new period value(enum)
            * @return True if setting new value is successful, else false.
            * @sa BlackLib::timeType
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(500000, BlackLib::nanosecond);
            *   myPwm.setSpaceRatioTime(125000);
            *
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *   std::cout << "Pwm duty time  : " << myPwm.getDutyValue() << " nanoseconds \n";
            *   std::cout << "Pwm duty ratio : " << myPwm.getValue() << "%";
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 500000 nanoseconds
            *   // Pwm duty time  : 125000 nanoseconds
            *   // Pwm duty ratio : 75.0%
            * @endcode
            *
            * @warning If your entered space time's nanosecond equivalent value is greater than the current period time,
            * the new value could not set. This feature is Beaglebone's default.
            */
            bool            setSpaceRatioTime(uint64_t space, timeType tType = nanosecond);

            /*! @brief Sets load time value of pwm signal.
            *
            * If input parameter's nanosecond equivalent is in range (from 0 to 10^9), this function changes duty value
            * by saving (current period value - calculated input value) to duty file. Users can select time type of entered
            * space time value like picosecond, nanosecond, microsecond, milisecond and second. This parameter's default
            * value is nanosecond.
            * @param [in] load new load time
            * @param [in] tType time type of your new period value(enum)
            * @return True if setting new value is successful, else false.
            * @sa BlackLib::timeType
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setDutyPercent(100.0);
            *   myPwm.setPeriodTime(500000, BlackLib::nanosecond);
            *   myPwm.setLoadRatioTime(125000);
            *
            *   std::cout << "Pwm period time: " << myPwm.getPeriodValue() << " nanoseconds \n";
            *   std::cout << "Pwm duty time  : " << myPwm.getDutyValue() << " nanoseconds \n";
            *   std::cout << "Pwm duty ratio : " << myPwm.getValue() << "%";
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Pwm period time: 500000 nanoseconds
            *   // Pwm duty time  : 375000 nanoseconds
            *   // Pwm duty ratio : 25.0%
            * @endcode
            *
            * @warning If your entered load time's nanosecond equivalent value is greater than the current period time,
            * the new value could not set. This feature is Beaglebone's default.
            */
            bool            setLoadRatioTime(uint64_t load, timeType tType = nanosecond);

            /*! @brief Sets polarity of pwm signal.
            *
            * The input parameter is converted to 1 or 0 and this value is saved to polarity file.
            * @param [in] polarity new polarity value(enum)
            * @return True if setting new polarity is successful, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setPolarity(BlackLib::straight);
            *   std::string val1 = myPwm.getPolarityValue();
            *
            *   myPwm.setPolarity(BlackLib::reverse);
            *   std::string val2 = myPwm.getPolarityValue();
            *
            *   std::cout << "Polarity value 1: " << val1 << std::endl;
            *   std::cout << "Polarity value 2: " << val2 << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Polarity value 1: 0
            *   // Polarity value 2: 1
            * @endcode
            *
            * @sa polarityType
            */
            bool            setPolarity(polarityType polarity);

            /*! @brief Sets run value of pwm signal.
            *
            * The input parameter is converted to 1 or 0 and this value is saved to run file.
            * @param [in] state new run value(enum)
            * @return True if setting new run value is successful, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setRunState(BlackLib::run);
            *   std::string val1 = myPwm.getRunValue();
            *
            *   myPwm.setRunState(BlackLib::stop);
            *   std::string val2 = myPwm.getRunValue();
            *
            *   std::cout << "Run value 1: " << val1 << std::endl;
            *   std::cout << "Run value 2: " << val2 << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Run value 1: 1
            *   // Run value 2: 0
            * @endcode
            *
            * @sa runValue
            */
            bool            setRunState(runValue state);

            /*! @brief Toggles run state of pwm signal.
            *
            * This function sets run value to 1 or 0, by value of current state. This new value is
            * saved to run file.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setRunState(BlackLib::run);
            *   std::string val1 = myPwm.getRunValue();
            *
            *   myPwm.toggleRunState();
            *   std::string val2 = myPwm.getRunValue();
            *
            *   std::cout << "Run value 1: " << val1 << std::endl;
            *   std::cout << "Run value 2: " << val2 << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Run value 1: 1
            *   // Run value 2: 0
            * @endcode
            */
            void            toggleRunState();

            /*! @brief Toggles polarity type of pwm signal.
            *
            * This function sets polarity value to 1 or 0, by value of current polarity. This new
            * value is saved to polarity file.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setPolarity(BlackLib::straight);
            *   std::string val1 = myPwm.getPolarityValue();
            *
            *   myPwm.tooglePolarity();
            *   std::string val2 = myPwm.getPolarityValue();
            *
            *   std::cout << "Polarity value 1: " << val1 << std::endl;
            *   std::cout << "Polarity value 2: " << val2 << std::endl;
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Polarity value 1: 0
            *   // Polarity value 2: 1
            * @endcode
            */
            void            tooglePolarity();

            /*! @brief Checks run state of pwm signal.
            *
            * This function calls getRunValue() function and evaluates return value.
            * @return True if return value equals to 1, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setRunState(BlackLib::run);
            *   std::cout << "Run value 1: " << std::boolalpha << myPwm.isRunning() << std::endl;
            *
            *   myPwm.setRunState(BlackLib::stop);
            *   std::cout << "Run value 2: " << std::boolalpha << myPwm.isRunning() << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Run value 1: true
            *   // Run value 2: false
            * @endcode
            *
            * @sa getRunValue()
            */
            bool            isRunning();

            /*! @brief Checks polarity of pwm signal.
            *
            * This function calls getPolarityValue() function and evaluates return value.
            * @return True if return value not equals to 1, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setPolarity(BlackLib::straight);
            *   std::cout << "Polarity value 1: " << std::boolalpha << myPwm.isPolarityStraight() << std::endl;
            *
            *   myPwm.setPolarity(BlackLib::reverse);
            *   std::cout << "Polarity value 2: " << std::boolalpha << myPwm.isPolarityStraight() << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Polarity value 1: true
            *   // Polarity value 2: false
            * @endcode
            *
            * @sa getPolarityValue()
            */
            bool            isPolarityStraight();

            /*! @brief Checks polarity of pwm signal.
            *
            * This function calls getPolarityValue() function and evaluates return value.
            * @return True if return value equals to 1, else false.
            *
            * @par Example
            * @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.setPolarity(BlackLib::straight);
            *   std::cout << "Polarity value 1: " << std::boolalpha << myPwm.isPolarityReverse() << std::endl;
            *
            *   myPwm.setPolarity(BlackLib::reverse);
            *   std::cout << "Polarity value 2: " << std::boolalpha << myPwm.isPolarityReverse() << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Polarity value 1: false
            *   // Polarity value 2: true
            * @endcode
            *
            * @sa getPolarityValue()
            */
            bool            isPolarityReverse();

            /*! @brief Is used for general debugging.
            *
            * @return True if any error occured, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   myPwm.getValue();
            *
            *   if( myPwm.fail() )
            *   {
            *       std::cout << "ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "EVERYTHING IS OK" << std::endl;
            *   }
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // EVERYTHING IS OK
            * @endcode
            *
            * @sa errorPWM
            */
            bool            fail();

            /*! @brief Is used for specific debugging.
            *
            * You can use this function, after call BlackPWM member functions in your code. The
            * input parameter is used for finding out status of selected error.
            * @param [in] f specific error type (enum)
            * @return Value of @a selected error.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackPWM myPwm(BlackLib::P8_19);
            *
            *   if( myPwm.fail(BlackLib::BlackPWM::dtErr) or
            *       myPwm.fail(BlackLib::BlackPWM::pwmTestErr) or
            *       myPwm.fail(BlackLib::BlackPWM::dtSubSystemErr) )
            *   {
            *        std::cout << "BlackPWM INITIALIZATION FAILED" << std::endl;
            *   }
            *
            *   myPwm.getValue();
            *   if( myPwm.fail(BlackLib::BlackPWM::dutyFileErr) or
            *       myPwm.fail(BlackLib::BlackPWM::periodFileErr) )
            *   {
            *        std::cout << "Duty and/or period file reading is failed." << std::endl;
            *   }
            *
            *
            *   myPwm.setDutyPercent(110.5);
            *   if( myPwm.fail(BlackLib::BlackPWM::outOfRangeErr) )
            *   {
            *        std::cout << "Your percentage is huge." << std::endl;
            *   }
            *
            *   if( myPwm.fail(BlackLib::BlackPWM::dutyFileErr) )
            *   {
            *        std::cout << "Duty file reading or range value is failed." << std::endl;
            *   }
            *
            *   if( myPwm.fail(BlackLib::BlackPWM::periodFileErr) )
            *   {
            *        std::cout << "Period file reading or range value is failed." << std::endl;
            *   }
            *
            *
            *  @endcode
            *  @code{.cpp}
            *   // Possible Output:
            *   // Your percentage is huge.
            *   // Duty file reading or range value is failed.
            *   // Period file reading or range value is failed.
            *  @endcode
            *
            * @sa errorPWM
            */
            bool            fail(BlackPWM::flags f);
    };
    // ########################################### BLACKPWM DECLARATION STARTS ############################################ //


// ######################################### BLACKSPIPROPERTIES DECLARATION STARTS ######################################### //

    /*! @brief Holds properties of SPI.
     *
     *    This struct holds bits per word, mode and speed properties of SPI. Also it has overloaded
     *    constructors and overloaded assign operator.
     */
    struct BlackSpiProperties
    {
        uint8_t     spiBitsPerWord;     /*!< @brief is used to hold the bits per word size of SPI */
        uint8_t     spiMode;            /*!< @brief is used to hold the mode of SPI */
        uint32_t    spiSpeed;           /*!< @brief is used to hold the speed of SPI */

        /*! @brief Default constructor of BlackSpiProperties struct.
         *
         *  This function sets default value to variables.
         */
        BlackSpiProperties()
        {
            spiBitsPerWord  = 0;
            spiMode         = 0;
            spiSpeed        = 0;
        }

        /*! @brief Overloaded constructor of BlackSpiProperties struct.
         *
         *  This function sets entered parameters to variables.
         */
        BlackSpiProperties(uint8_t bits, uint8_t mode, uint32_t speed)
        {
            spiBitsPerWord  = bits;
            spiMode         = mode;
            spiSpeed        = speed;
        }

        /*! @brief Overloaded constructor of BlackSpiProperties struct.
         *
         *  This function gets pointer of BlackSpiProperties struct and sets this pointed
         *  struct's variables to own variables.
         */
        BlackSpiProperties( BlackSpiProperties *self )
        {
            spiBitsPerWord  = self->spiBitsPerWord;
            spiMode         = self->spiMode;
            spiSpeed        = self->spiSpeed;
        }

        /*! @brief Overloaded assign operator of BlackSpiProperties struct.
         *
         *  This function assigns input struct's variables to own variables.
         */
        BlackSpiProperties& operator=(BlackSpiProperties equal)
        {
            spiBitsPerWord  = equal.spiBitsPerWord;
            spiMode         = equal.spiMode;
            spiSpeed        = equal.spiSpeed;
            return *this;
        }
    };
    // ########################################## BLACKSPIPROPERTIES DECLARATION ENDS ########################################### //





    // ########################################### BLACKSPI DECLARATION STARTS ############################################ //

    /*! @brief Interacts with end user, to use SPI.
     *
     *    This class is end node to use spi. End users interact with spi from this class.
     *    It includes public functions to set and get properties of spi's and to transfer datas.
     *    Spi has not capable of only read operation. For reading something from spi, user's
     *    must send dummy data.
     *
     *    @warning Users have to execute setup script before use spi. This is required for compiling
     *    and setting device tree overlays about spi.
     *
     *    @warning Spi speed can be 24000000 Hz(24 MHz) maximum.
     *
     *    @par Example
      @verbatim
      EXAMPLE PROJECT FILE TREE:
         mySpiProject
         |-> src
             |-> BlackLib
                 |-> BlackADC.cpp
                 |-> BlackADC.h
                 |-> BlackCore.cpp
                 |-> BlackCore.h
                 |-> BlackDef.h
                 |-> BlackErr.h
                 |-> BlackGPIO.cpp
                 |-> BlackGPIO.h
                 |-> BlackI2C.cpp
                 |-> BlackI2C.h
                 |-> BlackLib.h
                 |-> BlackPWM.cpp
                 |-> BlackPWM.h
                 |-> BlackSPI.cpp
                 |-> BlackSPI.h
                 |-> BlackUART.cpp
                 |-> BlackUART.h
             |-> mySpiProject.cpp
      @endverbatim
     *  @n@n If BlackLib source files are located in your project like above example project file tree, you have to
     *  include BlackSPI.h or another source files with adding this line to your project file (mySpiProject.cpp at
     *  the example):
     *  @code{.cpp}
     *      #include "BlackLib/BlackSPI.h"
     *  @endcode
     *  @n@n If BlackLib source files are located at same level with your project file (mySpiProject.cpp at the
     *  example), you have to include BlackSPI.h or another source files with adding this line to your project file:
     *  @code{.cpp}
     *      #include "BlackSPI.h"
     *  @endcode
     *  @n @n
     *  @code{.cpp}
     *  // Filename: mySpiProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackSPI.h"
     *
     *  int main()
     *  {
     *      BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
     *
     *      mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock);
     *
     *      uint8_t writeArr[4] = { 0x55, 0xaa, 0xf0, 0x0c };
     *      uint8_t readArr[4];
     *
     *      mySpi.transfer(writeArr, readArr, sizeof(writeArr), 100);
     *
     *      return 0;
     *  }
     * @endcode
     * @n @n
     * You can use "using namespace BlackLib" also. You can get rid of writing "BlackLib::", with using this method.
     * @code{.cpp}
     *  // Filename: mySpiProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackSPI.h"
     *  using namespace BlackLib;
     *
     *  int main()
     *  {
     *      BlackSPI  mySpi(SPI0_0, 8, SpiDefault, 2400000);
     *
     *      mySpi.open( ReadWrite|NonBlock );
     *
     *      uint8_t writeArr[4] = { 0x55, 0xaa, 0xf0, 0x0c };
     *      uint8_t readArr[4];
     *
     *      mySpi.transfer(writeArr, readArr, sizeof(writeArr), 100);
     *
     *      return 0;
     *  }
     * @endcode
     */
    class BlackSPI : virtual private BlackCore
    {
        private:
            BlackSpiProperties constructorProperties;   /*!< @brief is used to hold the user specified properties of spi */
            BlackSpiProperties defaultProperties;       /*!< @brief is used to hold the default properties of spi */
            BlackSpiProperties currentProperties;       /*!< @brief is used to hold the current properties of spi */

            errorSPI        *spiErrors;                 /*!< @brief is used to hold the errors of BlackSPI class */

            std::string     spiPortPath;                /*!< @brief is used to hold the spi's tty port path */
            std::string     dtSpiFilename;              /*!< @brief is used to hold the spi's device tree overlay name */

            int             spiFD;                      /*!< @brief is used to hold the spi's tty file's file descriptor */
            int             spiBusNumber;               /*!< @brief is used to hold the spi's bus number */
            int             spiChipNumber;              /*!< @brief is used to hold the spi's chip number */
            bool            isCurrentEqDefault;         /*!< @brief is used to hold the properties of spi is equal to default properties */
            bool            isOpenFlag;                 /*!< @brief is used to hold the spi's tty file's state */

            /*! @brief Loads SPI overlay to device tree.
            *
            *  This function loads @b BlackSPI::dtSpiFilename overlay to device tree. This file name changes with
            *  spiName constructor parameter. This overlay performs pinmuxing and generates device driver.
            *  @warning BlackSPI class uses custom device tree overlays. Users must install these overlays firstly
            *  by executing setup script.
            *  @return True if successful, else false.
            */
            bool            loadDeviceTree();

            /*! @brief Finds SPI's port path and bus number.
            *
            *  The spi's TTY file name which located at "/dev/" directory, is differed to device tree overlay loading order.
            *  For example if users want to use both of spi bus and loads spi1 and then spi0 overlay to device tree, Beaglebone
            *  generates TTY file "spidev1.x" for spi1 and "spidev2.x" for spi0. But if users loads first spi0 and then spi1
            *  overlay to device tree, this time Beaglebone generates TTY file "spidev1.x" for spi0 and "spidev2.x" for spi1.
            *  For this reason which file associated with which spi, must find. This function does these operations exactly.
            *  @return True if successful, else false.
            */
            bool            findPortPath();


        public:
            /*!
            * This enum is used to define SPI debugging flags.
            */
            enum flags      {   dtErr       = 1,    /*!< enumeration for @a errorSPI::dtError status */
                                openErr     = 2,    /*!< enumeration for @a errorSPI::openError status */
                                closeErr    = 3,    /*!< enumeration for @a errorSPI::closeError status */
                                portPathErr = 4,    /*!< enumeration for @a errorSPI::portPathError status */
                                transferErr = 5,    /*!< enumeration for @a errorSPI::transferError status */
                                modeErr     = 6,    /*!< enumeration for @a errorSPI::modeError status */
                                speedErr    = 7,    /*!< enumeration for @a errorSPI::speedError status */
                                bitSizeErr  = 8     /*!< enumeration for @a errorSPI::bitSizeError status */
                            };

            /*! @brief Constructor of BlackSPI class.
            *
            * This function initializes errorSPI struct, sets local variables. Then calls device tree loading function
            * and port path finding function. Objects which are initialized from BlackSPI class with this constructor,
            * uses default line properties.
            *
            * @param [in] spi            name of spi (enum),(SPIx_y)
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0);
            *   BlackLib::BlackSPI  *mySpiPtr = new BlackLib::BlackSPI(BlackLib::SPI0_1);
            *
            *   mySpi.open( BlackLib::ReadWrite );
            *   mySpiPtr->open( BlackLib::ReadWrite );
            *
            * @endcode
            * @sa loadDeviceTree()
            * @sa findPortPath()
            * @sa spiName
            */
            BlackSPI(spiName spi);

            /*! @brief Constructor of BlackSPI class.
            *
            * This function initializes errorSPI struct, sets value of constructorProperties struct and local variables.
            * Then calls device tree loading function and port path finding function.
            *
            * @param [in] spi            name of spi (enum),(SPIx_y)
            * @param [in] spiProperties  import properties from outside
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSpiProperties mySpiProps(8, BlackLib::SpiDefault, 2400000);
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, mySpiProps);
            *   BlackLib::BlackSPI  *mySpiPtr = new BlackLib::BlackSPI(BlackLib::SPI0_1, mySpiProps);
            *
            *   mySpi.open( BlackLib::ReadWrite );
            *   mySpiPtr->open( BlackLib::ReadWrite );
            *
            * @endcode
            *
            * @sa loadDeviceTree()
            * @sa findPortPath()
            * @sa spiName
            * @sa BlackSpiProperties
            */
            BlackSPI(spiName spi, BlackSpiProperties spiProperties);

            /*! @brief Constructor of BlackSPI class.
            *
            * This function initializes errorSPI struct, sets value of constructorProperties struct and local variables.
            * Then calls device tree loading function and port path finding function.
            *
            * @param [in] spi               name of spi (enum),(SPIx_y)
            * @param [in] spiBitsPerWord    word size of spi
            * @param [in] spiMode           mode of spi
            * @param [in] spiSpeed          transfer speed of spi
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *   BlackLib::BlackSPI  *mySpiPtr = new BlackLib::BlackSPI(BlackLib::SPI0_1, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite );
            *   mySpiPtr->open( BlackLib::ReadWrite );
            *
            * @endcode
            *
            * @sa loadDeviceTree()
            * @sa findPortPath()
            * @sa spiName
            * @sa BlackSpiProperties::spiBitsPerWord
            * @sa BlackSpiProperties::spiMode
            * @sa BlackSpiProperties::spiSpeed
            */
            BlackSPI(spiName spi, uint8_t spiBitsPerWord, uint8_t spiMode, uint32_t spiSpeed);

            /*! @brief Destructor of BlackSPI class.
            *
            * This function closes TTY file and deletes errorSPI struct pointer.
            */
            virtual ~BlackSPI();

            /*! @brief Opens TTY file of spi.
            *
            * This function opens spi's TTY file with selected open mode, gets default properties of SPI
            * and saves this properties to BlackSPI::defaultProperties struct. Then sets properties
            * which are specified at class initialization stage. Users can send "or"ed BlackLib::openMode
            * enums as parameter to this function.
            * @warning After initialization of BlackSPI class, this function must call. Otherwise users
            * could not use any of data transfering functions.
            *
            * @param [in] openMode          file opening mode
            * @return True if tty file opening successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            * @endcode
            *
            * @sa openMode
            */
            bool            open(uint openMode);

            /*! @brief Closes TTY file of spi.
            *
            * This function closes spi's TTY file and changes isOpenFlag's value.
            * @return True if tty file closing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   mySpi.close();
            *
            * @endcode
            *
            * @sa BlackSPI::isOpenFlag
            */
            bool            close();

            /*! @brief Transfers byte to/from slave.
            *
            * This function creates temporary read variable and then generates <i><b> SPI IOCTL
            * TRANSFER PACKAGE </b></i> with write byte's address, temporary read variable's address,
            * delay time, spi word's size and spi speed parameters. After doing ioctl kernel request, if request is
            * finished successfully, data which is held in temporary read variable, is returned from this function.
            *
            * @param [in] writeByte            one byte data
            * @param [in] wait_us              delay time
            * @return If transfer operation is successful this function returns received byte data, else returns 0.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   uint8_t sendByte = 0x87;
            *   uint8_t recvByte = mySpi.transfer(sendByte, 100);
            *
            *   std::cout << "Loopback spi test result: 0x" << std::hex << (int)recvByte;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Loopback spi test result: 0x87
            * @endcode
            */
            uint8_t         transfer(uint8_t writeByte, uint16_t wait_us = 10);

            /*! @brief Transfers datas to/from slave.
            *
            * This function creates temporary read buffer with specified size and then generates <i><b> SPI IOCTL
            * TRANSFER PACKAGE </b></i> with write buffer pointer, temporary read buffer pointer, buffer size,
            * delay time, spi word's size and spi speed parameters. After doing ioctl kernel request if request is
            * finished successfully, datas which are held in temporary read buffer are copied to @a @b readBuffer
            * pointer.
            *
            * @param [in] writeBuffer          data buffer pointer
            * @param [out] readBuffer          read buffer pointer
            * @param [in] bufferSize           buffer size
            * @param [in] wait_us              delay time
            * @return true if transfer operation successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   uint8_t sendBytes[4] = { 0x87, 0x41, 0xF1, 0x5A };
            *   uint8_t recvBytes[4];
            *   mySpi.transfer(sendBytes, recvBytes, sizeof(sendBytes), 100);
            *
            *   std::cout << "Loopback spi test result 0: 0x" << std::hex << (int)recvBytes[0] << std::endl
            *             << "Loopback spi test result 1: 0x" << std::hex << (int)recvBytes[1] << std::endl
            *             << "Loopback spi test result 2: 0x" << std::hex << (int)recvBytes[2] << std::endl
            *             << "Loopback spi test result 3: 0x" << std::hex << (int)recvBytes[3] << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Loopback spi test result 0: 0x87
            *   // Loopback spi test result 1: 0x41
            *   // Loopback spi test result 2: 0xF1
            *   // Loopback spi test result 3: 0x5A
            * @endcode
            */
            bool            transfer(uint8_t *writeBuffer, uint8_t *readBuffer, size_t bufferSize, uint16_t wait_us = 10);


            /*! @brief Changes word size of spi.
            *
            * This function changes bits per word size of spi on fly.
            *
            * @param [in] newBitSize         new word size
            * @return true if changing operation is successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Current bits per word size: " << (int)mySpi.getBitsPerWord() << std::endl;
            *   mySpi.setBitsPerWord(10);
            *   std::cout << "Current bits per word size: " << (int)mySpi.getBitsPerWord();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current bits per word size: 8
            *   // Current bits per word size: 10
            * @endcode
            *
            * @sa BlackSpiProperties::spiBitsPerWord
            */
            bool            setBitsPerWord(uint8_t newBitSize);

            /*! @brief Changes maximum speed of spi.
            *
            * This function changes maximum speed of spi on fly.
            *
            * @param [in] newSpeed         new speed value
            * @return true if changing operation is successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Current speed: " << mySpi.getMaximumSpeed() << " Hz\n";
            *   mySpi.setMaximumSpeed(18000000);
            *   std::cout << "Current speed: " << mySpi.getMaximumSpeed() << " Hz";
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current speed: 2400000 Hz
            *   // Current speed: 18000000 Hz
            * @endcode
            *
            * @sa BlackSpiProperties::spiSpeed
            */
            bool            setMaximumSpeed(uint32_t newSpeed);

            /*! @brief Changes mode of spi.
            *
            * This function changes mode of spi on fly. Users can send "or"ed BlackLib::spiMode
            * enums as parameter to this function.
            *
            * @param [in] newMode         new mode value
            * @return true if changing operation is successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   // 0 means SpiDefault. See the BlackLib::spiMode enums.
            *   std::cout << "Current mode: " << (int)mySpi.getMode() << std::endl;
            *
            *   mySpi.setMode( BlackLib::SpiPolarity | BlackLib::SpiPhase );
            *
            *   // 3 means SpiPolarity and SpiPhase. See the BlackLib::spiMode enums.
            *   std::cout << "Current mode: " << (int)mySpi.getMode();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current mode: 0
            *   // Current mode: 3
            * @endcode
            *
            * @sa BlackSpiProperties::spiMode
            * @sa spiMode
            */
            bool            setMode(uint8_t newMode);

            /*! @brief Changes properties of spi.
            *
            * This function changes properties of spi. These properties are composed of word size,
            * speed and mode.
            *
            * @param [in] &newProperties        new properties of spi
            * @return true if changing operation is successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   BlackLib::BlackSpiProperties currentProps = mySpi.getProperties();
            *
            *   std::cout << "First bits per word size: " << (int)currentProps.spiBitsPerWord << std::endl
            *             << "First mode              : " << (int)currentProps.spiMode        << std::endl
            *             << "First speed             : " << currentProps.spiSpeed       << std::endl;
            *
            *   currentProps = BlackLib::BlackSpiProperties(10, BlackLib::SpiPolarity | BlackLib::SpiPhase, 18000000);
            *
            *   mySpi.setProperties(currentProps);
            *
            *   std::cout << "Second bits per word size: " << (int)currentProps.spiBitsPerWord << std::endl
            *             << "Second mode              : " << (int)currentProps.spiMode        << std::endl
            *             << "Second speed             : " << currentProps.spiSpeed       << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // First bits per word size: 8
            *   // First mode              : 0
            *   // First speed             : 2400000
            *   // Second bits per word size: 10
            *   // Second mode              : 3
            *   // Second speed             : 18000000
            * @endcode
            *
            * @sa BlackSpiProperties
            */
            bool            setProperties(BlackSpiProperties &newProperties);

            /*! @brief Exports spi's port path.
            *
            * @return spi's port path as string.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Port path: " << mySpi.getPortName();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Port path: /dev/spidev1.0
            * @endcode
            *
            * @sa spiPortPath
            */
            string          getPortName();

            /*! @brief Exports word size of spi.
            *
            * @return bits per word size of spi
            *
            * @par Example
            *   Example usage is shown in BlackSPI::setBitsPerWord() function's example.
            *
            * @sa BlackSpiProperties::spiBitsPerWord
            */
            uint8_t         getBitsPerWord();

            /*! @brief Exports maximum speed of spi.
            *
            * @return maximum speed of spi
            *
            * @par Example
            *   Example usage is shown in BlackSPI::setMaximumSpeed() function's example.
            *
            * @sa BlackSpiProperties::spiSpeed
            */
            uint32_t        getMaximumSpeed();

            /*! @brief Exports mode of spi.
            *
            * @return mode of spi
            *
            * @par Example
            *   Example usage is shown in BlackSPI::setMode() function's example.
            *
            * @sa BlackSpiProperties::spiMode
            * @sa BlackLib::spiMode
            */
            uint8_t         getMode();

            /*! @brief Exports properties of spi.
            *
            * This function gets properties of spi. These properties are composed of word size,
            * speed and mode.
            *
            * @return BlackSPI::currentProperties struct with updated values.
            *
            * @par Example
            *   Example usage is shown in BlackSPI::setProperties() function's example.
            *
            * @sa BlackSpiProperties
            */
            BlackSpiProperties getProperties();

            /*! @brief Checks spi's tty file's open state.
            *
            * @return true if tty file is open, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *
            *   std::cout << "Is open?: " << std::boolalpha << mySpi.isOpen() << std::endl;
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Is open?: " << std::boolalpha << mySpi.isOpen();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Is open?: false
            *   // Is open?: true
            * @endcode
            *
            * @sa BlackSPI::isOpenFlag
            */
            bool            isOpen();

            /*! @brief Checks spi's tty file's close state.
            *
            * @return true if tty file is close, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *
            *   std::cout << "Is close?: " << std::boolalpha << mySpi.isClose() << std::endl;
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Is close?: " << std::boolalpha << mySpi.isClose();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Is close?: true
            *   // Is close?: false
            * @endcode
            *
            * @sa BlackSPI::isOpenFlag
            */
            bool            isClose();

            /*! @brief Is used for general debugging.
            *
            * @return True if any error occured, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   if( mySpi.fail() )
            *   {
            *       std::cout << "ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "EVERYTHING IS OK" << std::endl;
            *   }
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // EVERYTHING IS OK
            * @endcode
            *
            * @sa errorSPI
            */
            bool            fail();

            /*! @brief Is used for specific debugging.
            *
            * You can use this function, after call BlackSPI member functions in your code. The
            * input parameter is used for finding out status of selected error.
            * @param [in] f specific error type (enum)
            * @return Value of @a selected error.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackSPI  mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiDefault, 2400000);
            *
            *
            *   if( mySpi.fail(BlackLib::BlackSPI::dtErr) or mySpi.fail(BlackLib::BlackSPI::portPathErr) )
            *   {
            *       std::cout << "BlackSPI INITIALIZATION FAILED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "BlackSPI INITIALIZATION IS OK" << std::endl;
            *   }
            *
            *
            *   mySpi.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   if( mySpi.fail(BlackLib::BlackSPI::openErr) )
            *   {
            *       std::cout << "OPENNING ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "OPENNING IS OK" << std::endl;
            *   }
            *
            *
            *   uint8_t sendBytes[4] = { 0x87, 0x41, 0xF1, 0x5A };
            *   uint8_t recvBytes[4];
            *   mySpi.transfer(sendBytes, recvBytes, sizeof(sendBytes), 100);
            *
            *   if( mySpi.fail(BlackLib::BlackSPI::transferErr) )
            *   {
            *       if( mySpi.fail(BlackLib::BlackSPI::openErr) )
            *       {
            *           std::cout << "TRANSFER ERROR OCCURED BECAUSE SPI TTY IS NOT OPEN" << std::endl;
            *       }
            *       else
            *       {
            *           std::cout << "TRANSFER ERROR OCCURED" << std::endl;
            *       }
            *   }
            *   else
            *   {
            *       std::cout << "TRANSFER IS OK" << std::endl;
            *   }
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // BlackSPI INITIALIZATION IS OK
            *   // OPENNING IS OK
            *   // TRANSFER IS OK
            * @endcode
            *
            * @sa errorSPI
            */
            bool            fail(BlackSPI::flags f);

    };
    // ########################################### BLACKSPI DECLARATION ENDS ############################################## //



// ######################################### BLACKUARTPROPERTIES DECLARATION STARTS ######################################### //

    /*! @brief Holds properties of UART.
     *
     *    This struct holds input and output baud rate, parity, stop bits size and character size
     *    properties of UART. Also it has overloaded constructors and overloaded assign operator.
     *    @sa BlackLib::baudRate
     *    @sa BlackLib::parity
     *    @sa BlackLib::stopBits
     *    @sa BlackLib::characterSize
     */
    struct BlackUartProperties
    {
        baudRate        uartBaudIn;     /*!< @brief is used to hold the baud rate of UART RX */
        baudRate        uartBaudOut;    /*!< @brief is used to hold the baud rate of UART TX */
        parity          uartParity;     /*!< @brief is used to hold the parity type of UART */
        stopBits        uartStopBits;   /*!< @brief is used to hold the stop bits size of UART */
        characterSize   uartCharSize;   /*!< @brief is used to hold the character size of UART */

        /*! @brief Default constructor of BlackUartProperties struct.
         *
         *  This function sets default value to variables.
         */
        BlackUartProperties()
        {
            uartBaudIn      = Baud9600;
            uartBaudOut     = Baud9600;
            uartParity      = ParityDefault;
            uartStopBits    = StopDefault;
            uartCharSize    = CharDefault;
        }

        /*! @brief Overloaded constructor of BlackUartProperties struct.
         *
         *  This function sets input arguments to variables.
         * @sa BlackLib::baudRate
         * @sa BlackLib::parity
         * @sa BlackLib::stopBits
         * @sa BlackLib::characterSize
         */
        BlackUartProperties(baudRate S_baudIn, baudRate S_baudOut, parity S_parity, stopBits S_stopBits, characterSize S_charSize)
        {
            uartBaudIn      = S_baudIn;
            uartBaudOut     = S_baudOut;
            uartParity      = S_parity;
            uartStopBits    = S_stopBits;
            uartCharSize    = S_charSize;
        }

        /*! @brief Overloaded constructor of BlackUartProperties struct.
         *
         *  This function gets pointer of BlackUartProperties struct and sets input
         *  struct's variables to own variables.
         */
        BlackUartProperties( BlackUartProperties *S_properties )
        {
            uartBaudIn      = S_properties->uartBaudIn;
            uartBaudOut     = S_properties->uartBaudOut;
            uartParity      = S_properties->uartParity;
            uartStopBits    = S_properties->uartStopBits;
            uartCharSize    = S_properties->uartCharSize;
        }


        /*! @brief Overloaded assign operator of BlackUartProperties struct.
         *
         *  This function assigns input struct's variables to own variables.
         */
        BlackUartProperties& operator=(BlackUartProperties equal)
        {
            uartBaudIn      = equal.uartBaudIn;
            uartBaudOut     = equal.uartBaudOut;
            uartCharSize    = equal.uartCharSize;
            uartParity      = equal.uartParity;
            uartStopBits    = equal.uartStopBits;

            return *this;
        }
    };
    // ########################################## BLACKUARTPROPERTIES DECLARATION ENDS ########################################### //







    // ########################################### BLACKUART DECLARATION STARTS ############################################ //

    /*! @brief Interacts with end user, to use UART.
     *
     *    This class is end node to use uart. End users interact with uart from this class.
     *    It includes public functions to set and get properties of uart's and to read,
     *    write and transfer datas.
     *
     *    @par Example
      @verbatim
      EXAMPLE PROJECT FILE TREE:
         myUartProject
         |-> src
             |-> BlackLib
                 |-> BlackADC.cpp
                 |-> BlackADC.h
                 |-> BlackCore.cpp
                 |-> BlackCore.h
                 |-> BlackDef.h
                 |-> BlackErr.h
                 |-> BlackGPIO.cpp
                 |-> BlackGPIO.h
                 |-> BlackI2C.cpp
                 |-> BlackI2C.h
                 |-> BlackLib.h
                 |-> BlackPWM.cpp
                 |-> BlackPWM.h
                 |-> BlackSPI.cpp
                 |-> BlackSPI.h
                 |-> BlackUART.cpp
                 |-> BlackUART.h
             |-> myUartProject.cpp
      @endverbatim
     *  @n@n If BlackLib source files are located in your project like above example project file tree, you have to
     *  include BlackUART.h or another source files with adding this line to your project file (myUartProject.cpp at
     *  the example):
     *  @code{.cpp}
     *      #include "BlackLib/BlackUART.h"
     *  @endcode
     *  @n@n If BlackLib source files are located at same level with your project file (myUartProject.cpp at the
     *  example), you have to include BlackUART.h or another source files with adding this line to your project file:
     *  @code{.cpp}
     *      #include "BlackUART.h"
     *  @endcode
     *  @n @n
     *  @code{.cpp}
     *  // Filename: myUartProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackLib/BlackUART.h"
     *
     *  int main()
     *  {
     *      BlackLib::BlackUART  myUart(BlackLib::UART1,
     *                                  BlackLib::Baud9600,
     *                                  BlackLib::ParityEven,
     *                                  BlackLib::StopOne,
     *                                  BlackLib::Char8 );
     *
     *      myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock);
     *
     *      std::string testMessage = "This is uart test message.";
     *      std::cout << myUart.transfer(testMessage, 40000);
     *
     *      return 0;
     *  }
     * @endcode
     * @n @n
     * You can use "using namespace BlackLib" also. You can get rid of writing "BlackLib::", with using this method.
     * @code{.cpp}
     *  // Filename: myUartProject.cpp
     *  // Author:   Yiğit Yüce - ygtyce@gmail.com
     *
     *  #include <iostream>
     *  #include "BlackUART.h"
     *  using namespace BlackLib;
     *
     *  int main()
     *  {
     *      BlackUART  myUart(UART1,
     *                        Baud9600,
     *                        ParityEven,
     *                        StopOne,
     *                        Char8 );
     *
     *      myUart.open( ReadWrite | NonBlock);
     *
     *      std::string testMessage = "This is uart test message.";
     *      std::cout << myUart.transfer(testMessage, 40000);
     *
     *      return 0;
     *  }
     * @endcode
     *
     */
    class BlackUART : virtual private BlackCore
    {

        private:
            BlackUartProperties defaultUartProperties;      /*!< @brief is used to hold the default properties of uart */
            BlackUartProperties currentUartProperties;      /*!< @brief is used to hold the current properties of uart */
            BlackUartProperties constructorProperties;      /*!< @brief is used to hold the user specified properties of uart */

            errorUART       *uartErrors;                    /*!< @brief is used to hold the errors of BlackUART class */

            std::string     dtUartFilename;                 /*!< @brief is used to hold the uart's device tree overlay name */
            std::string     uartPortPath;                   /*!< @brief is used to hold the uart's tty port path */

            uint32_t        readBufferSize;                 /*!< @brief is used to hold the size of temporary buffer */
            int             uartFD;                         /*!< @brief is used to hold the uart's tty file's file descriptor */
            bool            isOpenFlag;                     /*!< @brief is used to hold the uart's tty file's state */
            bool            isCurrentEqDefault;             /*!< @brief is used to hold the properties of uart is equal to default properties */

            /*! @brief Loads UART overlay to device tree.
            *
            *  This function loads @b BlackUART::dtUartFilename named overlay to device tree.
            *  This file name changes with uartName constructor parameter.
            *  This overlay performs pinmuxing and generates device driver.
            *  @return True if successful, else false.
            */
            bool            loadDeviceTree();


        public:
            /*!
            * This enum is used to define UART debugging flags.
            */
            enum flags      {   dtErr       = 1,    /*!< enumeration for @a errorUART::dtError status */
                                openErr     = 2,    /*!< enumeration for @a errorUART::openError status */
                                closeErr    = 3,    /*!< enumeration for @a errorUART::closeError status */
                                directionErr= 4,    /*!< enumeration for @a errorUART::directionError status */
                                flushErr    = 5,    /*!< enumeration for @a errorUART::flushError status */
                                readErr     = 6,    /*!< enumeration for @a errorUART::readError status */
                                writeErr    = 7,    /*!< enumeration for @a errorUART::writeError status */
                                baudRateErr = 8,    /*!< enumeration for @a errorUART::baudRateError status */
                                parityErr   = 9,    /*!< enumeration for @a errorUART::parityError status */
                                stopBitsErr = 10,   /*!< enumeration for @a errorUART::stopBitsError status */
                                charSizeErr = 11    /*!< enumeration for @a errorUART::charSizeError status */
                            };

            /*! @brief Constructor of BlackUART class.
            *
            * This function initializes errorUART struct, sets value of constructorProperties struct and local variables.
            * Then calls device tree loading function.
            *
            * @param [in] uart          name of uart (enum),(UARTx)
            * @param [in] uartBaud      baud rate of uart (enum)
            * @param [in] uartParity    parity of uart (enum)
            * @param [in] uartStopBits  stop bits of uart (enum)
            * @param [in] uartCharSize  character size of uart (enum)
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   BlackLib::BlackUART  *myUartPtr = new BlackLib::BlackUART(BlackLib::UART4,
            *                                                             BlackLib::Baud9600,
            *                                                             BlackLib::ParityEven,
            *                                                             BlackLib::StopOne,
            *                                                             BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite );
            *   myUartPtr->open( BlackLib::ReadWrite );
            *
            * @endcode
            *
            * @sa loadDeviceTree()
            * @sa uartName
            * @sa baudRate
            * @sa parity
            * @sa stopBits
            * @sa characterSize
            */
            BlackUART(uartName uart, baudRate uartBaud, parity uartParity, stopBits uartStopBits, characterSize uartCharSize);

            /*! @brief Constructor of BlackUART class.
            *
            * This function initializes errorUART struct, sets value of constructorProperties struct and local variables.
            * Then calls device tree loading function.
            *
            * @param [in] uart            name of uart (enum),(UARTx)
            * @param [in] uartProperties  import properties from outside
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUartProperties myUartProps(BlackLib::Baud9600,
            *                                             BlackLib::Baud9600,
            *                                             BlackLib::ParityEven,
            *                                             BlackLib::StopOne,
            *                                             BlackLib::Char8);
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1, myUartProps);
            *   BlackLib::BlackUART  *myUartPtr = new BlackLib::BlackUART(BlackLib::UART4, myUartProps);
            *
            *   myUart.open( BlackLib::ReadWrite );
            *   myUartPtr->open( BlackLib::ReadWrite );
            *
            * @endcode
            *
            * @sa loadDeviceTree()
            * @sa uartName
            * @sa BlackUartProperties
            */
            BlackUART(uartName uart, BlackUartProperties uartProperties);

            /*! @brief Constructor of BlackUART class.
            *
            * This function initializes errorUART struct, sets local variables. Then calls device tree loading function.
            * Objects which are initialized from BlackUART class with this constructor, uses default uart properties.
            *
            * @param [in] uart            name of uart (enum),(UARTx)
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1);
            *   BlackLib::BlackUART  *myUartPtr = new BlackLib::BlackUART(BlackLib::UART4);
            *
            *   myUart.open( BlackLib::ReadWrite );
            *   myUartPtr->open( BlackLib::ReadWrite );
            *
            * @endcode
            * @sa loadDeviceTree()
            * @sa uartName
            */
            BlackUART(uartName uart);

            /*! @brief Destructor of BlackUART class.
            *
            * This function closes TTY file and deletes errorUART struct pointer.
            */
            virtual ~BlackUART();

            /*! @brief Opens TTY file of uart.
            *
            * This function opens uart's TTY file with selected open mode, gets default properties of UART
            * and saves this properties to BlackUART::defaultUartProperties struct. Then sets properties
            * which are specified at class initialization stage. Users can send "or"ed BlackLib::openMode
            * enums as parameter to this function.
            * @warning After initialization of BlackUART class, this function must call. Otherwise users
            * could not use any of data sending or receiving functions.
            *
            * @param [in] openMode          file opening mode
            * @return True if tty file opening successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            * @endcode
            * @sa openMode
            */
            bool            open(uint openMode);

            /*! @brief Closes TTY file of uart.
            *
            * This function closes uart's TTY file and changes BlackUART::isOpenFlag's value.
            * @return True if tty file closing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.close();
            *
            * @endcode
            * @sa BlackUART::isOpenFlag
            */
            bool            close();

            /*! @brief Flushes uart line.
            *
            * This function flushes uart line at specified direction.
            * @param [in] whichDirection flushing direction
            * @return True if flushing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            * @endcode
            * @sa direction
            */
            bool            flush(direction whichDirection);

            /*! @brief Reads values from uart line.
            *
            * This function reads values from uart line and returns read value as string.
            * It creates temporary read buffer with specified size and reads values to this
            * buffer. Then resize buffer to read value size. BlackUART::readBufferSize variable
            * is used to specify temporary buffer size.
            *
            * @return read value if reading successful, else returns BlackLib::UART_READ_FAILED string.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   std::string writeToUart  = "this is test.\n";
            *   myUart.write(writeToUart);
            *
            *   sleep(1);
            *
            *   std::string readFromUart = myUart.read();
            *
            *   std::cout << "Test output on loopback: " << readFromUart;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            * @sa BlackUART::readBufferSize
            */
            std::string     read();

            /*! @brief Reads values from uart line.
            *
            * This function reads values from uart line and saves read value to @a @b readBuffer pointer.
            * It creates temporary read buffer with specified size and reads values to this
            * buffer. Then copies buffer to  @a @b readBuffer pointer.
            *
            * @param [out] readBuffer          buffer pointer
            * @param [in] size                 buffer size
            * @return true if reading successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   std::string writeToUart  = "this is test.\n";
            *   myUart.write(writeToUart);
            *
            *   sleep(1);
            *
            *   char readBuffer[14];
            *   myUart.read(readBuffer, sizeof(readBuffer));
            *
            *   std::cout << "Test output on loopback: " << readBuffer;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            */
            bool            read(char *readBuffer, size_t size);

            /*! @brief Writes values to uart line.
            *
            * This function writes values to uart line. Values sent to this function as string type.
            *
            * @param [in] writeBuffer          values buffer
            * @return true if writing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   std::string writeToUart  = "this is test.\n";
            *   myUart.write(writeToUart);
            *
            *   sleep(1);
            *
            *   std::string readFromUart = myUart.read();
            *
            *   std::cout << "Test output on loopback: " << readFromUart;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            */
            bool            write(std::string writeBuffer);

            /*! @brief Writes values to uart line.
            *
            * This function writes values to uart line. Values sent to this function as c-style string(char array).
            *
            * @param [in] writeBuffer          values buffer
            * @param [in] size                 buffer size
            * @return true if writing successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   char writeBuffer[]  = "this is test.\n";
            *   myUart.write(writeBuffer, sizeof(writeBuffer));
            *
            *   sleep(1);
            *
            *   std::string readFromUart = myUart.read();
            *
            *   std::cout << "Test output on loopback: " << readFromUart;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            */
            bool            write(char *writeBuffer, size_t size);

            /*! @brief Writes and reads values sequentially to/from uart line.
            *
            * This function writes values to uart line firstly and then reads values from uart line and saves read
            * value to @a @b readBuffer pointer. It creates temporary read buffer with specified size and reads
            * values to this buffer. Then it copies buffer to  @a @b readBuffer pointer. This function waits between
            * writing and reading operations.
            *
            * @param [in] writeBuffer          values buffer
            * @param [out] readBuffer          read buffer pointer
            * @param [in] size                 buffer size
            * @param [in] wait_us              sleep time between writing and reading
            * @return true if transfering successful, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   char writeBuffer[]  = "this is test.\n";
            *   char readBuffer[ myUart.getReadBufferSize() ];
            *
            *   myUart.transfer(writeBuffer, readBuffer, sizeof(writeBuffer), 40000);
            *
            *   std::cout << "Test output on loopback: " << readBuffer;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            * @sa BlackUART::readBufferSize
            */
            bool            transfer(char *writeBuffer, char *readBuffer, size_t size, uint32_t wait_us);

            /*! @brief Writes and reads values sequentially to/from uart line.
            *
            * This function writes values to uart line firstly and then reads values from uart line and returns read
            * value as string. It creates temporary read buffer with specified size and reads values to this buffer.
            * Then resize buffer to read value size. BlackUART::readBufferSize variable is used to specify temporary
            * buffer size.
            *
            * @param [in] writeBuffer          write buffer
            * @param [in] wait_us              sleep time between writing and reading
            * @return read value if reading successful, else returns BlackLib::UART_READ_FAILED or
            * BlackLib::UART_WRITE_FAILED string.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   std::string writeToUart = "this is test.\n";
            *   std::string readFromUart;
            *
            *   readFromUart = myUart.transfer(writeToUart, 40000);
            *
            *   std::cout << "Test output on loopback: " << readFromUart;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            * @sa BlackUART::readBufferSize
            */
            std::string     transfer(std::string writeBuffer, uint32_t wait_us);

            /*! @brief Changes internal temporary buffers' sizes.
            *
            * This function changes internal buffers' sizes which are used at read and transfer operations.
            * This buffer size must be maximum possible read value size. Otherwise read value will truncate.
            *
            * @param [in] newBufferSize        new size of the temporary internal buffer
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   std::cout << "Current temporary buffer size: " << myUart.getReadBufferSize() << std::endl;
            *   myUart.setReadBufferSize(2048);
            *   std::cout << "Current temporary buffer size: " << myUart.getReadBufferSize();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current temporary buffer size: 1024
            *   // Current temporary buffer size: 2048
            * @endcode
            * @sa BlackUART::readBufferSize
            */
            void            setReadBufferSize(uint32_t newBufferSize);

            /*! @brief Changes baud rate of uart.
            *
            * This function changes baud rate of uart at specified direction. Also users can select apply
            * condition like ApplyNow, ApplyDrain, ApplyFlush.
            *
            * @param [in] newBaud         new baud rate value
            * @param [in] whichDirection  direction
            * @param [in] applyMode       new value's apply condition
            * @return true if changing operation is successful, else false.
            * @warning Before use this function, users must be called open function. If uart is not open,
            * this function returns false and sets errorUART::baudRateError and errorUART::openError flags.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   // 13 means Baud9600. See the BlackLib::baudRate enums.
            *   std::cout << "Current baud rate: " << myUart.getBaudRate(BlackLib::output) << std::endl;
            *
            *   myUart.setBaudRate(BlackLib::Baud19200, BlackLib::output, BlackLib::ApplyNow);
            *
            *   // 14 means Baud19200. See the BlackLib::baudRate enums.
            *   std::cout << "Current baud rate: " << myUart.getBaudRate(BlackLib::output);
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current baud rate: 13
            *   // Current baud rate: 14
            * @endcode
            *
            * @sa baudRate
            * @sa direction
            * @sa uartApplyMode
            */
            bool            setBaudRate(baudRate newBaud, direction whichDirection, uartApplyMode applyMode = ApplyNow );

            /*! @brief Changes parity of uart.
            *
            * This function changes parity of uart. Also users can select apply condition like ApplyNow,
            * ApplyDrain, ApplyFlush.
            *
            * @param [in] newParity       new parity value
            * @param [in] applyMode       new value's apply condition
            * @return true if changing operation is successful, else false.
            *
            * @warning Before use this function, users must be called open function. If uart is not open,
            * this function returns false and sets errorUART::parityError and errorUART::openError flags.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   // 2 means ParityEven. See the BlackLib::parity enums.
            *   std::cout << "Current parity: " << myUart.getParity() << std::endl;
            *
            *   myUart.setParity(BlackLib::ParityOdd, BlackLib::ApplyNow);
            *
            *   // 1 means ParityOdd. See the BlackLib::parity enums.
            *   std::cout << "Current parity: " << myUart.getParity();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current parity: 2
            *   // Current parity: 1
            * @endcode
            *
            * @sa parity
            * @sa uartApplyMode
            */
            bool            setParity(parity newParity, uartApplyMode applyMode = ApplyNow );

            /*! @brief Changes stop bits size of uart.
            *
            * This function changes stop bits size of uart. Also users can select apply condition like ApplyNow,
            * ApplyDrain, ApplyFlush.
            *
            * @param [in] newStopBits     new stop bits size value
            * @param [in] applyMode       new value's apply condition
            * @return true if changing operation is successful, else false.
            *
            * @warning Before use this function, users must be called open function. If uart is not open,
            * this function returns false and sets errorUART::stopBitsError and errorUART::openError flags.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   // 1 means StopOne. See the BlackLib::stopBits enums.
            *   std::cout << "Current stop bits size: " << myUart.getStopBits() << std::endl;
            *
            *   myUart.setStopBits(BlackLib::StopTwo, BlackLib::ApplyNow);
            *
            *   // 2 means StopTwo. See the BlackLib::stopBits enums.
            *   std::cout << "Current stop bits size: " << myUart.getStopBits();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current stop bits size: 1
            *   // Current stop bits size: 2
            * @endcode
            *
            * @sa stopBits
            * @sa uartApplyMode
            */
            bool            setStopBits(stopBits newStopBits, uartApplyMode applyMode = ApplyNow );

            /*! @brief Changes character size of uart.
            *
            * This function changes character size of uart. Also users can select apply condition like ApplyNow,
            * ApplyDrain, ApplyFlush.
            *
            * @param [in] newCharacterSize  new character size value
            * @param [in] applyMode         new value's apply condition
            * @return true if changing operation is successful, else false.
            *
            * @warning Before use this function, users must be called open function. If uart is not open,
            * this function returns false and sets errorUART::charSizeError and errorUART::openError flags.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   // 8 means Char8. See the BlackLib::characterSize enums.
            *   std::cout << "Current character size: " << myUart.getCharacterSize() << std::endl;
            *
            *   myUart.setCharacterSize(BlackLib::Char7, BlackLib::ApplyNow);
            *
            *   // 7 means Char7. See the BlackLib::characterSize enums.
            *   std::cout << "Current character size: " << myUart.getCharacterSize();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Current character size: 8
            *   // Current character size: 7
            * @endcode
            *
            * @sa characterSize
            * @sa uartApplyMode
            */
            bool            setCharacterSize(characterSize newCharacterSize, uartApplyMode applyMode = ApplyNow );

            /*! @brief Changes properties of uart.
            *
            * This function changes properties of uart. Also users can select apply condition like ApplyNow,
            * ApplyDrain, ApplyFlush. These properties are composed of baud rate, parity, stop bits size and
            * character size.
            *
            * @param [in] &props        new properties
            * @param [in] applyMode     new properties' apply condition
            * @return true if changing operation is successful, else false.
            *
            * @warning Before use this function, users must be called open function. If uart is not open,
            * this function returns false and sets errorUART::baudRateError, errorUART::charSizeError,
            * errorUART::parityError, errorUART::stopBitsError and errorUART::openError flags.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   BlackLib::BlackUartProperties currentProps = myUart.getProperties();
            *
            *   std::cout << "First bauds(in/out) : " << currentProps.uartBaudIn   << "/" << currentProps.uartBaudIn << std::endl
            *             << "First parity        : " << currentProps.uartParity   << std::endl
            *             << "First stop bits size: " << currentProps.uartStopBits << std::endl
            *             << "First character size: " << currentProps.uartCharSize << std::endl;
            *
            *   currentProps = BlackLib::BlackUartProperties(BlackLib::Baud19200, BlackLib::Baud19200,
            *                                                BlackLib::ParityOdd, BlackLib::StopTwo, BlackLib::Char7);
            *
            *   myUart.setProperties(currentProps, BlackLib::ApplyNow);
            *
            *   std::cout << "Second bauds(in/out) : " << currentProps.uartBaudIn   << "/" << currentProps.uartBaudIn << std::endl
            *             << "Second parity        : " << currentProps.uartParity   << std::endl
            *             << "Second stop bits size: " << currentProps.uartStopBits << std::endl
            *             << "Second character size: " << currentProps.uartCharSize << std::endl;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // First bauds(in/out) : 13/13
            *   // First parity        : 2
            *   // First stop bits size: 1
            *   // First character size: 8
            *   // Second bauds(in/out) : 14/14
            *   // Second parity        : 1
            *   // Second stop bits size: 2
            *   // Second character size: 7
            * @endcode
            *
            * @sa BlackUartProperties
            * @sa uartApplyMode
            */
            bool            setProperties(BlackUartProperties &props, uartApplyMode applyMode = ApplyNow );

            /*! @brief Exports properties of uart.
            *
            * This function gets properties of uart. These properties are composed of baud rate, parity, stop bits size and
            * character size.
            *
            * @return BlackUART::currentUartProperties struct with updated values.
            *
            * @par Example
            *   Example usage is shown in BlackUART::setProperties() function's example.
            *
            * @sa BlackUartProperties
            */
            BlackUartProperties getProperties();

            /*! @brief Exports uart's port path.
            *
            * @return uart's port path as string.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Port path: " << myUart.getPortName();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Port path: /dev/ttyO1
            * @endcode
            *
            * @sa uartPortPath
            */
            std::string     getPortName();

            /*! @brief Exports internal temporary buffers' size.
            *
            * @return size of internal temporary buffers.
            *
            * @par Example
            *   Example usage is shown in BlackUART::setReadBufferSize() function's example.
            *
            * @sa readBufferSize
            */
            uint32_t        getReadBufferSize();

            /*! @brief Exports baud rate value of uart.
            *
            * This function Exports baud rate value of uart at specified direction.
            *
            * @param [in] whichDirection  direction
            * @return baud rate value of uart.
            *
            * @par Example
            *   Example usage is shown in BlackUART::setBaudRate() function's example.
            *
            * @sa baudRate
            * @sa direction
            */
            baudRate        getBaudRate(direction whichDirection);

            /*! @brief Exports parity value of uart.
            *
            * @return parity value of uart.
            *
            * @par Example
            *   Example usage is shown in BlackUART::setParity() function's example.
            *
            * @sa parity
            */
            parity          getParity();

            /*! @brief Exports stop bits size value of uart.
            *
            * @return stop bits size value of uart.
            *
            * @par Example
            *   Example usage is shown in BlackUART::setStopBits() function's example.
            *
            * @sa stopBits
            */
            stopBits        getStopBits();

            /*! @brief Exports character size value of uart.
            *
            * @return character size value of uart.
            *
            * @par Example
            *   Example usage is shown in BlackUART::setCharacterSize() function's example.
            *
            * @sa characterSize
            */
            characterSize   getCharacterSize();

            /*! @brief Checks uart's tty file's open state.
            *
            * @return true if tty file is open, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   std::cout << "Is open?: " << std::boolalpha << myUart.isOpen() << std::endl;
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Is open?: " << std::boolalpha << myUart.isOpen();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Is open?: false
            *   // Is open?: true
            * @endcode
            *
            * @sa BlackUART::isOpenFlag
            */
            bool            isOpen();

            /*! @brief Checks uart's tty file's close state.
            *
            * @return true if tty file is close, else false.
            *
            * @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   std::cout << "Is close?: " << std::boolalpha << myUart.isClose() << std::endl;
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *
            *   std::cout << "Is close?: " << std::boolalpha << myUart.isClose();
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Is close?: true
            *   // Is close?: false
            * @endcode
            *
            * @sa BlackUART::isOpenFlag
            */
            bool            isClose();

            /*! @brief Is used for general debugging.
            *
            * @return True if any error occured, else false.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite );
            *
            *   if( myUart.fail() )
            *   {
            *       std::cout << "ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "EVERYTHING IS OK" << std::endl;
            *   }
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // EVERYTHING IS OK
            * @endcode
            *
            * @sa errorUART
            */
            bool            fail();

            /*! @brief Is used for specific debugging.
            *
            * You can use this function, after call BlackUART member functions in your code. The
            * input parameter is used for finding out status of selected error.
            * @param [in] f specific error type (enum)
            * @return Value of @a selected error.
            *
            * @par Example
            *  @code{.cpp}
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *
            *   if( myUart.fail(BlackLib::BlackUART::dtErr) )
            *   {
            *       std::cout << "BlackUART INITIALIZATION FAILED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "BlackUART INITIALIZATION IS OK" << std::endl;
            *   }
            *
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock  );
            *
            *   if( myUart.fail(BlackLib::BlackUART::openErr) )
            *   {
            *       std::cout << "OPENNING ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "OPENNING IS OK" << std::endl;
            *   }
            *
            *
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   if( myUart.fail(BlackLib::BlackUART::directionErr) or myUart.fail(BlackLib::BlackUART::flushErr) )
            *   {
            *       std::cout << "FLUSHING ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "FLUSHING IS OK" << std::endl;
            *   }
            *
            *   std::string writeThis = "Loopback test message.";
            *   std::string readThis  = myUart.transfer( writeThis, 40000 );
            *
            *   if( myUart.fail(BlackLib::BlackUART::readErr) or myUart.fail(BlackLib::BlackUART::writeErr) )
            *   {
            *       std::cout << "TRANSFER ERROR OCCURED" << std::endl;
            *   }
            *   else
            *   {
            *       std::cout << "TRANSFER IS OK" << std::endl;
            *   }
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // BlackUART INITIALIZATION IS OK
            *   // OPENNING IS OK
            *   // FLUSHING IS OK
            *   // TRANSFER IS OK
            * @endcode
            *
            * @sa errorUART
            */
            bool            fail(BlackUART::flags f);


            /*! @brief Writes values to uart line with "<<" operator.
            *
            *  This function writes values to uart line. Values sent to this function as string type.
            *  @param [in] &writeFromThis to uart
            *
            *  @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   std::string writeToUart  = "this is test.\n";
            *   myUart << writeToUart;
            *
            *   sleep(1);
            *
            *   std::string readFromUart;
            *   myUart >> readFromUart;
            *
            *   std::cout << "Test output on loopback: " << readFromUart;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            */
            BlackUART&      operator<<(std::string &writeFromThis);

            /*! @brief Reads values from uart line with ">>" operator.
            *
            *  This function reads values from uart line with ">>" operator. It creates temporary read
            *  buffer with specified size and reads values to this buffer. Then resize buffer to read
            *  value size. BlackUART::readBufferSize variable is used to specify temporary buffer size.
            *  @param [in] &readToThis from uart. If reading fails, this functions sets
            *  BlackLib::UART_READ_FAILED string to this variable.
            *
            *  @par Example
            *  @code{.cpp}
            *
            *   BlackLib::BlackUART  myUart(BlackLib::UART1,
            *                               BlackLib::Baud9600,
            *                               BlackLib::ParityEven,
            *                               BlackLib::StopOne,
            *                               BlackLib::Char8 );
            *
            *   myUart.open( BlackLib::ReadWrite | BlackLib::NonBlock );
            *   myUart.flush( BlackLib::bothDirection );
            *
            *   std::string writeToUart  = "this is test.\n";
            *   myUart << writeToUart;
            *
            *   sleep(1);
            *
            *   std::string readFromUart;
            *   myUart >> readFromUart;
            *
            *   std::cout << "Test output on loopback: " << readFromUart;
            *
            * @endcode
            * @code{.cpp}
            *   // Possible Output:
            *   // Test output on loopback: this is test.
            * @endcode
            */
            BlackUART&      operator>>(std::string &readToThis);
    };

    // ########################################### BLACKUART DECLARATION ENDS ############################################## //


/****************************************************************
 * Constants
 ****************************************************************/

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64
#define SYSFS_OMAP_MUX_DIR "/sys/kernel/debug/omap_mux/"

enum PIN_DIRECTION{
	INPUT_PIN=0,
	OUTPUT_PIN=1
};

enum PIN_VALUE{
	LOW=0,
	HIGH=1
};

/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag);
int gpio_set_value(unsigned int gpio, PIN_VALUE value);
int gpio_get_value(unsigned int gpio, unsigned int *value);
int gpio_set_edge(unsigned int gpio, char *edge);
int gpio_fd_open(unsigned int gpio);
int gpio_fd_close(int fd);
int gpio_omap_mux_setup(const char *omap_pin0_name, const char *mode);



/*
 * SimpleGPIO.h
 */



#endif /* BLACKLIB_H_ */
