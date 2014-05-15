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

#include "BlackLib.h"



//##############################################
//##############################################
//#######------BLACKLIB.CPP------###############
//##############################################
//##############################################




//namespace BlackLib
//{

    // ######################################### BLACKCOREADC DEFINITION STARTS ########################################## //
    BlackCoreADC::BlackCoreADC()
    {
        this->adcCoreErrors = new errorCoreADC( this->getErrorsFromCore() );

        this->loadDeviceTree();
        this->findHelperName();
    }

    BlackCoreADC::~BlackCoreADC()
    {
        delete this->adcCoreErrors;
    }


    bool        BlackCoreADC::loadDeviceTree()
    {
        std::string file = this->getSlotsFilePath();
        std::ofstream slotsFile;
        slotsFile.open(file.c_str(), std::ios::out);
        if(slotsFile.fail())
        {
            slotsFile.close();
            this->adcCoreErrors->dtError = true;
            return false;
        }
        else
        {
            slotsFile << "cape-bone-iio";
            slotsFile.close();
            this->adcCoreErrors->dtError = false;
            return true;
        }
    }

    bool        BlackCoreADC::findHelperName()
    {
        std::string limitedSearchResult = this->searchDirectoryOcp(BlackCore::ADC_helper);

        if(limitedSearchResult == SEARCH_DIR_NOT_FOUND)
        {
            this->helperName = "helper." + DEFAULT_HELPER_NUMBER;
            this->adcCoreErrors->helperError = true;
            return false;
        }
        else
        {
            this->helperName = limitedSearchResult;
            this->adcCoreErrors->helperError = false;
            return true;
        }
    }

    std::string BlackCoreADC::getHelperPath()
    {
        std::string temp = "/sys/devices/" + this->getOcpName() + "/" + this->helperName;
        return temp;
    }

    errorCoreADC *BlackCoreADC::getErrorsFromCoreADC()
    {
        return (this->adcCoreErrors);
    }

    // ########################################## BLACKCOREADC DEFINITION ENDS ########################################### //






    // ########################################### BLACKADC DEFINITION STARTS ############################################ //
    BlackADC::BlackADC(adcName adc)
    {
        this->adcErrors                 = new errorADC( this->getErrorsFromCoreADC() );
        this->ainName                   = adc;
        this->ainPath                   = this->getHelperPath() + "/AIN" + tostr(this->ainName);
    }


    BlackADC::~BlackADC()
    {
        delete this->adcErrors;
    }


    std::string BlackADC::getValue()
    {
        std::string returnStr = FILE_COULD_NOT_OPEN_STRING;
        std::ifstream adcValueFile;

        adcValueFile.open(ainPath.c_str(),std::ios::in);
        if(adcValueFile.fail())
        {
            adcValueFile.close();
            this->adcErrors->readError = true;
        }
        else
        {
            returnStr.clear();
            adcValueFile >> returnStr;

            adcValueFile.close();
            this->adcErrors->readError = false;
        }

        return returnStr;
    }

    adcName     BlackADC::getName()
    {
        return this->ainName;
    }


    int         BlackADC::getNumericValue()
    {
        int readValue = FILE_COULD_NOT_OPEN_INT;
        std::ifstream adcValueFile;

        adcValueFile.open(this->ainPath.c_str(),std::ios::in);
        if(adcValueFile.fail())
        {
            adcValueFile.close();
            this->adcErrors->readError=true;
        }
        else
        {
            adcValueFile >> readValue;
            adcValueFile.close();
            this->adcErrors->readError=false;
        }

        return readValue;
    }

    float       BlackADC::getConvertedValue(digitAfterPoint mode)
    {
        int valueInt = 0;

        std::ifstream adcValueFile;

        adcValueFile.open(this->ainPath.c_str(),std::ios::in);
        if(adcValueFile.fail())
        {
            adcValueFile.close();
            this->adcErrors->readError=true;
            return FILE_COULD_NOT_OPEN_FLOAT;
        }
        else
        {
            adcValueFile >> valueInt;
            adcValueFile.close();
            this->adcErrors->readError=false;
        }


        float valueFloat    = static_cast<float>(valueInt);

        if( mode == dap3 )
        {
            // shows 3 digit after point
            return (valueFloat)/1000;
        }

        if( mode == dap2 )
        {
            // shows 2 digit after point
            float tmp = round(valueFloat/10);
            return (tmp)/100;
        }


        if( mode == dap1 )
        {
            // shows 1 digit after point
            float tmp = round(valueFloat/100);
            return (tmp)/10;
        }

        return FILE_COULD_NOT_OPEN_FLOAT;
    }

    bool        BlackADC::fail()
    {
        return (this->adcErrors->adcCoreErrors->coreErrors->capeMgrError or
                this->adcErrors->adcCoreErrors->coreErrors->ocpError or
                this->adcErrors->adcCoreErrors->helperError or
                this->adcErrors->adcCoreErrors->dtError or
                this->adcErrors->readError);
    }

    bool        BlackADC::fail(BlackADC::flags f)
    {
        if(f==cpmgrErr) { return this->adcErrors->adcCoreErrors->coreErrors->capeMgrError;  }
        if(f==ocpErr)   { return this->adcErrors->adcCoreErrors->coreErrors->ocpError;      }
        if(f==helperErr){ return this->adcErrors->adcCoreErrors->helperError;               }
        if(f==dtErr)    { return this->adcErrors->adcCoreErrors->dtError;                   }
        if(f==readErr)  { return this->adcErrors->readError;                                }

        return true;
    }




    BlackADC&   BlackADC::operator>>(std::string &readToThis)
    {
        std::string readValue = FILE_COULD_NOT_OPEN_STRING;
        std::ifstream adcValueFile;

        adcValueFile.open(ainPath.c_str(),std::ios::in);
        if(adcValueFile.fail())
        {
            adcValueFile.close();
            this->adcErrors->readError = true;
        }
        else
        {
            readValue.clear();
            adcValueFile >> readValue;

            adcValueFile.close();
            this->adcErrors->readError = false;
        }

        readToThis = readValue;
        return *this;
    }


    BlackADC&   BlackADC::operator>>(int &readToThis)
    {
        int readValue = FILE_COULD_NOT_OPEN_INT;
        std::ifstream adcValueFile;

        adcValueFile.open(this->ainPath.c_str(),std::ios::in);
        if(adcValueFile.fail())
        {
            adcValueFile.close();
            this->adcErrors->readError=true;
        }
        else
        {
            adcValueFile >> readValue;
            adcValueFile.close();
            this->adcErrors->readError=false;
        }

        readToThis = readValue;
        return *this;
    }


    BlackADC&   BlackADC::operator>>(float &readToThis)
    {
        readToThis = this->getConvertedValue(dap3);
        return *this;
    }


    // ############################################ BLACKADC DEFINITION ENDS ############################################# //


// ########################################### BLACKCORE DEFINITION STARTS ########################################### //
    BlackCore::BlackCore()
    {
        this->coreErrors = new errorCore();

        this->findCapeMgrName();
        this->findOcpName();
        this->slotsFilePath = "/sys/devices/" + this->capeMgrName + "/slots";
    }

    BlackCore::~BlackCore()
    {
        delete this->coreErrors;
    }

    std::string BlackCore::executeCommand(std::string command)
    {
        FILE* pipe = popen(command.c_str(), "r");
        if ( pipe==NULL )
        {
            return "ERROR";
        }

        char buffer[128];
        std::string result = "";

        while( !feof(pipe) )
        {
            if( fgets(buffer, 128, pipe) != NULL )
            {
                result += buffer;
            }
        }

        pclose(pipe);
        return result;
    }

    std::string BlackCore::searchDirectory(std::string seachIn, std::string searchThis)
    {
        std::string str;
        DIR *path;
        dirent *entry;

        path = opendir(seachIn.c_str());
        if( path != NULL  )
        {
            while( (entry = readdir(path)) != NULL)
            {
                if( entry->d_name[0] == '.')
                {
                    continue;
                }

                str = entry->d_name;
                if(strstr(entry->d_name,searchThis.c_str()) != NULL )
                {
                    closedir(path);
                    return str;
                }
            }
        }
        closedir(path);

        return SEARCH_DIR_NOT_FOUND;
    }

    bool        BlackCore::findCapeMgrName()
    {
        std::string searchResult = this->searchDirectory("/sys/devices/","bone_capemgr.");

        if(searchResult == SEARCH_DIR_NOT_FOUND)
        {
            this->capeMgrName = "bone_capemgr." + DEFAULT_CAPE_MGR_NUMBER;
            this->coreErrors->capeMgrError = true;
            return false;
        }
        else
        {
            this->capeMgrName = searchResult;
            this->coreErrors->capeMgrError = false;
            return true;
        }
    }

    bool        BlackCore::findOcpName()
    {
        std::string searchResult = this->searchDirectory("/sys/devices/","ocp.");

        if(searchResult == SEARCH_DIR_NOT_FOUND)
        {
            this->ocpName = "ocp." + DEFAULT_OCP_NUMBER;
            this->coreErrors->ocpError = true;
            return false;
        }
        else
        {
            this->ocpName = searchResult;
            this->coreErrors->ocpError = false;
            return true;
        }
    }




    std::string BlackCore::searchDirectoryOcp(BlackCore::ocpSearch searchThis)
    {
        std::string searchResult;
        std::string searchPath = "/sys/devices/" + this->getOcpName() + "/";

        if( searchThis == this->SPI0 )
        {
            searchPath += DEFAULT_SPI0_PINMUX + ".spi/spi_master/";
        }
        else if( searchThis == this->SPI1 )
        {
            searchPath += DEFAULT_SPI1_PINMUX + ".spi/spi_master/";
        }


        switch(searchThis)
        {
            case ADC_helper:
            {
                searchResult = this->searchDirectory(searchPath,"helper.");
                break;
            }

            case PWM_P8_13:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P8_13.");
                break;
            }

            case PWM_P8_19:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P8_19.");
                break;
            }

            case PWM_P9_14:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P9_14.");
                break;
            }

            case PWM_P9_16:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P9_16.");
                break;
            }

            case PWM_P9_21:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P9_21.");
                break;
            }

            case PWM_P9_22:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P9_22.");
                break;
            }

            case PWM_P9_42:
            {
                searchResult = this->searchDirectory(searchPath,"pwm_test_P9_42.");
                break;
            }

            case SPI0:
            {
                searchResult = this->searchDirectory(searchPath,"spi");
                break;
            }

            case SPI1:
            {
                searchResult = this->searchDirectory(searchPath,"spi");
                break;
            }
        }


        return searchResult;
    }

    errorCore   *BlackCore::getErrorsFromCore()
    {
        return (this->coreErrors);
    }



    std::string BlackCore::getCapeMgrName()
    {
        return this->capeMgrName;
    }

    std::string BlackCore::getOcpName()
    {
        return this->ocpName;
    }

    std::string BlackCore::getSlotsFilePath()
    {
        return this->slotsFilePath;
    }

    // ############################################ BLACKCORE DEFINITION ENDS ############################################ //



// ######################################### BLACKCOREGPIO DEFINITION STARTS ######################################### //
                BlackCoreGPIO::BlackCoreGPIO(gpioName pin, direction dir)
    {
        this->pinNumericName    = static_cast<int>(pin);
        this->pinNumericType    = static_cast<int>(dir);
        this->gpioCoreError     = new errorCoreGPIO( this->getErrorsFromCore() );


        this->expPath           = "/sys/class/gpio/export";
        this->unExpPath         = "/sys/class/gpio/unexport";
        this->directionPath     = "/sys/class/gpio/gpio" + tostr(this->pinNumericName) + "/direction";


        this->doExport();
        this->setDirection();
    }

    BlackCoreGPIO::~BlackCoreGPIO()
    {
        this->doUnexport();
        delete this->gpioCoreError;
    }

    bool        BlackCoreGPIO::loadDeviceTree()
    {
        return false;
    }



    bool        BlackCoreGPIO::doExport()
    {
        std::ofstream expFile;

        expFile.open(this->expPath.c_str(),std::ios::out);
        if(expFile.fail())
        {
            expFile.close();
            this->gpioCoreError->exportFileError = true;
            return false;
        }
        else
        {
            expFile << this->pinNumericName;

            expFile.close();
            this->gpioCoreError->exportFileError = false;
            return true;
        }
    }

    bool        BlackCoreGPIO::setDirection()
    {
        std::ofstream directionFile;

        directionFile.open(this->directionPath.c_str(), std::ios::out);
        if(directionFile.fail())
        {
            directionFile.close();
            this->gpioCoreError->directionFileError = true;
            return false;
        }
        else
        {
            if(this->pinNumericType == static_cast<int>(output))
            {
                directionFile << "out";
            }
            else
            {
                directionFile << "in";
            }

            directionFile.close();
            this->gpioCoreError->directionFileError = false;
            return true;
        }
    }

    bool        BlackCoreGPIO::doUnexport()
    {
        std::ofstream unExpFile;

        unExpFile.open(this->unExpPath.c_str(), std::ios::out);
        if(unExpFile.fail())
        {
            unExpFile.close();
            return false;
        }
        else
        {
            unExpFile << this->pinNumericName;
            unExpFile.close();
            return true;
        }
    }



    std::string BlackCoreGPIO::getDirectionFilePath()
    {
        return this->directionPath;
    }


    std::string BlackCoreGPIO::getValueFilePath()
    {
        return ("/sys/class/gpio/gpio" + tostr(this->pinNumericName) + "/value");
    }


    errorCoreGPIO *BlackCoreGPIO::getErrorsFromCoreGPIO()
    {
        return (this->gpioCoreError);
    }

    // ########################################## BLACKCOREGPIO DEFINITION ENDS ########################################## //





    // ########################################### BLACKGPIO DEFINITION STARTS ########################################### //
    BlackGPIO::BlackGPIO(gpioName pin, direction dir, workingMode wm) : BlackCoreGPIO(pin, dir)
    {
        this->pinName       = pin;
        this->pinDirection  = dir;
        this->workMode      = wm;
        this->gpioErrors    = new errorGPIO( this->getErrorsFromCoreGPIO() );
        this->valuePath     = this->getValueFilePath();
    }

    BlackGPIO::~BlackGPIO()
    {
        delete this->gpioErrors;
    }


    bool        BlackGPIO::isReady()
    {
        return (this->isExported() and this->isDirectionSet());
    }

    bool        BlackGPIO::isExported()
    {
        std::ifstream exportCheck;

        exportCheck.open(valuePath.c_str(),std::ios::in|std::ios::binary);
        if(exportCheck.fail())
        {
            this->gpioErrors->exportError = true;
            return false;
        }
        else
        {
            this->gpioErrors->exportError = false;
            exportCheck.close();
            return true;
        }
    }

    bool        BlackGPIO::isDirectionSet()
    {
        std::ifstream directionCheck;

        directionCheck.open((this->getDirectionFilePath()).c_str(), std::ios::in|std::ios::binary);
        if(directionCheck.fail())
        {
            directionCheck.close();
            this->gpioErrors->directionError = true;
            return false;
        }
        else
        {
            std::string readValue;
            directionCheck >> readValue;

            if( (this->pinDirection == input and readValue=="in") or (this->pinDirection == output and readValue=="out") )
            {
                directionCheck.close();
                this->gpioErrors->directionError = false;
                return true;
            }
            else
            {
                directionCheck.close();
                this->gpioErrors->directionError = true;
                return false;
            }
        }
    }


    std::string BlackGPIO::getValue()
    {
        if( this->workMode == SecureMode )
        {
            if( ! this->isReady())
            {
                return GPIO_PIN_NOT_READY_STRING;
            }
        }


        std::ifstream valueFile;

        valueFile.open(valuePath.c_str(),std::ios::in);
        if(valueFile.fail())
        {
            valueFile.close();
            this->gpioErrors->readError = true;
            return FILE_COULD_NOT_OPEN_STRING;
        }
        else
        {
            std::string readValue;
            valueFile >> readValue;

            valueFile.close();
            this->gpioErrors->readError = false;
            return readValue;
        }
    }

    int         BlackGPIO::getNumericValue()
    {
        if( this->workMode == SecureMode )
        {
            if( ! this->isReady())
            {
                return GPIO_PIN_NOT_READY_INT;
            }
        }


        std::ifstream valueFile;

        valueFile.open(valuePath.c_str(),std::ios::in);
        if(valueFile.fail())
        {
            valueFile.close();
            this->gpioErrors->readError = true;
            return FILE_COULD_NOT_OPEN_INT;
        }
        else
        {
            int readValue;
            valueFile >> readValue;

            valueFile.close();
            this->gpioErrors->readError = false;
            return readValue;
        }
    }

    gpioName    BlackGPIO::getName()
    {
        return this->pinName;
    }

    direction   BlackGPIO::getDirection()
    {
        return this->pinDirection;
    }

    bool        BlackGPIO::setValue(digitalValue status)
    {
        if( !(this->pinDirection == output) )
        {
            this->gpioErrors->writeError = true;
            this->gpioErrors->forcingError = true;
            return false;
        }



        this->gpioErrors->forcingError = false;

        if( this->workMode == SecureMode )
        {
            if( ! this->isReady())
            {
                this->gpioErrors->writeError = true;
                return false;
            }
        }



        std::ofstream valueFile;
        valueFile.open(this->valuePath.c_str(), std::ios::out);
        if(valueFile.fail())
        {
            valueFile.close();
            this->gpioErrors->writeError = true;
            return false;
        }
        else
        {
            if( status == high )
            {
                valueFile << "1";
            }
            else
            {
                valueFile << "0";
            }

            valueFile.close();
            this->gpioErrors->writeError = false;
            return true;
        }
    }



    bool        BlackGPIO::isHigh()
    {
        return (this->getNumericValue() == 1);
    }

    void        BlackGPIO::toggleValue()
    {
        if( !(this->pinDirection == output) )
        {
            this->gpioErrors->forcingError = true;
        }
        else
        {
            this->gpioErrors->forcingError = false;
            if( (this->getNumericValue() == 1) )
            {
                this->setValue(low);
            }
            else
            {
                this->setValue(high);
            }
        }
    }


    void        BlackGPIO::setWorkingMode(workingMode newWM)
    {
        this->workMode = newWM;
    }

    workingMode BlackGPIO::getWorkingMode()
    {
        return this->workMode;
    }



    bool        BlackGPIO::fail()
    {
        return (this->gpioErrors->readError or
                this->gpioErrors->writeError or
                this->gpioErrors->exportError or
                this->gpioErrors->forcingError or
                this->gpioErrors->directionError
                );
    }

    bool        BlackGPIO::fail(BlackGPIO::flags f)
    {
        if(f==readErr)          { return this->gpioErrors->readError;                           }
        if(f==writeErr)         { return this->gpioErrors->writeError;                          }
        if(f==exportErr)        { return this->gpioErrors->exportError;                         }
        if(f==forcingErr)       { return this->gpioErrors->forcingError;                        }
        if(f==directionErr)     { return this->gpioErrors->directionError;                      }
        if(f==exportFileErr)    { return this->gpioErrors->gpioCoreErrors->exportFileError;     }
        if(f==directionFileErr) { return this->gpioErrors->gpioCoreErrors->directionFileError;  }

        return true;
    }




    BlackGPIO&  BlackGPIO::operator>>(std::string &readToThis)
    {
        if( this->workMode == SecureMode )
        {
            if( ! this->isReady())
            {
                readToThis = GPIO_PIN_NOT_READY_STRING;
                return *this;
            }
        }


        std::string readValue;
        std::ifstream valueFile;

        valueFile.open(valuePath.c_str(),std::ios::in);
        if(valueFile.fail())
        {
            valueFile.close();
            this->gpioErrors->readError = true;
            readValue = FILE_COULD_NOT_OPEN_STRING;
        }
        else
        {
            valueFile >> readValue;

            valueFile.close();
            this->gpioErrors->readError = false;
        }

        readToThis = readValue;
        return *this;
    }


    BlackGPIO&  BlackGPIO::operator>>(int &readToThis)
    {
        if( this->workMode == SecureMode )
        {
            if( ! this->isReady())
            {
                readToThis = GPIO_PIN_NOT_READY_INT;
                return *this;
            }
        }


        int readValue;
        std::ifstream valueFile;

        valueFile.open(valuePath.c_str(),std::ios::in);
        if(valueFile.fail())
        {
            valueFile.close();
            this->gpioErrors->readError = true;
            readValue = FILE_COULD_NOT_OPEN_INT;
        }
        else
        {
            valueFile >> readValue;

            valueFile.close();
            this->gpioErrors->readError = false;
        }

        readToThis = readValue;
        return *this;
    }


    BlackGPIO&  BlackGPIO::operator<<(digitalValue value)
    {
        if( !(this->pinDirection == output) )
        {
            this->gpioErrors->writeError = true;
            this->gpioErrors->forcingError = true;
            return *this;
        }



        this->gpioErrors->forcingError = false;

        if( this->workMode == SecureMode )
        {
            if( ! this->isReady())
            {
                this->gpioErrors->writeError = true;
                return *this;
            }
        }



        std::ofstream valueFile;
        valueFile.open(this->valuePath.c_str(), std::ios::out);
        if(valueFile.fail())
        {
            valueFile.close();
            this->gpioErrors->writeError = true;
            return *this;
        }
        else
        {
            if( value == high )
            {
                valueFile << "1";
            }
            else
            {
                valueFile << "0";
            }

            valueFile.close();
            this->gpioErrors->writeError = false;
            return *this;
        }
    }


    // ############################################ BLACKGPIO DECLARATION ENDS ############################################ //




BlackI2C::BlackI2C(i2cName i2c, unsigned int i2cDeviceAddress)
    {
        this->i2cPortPath   = "/dev/i2c-" + tostr(static_cast<int>(i2c));
        this->i2cDevAddress = i2cDeviceAddress;
        this->i2cFD         = -1;
        this->isOpenFlag    = false;

        this->i2cErrors     = new errorI2C( this->getErrorsFromCore() );
    }


    BlackI2C::~BlackI2C()
    {
        this->close();
        delete this->i2cErrors;
    }


    inline bool BlackI2C::useSmbusIOCTL(direction rwMode, uint8_t registerAddr, transactionType smbusTransaction, i2c_smbus_data &data)
    {
        if( rwMode == bothDirection ) { return false; }

        i2c_smbus_ioctl_data smbusPackage;

        smbusPackage.read_write = (rwMode == input ) ? I2C_SMBUS_READ : I2C_SMBUS_WRITE;
        smbusPackage.command    = registerAddr;
        smbusPackage.size       = smbusTransaction;
        smbusPackage.data       = &data;


        if( ::ioctl(this->i2cFD, I2C_SMBUS, &smbusPackage) < 0 )
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool    BlackI2C::setSlave()
    {
        if( ::ioctl(this->i2cFD, I2C_SLAVE, this->i2cDevAddress) < 0)
        {
            this->i2cErrors->setSlaveError = true;
            return false;
        }
        else
        {
            this->i2cErrors->setSlaveError = false;
            return true;
        }
    }

    bool    BlackI2C::loadDeviceTree()
    {
        return false;
    }





    bool    BlackI2C::open(uint openMode)
    {
        uint flags = 0;

        if( (openMode & ReadOnly)   == ReadOnly     ){  flags |= O_RDONLY;  }
        if( (openMode & WriteOnly)  == WriteOnly    ){  flags |= O_WRONLY;  }
        if( (openMode & ReadWrite)  == ReadWrite    ){  flags |= O_RDWR;    }
        if( (openMode & Append)     == Append       ){  flags |= O_APPEND;  }
        if( (openMode & Truncate)   == Truncate     ){  flags |= O_TRUNC;   }
        if( (openMode & NonBlock)   == NonBlock     ){  flags |= O_NONBLOCK;}


        this->i2cFD = ::open(this->i2cPortPath.c_str(), flags);


        if( this->i2cFD < 0 )
        {
            this->isOpenFlag = false;
            this->i2cErrors->openError = true;
            return false;
        }
        else
        {
            this->isOpenFlag = true;
            this->i2cErrors->openError = false;
            this->setSlave();
            return true;
        }
    }

    bool    BlackI2C::close()
    {
        if( ::close(this->i2cFD) < 0 )
        {
            this->i2cErrors->closeError = true;
            return false;
        }
        else
        {
            this->i2cErrors->closeError = false;
            this->isOpenFlag = false;
            return true;
        }
    }






    bool    BlackI2C::writeByte(uint8_t registerAddr, uint8_t value)
    {
        this->setSlave();

        i2c_smbus_data writeFromThis;
        writeFromThis.byte = value;

        if( this->useSmbusIOCTL(output, registerAddr, SMBUS_BYTE_DATA, writeFromThis) )
        {
            this->i2cErrors->writeError = false;
            return true;
        }
        else
        {
            this->i2cErrors->writeError = true;
            return false;
        }
    }

    uint8_t BlackI2C::readByte(uint8_t registerAddr)
    {
        this->setSlave();

        i2c_smbus_data readToThis;
        if( this->useSmbusIOCTL(input, registerAddr, SMBUS_BYTE_DATA, readToThis) )
        {
            this->i2cErrors->readError = false;
            return readToThis.byte;
        }
        else
        {
            this->i2cErrors->readError = true;
            return 0x00;
        }
    }



    bool    BlackI2C::writeWord(uint8_t registerAddr, uint16_t value)
    {
        this->setSlave();

        i2c_smbus_data writeFromThis;
        writeFromThis.word = value;

        if( this->useSmbusIOCTL(output, registerAddr, SMBUS_WORD_DATA, writeFromThis) )
        {
            this->i2cErrors->writeError = false;
            return true;
        }
        else
        {
            this->i2cErrors->writeError = true;
            return false;
        }
    }

    uint16_t BlackI2C::readWord(uint8_t registerAddr)
    {
        this->setSlave();

        i2c_smbus_data readToThis;

        if( this->useSmbusIOCTL(input, registerAddr, SMBUS_WORD_DATA, readToThis) )
        {
            this->i2cErrors->readError = false;
            return readToThis.word;
        }
        else
        {
            this->i2cErrors->readError = true;
            return 0x0000;
        }

    }



    bool    BlackI2C::writeBlock(uint8_t registerAddr, uint8_t *writeBuffer, size_t bufferSize)
    {
        this->setSlave();

        if( bufferSize > 32 )
        {
            bufferSize = 32;
        }


        i2c_smbus_data writeFromThis;

        memset(writeFromThis.block, 0, bufferSize+1);
        memcpy( &(writeFromThis.block[1]), writeBuffer, bufferSize);
        writeFromThis.block[0] = bufferSize;

        if( this->useSmbusIOCTL(output, registerAddr, SMBUS_I2C_BLOCK_DATA, writeFromThis) )
        {
            this->i2cErrors->writeError = false;
            return true;
        }
        else
        {
            this->i2cErrors->writeError = true;
            return false;
        }
    }

    uint8_t BlackI2C::readBlock(uint8_t registerAddr, uint8_t *readBuffer, size_t bufferSize)
    {
        this->setSlave();
        if( bufferSize > 32 )
        {
            bufferSize = 32;
        }

        i2c_smbus_data readToThis;
        readToThis.block[0] = bufferSize;

        if( this->useSmbusIOCTL(input, registerAddr, SMBUS_I2C_BLOCK_DATA, readToThis) )
        {
            this->i2cErrors->readError = false;
            memcpy(readBuffer, &(readToThis.block[1]), bufferSize);
            return readToThis.block[0];
        }
        else
        {
            this->i2cErrors->readError = true;
            return 0x00;
        }

    }



    bool    BlackI2C::writeLine(uint8_t *writeBuffer, size_t bufferSize)
    {
        this->setSlave();
        if( ::write(this->i2cFD, writeBuffer, bufferSize) < 0 )
        {
            this->i2cErrors->writeError = true;
            return false;
        }
        else
        {
            this->i2cErrors->writeError = false;
            return true;
        }
    }

    bool    BlackI2C::readLine(uint8_t *readBuffer, size_t bufferSize)
    {
        this->setSlave();
        if( ::read(this->i2cFD, readBuffer, bufferSize) < 0 )
        {
            this->i2cErrors->readError = true;
            return false;
        }
        else
        {
            this->i2cErrors->readError = false;
            return true;
        }
    }



    void    BlackI2C::setDeviceAddress(unsigned int newDeviceAddr)
    {
        this->i2cDevAddress = newDeviceAddr;
        this->setSlave();
    }

    int     BlackI2C::getDeviceAddress()
    {
        return this->i2cDevAddress;
    }





    std::string BlackI2C::getPortName()
    {
        return this->i2cPortPath;
    }

    bool        BlackI2C::isOpen()
    {
        return this->isOpenFlag;
    }

    bool        BlackI2C::isClose()
    {
        return !(this->isOpenFlag);
    }


    bool        BlackI2C::fail()
    {
        return (this->i2cErrors->openError or
                this->i2cErrors->closeError or
                this->i2cErrors->setSlaveError or
                this->i2cErrors->readError or
                this->i2cErrors->writeError
                );
    }

    bool        BlackI2C::fail(BlackI2C::flags f)
    {
        if(f==openErr)          { return this->i2cErrors->openError;        }
        if(f==closeErr)         { return this->i2cErrors->closeError;       }
        if(f==setSlaveErr)      { return this->i2cErrors->setSlaveError;    }
        if(f==readErr)          { return this->i2cErrors->readError;        }
        if(f==writeErr)         { return this->i2cErrors->writeError;       }

        return true;
    }




// ######################################### BLACKCOREPWM DEFINITION STARTS ########################################## //
    BlackCorePWM::BlackCorePWM(pwmName pwm)
    {
        this->pwmPinName    = pwm;
        this->pwmCoreErrors = new errorCorePWM( this->getErrorsFromCore() );

        this->loadDeviceTree();

        this->pwmTestPath   = "/sys/devices/" + this->getOcpName() + "/" + this->findPwmTestName( this->pwmPinName );
    }


    BlackCorePWM::~BlackCorePWM()
    {
        delete this->pwmCoreErrors;
    }


    bool        BlackCorePWM::loadDeviceTree()
    {
        std::string file    = this->getSlotsFilePath();
        std::ofstream slotsFile;

        slotsFile.open(file.c_str(), std::ios::out);
        if(slotsFile.fail())
        {
            slotsFile.close();
            this->pwmCoreErrors->dtSsError  = true;
            this->pwmCoreErrors->dtError    = true;
            return false;
        }
        else
        {
            slotsFile << "am33xx_pwm";
            slotsFile.close();
            this->pwmCoreErrors->dtSsError  = false;
        }


        slotsFile.open(file.c_str(), std::ios::out);
        if(slotsFile.fail())
        {
            slotsFile.close();
            this->pwmCoreErrors->dtError    = true;
            return false;
        }
        else
        {
            slotsFile << ("bone_pwm_" + pwmNameMap[this->pwmPinName]);
            slotsFile.close();
            this->pwmCoreErrors->dtError    = false;
            return true;
        }
    }

    std::string BlackCorePWM::findPwmTestName(pwmName pwm)
    {
        std::string searchResult = SEARCH_DIR_NOT_FOUND;
        switch (pwm)
        {
            case P8_13:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P8_13);
                break;
            }

            case P8_19:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P8_19);
                break;
            }

            case P9_14:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P9_14);
                break;
            }

            case P9_16:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P9_16);
                break;
            }

            case P9_21:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P9_21);
                break;
            }

            case P9_22:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P9_22);
                break;
            }

            case P9_42:
            {
                searchResult = this->searchDirectoryOcp(BlackCore::PWM_P9_42);
                break;
            }
        };


        if( searchResult == SEARCH_DIR_NOT_FOUND )
        {
            this->pwmCoreErrors->pwmTestError = true;
            return PWM_TEST_NAME_NOT_FOUND;
        }
        else
        {
            this->pwmCoreErrors->pwmTestError = false;
            return searchResult;
        }
    }



    std::string BlackCorePWM::getPeriodFilePath()
    {
        return (this->pwmTestPath + "/period");
    }

    std::string BlackCorePWM::getDutyFilePath()
    {
        return (this->pwmTestPath + "/duty");
    }

    std::string BlackCorePWM::getRunFilePath()
    {
        return (this->pwmTestPath + "/run");
    }

    std::string BlackCorePWM::getPolarityFilePath()
    {
        return (this->pwmTestPath + "/polarity");
    }

    errorCorePWM *BlackCorePWM::getErrorsFromCorePWM()
    {
        return (this->pwmCoreErrors);
    }
    // ########################################## BLACKCOREPWM DEFINITION ENDS ########################################### //





    // ########################################### BLACKPWM DEFINITION STARTS ############################################ //
    BlackPWM::BlackPWM(pwmName pwm) : BlackCorePWM(pwm)
    {
        this->pwmErrors     = new errorPWM( this->getErrorsFromCorePWM() );

        this->periodPath    = this->getPeriodFilePath();
        this->dutyPath      = this->getDutyFilePath();
        this->runPath       = this->getRunFilePath();
        this->polarityPath  = this->getPolarityFilePath();
    }

    BlackPWM::~BlackPWM()
    {
        delete this->pwmErrors;
    }

    std::string BlackPWM::getValue()
    {
        double period   = static_cast<long double>( this->getNumericPeriodValue() );
        double duty     = static_cast<long double>( this->getNumericDutyValue() );

        return tostr(static_cast<float>( (1.0 - (duty / period )) * 100 ));
    }

    std::string BlackPWM::getPeriodValue()
    {
        std::ifstream periodValueFile;

        periodValueFile.open(this->periodPath.c_str(),std::ios::in);
        if(periodValueFile.fail())
        {
            periodValueFile.close();
            this->pwmErrors->periodFileError = true;
            return FILE_COULD_NOT_OPEN_STRING;
        }
        else
        {
            std::string readValue;
            periodValueFile >> readValue;

            periodValueFile.close();
            this->pwmErrors->periodFileError = false;
            return readValue;
        }
    }

    std::string BlackPWM::getDutyValue()
    {
        std::ifstream dutyValueFile;

        dutyValueFile.open(this->dutyPath.c_str(),std::ios::in);
        if(dutyValueFile.fail())
        {
            dutyValueFile.close();
            this->pwmErrors->dutyFileError = true;
            return FILE_COULD_NOT_OPEN_STRING;
        }
        else
        {
            std::string readValue;
            dutyValueFile >> readValue;

            dutyValueFile.close();
            this->pwmErrors->dutyFileError = false;
            return readValue;
        }
    }

    std::string BlackPWM::getRunValue()
    {
        std::ifstream runValueFile;

        runValueFile.open(this->runPath.c_str(),std::ios::in);
        if(runValueFile.fail())
        {
            runValueFile.close();
            this->pwmErrors->runFileError = true;
            return FILE_COULD_NOT_OPEN_STRING;
        }
        else
        {
            std::string readValue;
            runValueFile >> readValue;

            runValueFile.close();
            this->pwmErrors->runFileError = false;
            return readValue;
        }
    }

    std::string BlackPWM::getPolarityValue()
    {
        std::ifstream polarityValueFile;

        polarityValueFile.open(this->polarityPath.c_str(),std::ios::in);
        if(polarityValueFile.fail())
        {
            polarityValueFile.close();
            this->pwmErrors->polarityFileError = true;
            return FILE_COULD_NOT_OPEN_STRING;
        }
        else
        {
            std::string readValue;
            polarityValueFile >> readValue;

            polarityValueFile.close();
            this->pwmErrors->polarityFileError = false;
            return readValue;
        }
    }

    float       BlackPWM::getNumericValue()
    {
        double period   = static_cast<long double>( this->getNumericPeriodValue() );
        double duty     = static_cast<long double>( this->getNumericDutyValue() );

        return static_cast<float>( (1.0 - (duty / period )) * 100 );
    }

    inline int64_t    BlackPWM::getNumericPeriodValue()
    {
        int64_t readValue = FILE_COULD_NOT_OPEN_INT;

        std::ifstream periodValueFile;

        periodValueFile.open(this->periodPath.c_str(),std::ios::in);
        if(periodValueFile.fail())
        {
            periodValueFile.close();
            this->pwmErrors->periodFileError = true;
        }
        else
        {
            periodValueFile >> readValue;

            periodValueFile.close();
            this->pwmErrors->periodFileError = false;
        }
        return readValue;
    }

    inline int64_t    BlackPWM::getNumericDutyValue()
    {
        int64_t readValue = FILE_COULD_NOT_OPEN_INT;
        std::ifstream dutyValueFile;

        dutyValueFile.open(this->dutyPath.c_str(),std::ios::in);
        if(dutyValueFile.fail())
        {
            dutyValueFile.close();
            this->pwmErrors->dutyFileError = true;
        }
        else
        {
            dutyValueFile >> readValue;

            dutyValueFile.close();
            this->pwmErrors->dutyFileError = false;
        }
        return readValue;
    }


    bool        BlackPWM::setDutyPercent(float percantage)
    {
        if( percantage > 100.0 or percantage < 0.0 )
        {
            this->pwmErrors->outOfRange      = true;
            this->pwmErrors->dutyFileError   = true;
            this->pwmErrors->periodFileError = true;
            return false;
        }

        this->pwmErrors->outOfRange = false;

        std::ofstream dutyFile;
        dutyFile.open(this->dutyPath.c_str(),std::ios::out);
        if(dutyFile.fail())
        {
            dutyFile.close();
            this->pwmErrors->dutyFileError = true;
            return false;
        }
        else
        {
            dutyFile << static_cast<int64_t>(std::round((this->getNumericPeriodValue()) * (1.0 - (percantage/100))));
            dutyFile.close();
            this->pwmErrors->dutyFileError = false;
            return true;
        }

    }

    bool        BlackPWM::setPeriodTime(uint64_t period, timeType tType)
    {
        uint64_t writeThis = static_cast<uint64_t>(period * static_cast<double>(pow( 10, static_cast<int>(tType)+9) ));

        if( writeThis > 1000000000)
        {
            this->pwmErrors->outOfRange = true;
            return false;
        }
        else
        {
            this->pwmErrors->outOfRange = false;
            std::ofstream periodFile;
            periodFile.open(this->periodPath.c_str(),std::ios::out);
            if(periodFile.fail())
            {
                periodFile.close();
                this->pwmErrors->periodFileError = true;
                return false;
            }
            else
            {
                periodFile << writeThis;
                periodFile.close();
                this->pwmErrors->periodFileError = false;
                return true;
            }
        }

    }

    bool        BlackPWM::setSpaceRatioTime(uint64_t space, timeType tType)
    {
        uint64_t writeThis = static_cast<int64_t>(space * static_cast<double>(pow( 10, static_cast<int>(tType)+9) ));

        if( writeThis > 1000000000)
        {
            this->pwmErrors->outOfRange = true;
            return false;
        }
        else
        {
            std::ofstream dutyFile;
            dutyFile.open(this->dutyPath.c_str(),std::ios::out);
            if(dutyFile.fail())
            {
                dutyFile.close();
                this->pwmErrors->dutyFileError = true;
                return false;
            }
            else
            {
                dutyFile << writeThis;
                dutyFile.close();
                this->pwmErrors->dutyFileError = false;
                return true;
            }
        }
    }

    bool        BlackPWM::setLoadRatioTime(uint64_t load, timeType tType)
    {
        uint64_t writeThis = (this->getNumericPeriodValue() - static_cast<int64_t>(load * static_cast<double>(pow( 10, static_cast<int>(tType)+9) )));

        if( writeThis > 1000000000)
        {
            this->pwmErrors->outOfRange = true;
            return false;
        }
        else
        {
            std::ofstream dutyFile;
            dutyFile.open(this->dutyPath.c_str(),std::ios::out);
            if(dutyFile.fail())
            {
                dutyFile.close();
                this->pwmErrors->dutyFileError = true;
                return false;
            }
            else
            {
                dutyFile << writeThis;
                dutyFile.close();
                this->pwmErrors->dutyFileError = false;
                return true;
            }
        }
    }

    bool        BlackPWM::setPolarity(polarityType polarity)
    {
        std::ofstream polarityFile;
        polarityFile.open(this->polarityPath.c_str(),std::ios::out);
        if(polarityFile.fail())
        {
            polarityFile.close();
            this->pwmErrors->polarityFileError = true;
            return false;
        }
        else
        {
            polarityFile << static_cast<int>(polarity);
            polarityFile.close();
            this->pwmErrors->polarityFileError = false;
            return true;
        }
    }

    bool        BlackPWM::setRunState(runValue state)
    {
        std::ofstream runFile;
        runFile.open(this->runPath.c_str(),std::ios::out);
        if(runFile.fail())
        {
            runFile.close();
            this->pwmErrors->runFileError = true;
            return false;
        }
        else
        {
            runFile << static_cast<int>(state);
            runFile.close();
            this->pwmErrors->runFileError = false;
            return true;
        }
    }



    bool        BlackPWM::isRunning()
    {
        return (this->getRunValue() == "1");
    }

    bool        BlackPWM::isPolarityStraight()
    {
        return !(this->getPolarityValue() == "1");
    }

    bool        BlackPWM::isPolarityReverse()
    {
        return (this->getPolarityValue() == "1");
    }



    void        BlackPWM::toggleRunState()
    {
        if( this->getRunValue() == "1" )
        {
            this->setRunState(stop);
        }
        else
        {
            this->setRunState(run);
        }
    }

    void        BlackPWM::tooglePolarity()
    {
        if( this->getPolarityValue() == "0" )
        {
            this->setPolarity(reverse);
        }
        else
        {
            this->setPolarity(straight);
        }
    }

    bool        BlackPWM::fail()
    {
        return (this->pwmErrors->outOfRange or
                this->pwmErrors->runFileError or
                this->pwmErrors->dutyFileError or
                this->pwmErrors->periodFileError or
                this->pwmErrors->polarityFileError or
                this->pwmErrors->pwmCoreErrors->dtError or
                this->pwmErrors->pwmCoreErrors->dtSsError or
                this->pwmErrors->pwmCoreErrors->pwmTestError or
                this->pwmErrors->pwmCoreErrors->coreErrors->ocpError or
                this->pwmErrors->pwmCoreErrors->coreErrors->capeMgrError
                );
    }

    bool        BlackPWM::fail(BlackPWM::flags f)
    {
        if(f==outOfRangeErr)    { return this->pwmErrors->outOfRange;                               }
        if(f==runFileErr)       { return this->pwmErrors->runFileError;                             }
        if(f==dutyFileErr)      { return this->pwmErrors->dutyFileError;                            }
        if(f==periodFileErr)    { return this->pwmErrors->periodFileError;                          }
        if(f==polarityFileErr)  { return this->pwmErrors->polarityFileError;                        }
        if(f==dtErr)            { return this->pwmErrors->pwmCoreErrors->dtError;                   }
        if(f==dtSubSystemErr)   { return this->pwmErrors->pwmCoreErrors->dtSsError;                 }
        if(f==pwmTestErr)       { return this->pwmErrors->pwmCoreErrors->pwmTestError;              }
        if(f==ocpErr)           { return this->pwmErrors->pwmCoreErrors->coreErrors->ocpError;      }
        if(f==cpmgrErr)         { return this->pwmErrors->pwmCoreErrors->coreErrors->capeMgrError;  }

        return true;
    }

    // ########################################### BLACKPWM DEFINITION STARTS ############################################ //




BlackSPI::BlackSPI(spiName spi)
    {
        this->spiChipNumber     = (static_cast<int>(spi) % 2);
        this->spiBusNumber      = ( (static_cast<int>(spi) - this->spiChipNumber)/2 );

        this->dtSpiFilename     = "BLACKLIB-SPI" + tostr(this->spiBusNumber);
        this->spiFD             = -1;
        this->isOpenFlag        = false;
        this->isCurrentEqDefault= true;
        this->spiErrors         = new errorSPI( this->getErrorsFromCore() );


        this->loadDeviceTree();
        this->findPortPath();

    }

    BlackSPI::BlackSPI(spiName spi, BlackSpiProperties spiProperties)
    {
        this->spiChipNumber     = (static_cast<int>(spi) % 2);
        this->spiBusNumber      = ( (static_cast<int>(spi) - this->spiChipNumber)/2 );

        this->dtSpiFilename     = "BLACKLIB-SPI" + tostr(this->spiBusNumber);
        this->spiFD             = -1;
        this->isOpenFlag        = false;
        this->isCurrentEqDefault= false;
        this->spiErrors         = new errorSPI( this->getErrorsFromCore() );

        constructorProperties   = spiProperties;

        this->loadDeviceTree();
        this->findPortPath();

    }

    BlackSPI::BlackSPI(spiName spi, uint8_t spiBitsPerWord, uint8_t spiMode, uint32_t spiSpeed)
    {
        this->spiChipNumber     = (static_cast<int>(spi) % 2);
        this->spiBusNumber      = ( (static_cast<int>(spi) - this->spiChipNumber)/2 );

        this->dtSpiFilename     = "BLACKLIB-SPI" + tostr(this->spiBusNumber);
        this->spiFD             = -1;
        this->isOpenFlag        = false;
        this->isCurrentEqDefault= false;
        this->spiErrors         = new errorSPI( this->getErrorsFromCore() );


        constructorProperties.spiBitsPerWord    = spiBitsPerWord;
        constructorProperties.spiMode           = spiMode;
        constructorProperties.spiSpeed          = spiSpeed;

        this->loadDeviceTree();
        this->findPortPath();
    }

    BlackSPI::~BlackSPI()
    {
        this->close();
        delete this->spiErrors;
    }






    bool        BlackSPI::loadDeviceTree()
    {
        std::string file = this->getSlotsFilePath();

        std::ofstream slotsFile;
        slotsFile.open(file.c_str(),std::ios::out);
        if(slotsFile.fail())
        {
            this->spiErrors->dtError = true;
            slotsFile.close();
            return false;
        }
        else
        {
            this->spiErrors->dtError = false;
            slotsFile << this->dtSpiFilename;
            slotsFile.close();
            return true;
        }
    }

    bool        BlackSPI::findPortPath()
    {
        std::string limitedSearchResult;

        if( this->spiBusNumber == 0 )
        {
            limitedSearchResult = this->searchDirectoryOcp(BlackCore::SPI0);
        }
        else if( this->spiBusNumber == 1 )
        {
            limitedSearchResult = this->searchDirectoryOcp(BlackCore::SPI1);
        }
        else
        {
            this->spiErrors->portPathError = true;
            return false;
        }


        if( limitedSearchResult == SEARCH_DIR_NOT_FOUND )
        {
            this->spiErrors->portPathError = true;
            return false;
        }
        else if( ::isdigit( static_cast<int>(limitedSearchResult[3]) ) == 0 )
        {
            this->spiErrors->portPathError = true;
            return false;
        }
        else
        {
            this->spiErrors->portPathError = false;
            this->spiPortPath = "/dev/spidev" + tostr( limitedSearchResult[3] ) + "." + tostr(this->spiChipNumber);
            return true;
        }

    }


    bool        BlackSPI::open(uint openMode)
    {
        uint flags = 0;

        if( (openMode & ReadOnly)   == ReadOnly     ){  flags |= O_RDONLY;  }
        if( (openMode & WriteOnly)  == WriteOnly    ){  flags |= O_WRONLY;  }
        if( (openMode & ReadWrite)  == ReadWrite    ){  flags |= O_RDWR;    }
        if( (openMode & Append)     == Append       ){  flags |= O_APPEND;  }
        if( (openMode & Truncate)   == Truncate     ){  flags |= O_TRUNC;   }
        if( (openMode & NonBlock)   == NonBlock     ){  flags |= O_NONBLOCK;}


        this->spiFD = ::open(this->spiPortPath.c_str(), flags);


        if( this->spiFD < 0 )
        {
            this->spiErrors->openError  = true;
            this->isOpenFlag            = false;
            return false;
        }


        this->spiErrors->openError  = false;
        this->isOpenFlag            = true;
        this->defaultProperties     = this->getProperties();

        if( this->isCurrentEqDefault )
        {
            this->currentProperties = this->defaultProperties;
        }
        else
        {
            if( this->setProperties( this->constructorProperties ))
            {
                this->currentProperties = this->constructorProperties;
            }
            else
            {
                this->currentProperties = this->defaultProperties;
            }
        }
        return true;
    }

    bool        BlackSPI::close()
    {
        if( ::close(this->spiFD) < 0 )
        {
            this->spiErrors->closeError = true;
            return false;
        }
        else
        {
            this->spiErrors->closeError = false;
            this->isOpenFlag = false;
            return true;
        }
    }








    bool        BlackSPI::setMode(uint8_t newMode)
    {
       if( ::ioctl(this->spiFD, SPI_IOC_WR_MODE, &newMode) == -1 )
       {
           this->spiErrors->modeError = true;
           return false;
       }
       else
       {
           this->spiErrors->modeError = false;
           this->currentProperties.spiMode = newMode;
           return true;
       }
    }

    uint8_t     BlackSPI::getMode()
    {
        uint8_t mode;

        if( ::ioctl(this->spiFD, SPI_IOC_RD_MODE, &mode) == -1 )
        {
            this->spiErrors->modeError = true;
            return 0;
        }
        else
        {
            this->spiErrors->modeError = false;
            this->currentProperties.spiMode = mode;
            return mode;
        }
    }



    bool        BlackSPI::setMaximumSpeed(uint32_t newSpeed)
    {
       if( ::ioctl(this->spiFD, SPI_IOC_WR_MAX_SPEED_HZ, &newSpeed) == -1 )
       {
           this->spiErrors->speedError = true;
           return false;
       }
       else
       {
           this->spiErrors->speedError = false;
           this->currentProperties.spiSpeed = newSpeed;
           return true;
       }
    }

    uint32_t    BlackSPI::getMaximumSpeed()
    {
        uint32_t speed;

        if( ::ioctl(this->spiFD, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1 )
        {
            this->spiErrors->speedError = true;
            return 0;
        }
        else
        {
            this->spiErrors->speedError = false;
            this->currentProperties.spiSpeed = speed;
            return speed;
        }
    }



    bool        BlackSPI::setBitsPerWord(uint8_t newBitSize)
    {
       if( ::ioctl(this->spiFD, SPI_IOC_WR_BITS_PER_WORD, &newBitSize) == -1 )
       {
           this->spiErrors->bitSizeError = true;
           return false;
       }
       else
       {
           this->spiErrors->bitSizeError = false;
           this->currentProperties.spiBitsPerWord = newBitSize;
           return true;
       }
    }

    uint8_t     BlackSPI::getBitsPerWord()
    {
        uint8_t bitsSize;

        if( ::ioctl(this->spiFD, SPI_IOC_RD_BITS_PER_WORD, &bitsSize) == -1 )
        {
            this->spiErrors->bitSizeError = true;
            return 0;
        }
        else
        {
            this->spiErrors->bitSizeError = false;
            this->currentProperties.spiBitsPerWord = bitsSize;
            return bitsSize;
        }
    }



    bool        BlackSPI::setProperties(BlackSpiProperties &newProperties)
    {
        return ( this->setBitsPerWord( newProperties.spiBitsPerWord ) and
                 this->setMaximumSpeed( newProperties.spiSpeed ) and
                 this->setMode( newProperties.spiMode )
               );
    }

    BlackSpiProperties BlackSPI::getProperties()
    {
        this->getBitsPerWord();
        this->getMode();
        this->getMaximumSpeed();
        return ( this->currentProperties );
    }



    uint8_t     BlackSPI::transfer(uint8_t writeByte, uint16_t wait_us)
    {
        uint8_t tempReadByte = 0x00;


        if( ! this->isOpenFlag )
        {
            this->spiErrors->openError      = true;
            this->spiErrors->transferError  = true;
            return tempReadByte;
        }



        this->spiErrors->openError          = false;
        spi_ioc_transfer package;

        package.tx_buf          = (unsigned long)&writeByte;
        package.rx_buf          = (unsigned long)&tempReadByte;
        package.len             = 1;
        package.delay_usecs     = wait_us;
        package.speed_hz        = this->currentProperties.spiSpeed;
        package.bits_per_word   = this->currentProperties.spiBitsPerWord;


        if( ::ioctl(this->spiFD, SPI_IOC_MESSAGE(1), &package) >= 0)
        {
            this->spiErrors->transferError = false;
            return tempReadByte;
        }
        else
        {
            this->spiErrors->transferError = true;
            return tempReadByte;
        }
    }

    bool        BlackSPI::transfer(uint8_t *writeBuffer, uint8_t *readBuffer, size_t bufferSize, uint16_t wait_us)
    {
        if( ! this->isOpenFlag )
        {
            this->spiErrors->openError      = true;
            this->spiErrors->transferError  = true;
            return false;
        }



        this->spiErrors->openError          = false;
        uint8_t tempReadBuffer[ bufferSize ];
        memset( tempReadBuffer, 0, bufferSize);

        spi_ioc_transfer package;
        package.tx_buf          = (unsigned long)writeBuffer;
        package.rx_buf          = (unsigned long)tempReadBuffer;
        package.len             = bufferSize;
        package.delay_usecs     = wait_us;
        package.speed_hz        = this->currentProperties.spiSpeed;
        package.bits_per_word   = this->currentProperties.spiBitsPerWord;


        if( ::ioctl(this->spiFD, SPI_IOC_MESSAGE(1), &package) >= 0)
        {
            this->spiErrors->transferError = false;
            memcpy(readBuffer, tempReadBuffer, bufferSize);
            return true;
        }
        else
        {
            this->spiErrors->transferError = true;
            return false;
        }
    }






    std::string BlackSPI::getPortName()
    {
        return this->spiPortPath;
    }

    bool        BlackSPI::isOpen()
    {
        return this->isOpenFlag;
    }

    bool        BlackSPI::isClose()
    {
        return !(this->isOpenFlag);
    }


    bool        BlackSPI::fail()
    {
        return (this->spiErrors->dtError or
                this->spiErrors->openError or
                this->spiErrors->closeError or
                this->spiErrors->portPathError or
                this->spiErrors->transferError or
                this->spiErrors->modeError or
                this->spiErrors->speedError or
                this->spiErrors->bitSizeError
                );
    }

    bool        BlackSPI::fail(BlackSPI::flags f)
    {
        if(f==dtErr)            { return this->spiErrors->dtError;         }
        if(f==openErr)          { return this->spiErrors->openError;       }
        if(f==closeErr)         { return this->spiErrors->closeError;      }
        if(f==portPathErr)      { return this->spiErrors->portPathError;   }
        if(f==transferErr)      { return this->spiErrors->transferError;   }
        if(f==modeErr)          { return this->spiErrors->modeError;       }
        if(f==speedErr)         { return this->spiErrors->speedError;      }
        if(f==bitSizeErr)       { return this->spiErrors->bitSizeError;    }

        return true;
    }




BlackUART::BlackUART(uartName uart, baudRate uartBaud, parity uartParity, stopBits uartStopBits, characterSize uartCharSize)
    {
        this->dtUartFilename            = "BB-UART" + tostr(static_cast<int>(uart));
        this->uartPortPath              = "/dev/ttyO" + tostr(static_cast<int>(uart));

        this->readBufferSize            = 1024;
        this->uartFD                    = -1;
        this->isOpenFlag                = false;
        this->isCurrentEqDefault        = false;

        this->uartErrors                = new errorUART( this->getErrorsFromCore() );
        this->constructorProperties     = BlackUartProperties(uartBaud, uartBaud, uartParity, uartStopBits, uartCharSize);

        this->loadDeviceTree();
    }


    BlackUART::BlackUART(uartName uart, BlackUartProperties uartProperties)
    {
        this->dtUartFilename            = "BB-UART" + tostr(static_cast<int>(uart));
        this->uartPortPath              = "/dev/ttyO" + tostr(static_cast<int>(uart));

        this->readBufferSize            = 1024;
        this->uartFD                    = -1;
        this->isOpenFlag                = false;
        this->isCurrentEqDefault        = false;

        this->uartErrors                = new errorUART( this->getErrorsFromCore() );
        this->constructorProperties     = uartProperties;

        this->loadDeviceTree();
    }


    BlackUART::BlackUART(uartName uart)
    {
        this->dtUartFilename            = "BB-UART" + tostr(static_cast<int>(uart));
        this->uartPortPath              = "/dev/ttyO" + tostr(static_cast<int>(uart));

        this->readBufferSize            = 1024;
        this->uartFD                    = -1;
        this->isOpenFlag                = false;
        this->isCurrentEqDefault        = true;

        this->uartErrors                = new errorUART( this->getErrorsFromCore() );

        this->loadDeviceTree();
    }


    BlackUART::~BlackUART()
    {
        this->close();
        delete this->uartErrors;
    }



    bool        BlackUART::loadDeviceTree()
    {
        std::string file = this->getSlotsFilePath();

        std::ofstream slotsFile;
        slotsFile.open(file.c_str(),std::ios::out);
        if(slotsFile.fail())
        {
            this->uartErrors->dtError = true;
            slotsFile.close();
            return false;
        }
        else
        {
            this->uartErrors->dtError = false;
            slotsFile << this->dtUartFilename;
            slotsFile.close();
            return true;
        }
    }


    bool        BlackUART::open(uint openMode)
    {
        int flags = 0;

        if( (openMode & ReadOnly)   == ReadOnly     ){  flags |= O_RDONLY;  }
        if( (openMode & WriteOnly)  == WriteOnly    ){  flags |= O_WRONLY;  }
        if( (openMode & ReadWrite)  == ReadWrite    ){  flags |= O_RDWR;    }
        if( (openMode & Append)     == Append       ){  flags |= O_APPEND;  }
        if( (openMode & Truncate)   == Truncate     ){  flags |= O_TRUNC;   }
        if( (openMode & NonBlock)   == NonBlock     ){  flags |= O_NONBLOCK;}

        this->uartFD = ::open(this->uartPortPath.c_str(), flags | O_NOCTTY);


        if( this->uartFD < 0 )
        {
            this->uartErrors->openError = true;
            this->isOpenFlag            = false;
            return false;
        }



        this->uartErrors->openError     = false;
        this->isOpenFlag                = true;
        this->defaultUartProperties     = this->getProperties();

        if( this->isCurrentEqDefault )
        {
            this->currentUartProperties = this->defaultUartProperties;
            return true;
        }
        else
        {
            if( this->setProperties(this->constructorProperties , ApplyNow) )
            {
                this->currentUartProperties = this->constructorProperties;
            }
            else
            {
                this->currentUartProperties = this->defaultUartProperties;
            }
        }
        return true;
    }

    bool        BlackUART::close()
    {
        if( ::close(this->uartFD) < 0 )
        {
            this->uartErrors->closeError    = true;
            return false;
        }
        else
        {
            this->uartErrors->closeError    = false;
            this->isOpenFlag                = false;
            return true;
        }
    }



    bool        BlackUART::flush(direction whichDirection)
    {
        int isFlushed = -1;


        if( whichDirection == input )
        {
            this->uartErrors->directionError = false;
            isFlushed = tcflush(this->uartFD, TCIFLUSH);
        }
        else if( whichDirection == output )
        {
            this->uartErrors->directionError = false;
            isFlushed = tcflush(this->uartFD, TCOFLUSH);
        }
        else if( whichDirection == bothDirection )
        {
            this->uartErrors->directionError = false;
            isFlushed = tcflush(this->uartFD, TCIOFLUSH);
        }
        else
        {
            this->uartErrors->directionError = true;
        }


        if( isFlushed == 0 )
        {
            this->uartErrors->flushError = false;
            return true;
        }
        else
        {
            this->uartErrors->flushError = true;
            return false;
        }
    }



    bool        BlackUART::read(char *readBuffer, size_t size)
    {
        char tempReadBuffer[size];
        memset(&tempReadBuffer, 0, size);

        if(::read(this->uartFD, tempReadBuffer, sizeof(tempReadBuffer)) > 0)
        {
            memcpy(readBuffer, tempReadBuffer, sizeof(tempReadBuffer));
            this->uartErrors->readError = false;
            return true;
        }
        else
        {
            this->uartErrors->readError = true;
            return false;
        }
    }

    std::string BlackUART::read()
    {
        std::string tempReadBuffer;
        tempReadBuffer.resize(this->readBufferSize);

        int readSize = ::read(this->uartFD, &tempReadBuffer[0], tempReadBuffer.size() );
        if( readSize > 0)
        {
            this->uartErrors->readError = false;
            tempReadBuffer.resize(readSize);
            return tempReadBuffer;
        }
        else
        {
            this->uartErrors->readError = true;
            return UART_READ_FAILED;
        }
    }



    bool        BlackUART::write(char *writeBuffer, size_t size)
    {
        if(::write(this->uartFD, writeBuffer, size) > 0)
        {
            this->uartErrors->writeError = false;
            return true;
        }
        else
        {
            this->uartErrors->writeError = true;
            return false;
        }
    }

    bool        BlackUART::write(std::string writeBuffer)
    {
        if(::write(this->uartFD, writeBuffer.c_str(), writeBuffer.size() ) > 0)
        {
            this->uartErrors->writeError = false;
            return true;
        }
        else
        {
            this->uartErrors->writeError = true;
            return false;
        }
    }




    bool        BlackUART::transfer(char *writeBuffer, char *readBuffer, size_t size, uint32_t wait_us)
    {
        if(::write(this->uartFD, writeBuffer, size ) > 0)
        {
            this->uartErrors->writeError = false;
        }
        else
        {
            this->uartErrors->writeError = true;
            return false;
        }

        usleep(wait_us);


        char tempReadBuffer[ size ];
        memset(&tempReadBuffer, 0, size);

        if(::read(this->uartFD, tempReadBuffer, sizeof(tempReadBuffer)) > 0)
        {
            memcpy(readBuffer,tempReadBuffer,sizeof(tempReadBuffer));
            this->uartErrors->readError = false;
            return true;
        }
        else
        {
            this->uartErrors->readError = true;
            return false;
        }
    }

    std::string BlackUART::transfer(std::string writeBuffer, uint32_t wait_us)
    {
        if(::write(this->uartFD,writeBuffer.c_str(),writeBuffer.size() ) > 0)
        {
            this->uartErrors->writeError = false;
        }
        else
        {
            this->uartErrors->writeError = true;
            return UART_WRITE_FAILED;
        }

        usleep(wait_us);

        std::string tempReadBuffer;
        tempReadBuffer.resize(this->readBufferSize);

        int readSize = ::read(this->uartFD, &tempReadBuffer[0], tempReadBuffer.size() );
        if( readSize > 0 )
        {
            this->uartErrors->readError = false;
            tempReadBuffer.resize(readSize);
            return tempReadBuffer;
        }
        else
        {
            this->uartErrors->readError = true;
            return UART_READ_FAILED;
        }



    }



    uint32_t    BlackUART::getReadBufferSize()
    {
        return this->readBufferSize;
    }

    void        BlackUART::setReadBufferSize(uint32_t newBufferSize)
    {
        this->readBufferSize = newBufferSize;
    }



    std::string BlackUART::getPortName()
    {
        return this->uartPortPath;
    }



    baudRate    BlackUART::getBaudRate(direction whichDirection)
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->baudRateError = true;
            this->uartErrors->openError     = true;
            return Baud0;
        }

        termios tempProperties;
        if( tcgetattr(this->uartFD, &tempProperties) != 0 )
        {
            this->uartErrors->baudRateError = true;
            return Baud0;
        }
        else
        {
            this->uartErrors->baudRateError = false;
        }



        if( whichDirection == input )
        {
            this->currentUartProperties.uartBaudIn = static_cast<baudRate>(cfgetispeed(&tempProperties));
            return this->currentUartProperties.uartBaudIn;
        }

        if( whichDirection == output )
        {
            this->currentUartProperties.uartBaudOut = static_cast<baudRate>(cfgetospeed(&tempProperties));
            return this->currentUartProperties.uartBaudOut;
        }
        else
        {
            return Baud0;
        }
    }

    bool        BlackUART::setBaudRate(baudRate newBaud, direction whichDirection, uartApplyMode applyMode )
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->baudRateError = true;
            this->uartErrors->openError     = true;
            return false;
        }

        termios tempProperties;
        tcgetattr(this->uartFD, &tempProperties);

        if( whichDirection == input)
        {
            cfsetispeed(&tempProperties, newBaud);
        }
        else
        if( whichDirection == output )
        {
            cfsetospeed(&tempProperties, newBaud);
        }
        else
        if( whichDirection == bothDirection )
        {
            cfsetispeed(&tempProperties, newBaud);
            cfsetospeed(&tempProperties, newBaud);
        }
        else
        {
            this->uartErrors->directionError    = true;
            this->uartErrors->baudRateError     = true;
            return false;
        }

        this->uartErrors->directionError        = false;

        if( tcsetattr(this->uartFD, applyMode, &tempProperties) == 0 )
        {
            if( whichDirection == input)
            {
                this->currentUartProperties.uartBaudIn = newBaud;
            }
            else
            if( whichDirection == output )
            {
                this->currentUartProperties.uartBaudOut = newBaud;
            }
            else
            if( whichDirection == bothDirection )
            {
                this->currentUartProperties.uartBaudIn = newBaud;
                this->currentUartProperties.uartBaudOut = newBaud;
            }

            this->uartErrors->baudRateError = false;
            return true;
        }
        else
        {
            this->uartErrors->baudRateError = true;
            return false;
        }
    }



    parity      BlackUART::getParity()
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->parityError   = true;
            this->uartErrors->openError     = true;
            return ParityDefault;
        }

        termios tempProperties;
        if( tcgetattr(this->uartFD, &tempProperties) != 0 )
        {
            this->uartErrors->parityError = true;
            return ParityDefault;
        }
        else
        {
            this->uartErrors->parityError = false;
        }

        tcflag_t controlFlag = tempProperties.c_cflag;

        if( (controlFlag & PARENB)      == PARENB )
        {
            if( (controlFlag & PARODD)  == PARODD )
            {
                this->currentUartProperties.uartParity = ParityOdd;
                return ParityOdd;
            }
            else
            {
                this->currentUartProperties.uartParity = ParityEven;
                return ParityEven;
            }
        }
        else
        {
            this->currentUartProperties.uartParity = ParityNo;
            return ParityNo;
        }
    }

    bool        BlackUART::setParity(parity newParity, uartApplyMode applyMode )
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->parityError   = true;
            this->uartErrors->openError     = true;
            return false;
        }

        termios tempProperties;
        tcgetattr(this->uartFD, &tempProperties);

        if( (newParity == ParityOdd) or (newParity == ParityEven) )
        {
            tempProperties.c_cflag |= PARENB;

            if( newParity == ParityOdd )
            {
                tempProperties.c_cflag |= PARODD;
            }
            else
            {
                tempProperties.c_cflag &= ~(PARODD);
            }
        }
        else
        {
            tempProperties.c_cflag &= ~(PARENB);
        }



        if( tcsetattr(this->uartFD, applyMode, &tempProperties) == 0 )
        {
            this->uartErrors->parityError = false;
            this->currentUartProperties.uartParity = (newParity == ParityDefault) ? ParityNo : newParity;;
            return true;
        }
        else
        {
            this->uartErrors->parityError = true;
            return false;
        }
    }



    stopBits    BlackUART::getStopBits()
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->stopBitsError = true;
            this->uartErrors->openError     = true;
            return StopDefault;
        }

        termios tempProperties;
        if( tcgetattr(this->uartFD, &tempProperties) != 0 )
        {
            this->uartErrors->stopBitsError = true;
            return StopDefault;
        }
        else
        {
            this->uartErrors->stopBitsError = false;
        }


        if( (tempProperties.c_cflag & CSTOPB) == CSTOPB )
        {
            this->currentUartProperties.uartStopBits = StopTwo;
            return StopTwo;
        }
        else
        {
            this->currentUartProperties.uartStopBits = StopOne;
            return StopOne;
        }

    }

    bool        BlackUART::setStopBits(stopBits newStopBits, uartApplyMode applyMode )
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->stopBitsError = true;
            this->uartErrors->openError     = true;
            return false;
        }

        termios tempProperties;
        tcgetattr(this->uartFD, &tempProperties);

        if( newStopBits == StopTwo)
        {
            tempProperties.c_cflag |= CSTOPB;
        }
        else
        {
            tempProperties.c_cflag &= ~(CSTOPB);
        }


        if( tcsetattr(this->uartFD, applyMode, &tempProperties) == 0 )
        {
            this->uartErrors->stopBitsError = false;
            this->currentUartProperties.uartStopBits = (newStopBits == StopDefault) ? StopOne : newStopBits;
            return true;
        }
        else
        {
            this->uartErrors->stopBitsError = true;
            return false;
        }
    }



    characterSize BlackUART::getCharacterSize()
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->charSizeError = true;
            this->uartErrors->openError     = true;
            return CharDefault;
        }

        termios tempProperties;
        if( tcgetattr(this->uartFD, &tempProperties) != 0 )
        {
            this->uartErrors->charSizeError = true;
            return CharDefault;
        }
        else
        {
            this->uartErrors->charSizeError = false;
        }

        tcflag_t controlFlag = tempProperties.c_cflag;

        if( (controlFlag & CS8) == CS8  ){ this->currentUartProperties.uartCharSize = Char8; return Char8; }
        if( (controlFlag & CS6) == CS6  ){ this->currentUartProperties.uartCharSize = Char6; return Char6; }
        if( (controlFlag & CS7) == CS7  ){ this->currentUartProperties.uartCharSize = Char7; return Char7; }
        else                             { this->currentUartProperties.uartCharSize = Char5; return Char5; }
    }

    bool        BlackUART::setCharacterSize(characterSize newCharacterSize, uartApplyMode applyMode )
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->charSizeError = true;
            this->uartErrors->openError     = true;
            return false;
        }

        termios tempProperties;
        tcgetattr(this->uartFD, &tempProperties);

        switch (newCharacterSize)
        {
            case Char5:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS5;
                break;
            }
            case Char6:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS6;
                break;
            }
            case Char7:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS7;
                break;
            }
            case Char8:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS8;
                break;
            }
            case CharDefault:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS8;
                break;
            }
            default:
                break;
        }



        if( tcsetattr(this->uartFD, applyMode, &tempProperties) == 0 )
        {
            this->currentUartProperties.uartCharSize = (newCharacterSize == CharDefault) ? Char8 : newCharacterSize;
            this->uartErrors->charSizeError = false;
            return true;
        }
        else
        {
            this->uartErrors->charSizeError = true;
            return false;
        }

    }




    BlackUartProperties BlackUART::getProperties()
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->baudRateError = true;
            this->uartErrors->charSizeError = true;
            this->uartErrors->parityError   = true;
            this->uartErrors->stopBitsError = true;
            this->uartErrors->openError     = true;
            return BlackUartProperties();
        }

        termios tempProperties;
        if( tcgetattr(this->uartFD, &tempProperties) != 0 )
        {
            this->uartErrors->baudRateError = true;
            this->uartErrors->charSizeError = true;
            this->uartErrors->parityError   = true;
            this->uartErrors->stopBitsError = true;
        }
        else
        {
            this->uartErrors->baudRateError = false;
            this->uartErrors->charSizeError = false;
            this->uartErrors->parityError   = false;
            this->uartErrors->stopBitsError = false;
        }

        tcflag_t controlFlag = tempProperties.c_cflag;

        baudRate        currentBaudIn   = static_cast<baudRate>(cfgetispeed(&tempProperties));
        baudRate        currentBaudOut  = static_cast<baudRate>(cfgetospeed(&tempProperties));
        stopBits        currentStopBits = ((controlFlag & CSTOPB)==CSTOPB) ? StopTwo : StopOne;
        parity          currentParity   = ((controlFlag & PARENB)==PARENB) ? ( ((controlFlag & PARODD)==PARODD) ? ParityOdd : ParityEven ) : ParityNo;
        characterSize   currentCharSize;

        if     ( (controlFlag & CS8) == CS8  ){ currentCharSize = Char8; }
        else if( (controlFlag & CS6) == CS6  ){ currentCharSize = Char6; }
        else if( (controlFlag & CS7) == CS7  ){ currentCharSize = Char7; }
        else                                  { currentCharSize = Char5; }

        this->currentUartProperties = BlackUartProperties(currentBaudIn, currentBaudOut, currentParity, currentStopBits, currentCharSize);
        return this->currentUartProperties;
    }

    bool        BlackUART::setProperties(BlackUartProperties &props, uartApplyMode applyMode)
    {
        if( !(this->isOpenFlag) )
        {
            this->uartErrors->baudRateError = true;
            this->uartErrors->charSizeError = true;
            this->uartErrors->parityError   = true;
            this->uartErrors->stopBitsError = true;
            this->uartErrors->openError     = true;
            return false;
        }

        termios tempProperties;
        tcgetattr(this->uartFD, &tempProperties);

        cfsetispeed(&tempProperties, props.uartBaudIn);
        cfsetospeed(&tempProperties, props.uartBaudOut);


        if( (props.uartParity == ParityOdd) or (props.uartParity == ParityEven) )
        {
            tempProperties.c_cflag |= PARENB;

            if( props.uartParity == ParityOdd )
            {
                tempProperties.c_cflag |= PARODD;
            }
            else
            {
                tempProperties.c_cflag &= ~(PARODD);
            }
        }
        else
        {
            tempProperties.c_cflag &= ~(PARENB);
        }


        if( props.uartStopBits == StopTwo)
        {
            tempProperties.c_cflag |= CSTOPB;
        }
        else
        {
            tempProperties.c_cflag &= ~(CSTOPB);
        }


        switch (props.uartCharSize)
        {
            case Char5:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS5;
                break;
            }
            case Char6:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS6;
                break;
            }
            case Char7:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS7;
                break;
            }
            case Char8:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS8;
                break;
            }
            case CharDefault:
            {
                tempProperties.c_cflag &= ~(CSIZE);
                tempProperties.c_cflag |= CS8;
                break;
            }
            default:
                break;
        }


        tempProperties.c_cflag |= CLOCAL;
        tempProperties.c_cflag |= CREAD;

        tempProperties.c_iflag |= ICRNL;
        tempProperties.c_oflag = 0;
        tempProperties.c_lflag = 0;

        tempProperties.c_cc[VTIME] = 0;
        tempProperties.c_cc[VMIN]  = 1;



        if( tcsetattr(this->uartFD, applyMode, &tempProperties) == 0 )
        {
            this->uartErrors->baudRateError = false;
            this->uartErrors->charSizeError = false;
            this->uartErrors->parityError   = false;
            this->uartErrors->stopBitsError = false;

            this->currentUartProperties.uartBaudIn   = props.uartBaudIn;
            this->currentUartProperties.uartBaudOut  = props.uartBaudOut;
            this->currentUartProperties.uartCharSize = (props.uartCharSize == CharDefault)   ? Char8    : props.uartCharSize;
            this->currentUartProperties.uartParity   = (props.uartParity   == ParityDefault) ? ParityNo : props.uartParity;
            this->currentUartProperties.uartStopBits = (props.uartStopBits == StopDefault)   ? StopOne  : props.uartStopBits;
            return true;
        }
        else
        {
            this->uartErrors->baudRateError = true;
            this->uartErrors->charSizeError = true;
            this->uartErrors->parityError   = true;
            this->uartErrors->stopBitsError = true;
            return false;
        }


    }





    bool        BlackUART::isOpen()
    {
        return this->isOpenFlag;
    }

    bool        BlackUART::isClose()
    {
        return !(this->isOpenFlag);
    }




    bool        BlackUART::fail()
    {
        return (this->uartErrors->dtError or
                this->uartErrors->readError or
                this->uartErrors->writeError or
                this->uartErrors->flushError or
                this->uartErrors->openError or
                this->uartErrors->closeError or
                this->uartErrors->directionError or
                this->uartErrors->baudRateError or
                this->uartErrors->charSizeError or
                this->uartErrors->stopBitsError or
                this->uartErrors->parityError
                );
    }

    bool        BlackUART::fail(BlackUART::flags f)
    {
        if(f==dtErr)            { return this->uartErrors->dtError;         }
        if(f==readErr)          { return this->uartErrors->readError;       }
        if(f==writeErr)         { return this->uartErrors->writeError;      }
        if(f==flushErr)         { return this->uartErrors->flushError;      }
        if(f==closeErr)         { return this->uartErrors->closeError;      }
        if(f==openErr)          { return this->uartErrors->openError;       }
        if(f==directionErr)     { return this->uartErrors->directionError;  }
        if(f==parityErr)        { return this->uartErrors->parityError;     }
        if(f==baudRateErr)      { return this->uartErrors->baudRateError;   }
        if(f==charSizeErr)      { return this->uartErrors->charSizeError;   }
        if(f==stopBitsErr)      { return this->uartErrors->stopBitsError;   }

        return true;
    }




    BlackUART&      BlackUART::operator<<(std::string &writeFromThis)
    {
        if(::write(this->uartFD,writeFromThis.c_str(),writeFromThis.size() ) > 0)
        {
            this->uartErrors->writeError = false;
        }
        else
        {
            this->uartErrors->writeError = true;
        }

        return *this;
    }

    BlackUART&      BlackUART::operator>>(std::string &readToThis)
    {
        std::string tempReadBuffer;
        tempReadBuffer.resize(this->readBufferSize);

        int readSize = ::read(this->uartFD, &tempReadBuffer[0], tempReadBuffer.size() );
        if( readSize > 0)
        {
            this->uartErrors->readError = false;
            tempReadBuffer.resize(readSize);
            readToThis = tempReadBuffer;
        }
        else
        {
            this->uartErrors->readError = true;
            readToThis = UART_READ_FAILED;
        }

        return *this;
    }

//}

/*
 * SimpleGPIO.cpp
 */


/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}

	if (out_flag == OUTPUT_PIN)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);

	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int gpio_set_value(unsigned int gpio, PIN_VALUE value)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	if (value==LOW)
		write(fd, "0", 2);
	else
		write(fd, "1", 2);

	close(fd);
	return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd;
	char buf[MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}

	close(fd);
	return 0;
}


/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}

	write(fd, edge, strlen(edge) + 1);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(unsigned int gpio)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
	return close(fd);
}


/****************************************************************
 * gpio_omap_mux_setup - Allow us to setup the omap mux mode for a pin
 ****************************************************************/
int gpio_omap_mux_setup(const char *omap_pin0_name, const char *mode)
{
	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), SYSFS_OMAP_MUX_DIR "%s", omap_pin0_name);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("failed to open OMAP_MUX");
		return fd;
	}
	write(fd, mode, strlen(mode) + 1);
	close(fd);
	return 0;
}
