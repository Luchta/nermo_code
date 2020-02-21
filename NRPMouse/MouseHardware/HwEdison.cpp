
#include <mutex>
#include <array>

//external libraries
#include "mraa.hpp"
#include "SparkFun_pca9685_Edison.h"//PWM
#include "SparkFunADS1015.h"        //ADC
#include "SFE_LSM9DS0.h"          //IMU

#include "HwEdison.h"

std::mutex mutex;

//PWM_BLOCK****************************************************************************************************
//init I2C Bus to servos on pca9685 PWM_Board
mraa::I2c* pwmblock_i2c = new mraa::I2c(1); // Create and tell the I2c object which bus it's on.
pca9685 pwmblock(pwmblock_i2c, 0x40);       // 0x40 is the default address for the PCA9685.

//ADC_BLOCK****************************************************************************************************
mraa::I2c* adc_i2c2 = new mraa::I2c(1); //create I²C Object and set channel
ads1015 adc(adc_i2c2, 0x48);            //create adc object from sparkfun (four different addresses available)

//IMU_BLOCK****************************************************************************************************
LSM9DS0 *imu = new LSM9DS0(0x6B, 0x1D);
uint16_t imuResult = imu->begin();

//GPIO*********************************************************************************************************
//init Gpio pins
mraa_gpio_context gpios[NUMGPIOS] = {
    mraa_gpio_init(GPIO44),
    mraa_gpio_init(GPIO45),
    mraa_gpio_init(GPIO46),
    mraa_gpio_init(GPIO47),
    mraa_gpio_init(GPIO48),
    mraa_gpio_init(GPIO49),
    mraa_gpio_init(GPIO15),
    mraa_gpio_init(GPIO14)
};
mraa_pwm_context pwms[NUMPWM] = {
    mraa_pwm_init(GPIO12),
    mraa_pwm_init(GPIO13),
    mraa_pwm_init(GPIO182),
    mraa_pwm_init(GPIO183)
};
std::array<bool,NUMGPIOS> in = {false};
std::array<bool,NUMGPIOS> out = {false};

/*
mraa_gpio_context pin44 = mraa_gpio_init(GPIO44);
mraa_gpio_context pin45 = mraa_gpio_init(GPIO45);
mraa_gpio_context pin46 = mraa_gpio_init(GPIO46);
mraa_gpio_context pin47 = mraa_gpio_init(GPIO47);
mraa_gpio_context pin48 = mraa_gpio_init(GPIO48);
mraa_gpio_context pin49 = mraa_gpio_init(GPIO49);
mraa_gpio_context pin15 = mraa_gpio_init(GPIO15);
mraa_gpio_context pin14 = mraa_gpio_init(GPIO14);
//init pwm pins
mraa_pwm_context pwm_3 = mraa_pwm_init(GPIO183); //init pin via mraa library
mraa_pwm_context pwm_2 = mraa_pwm_init(GPIO182);
mraa_pwm_context pwm_0 = mraa_pwm_init(GPIO12);
mraa_pwm_context pwm_1 = mraa_pwm_init(GPIO13);
*/

////**************************************************************************************************************************************
/// Init and Constructor functions
/// **************************************************************************************************************************************

HwEdison::HwEdison()
{
    mraa_init();
    HwEdison::pwmBlockInit();
    HwEdison::adcBlockInit();
    HwEdison::pwmPinsInit();
    std::cout<<std::hex<<"IMU Chip ID: 0x"<<imuResult<<std::dec<<" (should be 0x49d4)"<<std::endl;

}

void HwEdison::pwmBlockInit(void)
{
    //Enable Servo mode for the pca9685 PWM_Board
    pwmblock.enableServoMode();

    //SET Default Angle and Pulse limits
    pwmblock.setServoAngleLimits(DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE);
    pwmblock.setServoAnglePulseLimits(DEFAULT_MIN_PULSE, DEFAULT_MAX_PULSE);
    //SET Default Angles
    pwmblock.setChlAngle(FOOT_LEFT_FRONTFOOT, DEFAULT_FRNT_LT_FOOT);
    pwmblock.setChlAngle(HIP_LEFT_FRONTFOOT, DEFAULT_FRNT_LT_HIP);
    pwmblock.setChlAngle(FOOT_RIGHT_FRONTFOOT, DEFAULT_FRNT_RT_FOOT);
    pwmblock.setChlAngle(HIP_RIGHT_FRONTFOOT, DEFAULT_FRNT_RT_HIP);
    pwmblock.setChlAngle(FOOT_LEFT_HINDFOOT, DEFAULT_HIND_LT_FOOT);
    pwmblock.setChlAngle(HIP_LEFT_HINDFOOT, DEFAULT_HIND_LT_HIP);
    pwmblock.setChlAngle(FOOT_RIGHT_HINDFOOT, DEFAULT_HIND_RT_FOOT);
    pwmblock.setChlAngle(HIP_RIGHT_HINDFOOT, DEFAULT_HIND_RT_HIP);
}

void HwEdison::adcBlockInit(void)
{
    // There are 6 settable ranges:
    //  _0_256V - Range is -0.256V to 0.255875V, and step size is 125uV.
    //  _0_512V - Range is -0.512V to 0.51175V, and step size is 250uV.
    //  _1_024V - Range is -1.024V to 1.0235V, and step size is 500uV.
    //  _2_048V - Range is -2.048V to 2.047V, and step size is 1mV.
    //  _4_096V - Range is -4.096V to 4.094V, and step size is 2mV.
    //  _6_144V - Range is -6.144V to 6.141V, and step size is 3mV.
    // The default setting is _2_048V.
    // NB!!! Just because FS reading is > 3.3V doesn't mean you can take an
    //  input above 3.3V! Keep your input voltages below 3.3V to avoid damage!
    adc.setRange(_4_096V);
}

void HwEdison::init(void)//NOT IN USE
{
    //Enable Servo mode for the pca9685 PWM_Board
    pwmblock.enableServoMode();
    //SET Default Angle and Pulse limits
    pwmblock.setServoAngleLimits(DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE);
    pwmblock.setServoAnglePulseLimits(DEFAULT_MIN_PULSE, DEFAULT_MAX_PULSE);
}

void HwEdison::pwmPinsInit()
{
    for(uint8_t i=0; i<NUMPWM; i++)
    {
        mraa_pwm_enable(pwms[i], MRAA_PWM_DISABLE );  //disable output
        mraa_pwm_period_ms(pwms[i], MRAA_PWM_PULSE);  //set pwm pulse length
        mraa_pwm_enable(pwms[i], MRAA_PWM_ENABLE );   //enalbe output
    };
    /*
    mraa_pwm_enable(pwm_0, MRAA_PWM_DISABLE );    //disable output
    mraa_pwm_period_ms(pwm_0, MRAA_PWM_PULSE);   //set pwm pulse length
    mraa_pwm_enable(pwm_0, MRAA_PWM_ENABLE );   //enalbe output

    mraa_pwm_enable(pwm_1, MRAA_PWM_DISABLE );    //disable output
    mraa_pwm_period_ms(pwm_1, MRAA_PWM_PULSE);   //set pwm pulse length
    mraa_pwm_enable(pwm_1, MRAA_PWM_ENABLE );   //enalbe output

    mraa_pwm_enable(pwm_2, MRAA_PWM_DISABLE );    //disable output
    mraa_pwm_period_ms(pwm_2, MRAA_PWM_PULSE);   //set pwm pulse length
    mraa_pwm_enable(pwm_2, MRAA_PWM_ENABLE );   //enalbe output

    mraa_pwm_enable(pwm_3, MRAA_PWM_DISABLE );    //disable output
    mraa_pwm_period_ms(pwm_3, MRAA_PWM_PULSE);   //set pwm pulse length
    mraa_pwm_enable(pwm_3, MRAA_PWM_ENABLE );   //enalbe output
*/
}

////****************************************************************************************************************************************
/// GPIO_Block functions
/// ****************************************************************************************************************************************

void HwEdison::gpioSetIn(int pin)
{
    for(int i=0; i<NUMGPIOS; i++)
    {
        if(i == pin)
        {
            mutex.lock();
            mraa_gpio_dir(gpios[pin], MRAA_GPIO_IN);
            in[pin] = true;
            out[pin] = false;
            mutex.unlock();
        }
    }
    return;
}

void HwEdison::gpioSetOut(int pin)
{
    for(int i=0; i<NUMGPIOS; i++)
    {
        if(i == pin)
        {
            mutex.lock();
            mraa_gpio_dir(gpios[pin], MRAA_GPIO_OUT);
            out[pin] = true;
            in[pin] = false;
            mutex.unlock();
        }
    }
    return;
}

void HwEdison::gpioOut(int pin, int value)
{
    for(int i=0; i<NUMGPIOS; i++)
    {
        if(i == pin && out[i])
        {
            mutex.lock();
            mraa_gpio_write(gpios[pin], value);
            mutex.unlock();
        }
    }
    /*
    if(value == 1|| value == 0){

        switch (pin) {
        case 44:
            mraa_gpio_dir(pin44, MRAA_GPIO_OUT);
            mraa_gpio_write(pin44, value);
            break;
        case 45:
            mraa_gpio_dir(pin45, MRAA_GPIO_OUT);
            mraa_gpio_write(pin45, value);
            break;
        case 46:
            mraa_gpio_dir(pin46, MRAA_GPIO_OUT);
            mraa_gpio_write(pin46, value);
            break;
        case 47:
            mraa_gpio_dir(pin47, MRAA_GPIO_OUT);
            mraa_gpio_write(pin47, value);
            break;
        case 48:
            mraa_gpio_dir(pin48, MRAA_GPIO_OUT);
            mraa_gpio_write(pin48, value);
            break;
        case 49:
            mraa_gpio_dir(pin49, MRAA_GPIO_OUT);
            mraa_gpio_write(pin49, value);
            break;
        case 15:
            mraa_gpio_dir(pin15, MRAA_GPIO_OUT);
            mraa_gpio_write(pin15, value);
            break;
        case 14:
            mraa_gpio_dir(pin14, MRAA_GPIO_OUT);
            mraa_gpio_write(pin14, value);
            break;
        default:
            std::cerr<<"wrong pin number in mouse_service::gpioOut"<<std::endl;
            break;
        }
    }
    else
        std::cerr<<"wrong value in mouse_service::gpioOut"<<std::endl;
*/
    return;
}

void HwEdison::gpioIN(int pin, int* in)
{
    for(int i=0; i<NUMGPIOS; i++)
    {
        if(i == pin && in[i])
        {
            mutex.lock();
            *in = mraa_gpio_read(gpios[pin]);
            mutex.unlock();
        }
    }
    /*
        switch (pin) {
        case 44:
            mraa_gpio_dir(pin44, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin44);
            break;
        case 45:
            mraa_gpio_dir(pin45, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin45);
            break;
        case 46:
            mraa_gpio_dir(pin46, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin46);
            break;
        case 47:
            mraa_gpio_dir(pin47, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin47);
            break;
        case 48:
            mraa_gpio_dir(pin48, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin48);
            break;
        case 49:
            mraa_gpio_dir(pin49, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin49);
            break;
        case 15:
            mraa_gpio_dir(pin15, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin15);
            break;
        case 14:
            mraa_gpio_dir(pin14, MRAA_GPIO_IN);
            *in = mraa_gpio_read(pin14);
            break;
        default:
            std::cerr<<"wrong pin number in mouse_service::gpioOut"<<std::endl;
            break;
        }
*/
    return;
}

void HwEdison::pwm(int pin, int position)
{
    for(int i=0; i<NUMPWM; i++)
    {
        if(i == pin)
        {
            mutex.lock();
            mraa_pwm_pulsewidth_us (pwms[i],position);
            mutex.unlock();
        }
    }
    return;
    /*
    switch (pin) {
    case 0:
        mraa_pwm_pulsewidth_us (pwm_0,position);
        break;
    case 1:
        mraa_pwm_pulsewidth_us (pwm_1,position);
        break;
    case 2:
        mraa_pwm_pulsewidth_us (pwm_2,position);
        break;
    case 3:
        mraa_pwm_pulsewidth_us (pwm_3,position);
        break;
    default:
        std::cout<<"wrong input in mouse_service::pwm pin = <0-3>"<<std::endl;
        break;
    }
*/
}

///****************************************************************************************************************************************
/// ADC_Block functions
///****************************************************************************************************************************************

void HwEdison::analogRead(int ch, float* in)
{
    mutex.lock();
    *in = adc.getResult(ch);
    mutex.unlock();
}

///****************************************************************************************************************************************
/// IMU_Block functions
///****************************************************************************************************************************************

void HwEdison::imuDump()
{
    imu->readAccel();
    imu->readMag();
    imu->readGyro();
    imu->readTemp();

    std::cout<<"-------------------------------------"<<std::endl;
    std::cout<<"Gyro x: "<<imu->calcGyro(imu->gx)<<" deg/s"<<std::endl;
    std::cout<<"Gyro y: "<<imu->calcGyro(imu->gy)<<" deg/s"<<std::endl;
    std::cout<<"Gyro z: "<<imu->calcGyro(imu->gz)<<" deg/s"<<std::endl;
    std::cout<<"Accel x: "<<imu->calcAccel(imu->ax)<<" g"<<std::endl;
    std::cout<<"Accel y: "<<imu->calcAccel(imu->ay)<<" g"<<std::endl;
    std::cout<<"Accel z: "<<imu->calcAccel(imu->az)<<" g"<<std::endl;
    std::cout<<"Mag x: "<<imu->calcMag(imu->mx)<<" Gauss"<<std::endl;
    std::cout<<"Mag y: "<<imu->calcMag(imu->my)<<" Gauss"<<std::endl;
    std::cout<<"Mag z: "<<imu->calcMag(imu->mz)<<" Gauss"<<std::endl;
}

/*
std::array< int, 3 > &GetMag( std::array< int, 3 > &arr ) {
    imu->readMag();
    arr[0] = imu->calcMag(imu->mx);
    arr[1] = imu->calcMag(imu->my);
    arr[2] = imu->calcMag(imu->mz);
    return arr;
}

std::array< int, 3 > &GetGyro( std::array< int, 3 > &arr ) {
    imu->readGyro();
    arr[0] = imu->calcGyro(imu->gx);
    arr[1] = imu->calcGyro(imu->gy);
    arr[2] = imu->calcGyro(imu->gz);
    return arr;
}

std::array< int, 3 > &GetAccel( std::array< int, 3 > &arr ) {
    imu->readAccel();
    arr[0] = imu->calcAccel(imu->ax);
    arr[1] = imu->calcAccel(imu->ay);
    arr[2] = imu->calcAccel(imu->az);
    return arr;
}
*/

///**************************************************************************************************************************************
///Servo_Block functions
///**************************************************************************************************************************************

void HwEdison::setServoAngle(uint8_t servo, uint16_t angle)
{
    mutex.lock();
    pwmblock.setChlAngle(servo, angle);
    mutex.unlock();
    return;
}

void HwEdison::setServoDutycycle(uint8_t servo, float value)
{
    mutex.lock();
    //std::cout<<"duty cycle range: <3-16%>, back <-1>"<<std::endl; //3-16 is a tested value with the HS-35HD Servo
    pwmblock.setChlDuty(servo, value);
    mutex.unlock();
    return;
}

void HwEdison::setAngleRange(int16_t mina, int16_t maxa)
{
    mutex.lock();
    pwmblock.setServoAngleLimits(mina, maxa);
    mutex.unlock();
    return;
}

void HwEdison::getAngleRange(int16_t* mina, int16_t* maxa)
{
    mutex.lock();
    pwmblock.getServoAngleLimits(mina, maxa);
    mutex.unlock();
    return;
}

void HwEdison::getChlTime(uint8_t channel, uint16_t *start, uint16_t *stop)
{
    mutex.lock();
    pwmblock.getChlTime(channel, start, stop);
    mutex.unlock();
    return;
}


void HwEdison::setDutyRange(int16_t minp, int16_t maxp)
{
    mutex.lock();
    pwmblock.setServoAnglePulseLimits(minp, maxp);
    mutex.unlock();
    return;
}

void HwEdison::getDutyRange(uint16_t* minp, uint16_t* maxp)
{
    mutex.lock();
    pwmblock.getServoAnglePulseLimits(minp, maxp);
    mutex.unlock();
    return;
}

void HwEdison::disableLegs()
{
    mutex.lock();
    mutex.unlock();
    return;
}

void HwEdison::enableLegs()
{
    mutex.lock();
    mutex.unlock();
}


