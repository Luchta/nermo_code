/*mouse hardware layer class
 *
 * Author: Peer Lucas
 * Email: peer.lucas@tum.de
 *
 * Last edited: 20.03.2017
 *
 */

#ifndef HwEdison_h
#define HwEdison_h

//standard includes
#include <unistd.h>
#include <iostream>
#include <stdio.h>

#include "HwInterface.h"

class HwEdison : public HwInterface
{
public:
    // Constructor and destructor
    HwEdison();
    ~HwEdison(){} //

    //Sparkfun GPIO-Block Pin Defines
#define GP44    0
#define GP45    1
#define GP46    2
#define GP47    3
#define GP48    4
#define GP49    5
#define GP15    6
#define GP14    7
    //UART1
    //#define GP131   35 //UART-1-TX
    //#define GP130   26 //UART-1-RX
    //#define GP129   25 //UART-1-RTS
    //#define GP128   13 //UART-1-CTS
#define PWM0    0
#define PWM1    1
#define PWM2    2
#define PWM3    3

//#define Ch0 0
//#define Ch1 1
//#define Ch2 2
//#define Ch3 3
    //enum AnalogCh : uint8_t {Ch0, Ch1, Ch2, Ch3};

    // functions
    void init();

    //Servo functions
    void setServoAngle(uint8_t servo, uint16_t angle);      //setting servo angles (not actual angele --see SetAngleRange)
    void setServoDutycycle(uint8_t servo, float value);     //setting dutycycle for servo
    //Setup
    void setDutyRange(int16_t minp, int16_t maxp);          //Setup for range of dutycycle
    void setAngleRange(int16_t mina, int16_t maxa);         //Setup for Range of angular movment (not actual angles, 1deg depends on dutycycle range)
    void getDutyRange(uint16_t* minp, uint16_t* maxp);      //Get for range of dutycycle
    void getAngleRange(int16_t* mina, int16_t* maxa);       //Get Range of angular movment
    void getChlTime(uint8_t channel, uint16_t *start, uint16_t *stop); //Get start and stop time of servo channel


    //GPIO functions
    void gpioOut(int pin, int value);
    void gpioIN(int pin, int *in);
    void gpioSetOut(int pin);
    void gpioSetIn(int pin);

    //ADC function
    void analogRead(int ch, float* in);

    //PWM Pins
    void pwm(int pin, int position);


    void imuDump();
private:
#define NUMGPIOS 8
#define NUMPWM 4

    //Sparkfun GPIO-Block Value Defines
#define MRAA_PWM_ENABLE     1
#define MRAA_PWM_DISABLE    0
#define MRAA_PWM_PULSE      20

    //Sparkfun GPIO-Block Pin Defines
    //MRAA GPIO Mapping
#define GPIO44    31
#define GPIO45    45
#define GPIO46    32
#define GPIO47    46
#define GPIO48    33
#define GPIO49    47
#define GPIO15    48
#define GPIO14    36

    //UART1
#define GPIO131   35 //UART-1-TX
#define GPIO130   26 //UART-1-RX
#define GPIO129   25 //UART-1-RTS
#define GPIO128   13 //UART-1-CTS

    //PWM3
#define GPIO183   21
    //PWM2
#define GPIO182   0
    //PWM0
#define GPIO12    20
    //PWM1
#define GPIO13    14


    //Sparkfun PWM-Block Pin Defines

    //Range Defaults
    int DEFAULT_MAX_PULSE = 620;
    int DEFAULT_MIN_PULSE = 120;

    //Angles are not accurate,
    //the angle function only sets the number of steps between min and max pulse
    int DEFAULT_MIN_ANGLE = 0;
    int DEFAULT_MAX_ANGLE = 270;

    //++               *               ++
    //++              ***              ++
    //++               *               ++
    //++ <270-0>     2 * 5     <0-270> ++
    //++ <0-270>     3 * 4     <270-0> ++
    //++              ***              ++
    //++              ***              ++
    //++              ***              ++
    //++ <270-0>     1 * 6     <0-270> ++
    //++ <0-270>     0 * 7     <270-0> ++
    //++               *               ++
    //++               *               ++
    //++               *               ++
    //++               *               ++
    //++               *               ++
    //++               *               ++
    //+++++++++++++++++++++++++++++++++++

#define HIP_LEFT_HINDFOOT    0
#define FOOT_LEFT_HINDFOOT   1
#define HIP_RIGHT_HINDFOOT   7
#define FOOT_RIGHT_HINDFOOT  6
#define HIP_LEFT_FRONTFOOT   2
#define FOOT_LEFT_FRONTFOOT  3
#define HIP_RIGHT_FRONTFOOT  5
#define FOOT_RIGHT_FRONTFOOT 4

    //Safe Position Defaults
    //Left Hindfoot
#define DEFAULT_HIND_LT_HIP 225
#define DEFAULT_HIND_LT_FOOT 0

    //Right Hindfoot
#define DEFAULT_HIND_RT_HIP 11
#define DEFAULT_HIND_RT_FOOT 270

    //Left Frontfoot
#define DEFAULT_FRNT_LT_HIP 10
#define DEFAULT_FRNT_LT_FOOT 240

    //Right Frontfoot
#define DEFAULT_FRNT_RT_HIP 60
#define DEFAULT_FRNT_RT_FOOT 35

/*

    //Left Hindfoot Positions
    #define POS_HIND_LT_HIP_CENTER  225 //230

    //Right Hindfoot Positions
    #define POS_HIND_RT_HIP_CENTER  40  //

    //Left Frontfoot Positions
    #define POS_FORE_LT_HIP_CENTER  40 //

    //Right Frontfoot Positions

    #define POS_FORE_RT_HIP_CENTER  220 //

*/


    void disableLegs();
    void enableLegs();
    void pwmBlockInit();
    void adcBlockInit();
    void pwmPinsInit();
};

#endif
