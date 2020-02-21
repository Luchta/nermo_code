/*mouse hardware interface definition
 *
 * Author: Peer Lucas
 * Email: peer.lucas@tum.de
 *
 * Last edited: 20.03.2017
 *
 */

#ifndef HwInterface_h
#define HwInterface_h

//standard includes
#include <unistd.h>
#include <iostream>
#include <stdio.h>

class HwInterface
{
public:
    virtual void init() = 0;

    //Servo functions
    virtual void setServoAngle(uint8_t servo, uint16_t angle) = 0;  //setting servo angles (not actual angele --see SetAngleRange)
    //Setup   
    virtual void setAngleRange(int16_t mina, int16_t maxa) = 0;      //Setup for Range of angular movment (not actual angles, 1deg depends on dutycycle range)
    virtual void getAngleRange(int16_t* mina, int16_t* maxa) = 0;        //Get Range of angular movment


    virtual void setServoDutycycle(uint8_t servo, float value) = 0; //setting dutycycle for servo
    virtual void setDutyRange(int16_t minp, int16_t maxp) = 0;         //Setup for range of dutycycle
    virtual void getDutyRange(uint16_t* minp, uint16_t* maxp) = 0;     //Get for range of dutycycle
    virtual void getChlTime(uint8_t channel, uint16_t *start, uint16_t *stop) = 0;  //Get start and stop time of servo channel


    //GPIO functions
    virtual void gpioOut(int pin, int value) = 0;
    virtual void gpioIN(int pin, int *in) = 0;
    virtual void gpioSetOut(int pin) = 0;
    virtual void gpioSetIn(int pin) = 0;

    //ADC function
    virtual void analogRead(int ch, float* in) = 0;

    //IMU function
    virtual void imuDump() = 0;

    //PWM Pins
    virtual void pwm(int pin, int position) = 0;

};

#endif
