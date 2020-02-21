
#include <mutex>

#include "HwEdison.h"
#include "NRPMouse.h"
#include "Servo.h"

std::mutex safety;
Servo * _legs = NULL;

NRPMouse::NRPMouse()
{
    _hw = new HwEdison();
    _legs = new Servo(_hw);
}

//void NRPMouse::moveto(legservo servo, uint16_t pos, uint16_t speed)
//{
//    safety.lock();
//    _hw->setServoAngle(servo, pos);
//    safety.unlock();
//}

NRPMouse &NRPMouse::Instance()
{
    //create unique object and return as reference.
    static NRPMouse instance;
    return instance;
}

void NRPMouse::moveLeg(legservo servo, uint16_t angle, uint16_t duration)
{
    safety.lock();
    _legs->moveLeg(servo, angle, duration);
    safety.unlock();
}

void NRPMouse::legStatus(leg ch, float* status)
{
    safety.lock();
    _hw->analogRead(ch, status);
    safety.unlock();
}



void NRPMouse::squeak(int beeps, int speedms){ //0-255
    safety.lock();
    _hw->gpioSetOut(GP14);
    for (int i = 0; i < beeps; i++){
        for (int i = 0; i < 255; i++){
            _hw->gpioOut(GP14,1);
            usleep((355-i)+  (speedms*2));
            _hw->gpioOut(GP14,0);
            usleep((355-i)+  (speedms*2));
        }
        usleep(30000);
    }
    safety.unlock();
}

//void NRPMouse::LEDNeoPixel(int R,int G,int B)
//{
//    int i=0;

//    //R = R & 0xFF;
//    //G = G & 0xFF;
//    //B = B & 0xFF;

//    _hw->gpioSetOut(GP44);

//    for(i=0;i<8;i++){
//        if( (G & (0x80>>i)) ==0) NRPMouse::LowCode();
//        else NRPMouse::HiCode();
//    }
//    for(i=0;i<8;i++){
//        if( (R & (0x80>>i)) ==0)  NRPMouse::LowCode();
//            else NRPMouse::HiCode();
//    }
//    for(i=0;i<8;i++){
//        if( (B & (0x80>>i)) ==0) NRPMouse::LowCode();
//        else NRPMouse::HiCode();
//    }
//}

//void NRPMouse::LowCode()
//{
//    //---- T0H ----
//    _hw->gpioOut(GP44,1);
//    //---- T0L ----
//    _hw->gpioOut(GP44,0);
//    _hw->gpioOut(GP44,0);
//    _hw->gpioOut(GP44,0);
//    _hw->gpioOut(GP44,0);
//}
//void NRPMouse::HiCode()
//{
//    //---- T1H ----
//    _hw->gpioOut(GP44,1);
//    _hw->gpioOut(GP44,1);
//    _hw->gpioOut(GP44,1);
//    _hw->gpioOut(GP44,1);
//    //---- T1L ----
//    _hw->gpioOut(GP44,0);

//}

//void NRPMouse::Reset()
//{
//    int i=0;
//    for(i=0;i<1000;i++) _hw->gpioOut(GP44,0);
//}


