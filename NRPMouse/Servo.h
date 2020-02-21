
#ifndef servo_h
#define servo_h

#include "HwInterface.h"
#include <atomic>
#include <thread>

#include "Anatomy.h"
#define MAXDURATION 255
#define MINDURATION 1

class Servo
{
public:
    Servo(HwInterface* hw);

    ~Servo();

    void moveLeg(legservo servo, uint16_t pos, uint16_t duration = MINDURATION);

private:

#define NUMPARAMETERS 2
#define NUMSERVOS 8
#define POSITION 0
#define DURATION 1
#define CLOSETHREAD 666

    void startServo(std::atomic<int>* ctrl, HwInterface* hw, legservo pin, bool inverted) {
        //std::cout << "starting Servo thread"<<std::endl;
        t1 = std::thread ([=] { move(ctrl, hw, pin, inverted); });
        t1.detach();
        //std::cout << "Servo thread detached"<<std::endl;
    }


    void move(std::atomic<int>* ctrl, HwInterface* hw, legservo pin, bool _inverted);    //move foot to position pos with foot in lift pos/ down pos


    std::atomic<int>* hindleft_hip     = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* hindleft_foot    = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* hindright_hip    = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* hindright_foot   = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* foreleft_hip     = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* foreleft_foot    = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* foreright_hip    = new std::atomic<int>[NUMPARAMETERS];
    std::atomic<int>* foreright_foot   = new std::atomic<int>[NUMPARAMETERS];

    //std::atomic<int> wtf = new std::atomic<int>[NUMSERVOS][NUMPARAMETERS];

    std::thread t1;
};


#endif
