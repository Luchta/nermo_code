#ifndef GAIT_CTRL_H
#define GAIT_CTRL_H

#include <thread>
#include <atomic>
#include "NRPMouse.h"
#include <stdbool.h>

enum state {QUIT, INIT,GotoSTOP, STOP, FORWARD, BACKWARD, LEFT, RIGHT, HUMP, FWD2};

class GaitCtrl
{
public:
    GaitCtrl(){
        //        std::thread walker(&gait_ctrl::state, std::ref(gait), this);
        //        walker.detach();
    }

    void startWalker(std::atomic<state>* gait,std::atomic<int> *pace) {
        std::cout << "starting .Walker thread"<<std::endl;
        t1 = std::thread ([=] { walker(gait, pace); });
        t1.detach();
        std::cout << "Walker thread detached"<<std::endl;
    }


private:
    //std::thread walker;
    std::thread t1;
    void walker(std::atomic<state>* gait, std::atomic<int> *pace);   //regulates walker state inside thread

    bool left = false;
    int slowness = 1;

    void rightStepFwd();   //standard gait, right step
    void leftStepFwd();    //standard gait left step
    void rightStepBkwd();   //standard gait, right step
    void leftStepBkwd();    //standard gait left step
    void initpos();     //go to start position for walking

    void slowWalk();        //walk with slowness value
    void rightStepLeft();   //tourn right from left step
    void leftStepRight();   //turn left from right step
    void leftStepLeftTight();   //tight turn
    void rightStepLeftTight();  //tight turn
    void rightStepCenter(); //go to center position from right
    void leftStepCenter();  //g to center position from left

    //up down movments
    void hump();
    void humpd();
    void humpe();
    void humpa();
    void humpb();
    void humpc();
};

#endif // GAIT_CTRL_H
