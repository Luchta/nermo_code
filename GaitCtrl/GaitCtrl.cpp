
#include <iostream>
#include <unistd.h>

#include "GaitCtrl.h"
//#include "leg_service.h"
#include "mraa.hpp"


void GaitCtrl::walker(std::atomic<state> *gait, std::atomic<int> *pace){

    std::cout<<"walker started"<<std::endl;

    //start on stop postition
    int state = STOP;
    //go to init position, only called once




    //stay in loop until thread is closed
    do{
        state = *gait;
        slowness = *pace;

        switch (state) {
        case INIT:
            GaitCtrl::initpos();
            gait->store(STOP);
            std::cout<<"walker ready"<<std::endl;
            break;
        case GotoSTOP:
            if(left){
                GaitCtrl::leftStepCenter();
                left = false;
            }else{
                GaitCtrl::rightStepCenter();
                left = true;
            }
            gait->store(STOP);
            break;
        case STOP:
            //do nothing
            usleep(200000);//break for the processor
            break;
        case FORWARD:
            if(left){
                GaitCtrl::leftStepFwd();
                left = false;
            }else{
                GaitCtrl::rightStepFwd();
                left = true;
            }
            break;
        case BACKWARD:
            if(left){
                GaitCtrl::leftStepBkwd();
                left = false;
            }else{
                GaitCtrl::rightStepBkwd();
                left = true;
            }
            break;
        case LEFT:
            if(left){
                GaitCtrl::leftStepFwd();
                //GaitCtrl::leftStepLeftTight();
                left = false;
            }else{
                GaitCtrl::rightStepLeft();
                //GaitCtrl::rightStepLeftTight();
                left = true;
            }
            break;
        case RIGHT:
            if(left){
                GaitCtrl::leftStepRight();
                left = false;
            }else{
                GaitCtrl::rightStepFwd();
                left = true;
            }
            break;
        case FWD2:
            GaitCtrl::slowWalk();
            break;
        case HUMP:
            hump();
            break;
        case QUIT:
            std::cout<<"stop signal"<<std::endl;
            break;
        }

    }while(state != QUIT);

    std::cout<<"walker stopped"<<std::endl;

}



//function to stop all movement instantly (kill all threads)
//void GaitCtrl::eStop(void)
//{
//    std::cout<<"killing everyone"<<std::endl;
//    std::terminate();
//}

void GaitCtrl::initpos()
{

    //foreright
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot up
    usleep(350000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD); //Foreright Hip bkwd
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot down

    //hindleft
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot up
    usleep(350000);
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD); //Hindleft Hip bkwd
    usleep(200000);
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot down

    //foreleft
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP); //Foreleft Foot up
    usleep(350000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD); //Foreleft Hip fwd
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN); //Foreleft Foot down

    //hindright
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot up
    usleep(350000);
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);  //Hindright Hip fwd
    usleep(200000);
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);; //Hindright Foot down

    left = true;

}

void GaitCtrl::slowWalk()
{

    /*
     *|----------       |LH
     *|   ----------    |LV
     *|------      -----|RV 2/3-/1/3
     *|--       --------|RH 1/4-3/4
     *
     *   1 2   3   4 5  6 7
     *|--|-|---|---|-|  | |    |LH
     *|  | |---|---|-|--|-|    |LV
     *|--|-|---|   | |  |-|----|RV 2/3-/1/3
     *|--| |   |   |-|--|-|----|RH 1/4-3/4
     *
     *   1 2   3   4 5  6 7
     *|--|-|---|---|-u  f | u  |LH
     *|  d b---|---|-|--|-u f  |LV
     *|--|-|---u  f| d  b-|----|RV 2/3-/1/3
     *|--u |  f|  db-|--|-|----|RH 1/4-3/4
     *
     *
     * one Step:
     *
     *  Bkwd      Up   FwdDown
     * |b---------u    f  d  |
     *
     * overall stepd = 17
     * time for all steps as from standard gait = 1100000
     * step 1,2,3,4 (64706, 129412, 194228, 258824)
     */

#define ONE_TIMESTEP 64706
#define TWO_TIMESTEP 129412
#define THREE_TIMESTEP 194228
#define FOUR_TIMESTEP 258824

    //startpos

    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);   //Hindleft Foot Up
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD);      //Hindleft Hip Fwd

    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);     //Foreleft Foot Up
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);   //Foreright Hip Center

    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);  //Foreright Hip 2/3 Bkwd
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN);  //Foreright Foot Down

    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER);  //Hindright Hip 1/4 Bkwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);  //Hindright Foot Down


    //walk

    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD,40);  //Hindleft Hip Bkwd
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD);      //Foreleft Hip Fwd
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD,40); //Foreright Hip Bkwds
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD,40); //Hindright Hip Bkwd

    //walking loop start
    //1
    usleep(TWO_TIMESTEP);// 2/2
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP);    //RH Foot Up
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);   //LV Foot Down
    //2
    usleep(ONE_TIMESTEP);// 1/1
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD,40);  //LV Hip Bkwd
    //2.75
    usleep(TWO_TIMESTEP);// 2/3
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);     //RH Hip Fwd
    //3
    usleep(ONE_TIMESTEP);// 3/3
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP);    //RV Foot Up
    //3.5
    usleep(TWO_TIMESTEP);// 2/3
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);  //RH Foot Up
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD);     //RV Hip Fwds
    //4
    usleep(ONE_TIMESTEP);// 3/3
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD,40); //RH Hip Bkwd
    //5
    usleep(ONE_TIMESTEP);// 1/1
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP);     //LH Foot Up
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN);  //RV Foot Down
    //6
    usleep(TWO_TIMESTEP);// 2/2
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD,40); //RV Hip Bkwds
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD);      //LH Hip Fwd
    //7
    usleep(ONE_TIMESTEP);// 1/1
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);     //LV Foot Up
    //7.5
    usleep(TWO_TIMESTEP);// 2/4
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD);      //LV Hip Fwd
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);   //LH Foot Up
    //8
    usleep(TWO_TIMESTEP);// 4/4

    //walking loop stop

}

/*
     *|----------          |LH
     *|          ----------|LV
     *|----------          |RV
     *|          ----------|RH
     *
     */

//FORWARD++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void GaitCtrl::rightStepFwd()
{
    //foreright walk, foreleft lift
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_LIFT);  //Foreleft Hip bkwd to lift
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);  //Foreleft Foot Up
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD, 25); //Foreright Hip Bkwds
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD, 30); //Hindleft Hip Bkwd
    //getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, 80, 25); //Hindleft Foot semi up

    //set foreleft front and down
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD);  //Foreleft Hip Fwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD); //Hindright Hip Fwd
    //getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN,10);  //Foreleft Foot Down
    usleep(100000);//longer for better biomimeticity
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);  //Foreleft Foot Down
    //usleep(100000);
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN); //Hindright Foot Down
    usleep(300000);
    //usleep(200000);
}
void GaitCtrl::leftStepFwd()
{
    //foreleft walk foreright lift
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_LIFT); //Foreright Hip bkwd to lift
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot Up
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD, 25);  //Foreleft Hip Bkwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD, 30); //Hindright Hip Bkwd
    //getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, 160, 25); //Hindright Foot semi up

    //set foreright fwd and down
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD); //Hindleft Hip Fwd
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD); //Foreright Hip Fwd
    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot Down
    usleep(300000);
}

//BACKWARD++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void GaitCtrl::rightStepBkwd()
{
    //foreleft walk, foreright lift
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot Up
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD,40);  //Foreleft Hip Fwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD,40); //Hindright Hip Fwd

    //set foreright bkwd and down
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD); //Foreright Hip Bkwd
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD); //Hindleft Hip Bkwd
    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot Down
    usleep(300000);
}
void GaitCtrl::leftStepBkwd()
{
    //foreright walk, foreleft lift
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);  //Foreleft Foot Up
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD, 40); //Foreright Hip Fwd
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD, 40); //Hindleft Hip Fwd

    //set foreleft bkwd and down
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD);  //Foreleft Hip Bkwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD); //Hindright Hip Bkwd
    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);  //Foreleft Foot Down
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN); //Hindright Foot Down
    usleep(300000);
}

//Centering++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void GaitCtrl::rightStepCenter()
{
    //foreright walk, foreleft lift
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);  //Foreleft Foot Up
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER, 10); //Foreright Hip Bkwds
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER, 10); //Hindleft Hip Bkwd
    //getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, 80, 25); //Hindleft Foot semi up

    //set foreleft front and down
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);  //Foreleft Hip Fwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER); //Hindright Hip Fwd
    //getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN,10);  //Foreleft Foot Down
    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);  //Foreleft Foot Down
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN); //Hindright Foot Down
    usleep(300000);
}
void GaitCtrl::leftStepCenter()
{
    //foreleft walk foreright lift
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot Up
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER, 10);  //Foreleft Hip Bkwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER, 10); //Hindright Hip Bkwd
    //getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, 160, 25); //Hindright Foot semi up

    //set foreright fwd and down
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER); //Hindleft Hip Fwd
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER); //Foreright Hip Fwd
    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot Down
    usleep(300000);
}

//TURN+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TURN LEFT++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void GaitCtrl::rightStepLeft()//left side shorter walk !!no change for leftstep!!
{
    //foreright walk, foreleft lift
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);  //Foreleft Foot Up
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD,40); //Foreright Hip Bkwds
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_TURN,40); //Hindleft Hip TURN

    //set foreleft front and down
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_TURN);  //Foreleft Hip TURN
    //getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD);  //Foreleft Hip TURN

    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD); //Hindright Hip Fwd
    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);  //Foreleft Foot Down
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN); //Hindright Foot Down
    usleep(300000);
}
//TURN RIGHT++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void GaitCtrl::leftStepRight()//left side shorter walk !!no change for rightstep!!
{
    //foreleft walk foreright lift
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot Up
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot Up
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD,40);  //Foreleft Hip Bkwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_TURN,40); //Hindright Hip Turn

    //set foreright fwd and down
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD); //Hindleft Hip Fwd
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_TURN); //Foreright Hip Turn

    usleep(100000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot Down
    usleep(300000);
}

////TURN LEFT TIGHT++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//void GaitCtrl::rightStepLeftTight()//left side shorter walk !!no change for leftstep!!
//{
//    //foreright walk, foreleft lift
//    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);  //Foreleft Foot Up
//    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot Up
//    usleep(200000);
//    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD,40); //Foreright Hip Bkwds
//    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD,40); //Hindleft Hip Fwd

//    //set foreleft front and down
//    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD);  //Foreleft Hip Bkwd
//    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD); //Hindright Hip Fwd
//    usleep(100000);
//    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);  //Foreleft Foot Down
//    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN); //Hindright Foot Down
//    usleep(300000);
//}
//void GaitCtrl::leftStepLeftTight()//left side shorter walk !!no change for rightstep!!
//{
//    //foreleft walk foreright lift
//    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot Up
//    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot Up
//    usleep(200000);
//    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD,40);  //Foreleft Hip Fwd
//    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD,40); //Hindright Hip Bkwd

//    //set foreright fwd and down
//    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD); //Hindleft Hip Bkwd
//    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD); //Foreright Hip Fwd

//    usleep(100000);
//    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot Down
//    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot Down
//    usleep(300000);
//}


//glorious Humping and jumping++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void GaitCtrl::hump()
{
    //GaitCtrl::humpa();
    //GaitCtrl::humpb();
    //GaitCtrl::humpc();
    GaitCtrl::humpd();
    //GaitCtrl::humpe();

}

void GaitCtrl::humpa()
{
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);   //Foreleft Foot Down
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN);  //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);  //Hindright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);   //Hindleft Foot Down

    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER-10);    //Foreleft Hip CENTER+10
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER+10);   //Foreright Hip CENTER+10
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER);       //Hindleft Hip CENTER
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER);      //Hindright Hip CENTER

    usleep(200000);

    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);       //Foreleft Foot CENTER
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);      //Foreright Foot CENTER
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER-10);    //Hindleft Foot CENTER+10
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER+10);   //Hindright Foot CENTER+10

    usleep(200000);
}

void GaitCtrl::humpb()
{
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);    //Foreleft Hip CENTER+10
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);   //Foreright Hip CENTER+10
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER);       //Hindleft Hip CENTER
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER);      //Hindright Hip CENTER

    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN-100,10);   //Foreleft Foot Down
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN+100.10);  //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN-100,10);  //Hindright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN+100, 10);   //Hindleft Foot Down
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);   //Foreleft Foot Down
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN);  //Foreright Foot Down
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);  //Hindright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);   //Hindleft Foot Down
    usleep(200000);
}

void GaitCtrl::humpc()
{
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);    //Foreleft Hip CENTER+10
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);   //Foreright Hip CENTER+10
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER);       //Hindleft Hip CENTER
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER);      //Hindright Hip CENTER

    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP,10);  //Hindright Foot up
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP, 10);   //Hindleft Foot up
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);    //Foreleft Hip CENTER
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);   //Foreright Hip CENTER
    usleep(200000);

    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);  //Hindright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);   //Hindleft Foot Down
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD);    //Foreleft Hip Bkwd
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD);   //Foreright Hip Bkwd
    usleep(200000);
}

void GaitCtrl::humpd()
{
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);    //Foreleft Hip CENTER+10
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);   //Foreright Hip CENTER+10
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD);       //Hindleft Hip CENTER
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);      //Hindright Hip CENTER

    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN-100,10);  //Hindright Foot up
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN+100, 10);   //Hindleft Foot up
    usleep(200000);
    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);  //Hindright Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);   //Hindleft Foot Down
    usleep(200000);
}

void GaitCtrl::humpe()
{
    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_CENTER);    //Foreleft Hip CENTER+10
    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_CENTER);   //Foreright Hip CENTER+10
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER);       //Hindleft Hip CENTER
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER);      //Hindright Hip CENTER

    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP, slowness+5);  //Hindright Foot up
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP, slowness+5);   //Hindleft Foot up
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_CENTER, slowness);    //Foreleft Hip CENTER
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_CENTER, slowness);   //Foreright Hip CENTER
    usleep(200000);
    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN, slowness);  //Hindright Foot Down
    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN, slowness);   //Hindleft Foot Down
    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD, slowness);    //Foreleft Hip Bkwd
    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD, slowness);   //Foreright Hip Bkwd
    usleep(200000);
}
