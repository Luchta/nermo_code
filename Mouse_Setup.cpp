
/*Simple terminal based mouse control programm for Sero setup, pin control and walking tests
 *
 * Author: Peer Lucas
 * Email: peer.lucas@tum.de
 *
 * Last edited: 06.04.2017
 *
 */


#include "HwInterface.h"
#include "Mouse_Setup.h"
//#include <atomic>

#include "GaitCtrl.h"



//needed variables and objects
NRPMouse &mouse = NRPMouse::Instance();   //create mouse
GaitCtrl* walker = new GaitCtrl();        //create gait controller
std::atomic<state> gait = {STOP};           //atomic for gait selection
std::atomic<int> pace = {1};                //atomic for movement duration

//hardware only needed for setup menu
HwInterface* hardware;

bool isgood = false;

void MouseSetup::init(void)
{
    hardware = getNrpMouseInstance().request_hw_accesss(); //request direkt hardware access to all hardware functions (for setup only never use for final design)
    //gait.store(INIT); //init walking position
    usleep(600000);

}


MouseSetup::MouseSetup()
{
    std::cout << "Starting Mouse_Setup"<<std::endl;
}

//int main(void) {
//    std::cout << "Hello Edison!\n";
//    std::cout << "Starting Init"<<std::endl;
//    init();

//    mainmenu();

//    return MRAA_SUCCESS;
//}


///****************************************************************************************************************************************
/// Main Menu
///****************************************************************************************************************************************

void MouseSetup::mainMenu(void){
    char choice1;
    int done = 0;

    std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
    std::cout<<"++       NRPMouse Control       ++"<<std::endl;
    std::cout<<"++         Version: 0.5          ++"<<std::endl;
    std::cout<<"++     Autohor: Peer Lucas       ++"<<std::endl;
    std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

    do {
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++           Main Menu           ++"<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout << "<1> Walking\n";
        std::cout << "<2> Servos\n";
        std::cout << "<3> GPIO\n";
        std::cout << "<4> Sensors\n";
        std::cout << "<q> Quit\n";

        do{
            std::cout << "choose: ";
            std::cin >> choice1;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        switch(choice1) {
        case '1':

            walking();
            break;

        case '2':
            servos();
            break;

        case '3':
            gpio();
            break;
        case '4':
            sense();
            break;

        case 'q':
            done = 1;
            break;

        default:
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;
            break;
        }
    } while(done != 1);

    return;
}

///***************************************************************************************************************************************
/// GPIO Settings and Control
///***************************************************************************************************************************************

void MouseSetup::gpio(void)
{
    char choicegp;
    int pin, value;
    int done = 0;

    do {
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++           GPIO Menu           ++"<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

        std::cout<<"<w> write pin"<<std::endl;
        std::cout<<"<r> read pin"<<std::endl;
        std::cout<<"<p> set pwm"<<std::endl;
        std::cout<<"<c> chirp"<<std::endl;
        //std::cout<<"<l> rgb led"<<std::endl;
        std::cout<<"<q> quit "<<std::endl;

        do{
            std::cout << "choose: ";
            std::cin >> choicegp;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        switch(choicegp) {
        case 'w':
            std::cout<<"pin <> value <0/1>"<<std::endl;
            std::cout<<"set: ";
            std::cin >> pin >> value;
            hardware->gpioOut(pin, value);
            break;
        case 'r':
            std::cout<<"pin: ";
            std::cin >> pin;
            hardware->gpioIN(pin, &value);
            std::cout<<"value: "<<std::dec<<value<<std::endl;
            break;
        case 'p':
            std::cout<<"pin <0-3> value <500-2100>"<<std::endl;
            std::cout<<"set: ";
            std::cin >> pin >> value;
            hardware->pwm(pin, value);
            break;
        case 'c':
            // Chirp for ready
            mouse.squeak(1,50);
            mouse.squeak(1,255);
            mouse.squeak(3,0);
            // chirp(1, 50);
            // chirp(1, 255);
            // chirp(3, 0);
            break;
            //        case 'l':
            //            int r;
            //            //std::cout<<"r g b <0-255>"<<std::endl;
            //            std::cout<<"set: ";
            //            std::cin >> r;
            //            switch(r){

            //            case '1':
            //                mouse.LEDNeoPixel(255,0,0);
            //                break;
            //            case '2':
            //                mouse.LEDNeoPixel(0,255,0);
            //                break;
            //            case '3':
            //                mouse.LEDNeoPixel(0,0,255);
            //                break;
            //            case '4':
            //                mouse.LEDNeoPixel(0,0,0);
            //                break;
            //            case '5':
            //                mouse.LEDNeoPixel(255,255,255);
            //                break;
            //            default:
            //                mouse.Reset();
            //                break;
            //            }

            //            //mouse.LEDNeoPixel(r,g,b);
            //            break;
        case 'q':
            std::cout << "bye!\n";
            done = 1;
            break;
        default:
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;
            break;
        }

    } while(done != 1);

    return;
}

///***************************************************************************************************************************************
/// Sensor Settings and Control
///***************************************************************************************************************************************

void MouseSetup::sense(void)
{
    char choiceS;
    int pin;
    float value;
    int done = 0;

    do {
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++          Sensor Menu          ++"<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

        std::cout<<"<1> dump IMU"<<std::endl;
        std::cout<<"<2> read analog pin"<<std::endl;
        std::cout<<"<q> quit "<<std::endl;

        do{
            std::cout << "choose: ";
            std::cin >> choiceS;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        switch(choiceS) {

        case '1':
            for(int i=0; i<100; i++){
                hardware->imuDump();
                sleep(1);
            }
            break;

        case '2':
            do{
                std::cout << "pin: ";
                std::cin >> pin;
                isgood = std::cin.good();
                std::cin.clear();
                std::cin.ignore(999999,'\n');
            } while (!isgood);
            for(int i=0; i<100; i++){
                mouse.legStatus(intToLeg(pin),&value);
                std::cout<<"value: "<<std::dec<<value<<std::endl;
                sleep(1);
            }
            break;

        case 'q':
            std::cout << "bye!\n";
            done = 1;
            break;

        default:
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;
            break;
        }

    } while(done != 1);

    return;
}


///***************************************************************************************************************************************
///Walk Menu Settings and Control
///***************************************************************************************************************************************

void MouseSetup::walking(void)
{
    char choicew;
    int done = 0, slow = 1;

    do {
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++           Walk Menu           ++"<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++               W               ++"<<std::endl;
        std::cout<<"++            A  S  D            ++"<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

        std::cout<<"<w> forward"<<std::endl;
        std::cout<<"<2> forward2"<<std::endl;
        std::cout<<"<s> stop"<<std::endl;
        std::cout<<"<y> stopposition"<<std::endl;
        std::cout<<"<a> left"<<std::endl;
        std::cout<<"<d> right"<<std::endl;
        std::cout<<"<x> backwards"<<std::endl;
        std::cout<<"<r> set slowness"<<std::endl;
        std::cout<<"<i> init"<<std::endl;
        std::cout<<"<1> start walker"<<std::endl;
        std::cout<<"<0> stop walker"<<std::endl;
        std::cout<<"<l> legs menu"<<std::endl;
        std::cout<<"<q> quit "<<std::endl;

        do{
            std::cout << "choose: ";
            std::cin >> choicew;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        switch(choicew) {
        case 'w':   //forward walking
            gait.store(FORWARD);
            break;
        case 's':   //stop walking
            gait.store(STOP);
            break;
        case 'a':   //left courve
            gait.store(LEFT);
            break;
        case 'd':   //right curve
            gait.store(RIGHT);
            break;
        case 'x':   //backward walking
            gait.store(BACKWARD);
            break;
        case 'h':
            gait.store(HUMP);
            break;
        case 'i':
            gait.store(INIT);
            break;
        case'y':
            gait.store(GotoSTOP);
            break;
        case 'r':
            do{
                std::cout << "slowness: ";
                std::cin >> slow;
                isgood = std::cin.good();
                std::cin.clear();
                std::cin.ignore(999999,'\n');
            } while (!isgood);
            pace.store(slow);
            break;
        case '1':
            gait.store(QUIT);
            usleep(10000);
            gait.store(STOP);
            walker->startWalker(&gait, &pace);
            break;
        case'0':
            gait.store(QUIT);
            break;
        case '2':
            gait.store(FWD2);
            break;
        case 'l':   //leg setup
            legSetup();
            break;
        case 'q':   //quit
            std::cout << "bye!\n";
            done = 1;
            break;
        default:
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;
            break;
        }
    } while(done != 1);

    return;
}


void MouseSetup::legSetup(void)
{
    char choicew;
    int done = 0;

    do {
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++           Leg Setup           ++"<<std::endl;
        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

        std::cout<<"<1> left hindleg"<<std::endl;
        std::cout<<"<2> right hindleg"<<std::endl;
        std::cout<<"<3> left foreleg"<<std::endl;
        std::cout<<"<4> right foreleg"<<std::endl;
        //std::cout<<"<7> print positions"<<std::endl;
        //std::cout<<"<8> read positions"<<std::endl;
        std::cout<<"<q> quit "<<std::endl;

        do{
            std::cout << "choose: ";
            std::cin >> choicew;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        switch(choicew) {
        case '1':   //left hindleg
            legCtrl(LEFT_HINDLEG);
            break;
        case '2':   //right hindleg
            legCtrl(RIGHT_HINDLEG);
            break;
        case '3':   //left foreleg
            legCtrl(LEFT_FORELEG);
            break;
        case '4':   //right foreleg
            legCtrl(RIGHT_FORELEG);
            break;
            //        case '7':   //print positions
            //            //NRPMouse->printPositions();
            //            std::cout << "NOT IMPLEMENTED\n";
            //            break;
            //        case '8':   //read positions
            //            //NRPMouse->readPos();
            //            std::cout << "NOT IMPLEMENTED\n";
            //            break;
        case 'q':   //quit
            std::cout << "bye!\n";
            done = 1;
            break;
        default:
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;
            break;
        }

    } while(done != 1);

    return;
}


///****************************************************************************************************************************************
///Feet Tests (one leg)
///****************************************************************************************************************************************

void MouseSetup::legCtrl(int pos)
{
    std::string menu = "Error";
    char        action;
    int         done = 0;

    switch (pos) {
    case LEFT_FORELEG:
        menu = " left fore";
        break;
    case LEFT_HINDLEG:
        menu = " left hind";
        break;
    case RIGHT_FORELEG:
        menu = "right fore";
        break;
    case RIGHT_HINDLEG:
        menu = "right hind";
        break;
    default:
        std::cout << "ERROR! You have selected an invalid leg position." << std::endl;
        return;
    }

    std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
    std::cout<<"++       "<<menu<<"Leg         ++"<<std::endl;
    std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
    std::cout<<"<w> up"<<std::endl;
    std::cout<<"<s> down"<<std::endl;
    std::cout<<"<a> front"<<std::endl;
    std::cout<<"<d> back"<<std::endl;
    std::cout<<"<r> set fwd"<<std::endl;
    std::cout<<"<e> step"<<std::endl;
    std::cout<<"<m> multiple steps"<<std::endl;
    std::cout<<"<q> quit "<<std::endl;

    do {

        do{
            std::cout << "set: ";
            std::cin >> action;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        switch(action) {
        case 'w': //move foot up
            switch (pos) {
            case LEFT_FORELEG:
                mouse.moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP);
                break;
            case LEFT_HINDLEG:
                mouse.moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP);
                break;
            case RIGHT_FORELEG:
                mouse.moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP);
                break;
            case RIGHT_HINDLEG:
                mouse.moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP);
                break;
            }
            break;

        case 's'://move foot down
            switch (pos) {
            case LEFT_FORELEG:
                //                NRPMouse->foreleft.footDown();
                mouse.moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN);
                break;
            case LEFT_HINDLEG:
                //                NRPMouse->hindleft.footDown();
                mouse.moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN);
                break;
            case RIGHT_FORELEG:
                //                NRPMouse->foreright.footDown();
                mouse.moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN);
                break;
            case RIGHT_HINDLEG:
                //                NRPMouse->hindright.footDown();
                mouse.moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);
                break;
            }
            break;

        case 'a':   //move hip forward
            switch (pos) {
            case LEFT_FORELEG:
                //                NRPMouse->foreleft.hipFwd();
                mouse.moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD);
                break;
            case LEFT_HINDLEG:
                //                NRPMouse->hindleft.hipFwd();
                mouse.moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD);
                break;
            case RIGHT_FORELEG:
                //                NRPMouse->foreright.hipFwd();
                mouse.moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD);
                break;
            case RIGHT_HINDLEG:
                //                NRPMouse->hindright.hipFwd();
                mouse.moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);
                break;
            }
            break;

        case 'd':   //move hip backward
            switch (pos) {
            case LEFT_FORELEG:
                //                NRPMouse->foreleft.hipBkwd();
                mouse.moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD);
                break;
            case LEFT_HINDLEG:
                //                NRPMouse->hindleft.hipBkwd();
                mouse.moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD);
                break;
            case RIGHT_FORELEG:
                //                NRPMouse->foreright.hipBkwd();
                mouse.moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD);
                break;
            case RIGHT_HINDLEG:
                //                NRPMouse->hindright.hipBkwd();
                mouse.moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD);
                break;
            }
            break;

        case 'r': //set foot forward
            switch (pos) {
            case LEFT_FORELEG:
                getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP); //Foreleft Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD); //Foreleft Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN); //Foreleft Foot down
                break;
            case LEFT_HINDLEG:
                getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD); //Hindleft Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot down
                break;
            case RIGHT_FORELEG:
                getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD); //Foreright Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot down
                break;
            case RIGHT_HINDLEG:
                getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);  //Hindright Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);; //Hindright Foot down
                break;
            }
            break;

        case 'e': //DO ONE STEP
            switch (pos) {
            case LEFT_FORELEG:
                getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP); //Foreleft Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD); //Foreleft Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN); //Foreleft Foot down
                usleep(500000);
                getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD,40); //Foreleft Hip bkwd
                break;
            case LEFT_HINDLEG:
                getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD); //Hindleft Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot down
                usleep(500000);
                getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD,40); //Hindleft Hip bkwd
                break;
            case RIGHT_FORELEG:
                getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD); //Foreright Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot down
                usleep(500000);
                getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD,40); //Foreright Hip bkwd
                break;
            case RIGHT_HINDLEG:
                getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot up
                usleep(200000);
                getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);  //Hindright Hip fwd
                usleep(100000);
                getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);; //Hindright Foot down
                usleep(500000);
                getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD,40);  //Hindright Hip bkwd
                break;
            }
            break;

        case 'm': //set number of steps
            uint16_t num;
            isgood = false;

            do
            {
                std::cout << "set #steps: ";
                std::cin >> num;
                isgood = std::cin.good();
                std::cin.clear();
                std::cin.ignore(999999,'\n');
            } while (!isgood);

            for(int i = 0; i<num; i++)
            {
                switch (pos) {
                case LEFT_FORELEG:
                    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_UP); //Foreleft Foot up
                    usleep(200000);
                    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_FWD); //Foreleft Hip fwd
                    usleep(100000);
                    getNrpMouseInstance().moveLeg(FORE_LEFT_FOOT, POS_FORE_LT_FOOT_DOWN); //Foreleft Foot down
                    usleep(500000);
                    getNrpMouseInstance().moveLeg(FORE_LEFT_HIP, POS_FORE_LT_HIP_BKWD,40); //Foreleft Hip bkwd
                    break;
                case LEFT_HINDLEG:
                    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_UP); //Hindleft Foot up
                    usleep(200000);
                    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_FWD); //Hindleft Hip fwd
                    usleep(100000);
                    getNrpMouseInstance().moveLeg(HIND_LEFT_FOOT, POS_HIND_LT_FOOT_DOWN); //Hindleft Foot down
                    usleep(500000);
                    getNrpMouseInstance().moveLeg(HIND_LEFT_HIP, POS_HIND_LT_HIP_BKWD,40); //Hindleft Hip bkwd
                    break;
                case RIGHT_FORELEG:
                    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_UP); //Foreright Foot up
                    usleep(200000);
                    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_FWD); //Foreright Hip fwd
                    usleep(100000);
                    getNrpMouseInstance().moveLeg(FORE_RIGHT_FOOT, POS_FORE_RT_FOOT_DOWN); //Foreright Foot down
                    usleep(500000);
                    getNrpMouseInstance().moveLeg(FORE_RIGHT_HIP, POS_FORE_RT_HIP_BKWD,40); //Foreright Hip bkwd
                    break;
                case RIGHT_HINDLEG:
                    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_UP); //Hindright Foot up
                    usleep(200000);
                    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_FWD);  //Hindright Hip fwd
                    usleep(100000);
                    getNrpMouseInstance().moveLeg(HIND_RIGHT_FOOT, POS_HIND_RT_FOOT_DOWN);; //Hindright Foot down
                    usleep(500000);
                    getNrpMouseInstance().moveLeg(HIND_RIGHT_HIP, POS_HIND_RT_HIP_BKWD,40);  //Hindright Hip bkwd
                    break;
                }
                usleep(400000);
            }
            break;

        case 'q':
            std::cout << "bye!\n";
            done = 1;
            break;

        default:
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;
            break;
        }

    } while(done != 1);

    return;

}


///****************************************************************************************************************************************
///Servo Menu Settings and Control
///****************************************************************************************************************************************

void MouseSetup::servos(void) {
    int choice2 = 8;

    do {

        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++          Servo Menu           ++"<<std::endl;

        int16_t servoMinAngle, servoMaxAngle;
        hardware->getAngleRange(&servoMinAngle, &servoMaxAngle);

        uint16_t minServoPL, maxServoPL;
        hardware->getDutyRange(&minServoPL, &maxServoPL);

        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

        std::cout<<"current settings:"<<std::endl;
        std::cout<<" angle: \tmin: "<<std::dec<<servoMinAngle<<"   max: "<<std::dec<<servoMaxAngle<<std::endl;
        //  each count about 4.5us, depending on the clock's accuracy.
        std::cout<<" pulse length: \tmin: "<<std::dec<<minServoPL<<" max: "<<std::dec<<maxServoPL<<std::endl;

        std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"<1>  servo ctrl angle"<<std::endl;
        std::cout<<"<2>  servo ctrl duty"<<std::endl;
        std::cout<<"<8>  set angle range"<<std::endl;
        std::cout<<"<9>  set pulse range"<<std::endl;
        std::cout<<"<-1> back "<<std::endl;

        do{
            std::cout << "choose: ";
            std::cin >> choice2;
            isgood = std::cin.good();
            std::cin.clear();
            std::cin.ignore(999999,'\n');
        } while (!isgood);

        if (choice2 == 1)
        {
            setServoAngle();
        }
        else if (choice2 == 2)
        {
            setServoDutycycle();
        }
        else if (choice2 == 8)
        {
            int done = 0;
            int16_t mina=0;
            int16_t maxa=0;

            do
            {
                mina = 0;
                maxa = 0;
                std::cout<<"++++++++set angle <min max>++++++++"<<std::endl;

                hardware->getAngleRange(&mina, &maxa);

                std::cout<<"current: "<<std::dec<<mina<<" "<<std::dec<<maxa<<std::endl;
                std::cin>>mina>>maxa;

                hardware->setAngleRange(mina, maxa);

                std::cout<<"new: "<<std::dec<<mina<<" "<<std::dec<<maxa<<std::endl;
                std::cout << "<Y=1/N=0> ";

                std::cin>>done;
            }while (done != 1);
        }
        else if (choice2 == 9)
        {
            int done = 0;
            uint16_t minp=0;
            uint16_t maxp=0;
            do
            {
                minp=0;
                maxp=0;

                std::cout<<"++++set pulse length <min max>+++++"<<std::endl;

                hardware->getDutyRange(&minp, &maxp);

                std::cout<<"current pulse: min: "<<std::dec<<minp<<" max: "<<std::dec<<maxp<<std::endl;
                std::cin>>minp>>maxp;

                hardware->setDutyRange(minp, maxp);

                std::cout<<"new "<<std::dec<<minp<<" "<<std::dec<<maxp<<std::endl;
                std::cout << "<Y=1/N=0> ";

                std::cin>>done;
            }while (done != 1);
        }
        else if (choice2 == -1)
        {
            //break at while
        }
        else
        {
            std::cout << "ERROR! You have selected an invalid choice." << std::endl;

        }

    } while(choice2 >= 0);

    return;
}


// change angle of given servo
void MouseSetup::setServoAngle(void){

    int value, servo, speed;
    //uint16_t start, stop;
    int16_t mina=0;
    int16_t maxa=0;

    hardware->getAngleRange(&mina, &maxa);   //get current range

    std::cout<<"++++++++++Set Servo Angle++++++++++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++ <0-270>     2 * 5     <270-0> ++"<<std::endl;
    std::cout<<"++ <0-270>     3 * 4     <270-0> ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++ <270-0>     1 * 6     <0-270> ++"<<std::endl;
    std::cout<<"++ <0-270>     0 * 7     <270-0> ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

    //Angle range is a psydo value deviding the pulse range in respective parts
    std::cout<<"<0-7> <"<<std::dec<<mina<<"-"<<std::dec<<maxa<<">deg, duration <1-255>ms, back <-1>"<<std::endl;

    //menu loop
    do {
        std::cout<<"set: ";
        //std::cin >> servo >> value >> speed;

        std::cin >> servo;
        if(servo == -1)
            continue;
        std::cin >> value >> speed;

        if(servo <= 7 || servo >= 0)
        {
            mouse.moveLeg(intToServo(servo), (uint16_t)value, (uint16_t)speed);
        }else if(servo != -1)
            std::cout <<"Servo number not available!"<<std::endl;
        //        if((value <= 360 || value >= 0) && (servo <= 7 || servo >= 0))
        //        {
        //            hardware->setServoAngle((uint8_t)servo, (uint16_t)value);
        //        }
        //        else
        //        {
        //            std::cout <<"out of range!"<<std::endl;
        //            servo = -1;
        //        }
    }while(servo != -1);

    return;
}

legservo MouseSetup::intToServo(int pin)
{
    switch(pin){
    case 0:
        return HIND_LEFT_HIP;
    case 1:
        return HIND_LEFT_FOOT;
    case 2:
        return FORE_LEFT_HIP;
    case 3:
        return FORE_LEFT_FOOT;
    case 4:
        return FORE_RIGHT_FOOT;
    case 5:
        return FORE_RIGHT_HIP;
    case 6:
        return HIND_RIGHT_FOOT;
    case 7:
        return HIND_RIGHT_HIP;
    default:
        std::cout <<"unknown servo!"<<std::endl;
        return HIND_LEFT_FOOT;
    }
    return HIND_LEFT_FOOT;
}

leg MouseSetup::intToLeg(int pin)
{
    switch(pin){
    case 0:
        return HIND_LEFT;
    case 1:
        return FORE_LEFT;
    case 2:
        return FORE_RIGHT;
    case 3:
        return HIND_RIGHT;
    default:
        std::cout <<"unknown servo!"<<std::endl;
        return HIND_LEFT;
    }
    return HIND_LEFT;
}


// change dury cycle of given servo
void MouseSetup::setServoDutycycle(void){

    int servo;
    float value;
    uint16_t start, stop;

    std::cout<<"++++++++++Set Servo Angle++++++++++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++             2 * 5             ++"<<std::endl;
    std::cout<<"++             3 * 4             ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++              ***              ++"<<std::endl;
    std::cout<<"++             1 * 6             ++"<<std::endl;
    std::cout<<"++             0 * 7             ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"++               *               ++"<<std::endl;
    std::cout<<"+++++++++++++++++++++++++++++++++++"<<std::endl;

    std::cout<<"duty cycle range: <3-16%>, back <-1>"<<std::endl; //3-16 is a tested value with the HS-35HD Servo
    std::cout<<"<0-7> <0-100>, back <-1>"<<std::endl;

    do {
        std::cout<<"set: ";
        std::cin >> servo;
        if(servo == -1)
            continue;
        std::cin >>value;
        if((value <= 100 && value >= 0) && (servo <= 7 || servo >= 0))
        {
            hardware->setServoDutycycle(servo, value);
            //get start stop time for control
            hardware->getChlTime(servo, &start, &stop);
            std::cout<<"t: "<<std::dec<<start<<" "<<std::dec<<stop<<std::endl;
        }
        else
        {
            std::cout <<"out of range!"<<std::endl;
            servo = -1;
        }
    }while(servo != -1);

    return;
}
