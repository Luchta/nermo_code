#include "cmouse_ctrl.h"
#include <cmath>
#include <iostream>
#include <termios.h>
#include <unistd.h>



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//defines for Motors
#define TIMESTAMP 0
#define FORELEFT_HIP 1
#define FORELEFT_KNEE 2
#define FORERIGHT_HIP 3
#define FORERIGHT_KNEE 4
#define HINDLEFT_HIP 5
#define HINDLEFT_KNEE 6
#define HINDRIGHT_HIP 7
#define HINDRIGHT_KNEE 8
#define SPINE 9
#define TAIL 10
#define SPINE_FLEX 11
#define HEAD_PAN 12
#define HEAD_TILT 13

// motion is created via motionarray
// speed is done via amount of points to be published (old setup: 100 values at 500hz?!)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS CLASS
#if defined ROS
CMouseRos::CMouseRos()
{
    msgarr.data.resize(14); //need to declare the size, else it wont work
    pub = n.advertise<std_msgs::Float64MultiArray>("nrpmouse_servotopic", 512);
    n.setParam("length", 50);
    //ros::Rate loop_rate(10); //run at 10Hz, not sleep time
}

//starting the loop thread
void CMouseRos::ROSstartThread() {
    std::cout << "starting UART and MOUSE thread"<<std::endl;
    std::thread t1 ([=] { RosCtrl(); });
    t1.detach();
    std::cout << "Walker thread detached"<<std::endl;
}

void CMouseRos::RosCtrl()
{
    int motionlength = 50;
    int cmd = '0';
    int state = '0';
    //bool newArray = true;
    clearArr(); //set everything to 90 deg, to avoid damage.

    while (ros::ok())
    {
        cmd = messages; //just one access to messages per run - not yet atomic!!
        if (cmd != state && cmd != 0){
            //newArray = true;
            state = cmd;
        }
        n.param("length", motionlength, 50);
        switch (state) {
        case 'i':   //initalize pose
            dir = stop;
            std::cout << "init" << std::endl;
            messages = 0;
            Init(3);
            Publish(3);
            state = 'h';
            break;
        case 'w': //walk forward
            dir = Fwd;
            std::cout << "Straight ahead" << std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 's': //walk backward
            dir = Bkwd;
            std::cout<<"Backwards"<<std::endl;
            TrotBkw(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'a': //walk left
            dir = left;
            std::cout<<"Left Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'd':   //walk right
            dir = right;
            std::cout<<"Right Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'y':   //quit programm
            std::cout<<"Sitting"<<std::endl;
            SitUp(80);
            Publish(80);
            messages = 0;
            state = 'h';
            break;
        case 'q':   //quit programm
            std::cout<<"Quitting"<<std::endl;
            clearArr();
            Publish();
            return;
        case 'm':   //publish motions to ros
            Publish(motionlength);
            break;
        case 'h':   //idle
            usleep(90);

            break;
        }
    }
}

void CMouseRos::Publish(int length)
{
    ros::Rate loop_rate(40); //run at 40Hz, not sleep time
    for(int i=0;i<length;i++){

        msgarr.data[TIMESTAMP] = TrottArray[i][TIMESTAMP];
        msgarr.data[FORELEFT_HIP]=(TrottArray[i][FORELEFT_HIP]);
        msgarr.data[FORELEFT_KNEE]=(TrottArray[i][FORELEFT_KNEE]);
        msgarr.data[FORERIGHT_HIP]=(TrottArray[i][FORERIGHT_HIP]);
        msgarr.data[FORERIGHT_KNEE]=(TrottArray[i][FORERIGHT_KNEE]);
        msgarr.data[HINDLEFT_HIP]=(TrottArray[i][HINDLEFT_HIP]);
        msgarr.data[HINDLEFT_KNEE]=(TrottArray[i][HINDLEFT_KNEE]);
        msgarr.data[HINDRIGHT_HIP]=(TrottArray[i][HINDRIGHT_HIP]);
        msgarr.data[HINDRIGHT_KNEE]=(TrottArray[i][HINDRIGHT_KNEE]);
        msgarr.data[SPINE]=(TrottArray[i][SPINE]);
        msgarr.data[TAIL]=(TrottArray[i][TAIL]);
        msgarr.data[SPINE_FLEX]=(TrottArray[i][SPINE_FLEX]);
        msgarr.data[HEAD_PAN]=(TrottArray[i][HEAD_PAN]);
        msgarr.data[HEAD_TILT]=(TrottArray[i][HEAD_TILT]);

        //std::cout << (TrottArray[i][SPINE_FLEX]) << "\n";

        pub.publish(msgarr);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

#endif



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UI methods

void CMouseUI::process()
{

    do {
        _msg = getch();
        usleep(100);
    }while (_msg != 'q');
}

int CMouseUI::getch()
{
    //a non locking getchar() which will always wait for input
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int dir = getchar();  // read character (non-blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return dir;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CMouseCtrl Methods

void CMouseCtrl::Init(int length) //initalizes all legs to zero position
{
    int i;
    CLegPos tmpLeg;
    CSpinePos tmpSpine;

    clearArr();

    //Spine positions
    tmpSpine = Spine.centre();


    //initalize Leg motion with Right leg forward
    LHindLeft.StartLeg(0, 0, length, CMouseLeg::Stance);
    LHindRight.StartLeg(0, 0, length, CMouseLeg::Stance);
    LForeLeft.StartLeg(0, 0, length, CMouseLeg::Stance);
    LForeRight.StartLeg(0, 0, length, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<length; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmpLeg.coil;

        TrottArray[i][SPINE] = tmpSpine.spine;
        TrottArray[i][TAIL] = tmpSpine.tail;
        TrottArray[i][SPINE_FLEX] = 180;
    }

}

void CMouseCtrl::SitUp(int length) //initalizes all legs to zero position
{
    int i, leng_init, leng_up;
    CLegPos tmpLeg;
    CSpinePos tmpSpine;

    //caculate array segmentation
    // left leg to start - right leg to start - Spine to fix + Sit up
    leng_init = (int)round(length * 0.25);
    leng_up = (leng_init * 2);
    //leng_up = length - l2;

    clearArr();

    //Spine positions
    tmpSpine = Spine.centre();

    //initalize Leg motion with Right leg forward
    LForeLeft.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init, CMouseLeg::Stance);
    LHindLeft.StartLeg(uSitting_x, uSitting_y, leng_init, CMouseLeg::Swing);
    LHindRight.StartLeg(uSitting_x, uSitting_y, leng_init, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<leng_init; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        //move Right foreleg to stance position
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmpLeg.coil;
        //move hindleft leg to maximal forward position
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmpLeg.coil;
    }
    int ll = 180; //leg most forward
    int lc = 180; //coil most released
    int changeAbsolute = leng_up - (leng_init/4);

    for (i=leng_init; i<leng_up; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        //set Left hindleg to maximum forward value of leg
        TrottArray[i][HINDLEFT_HIP] = ll;
        TrottArray[i][HINDLEFT_KNEE] = lc;
        //keep Foreleg to previous values
        TrottArray[i][FORERIGHT_HIP] = TrottArray[i-1][FORERIGHT_HIP];
        TrottArray[i][FORERIGHT_KNEE] = TrottArray[i-1][FORERIGHT_KNEE];
        // move Right hindleg forward until almost done, then move to absolute max
        if (i < changeAbsolute)
        {
            tmpLeg = LHindRight.GetNext();
            TrottArray[i][HINDRIGHT_HIP] = tmpLeg.leg;
            TrottArray[i][HINDRIGHT_KNEE] = tmpLeg.coil;
        }else{
            TrottArray[i][HINDRIGHT_HIP] = 180;
            TrottArray[i][HINDRIGHT_KNEE] = 0;
        }
        //move foreleft leg in stance position
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][FORELEFT_KNEE] = tmpLeg.coil;
    }

    LHindLeft.StartLeg(50, 0, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(50, 0, leng_init, CMouseLeg::Stance);

    int posSpineSit = 150;
    int posSpineStart = 180;
    double spineCurr = posSpineStart;
    double spineStep = (length-leng_up)/posSpineSit;


    for (i=leng_up; i<length; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        //move both hindlegs simultaniously to sit up body
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmpLeg.coil;
        //keep forelegs in position
        TrottArray[i][FORELEFT_HIP] = TrottArray[i-1][FORELEFT_HIP];
        TrottArray[i][FORELEFT_KNEE] = TrottArray[i-1][FORELEFT_KNEE];
        TrottArray[i][FORERIGHT_HIP] = TrottArray[i-1][FORERIGHT_HIP];
        TrottArray[i][FORERIGHT_KNEE] = TrottArray[i-1][FORERIGHT_KNEE];

        //iterate spine to stretch to move COG backward when sitting
        spineCurr += spineStep;
        TrottArray[i][SPINE_FLEX] = spineStep;
    }

}

void CMouseCtrl::Trot(int motionlength) //calculates trott gait
{
    //Variables
    int halfMotion = (int)round(motionlength/2);
    int i;
    bool tail = true;
    CLegPos tmp;
    CSpinePos tmpSpine;

    tmpSpine = Spine.centre();
    //Spine positions left/right/centre
    switch (dir){
    case Bkwd:
    case stop:
    case stance:
    case Fwd:
        tmpSpine = Spine.centre();
        break;
    case left:
        tmpSpine = Spine.moveStepRight(motionlength);
        break;
    case right:
        tmpSpine = Spine.moveStepLeft(motionlength);
        break;
    }

    //Setting Goals starting with Right leg forward
    LHindLeft.StartLeg(uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, 0, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart, 0, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmp.leg;
        TrottArray[i][FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        if (tail) {TrottArray[i][TAIL] = Spine.moveTailLeft(halfMotion);}
        else {TrottArray[i][TAIL] = tmpSpine.tail;}
        TrottArray[i][SPINE_FLEX] = Spine.stretch();
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uFrontLegStart, 0, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, 0, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmp.leg;
        TrottArray[i][FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        if (tail) {TrottArray[i][TAIL] = Spine.moveTailRight(motionlength-halfMotion);}
        else {TrottArray[i][TAIL] = tmpSpine.tail;}
        TrottArray[i][SPINE_FLEX] = Spine.stretch();
    }

}

void CMouseCtrl::TrotBkw(int motionlength) //calculates trott gait moving backwards
{
    //Variables
    int halfMotion = motionlength/2;
    int i;
    CLegPos tmp;
    CSpinePos tmpSpine;

    tmpSpine = Spine.centre();
    //Spine positions
    switch (dir){
    case Fwd:
    case stop:
    case stance:
    case Bkwd:
        tmpSpine = Spine.centre();
        break;
    case left:
        tmpSpine = Spine.moveStepRight(motionlength);
        break;
    case right:
        tmpSpine = Spine.moveStepLeft(motionlength);
        break;
    }


    //Setting Goals starting with Right leg forward
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmp.leg;
        TrottArray[i][FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        TrottArray[i][TAIL] = tmpSpine.tail;
        TrottArray[i][SPINE_FLEX] = Spine.stretch();
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmp.leg;
        TrottArray[i][FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        TrottArray[i][TAIL] = tmpSpine.tail;
        TrottArray[i][SPINE_FLEX] = Spine.stretch();
    }

}

void CMouseCtrl::Ctrl() //control setup - deprecated is only used in stand alone c++
{
    int motionlength = 50;
    char dir = '0';
    int state = '0';
    int cmd;
    bool OK = true;

    clearArr(); //set everything to 90 deg, to avoid damage.

    //while(ros.OK)
    while (OK)
    {
        cmd = messages; //just one access to messages per run - not yet atomic!!
        if (cmd != state && cmd != 0){
            //newArray = true;
            state = cmd;
        }
        switch (state) {
        case 'i':   //initalize pose
            dir = stop;
            std::cout << "init" << std::endl;
            messages = 0;
            Init(3);
            Print(3);
            state = 'h';
            break;
        case 'w': //walk forward
            dir = Fwd;
            std::cout << "Straight ahead" << std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 's': //walk backward
            dir = Bkwd;
            std::cout<<"Backwards"<<std::endl;
            TrotBkw(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'a': //walk left
            dir = left;
            std::cout<<"Left Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'd':   //walk right
            dir = right;
            std::cout<<"Right Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'y':   //quit programm
            std::cout<<"Sitting"<<std::endl;
            SitUp(80);
            Print(80);
            messages = 0;
            state = 'h';
            break;
        case 'q':   //quit programm
            std::cout<<"Quitting"<<std::endl;
            clearArr();
            Print();
            return;
        case 'm':   //publish motions to ros
            Print(motionlength);
            break;
        case 'h':   //idle
            usleep(90);

            break;
        }
    }


}

/*
int CMouseCtrl::getch()
{
    int c = std::cin.peek();

    if (c == EOF){
        return -1;
    }else{
    //a non blocking getchar()
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int dir = getchar();  // read character (non-blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return dir;
    }
}
*/

void CMouseCtrl::Print(int length) //print the array values for calculated lengthss
{
    std::cout << "TIMESTAMP; "
              << "FORELEFT_HIP; "
              << "FORELEFT_KNEE; "
              << "FORERIGHT_HIP; "
              << "FORERIGHT_KNEE; "
              << "HINDLEFT_HIP; "
              << "HINDLEFT_KNEE; "
              << "HINDRIGHT_HIP; "
              << "HINDRIGHT_KNEE; "
              << "SPINE; "
              << "TAIL; "
              << "SPINE_FLEX; "
              << "HEAD_PAN; "
              << "HEAD_TILT \n ";

    for(int i=0;i<length;i++)
    {
        std::cout << (int)TrottArray[i][TIMESTAMP] << "; "
                  << (int)TrottArray[i][FORELEFT_HIP] << "; "
                  << (int)TrottArray[i][FORELEFT_KNEE] << "; "
                  << (int)TrottArray[i][FORERIGHT_HIP] << "; "
                  << (int)TrottArray[i][FORERIGHT_KNEE] << "; "
                  << (int)TrottArray[i][HINDLEFT_HIP] << "; "
                  << (int)TrottArray[i][HINDLEFT_KNEE] << "; "
                  << (int)TrottArray[i][HINDRIGHT_HIP] << "; "
                  << (int)TrottArray[i][HINDRIGHT_KNEE] << "; "
                  << (int)TrottArray[i][SPINE] << "; "
                  << (int)TrottArray[i][TAIL] << "; "
                  << (int)TrottArray[i][SPINE_FLEX] << "; "
                  << (int)TrottArray[i][HEAD_PAN] << "; "
                  << (int)TrottArray[i][HEAD_TILT] << "\n ";
    }
}

void CMouseCtrl::clearArr(){    //clear the TrottArray

    CLegPos tmpLeg;

    //initalize Leg motion with Right leg forward
    LHindLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LHindRight.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LForeLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LForeRight.StartLeg(0, 0, 1, CMouseLeg::Stance);

    for(int i=0;i<ArrayBuffer;i++)
    {
        TrottArray[i][TIMESTAMP] = 0 ;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmpLeg.coil;
        TrottArray[i][SPINE] = 90 ;
        TrottArray[i][TAIL] = 90 ;
        TrottArray[i][SPINE_FLEX] = 180 ;
        TrottArray[i][HEAD_PAN] = 90 ;
        TrottArray[i][HEAD_TILT] = 90 ;
    }
}

void CMouseCtrl::startThread() { //starting the loop thread
    std::cout << "starting UART and MOUSE thread"<<std::endl;
    std::thread t1 ([=] { Ctrl(); });
    //t1 = std::thread { [] { Ctrl {} (); } };
    t1.detach();
    std::cout << "Walker thread detached"<<std::endl;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// leg machine methods

//initalizes trajectory
void CMouseLeg::StartLeg(double x, double y, int length, typPhase phase)
{
    //length is the length of the array to be filled = number of steps
    stepcount = length;
    currPhase = phase;
    StepStart(x,y);
}

//dumps debugging info
void CMouseLeg::Dump()
{
    std::cout << leg << "; " << side << "; " << pawLift << "\n";
    for (int i=0;i<stepcount;i++) {
        std::cout << docu[i].x << "; "
                  << docu[i].y << "\n";
    }
}

//returns the next waypoint in the given trajectory and the endpoint if invoked after end of Trajectory
CLegPos CMouseLeg::GetNext()
{
    if (StepNext()){
        return output;
    }else {
        return output; //currently always same output. when done, then always endposition
    }
}

void CMouseLeg::MoveTo(double x, double y)      // direkt single step to x/y, no trajectory,
{
    vx = vy = 0;
    step = stepcount = 0;   // zurücksetzen der Werte
    ptLeg = CKoord(x,y);    //aktuelle Positioen auf aaktuelles Ziel setzen
    output = SetPosition(NextWayPoint()); //Next waypoint berechnet Kinematik und in Output Speichern.
}

// Punkte werden im Voraus gerechnet (dgNext), w�hrend der Motor l�uft, besser
void CMouseLeg::StepStart(double x, double y)   // step mode by trajectory
{
    step = 0;                              // cast: Abschneiden gewollt s. jobTime
    if (stepcount <= 1)
        MoveTo(x, y);                  // if Dist < Resolution then Singlestep.
    else {
        vx = (x-ptLeg.x)/stepcount;          // compute step vector from known position (= at start!)
        vy = (y-ptLeg.y)/stepcount;
        dgNext = NextWayPoint();             // first step of kinematik move
    }
}

//invokes NextWayPoint until trajectory is finished
bool CMouseLeg::StepNext()
{
    if (++step > stepcount) return false;  // fertig
    output = SetPosition(dgNext);                   // vorausberechneten Punkt ausgeben
    if (step < stepcount)                  // wenn nicht letzter Punkt:
        dgNext = NextWayPoint();             //   neuen Punkt berechnen, solange Motor l�uft
    return true;                           // weiter gehts
}

//calculates and returns next point of trajectory - when given the swing phase it lifts the leg up
CLegPos CMouseLeg::NextWayPoint()
{
    double X = ptLeg.x+vx, Y = ptLeg.y+vy;        // N�chsten Punkt ab current ptLeg errechnen
    if (currPhase == Swing && step > 0 && step<stepcount-1) Y += pawLift;  // Rueckweg; 1 cm anheben, letzter Step wieder runter
    return (leg == 'f') ? ikforeleg(X, Y, side)
                        : ikhindleg(X, Y, side);
}

// returns the servo values for the leg and sets current internal positios
CLegPos CMouseLeg::SetPosition(CLegPos ang)
{
    docu[step] = ptLeg; //documentation for debugging
    ptLeg.x += vx;  ptLeg.y += vy;      // Vektor auf letzten Punkt addieren
    return ang; //output zurückgeben
}

//calculates distance between two points
float CMouseLeg::Distance(float x1, float y1, float x2, float y2)
{
    return std::hypot((x2-x1),(y2-y1));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// spine machine methods

//returns a stretched position value
double CSpine::stretch()
{
    return posStreched;
}

//returns a crouched position value
double CSpine::crouch()
{
    return posCrouched;
}


//move right centre to 50 and back
double CSpine::moveTailLeft(int length)
{
    //catch first function call to initialize
    if(leftTailStart){
        //set stepsize to got there and back again
        TailStepsize = (((double)RangeLeft)/((double)length/2));
        leftTailStart = false;
        dir = true;
        curTL = posTailCentre;
        return curTL;
    }
    //set new positions of the spine
    if (dir && (curTL > posTailFarLeft)){
        curTL = curTL - TailStepsize;  //go left
    }else if(!dir && (curTL < posTailCentre)) {
        curTL = curTL + TailStepsize; //go centre
    }else {
        //check wether motion completed
        if (curTL >= posTailCentre) {
            curTL = posTailCentre;
            leftTailStart = true;
            return curTL;
        }
        dir = !dir; //change direction
    }
    return curTL;
}

//move right centre to 130 and back
double CSpine::moveTailRight(int length)
{
    //catch first function call to initialize
    if(rightTailStart){
        //set stepsize to got there and back again
        TailStepsize = (((double)RangeRight)/((double)length/2));
        rightTailStart = false;
        dir = true;
        curTL = posTailCentre;
        return curTL;
    }
    //set new positions of the spine
    if (dir && (curTL < posTailFarRight)){
        curTL = curTL + TailStepsize;  //go right
    }else if(!dir && (curTL > posTailCentre)) {
        curTL = curTL - TailStepsize; //go centre
    }else {
        //check wether motion completed
        if (curTL <= posTailCentre) {
            curTL = posTailCentre;
            rightTailStart = true;
            return curTL;
        }
        dir = !dir; //change direction
    }
    return curTL;
}

//moving the Spine and Tail smoothly during walking
//every call to this funtion gives the next value for a given amount of iterations (length)
CSpinePos CSpine::moveLeft(int length)
{
    if(leftStart){
        leftStepsize = length/posFarLeft;
        leftStart = false;
    }
    curSP += leftStepsize;
    curTL += leftStepsize;
    if (curSP > posFarLeft) {curSP = posFarLeft; leftStart = true;}
    if (curTL > posFarLeft) {curTL = posFarLeft; leftStart = true;}

    return CSpinePos(curSP, curTL);
}

CSpinePos CSpine::moveRight(int length)
{
    if(rightStart){
        rightStepsize = length/posFarRight;
        rightStart = false;
    }
    curSP += rightStepsize;
    curTL += rightStepsize;
    if (curSP > posFarRight) {curSP = posFarRight; rightStart=true;}
    if (curTL > posFarRight) {curTL = posFarRight; rightStart=true;}

    return CSpinePos(curSP, curTL);
}

//moving the Spine and Tail stepwise for direction control
//every call to this funtion gives the next hihger bending iteration until the maximum bending is reached.
CSpinePos CSpine::moveStepLeft(int length)
{
    if ((curSP -= spineStep) < posFarLeft){
        curSP = posFarLeft;
    }
    return CSpinePos(curSP, curTL);
}

CSpinePos CSpine::moveStepRight(int length)
{
    if ((curSP += spineStep) > posFarRight){
        curSP = posFarRight;
    }
    return CSpinePos(curSP, curTL);
}

//centering the Spine
CSpinePos CSpine::centre()
{
    return CSpinePos((posCentre+cOffsetSpine), (posCentre+cOffsetTail));
}






