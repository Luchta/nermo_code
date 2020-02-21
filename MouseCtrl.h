#ifndef CMOUSE_CTRL_H
#define CMOUSE_CTRL_H

#include "Kinematics.h"
#include "MouseCom.h"
//#include <thread>


#ifdef ROS
// ROS includes
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#endif

class CSpinePos
{
public:
    CSpinePos() { spine=tail=0; }
    CSpinePos(double s, double t) { spine=s; tail=t; }
    double spine;
    double tail;
};

class CMove
{
public:
    CMove(int min, int max, int offset, int step){
        uRangeLeft = min;
        uRangeRight = max;
        uOffset = offset;
        uStep = step;

        posCentre = uMotorCentre + uOffset;
        posFarLeft = posCentre - uRangeLeft;
        posFarRight = posCentre + uRangeRight;

        curPos = posCentre + uOffset;
    }

    virtual ~CMove() {}

    double max();
    double min();
    double moveLeft(int length);
    double moveRight(int length);
    double moveStepLeft(int length);
    double moveStepRight(int length);
    double centre();

private:
    //Variables init in constructor
    int uRangeLeft, uRangeRight, uOffset, uStep;
    double posCentre, posFarLeft, posFarRight;
    //runtime variables
    double leftStepsize, rightStepsize, TailStepsize, curPos;
    //state variables
    bool dir = true;
    bool leftStart = true;
    bool rightStart = true;
    //constants
    const int uMotorCentre = 90;

};

class CSpine
{
public:
    CSpine(){}

    virtual ~CSpine() {}

    double stretch();
    double crouch();
    double moveTailLeft(int length);
    double moveTailRight(int length);
    CSpinePos moveLeft(int length);
    CSpinePos moveRight(int length);
    CSpinePos moveStepLeft(int length);
    CSpinePos moveStepRight(int length);
    CSpinePos centre();

private:
    const int RangeLeft = 40;
    const int RangeRight = 40;
    const int posStreched = 120;
    const int posCrouched = 180;
    const int cOffsetSpine = -5;
    const int cOffsetTail = 0;
    const int cOffsetFlex = 0;
    const int posCentre = 180;
    const int spineStep = 10;
    const int posSpineCentre = posCentre + cOffsetSpine;
    const int posTailCentre = posCentre + cOffsetTail;
    const int posFarLeft = posSpineCentre - RangeLeft;
    const int posFarRight = posSpineCentre + RangeRight;
    const int posTailFarLeft = posTailCentre - RangeLeft;
    const int posTailFarRight = posTailCentre + RangeRight;

    bool dir = true;
    bool leftStart = true;
    bool rightStart = true;
    bool rightTailStart = true;
    bool leftTailStart = true;
    double leftStepsize, rightStepsize, TailStepsize;
    double curSP = posCentre + cOffsetSpine;
    double curTL = posCentre + cOffsetTail;

};

// class for the 4 legs
class CMouseLeg : public CKinematics
{
public:
    CMouseLeg(char _leg, char _side, int _pawLift) { vx=vy=0; leg=_leg; side=_side; pawLift=_pawLift;}
    virtual ~CMouseLeg() {}

    typedef enum Phase{ Swing, Stance} typPhase;
    //FUNCTIONS
    void StartLeg(double x, double y, int length, typPhase phase);
    CLegPos GetNext();
    void MoveTo   (double x, double y);
    void Dump();

private:
    //FUNCTIONS
    void StepStart(double x, double y);
    bool StepNext ();
    CLegPos NextWayPoint();
    CLegPos SetPosition(CLegPos ang);//double deg1, double deg2);
    float Distance(float x1, float y1, float x2, float y2);

    //OBJECTS
    CKoord  ptLeg;   // x/y destination pos in move, current pos !in move
    CLegPos dgNext;  // Next Point on walking line
    CLegPos output;  // Output Value
    typPhase currPhase;
    CKoord docu[100]; //docu array

    //VARIABLES
    double vx, vy, riseStep;   // vector from current to destination x/y-point, divided by step stepcount
    int  stepcount,  // number of kinematic-steps
    step,       // current step number
    pawLift,     // how high does the leg lift
    risetime;    //time till the leg is lifted/set down
    char leg;    // Motor id of hip motor (knee++)
    char side;
};

class CMouseCtrl : public CMouseCom
{
public:
    CMouseCtrl();
    virtual ~CMouseCtrl() {}
    //defines for Motor IDs
#define ID_FORELEFT_HIP 00
#define ID_FORELEFT_KNEE 01
#define ID_HINDLEFT_HIP 10
#define ID_HINDLEFT_KNEE 11
#define ID_FORERIGHT_HIP 20
#define ID_FORERIGHT_KNEE 21
#define ID_HINDRIGHT_HIP 30
#define ID_HINDRIGHT_KNEE 31
#define ID_SPINE_FLEX 40
#define ID_SPINE 41
#define ID_TAIL 42
#define ID_HEAD_PAN 43
#define ID_HEAD_TILT 44

    //VARIABLES
    int messages; // message to UI thread

    static const int ArrayBuffer = 200;
    static const int Motors = 13;
    double TrottArray[ArrayBuffer][Motors+1];

    int MotorID[13] = {00,01,10,11,20,21,30,31,40,41,42,43,44};

    typedef enum Direction{ Fwd, Bkwd, left, right, stop, stance} typDir;
    typDir dir = stop;

    //FUNCTIONS
    void Ctrl();
    void startCtrlThread();
    void StopAllMotors();

private:

    //OBJECTS
    CMouseLeg LForeLeft = CMouseLeg('f','l', FLift); //fwd=0 bkwd=180 up=0 down=180
    CMouseLeg LForeRight = CMouseLeg('f','r', FLift);//fwd=180 bkwd=0 up=180 down=0
    CMouseLeg LHindLeft = CMouseLeg('h','l', HLift); //fwd=0 bkwd=180 up=0 down=180
    CMouseLeg LHindRight = CMouseLeg('h','r', HLift); //fwd=180 bkwd=0 up=180 down=0
    CSpine Spine = CSpine();

    //Global Variables
    int state = '0';
    bool Estop = false;

    //Motion Parameters
    unsigned int CommandDelay = 30000;
    static const int FLift = 25; //height of the foot lift
    static const int HLift = 20; //height of the foot lift

    const int uFrontLegStart  = -10;  // x start pos. of pace
    const int uHindLegStart   = -20;
    const int uStepLengthF    = 60;  // length of one leg move on the ground
    const int uStepLengthH    = 60;  // length of one leg move on the ground
    const int uHWalkLevel      = 0;//10;  // y walking level of init and walking
    const int uFWalkLevel      = 10;//10;  // y walking level of init and walking
    const int uSitting_x      = -30;  // X position for foot in sitting
    const int uSitting_y      = 30;  // Y position for foot in sitting
    const int uPosSpineFlexRelax = 180; //- is flex  NO+!!
    const int uPosHeadPan = 180; //+ is right
    const int uPosHeadTilt = 202; //+ is down

    //Sitting Parameters
    const int sitPosSpine = 168;
    const int sitPosFL = uFrontLegStart+uStepLengthF;   
    const int sitPosHL = 40;
    const int sitPosTail = 180;
    const int sitPosSpineFlex = 130;
    const int sitPosHeadPan = 180;
    const int sitPosHeadTilt = 236;

    //Bound Parameters
    static const int FLiftHigh = 35; //height of the foot lift


    //Head Parameters
    const int uPosHeadPanCentre = 180;
    const int uPosHeadTiltCentre = 236;

    //Lever Pressing Parameters
    const int sitStrechPosFL = -7;
    const int sitPushPosFL = uFrontLegStart+30;
    const int sitLiftPosFL = sitPushPosFL-15;

    // Motion Storage
    static const int storageBuffer = 10000;
    static const int storagevariables = 2; //time, cmd
    double StoreArray[storageBuffer][Motors+storagevariables];
    long int TimeStamp[storageBuffer];
    bool storeData = true;
    int si = 0; //storage index
    int fileNr = 0; //File index
    long int StartTime;

    // Motion Reading from File
    static const int inputBuffer = 10000;
    static const int infovariables = 2; //time, cmd
    double InputArray[inputBuffer][Motors+infovariables];
    int uMotionLines = 0;


    //FUNCTIONS
    //Control
    void Greeting();
    void Publish(int length = 1);
    int Remap(double in);
    void SendMotorMsgs(int i);
    //debug
    void Print(int i = 1);
    //storage
    void Store(int i);
    void StoreFile();
    void ReadFile();
    void PlayFile();
    void SendMotorFileMsgs(int i);
    void PrintFile(int i);
    void PlayFileReverse();

    std::chrono::milliseconds GetCurTime();
    //array fkts
    void clearStoreArr();
    void clearArr();
    //walking
    void Trot(int motionlength);
    void Init(int length = 1);
    void TrotBkw(int motionlength);
    void Bound(int motionlength);
    void Bound2(int motionlength);
    //Sitting
    void SitDown(int length);
    void SitUp(int length);
    //Levers
    void ReleaseLever(int length, char side);
    //void LiftHand(int length, char side);
    void PushBothHands(int length);
    void LiftBothHands(int length);
    //void SwitchLever(int length, char side);

};

#ifdef ROS
class CMouseRos : public CMouseCtrl
{
public:
    CMouseRos();
    virtual ~CMouseRos() {}
    //FUNCTIONS
    void RosCtrl();
    void ROSstartThread();
private:
    //FUNCTIONS
    void Publish(int length = 1);
    //OBJECTS
    std_msgs::Float64MultiArray msgarr;
    ros::NodeHandle n;
    ros::Publisher pub;

};
#endif

class CMouseUI
{
public:
    CMouseUI(int& msg) : _msg(msg) { }
    virtual ~CMouseUI() {}

    void process();
private:
    int getch();
    int& _msg;
};

#endif // CMOUSE_CTRL_H
