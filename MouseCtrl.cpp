#include "MouseCtrl.h"
#include <cmath> //pi and others
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread> // multithreading
#include <fstream> //file storage
#include <chrono> //timestamp
//#include <stdio.h> //strtok
#include <string.h> //strtok

#include <string>
#include <sstream>
#include <vector>





/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//defines for Motors in Array
#define A_TIMESTAMP 0
#define A_FORELEFT_HIP 1
#define A_FORELEFT_KNEE 2
#define A_FORERIGHT_HIP 3
#define A_FORERIGHT_KNEE 4
#define A_HINDLEFT_HIP 5
#define A_HINDLEFT_KNEE 6
#define A_HINDRIGHT_HIP 7
#define A_HINDRIGHT_KNEE 8
#define A_SPINE 9
#define A_TAIL 10
#define A_SPINE_FLEX 11
#define A_HEAD_PAN 12
#define A_HEAD_TILT 13

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//defines for Motors in File Array
#define F_TIMESTAMP 0
#define F_TIME 1
#define F_FORELEFT_HIP 2
#define F_FORELEFT_KNEE 3
#define F_FORERIGHT_HIP 4
#define F_FORERIGHT_KNEE 5
#define F_HINDLEFT_HIP 6
#define F_HINDLEFT_KNEE 7
#define F_HINDRIGHT_HIP 8
#define F_HINDRIGHT_KNEE 9
#define F_SPINE 10
#define F_TAIL 11
#define F_SPINE_FLEX 12
#define F_HEAD_PAN 13
#define F_HEAD_TILT 14

// motion is created via motionarray
// speed is done via amount of points to be published (old setup: 100 values at 500hz?!)

#define DEBUG false
#define DEBUG2 false



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UI methods

void CMouseUI::process()
{
    std::cout << "waiting for input\n";
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CMouseCtrl Methods
//Initalizations
CMouseCtrl::CMouseCtrl()
{
    clearArr();
    if (DEBUG) {std::cout<<"Mouse Ctrl in Debug Mode\n";}
    else { startUART();}
}

void CMouseCtrl::startCtrlThread() { //starting the loop thread
    //std::cout << "Starting Ctrl thread"<<std::endl;
    std::thread t1 ([=] { Ctrl(); });
    //std::thread t1 ([=] { RosCtrl(); });
    t1.detach();
    std::cout << "Ctrl thread started"<<std::endl;
}

//Control++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CMouseCtrl::Greeting(){
    std::cout<<"*****************************************\n";
    std::cout<<"Welcome to the NRP_Mouse Control Software\n";
    std::cout<<"*****************************************\n";
    std::cout<<"You have the following Options:\n";
    std::cout<<"Control:--------------------------\n";
    std::cout<<"s: Stop Motors\n";
    std::cout<<"h: Hold programm\n";
    std::cout<<"q: Quit programm\n";
    std::cout<<"+: Increase Speed\n";
    std::cout<<"-: Decrease Speed\n";
    std::cout<<"Walking:--------------------------\n";
    std::cout<<"i: Initialize pose\n";
    std::cout<<"w: Walk forward Trott\n";
    std::cout<<"a: Walk right the more the often you press\n";
    std::cout<<"d: Walk left the more the often you press\n";
    std::cout<<"b: Walk forward Bound\n";
    std::cout<<"n: Walk forward Bound2\n";
    std::cout<<"Sitting:--------------------------\n";
    std::cout<<"y: Sit up\n";
    std::cout<<"f: Press both paws up\n";
    std::cout<<"t: Lift both paws up\n";
    std::cout<<"e: Lift left paw\n";
    std::cout<<"r: Lift right paw\n";
    std::cout<<"x: Sit down\n";
    std::cout<<"Files:----------------------------\n";
    std::cout<<"o: Read motion data from motion.txt\n";
    std::cout<<"l: Playback data once\n";
    std::cout<<"k: Playback data reapeatingly\n";
    std::cout<<"p: Print position data to file\n";
}

void CMouseCtrl::Ctrl() //control setup - deprecated is only used in stand alone c++
{
    int cmd;
    bool OK = true;
    //Amount of Waypoints generated per motion:
    int motionlength = 50;
    int boundlength = 20;
    int situpDownTime = 80;
    int switchingTime = 20;
    int pushingTime = 20;
    int initTime = 1;

    //get Programm starting time
    StartTime = GetCurTime().count();

    //set everything to 180 deg(neutral position) to avoid damage.
    clearArr();
    //print greeting message
    Greeting();

    while (OK)
    {
        cmd = messages; //just one access to messages per run - not yet atomic!!
        //cmd = 'a';
        if (cmd != state && cmd != 0){
            state = cmd;
        }
        switch (state) {
        case 'i':   //initalize pose
            dir = stop;
            std::cout << "init" << std::endl;
            Init(initTime);
            Publish(initTime);
            messages = 0;
            state = 'h';
            break;
        case 'w': //walk forward+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            dir = Fwd;
            std::cout << "Straight ahead" << std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 's': //Stop Motors
            dir = Bkwd;
            std::cout<<"Stop Motors"<<std::endl;
            StopAllMotors();
            messages = 0;
            state = 'h';
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
        case 'b':   //Bound Gait+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            std::cout<<"Bound"<<std::endl;
            Bound(boundlength);
            Publish(boundlength);
            messages = 0;
            state = '.';
            break;
        case 'n':   //Bound2 Gait
            std::cout<<"Bound2"<<std::endl;
            Bound2(boundlength);
            Publish(boundlength);
            messages = 0;
            state = '.';
            break;
        case 'y':   //Sitting+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            std::cout<<"Sitting"<<std::endl;
            SitUp(situpDownTime);
            Publish(situpDownTime);
            messages = 0;
            state = 'h';
            break;
        case 'f':   //press both paws
            std::cout<<"press both paws"<<std::endl;
            PushBothHands(switchingTime);
            Publish(switchingTime);
            messages = 0;
            state = 'h';
            break;
        case 't':   //lift both paws
            std::cout<<"lift both paws"<<std::endl;
            LiftBothHands(switchingTime);
            Publish(switchingTime);
            messages = 0;
            state = 'h';
            break;
        case 'e':   //push left paw
            std::cout<<"push left paw"<<std::endl;
            ReleaseLever(pushingTime, 'l');
            Publish(pushingTime);
            messages = 0;
            state = 'h';
            break;
        case 'r':   //push right paw
            std::cout<<"push right paw"<<std::endl;
            ReleaseLever(pushingTime, 'r');
            Publish(pushingTime);
            messages = 0;
            state = 'h';
            break;
        case 'x':   //sit down
            std::cout<<"sit down"<<std::endl;
            SitDown(situpDownTime);
            Publish(situpDownTime);
            messages = 0;
            state = 'h';
            break;
        case '+':   //increase speed+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            CommandDelay = CommandDelay + 5000;
            std::cout<<"new Speed: "<<CommandDelay<<"\n";
            messages = 0;
            state = 'h';
            break;
        case '-':   //decrease speed
            CommandDelay = CommandDelay - 5000;
            std::cout<<"new Speed: "<<CommandDelay<<"\n";
            messages = 0;
            state = 'h';
            break;
        case 'm':   //publish motions to uart
            Publish(motionlength);
            break;
        case '.':   //publish motions to uart
            Publish(boundlength);
            break;
        case 'h':   //idle-hold
            dir = stop;
            usleep(90);
            break;
        case 'p':   //save motion data++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            StoreFile();
            messages = 0;
            state = 'h';
            break;
        case 'o':   //read motion data
            std::cout<<"reading data"<<std::endl;
            ReadFile();
            messages = 0;
            state = 'h';
            break;
        case 'l':   //playback motion data once
            PlayFile();
            messages = 0;
            state = 'h';
            break;
        case ',':   //playback motion data reverse once
            PlayFileReverse();
            messages = 0;
            state = 'h';
            break;
        case 'k':   //playback motion data continous
            PlayFile();
            break;
        case 'j':   //playback motion data reverse continous
            PlayFileReverse();
            break;
        case 'q':   //quit programm
            std::cout<<"Quitting"<<std::endl;
            StopAllMotors();
            return;
        }
    }
}

void CMouseCtrl::Publish(int length) //handle output of the array values for calculated lengths
{
    int i=0;
    //unsigned int CommandDelay = 30000;

    for(i=0;i<length;i++)
    {
        if(Estop){return;} //break
        if (DEBUG){
            Print(i);
            usleep(CommandDelay); //send delay between points
        }else {
            SendMotorMsgs(i);
            usleep(CommandDelay); //send delay between points
        }
        if(messages == 's'){std::cout << "EStop in Publish!\n";return;} //break for EStop
        //Storage
        if ((si+length) > storageBuffer){storeData = false;}
        if (storeData){
            Store(i);
        }
    }
    if (storeData){si = si + i;}

}

int CMouseCtrl::Remap(double in) //remap from 360 degree to servo value range
{
    double maxPos = 4095;
    //int minPos = 0;
    double maxDeg = 360;
    //int minDeg = 0;
    double map = (maxPos*in)/maxDeg;

    return (int)std::abs(map);
}

void CMouseCtrl::SendMotorMsgs(int i) // send Motor commands to MouseCom
{
    ProcessSpine(SetMotorPos, ID_FORELEFT_HIP, Remap(TrottArray[i][A_FORELEFT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_FORELEFT_KNEE, Remap(TrottArray[i][A_FORELEFT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_FORERIGHT_HIP, Remap(TrottArray[i][A_FORERIGHT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_FORERIGHT_KNEE, Remap(TrottArray[i][A_FORERIGHT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_HINDLEFT_HIP, Remap(TrottArray[i][A_HINDLEFT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_HINDLEFT_KNEE, Remap(TrottArray[i][A_HINDLEFT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_HINDRIGHT_HIP, Remap(TrottArray[i][A_HINDRIGHT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_HINDRIGHT_KNEE, Remap(TrottArray[i][A_HINDRIGHT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_SPINE, Remap(TrottArray[i][A_SPINE]), 1);
    ProcessSpine(SetMotorPos, ID_TAIL, Remap(TrottArray[i][A_TAIL]), 1);
    ProcessSpine(SetMotorPos, ID_SPINE_FLEX, Remap(TrottArray[i][A_SPINE_FLEX]), 1);
    ProcessSpine(SetMotorPos, ID_HEAD_PAN, Remap(TrottArray[i][A_HEAD_PAN]), 1);
    ProcessSpine(SetMotorPos, ID_HEAD_TILT, Remap(TrottArray[i][A_HEAD_TILT]), 1);
}

void CMouseCtrl::PlayFile() //handle output of the array values read from file
{
    int i=0;
    //unsigned int CommandDelay = 30000;

    for(i=0;i<uMotionLines;i++)
    {
        if (DEBUG){
            PrintFile(i);
            usleep(CommandDelay); //send delay between points
        }else {
           SendMotorFileMsgs(i);
            usleep(CommandDelay); //send delay between points
        }
        if(messages == 's'){std::cout << "EStop in Playback!\n"; return;} //break for EStop
    }
    std::cout << "played file\n";
}

void CMouseCtrl::PlayFileReverse() //handle output of the array values read from file
{
    int i=0;
    //unsigned int CommandDelay = 30000;

    //for(i=0;i<uMotionLines;i++)
    for(i=(uMotionLines-1);i>=0;i--)
    {
        if (DEBUG){
            PrintFile(i);
            usleep(CommandDelay); //send delay between points
        }else {
           SendMotorFileMsgs(i);
            usleep(CommandDelay); //send delay between points
        }
        if(messages == 's'){std::cout << "EStop in Playback!\n"; return;} //break for EStop
    }
    std::cout << "played file reverse\n";
}

void CMouseCtrl::SendMotorFileMsgs(int i)
{
    ProcessSpine(SetMotorPos, ID_FORELEFT_HIP, Remap(InputArray[i][F_FORELEFT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_FORELEFT_KNEE, Remap(InputArray[i][F_FORELEFT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_FORERIGHT_HIP, Remap(InputArray[i][F_FORERIGHT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_FORERIGHT_KNEE, Remap(InputArray[i][F_FORERIGHT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_HINDLEFT_HIP, Remap(InputArray[i][F_HINDLEFT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_HINDLEFT_KNEE, Remap(InputArray[i][F_HINDLEFT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_HINDRIGHT_HIP, Remap(InputArray[i][F_HINDRIGHT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_HINDRIGHT_KNEE, Remap(InputArray[i][F_HINDRIGHT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_SPINE, Remap(InputArray[i][F_SPINE]), 1);
    ProcessSpine(SetMotorPos, ID_TAIL, Remap(InputArray[i][F_TAIL]), 1);
    ProcessSpine(SetMotorPos, ID_SPINE_FLEX, Remap(InputArray[i][F_SPINE_FLEX]), 1);
    ProcessSpine(SetMotorPos, ID_HEAD_PAN, Remap(InputArray[i][F_HEAD_PAN]), 1);
    ProcessSpine(SetMotorPos, ID_HEAD_TILT, Remap(InputArray[i][F_HEAD_TILT]), 1);
}


//Debug++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CMouseCtrl::Print(int i) //print the array values for calculated lengthss
{
    std::cout << (int)TrottArray[i][A_TIMESTAMP] << "; FH:"
              << (int)TrottArray[i][A_FORELEFT_HIP] << "; FK:"
              << (int)TrottArray[i][A_FORELEFT_KNEE] << "; FH:"
              << (int)TrottArray[i][A_FORERIGHT_HIP] << "; FK:"
              << (int)TrottArray[i][A_FORERIGHT_KNEE] << "; HH:"
              << (int)TrottArray[i][A_HINDLEFT_HIP] << "; HK:"
              << (int)TrottArray[i][A_HINDLEFT_KNEE] << "; HH:"
              << (int)TrottArray[i][A_HINDRIGHT_HIP] << "; HK:"
              << (int)TrottArray[i][A_HINDRIGHT_KNEE] << "; SP:"
              << (int)TrottArray[i][A_SPINE] << "; T:"
              << (int)TrottArray[i][A_TAIL] << "; SF:"
              << (int)TrottArray[i][A_SPINE_FLEX] << "; HP:"
              << (int)TrottArray[i][A_HEAD_PAN] << "; HT: "
              << (int)TrottArray[i][A_HEAD_TILT] << "\n ";
}

void CMouseCtrl::PrintFile(int i)
{
    std::cout   << InputArray[i][F_TIMESTAMP] << "; TS:"
                                    << InputArray[i][F_TIME] << "; FLH:"
                                    << InputArray[i][F_FORELEFT_HIP] << "; FLK:"
                                    << InputArray[i][F_FORELEFT_KNEE] << "; FRH:"
                                    << InputArray[i][F_FORERIGHT_HIP] << "; FRK:"
                                    << InputArray[i][F_FORERIGHT_KNEE] << "; HLH:"
                                    << InputArray[i][F_HINDLEFT_HIP] << "; HLK:"
                                    << InputArray[i][F_HINDLEFT_KNEE] << "; HRH:"
                                    << InputArray[i][F_HINDRIGHT_HIP] << "; HRK:"
                                    << InputArray[i][F_HINDRIGHT_KNEE] << "; SPI:"
                                    << InputArray[i][F_SPINE] << "; TAL:"
                                    << InputArray[i][F_TAIL] << "; SPF:"
                                    << InputArray[i][F_SPINE_FLEX] << "; HP:"
                                    << InputArray[i][F_HEAD_PAN] << "; HT: "
                                    << InputArray[i][F_HEAD_TILT] << "\n ";
}

//Storage++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CMouseCtrl::Store(int i){
    TimeStamp[si+i] = GetCurTime().count();
    StoreArray[si+i][A_TIMESTAMP] = si+i;
    StoreArray[si+i][A_FORELEFT_HIP] = TrottArray[i][A_FORELEFT_HIP];
    StoreArray[si+i][A_FORELEFT_KNEE] = TrottArray[i][A_FORELEFT_KNEE];
    StoreArray[si+i][A_FORERIGHT_HIP] = TrottArray[i][A_FORERIGHT_HIP];
    StoreArray[si+i][A_FORERIGHT_KNEE] = TrottArray[i][A_FORERIGHT_KNEE];
    StoreArray[si+i][A_HINDLEFT_HIP] = TrottArray[i][A_HINDLEFT_HIP];
    StoreArray[si+i][A_HINDLEFT_KNEE] = TrottArray[i][A_HINDLEFT_KNEE];
    StoreArray[si+i][A_HINDRIGHT_HIP] = TrottArray[i][A_HINDRIGHT_HIP];
    StoreArray[si+i][A_HINDRIGHT_KNEE] = TrottArray[i][A_HINDRIGHT_KNEE];
    StoreArray[si+i][A_SPINE] = TrottArray[i][A_SPINE];
    StoreArray[si+i][A_TAIL] = TrottArray[i][A_TAIL];
    StoreArray[si+i][A_SPINE_FLEX] = TrottArray[i][A_SPINE_FLEX];
    StoreArray[si+i][A_HEAD_PAN] = TrottArray[i][A_HEAD_PAN];
    StoreArray[si+i][A_HEAD_TILT] = TrottArray[i][A_HEAD_TILT];
    StoreArray[si+i][14] = state;
}

void CMouseCtrl::StoreFile() //print the array values for calculated lengthss
{
    std::ofstream myfile;
    int l;
    char buffer [18];

    //wirte new Filename
    l = sprintf (buffer, "MotionStorage_%d.txt", fileNr);
    //open File
    myfile.open (&buffer[0]);

    if (myfile.is_open())
    {
        for(int i=0;i<storageBuffer;i++)
        {
            myfile << (TimeStamp[i]-StartTime) << ","
                   << StoreArray[i][A_TIMESTAMP] << ","
                   << StoreArray[i][A_FORELEFT_HIP] << ","
                   << StoreArray[i][A_FORELEFT_KNEE] << ","
                   << StoreArray[i][A_FORERIGHT_HIP] << ","
                   << StoreArray[i][A_FORERIGHT_KNEE] << ","
                   << StoreArray[i][A_HINDLEFT_HIP] << ","
                   << StoreArray[i][A_HINDLEFT_KNEE] << ","
                   << StoreArray[i][A_HINDRIGHT_HIP] << ","
                   << StoreArray[i][A_HINDRIGHT_KNEE] << ","
                   << StoreArray[i][A_SPINE] << ","
                   << StoreArray[i][A_TAIL] << ","
                   << StoreArray[i][A_SPINE_FLEX] << ","
                   << StoreArray[i][A_HEAD_PAN] << ","
                   << StoreArray[i][A_HEAD_TILT] << ","
                   << StoreArray[i][14] << "\n";
        }

        myfile.close();

        fileNr++;
        si = 0;
        clearStoreArr();
        std::cout << "Storage Completed \n ";
    }
    else
    {
        std::cout << "Error opening file";
    }

}

void CMouseCtrl::ReadFile()
{
    std::ifstream infile;
    std::string st, st2;
    std::istringstream iss, iss2;
    std::vector<double> vec;
    double val;
    const int MAX_ARG_PER_LINE = 15;
    int linecount = 0;
    char const separator = ',';

    infile.open("motions/motion.txt");
    if (infile.is_open())
    {
        // get one line of File
        while (!infile.eof()) {
            getline(infile, st);
            if (st.empty()) continue;
            iss.clear();
            iss.str(st);
            vec.clear();
            while (std::getline(iss, st2, separator)) {
                std::cerr << st2 << ",";
                iss2.str(st2);
                iss2.clear();
                iss2 >> val;
                if (!iss2.fail()) vec.push_back(val);
            }
            std::cerr << std::endl;
            if (vec.size() != MAX_ARG_PER_LINE) {
                std::cerr << "format error: " << vec.size() << std ::endl;
                continue;
            }
            for (size_t i = 0; i < vec.size(); i++) {
                InputArray[linecount][i] = vec.at(i);
            }
            linecount++;
        }
        infile.close();
    }
    else {
        std::cerr << "File could not be opened!\n";
    }
    uMotionLines = linecount;
    std::cout << "Data Read!\n";
}

std::chrono::milliseconds CMouseCtrl::GetCurTime(){
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()
                );
    return ms;

}
//##################################################################################################
// CMouseCtrl Motions

void CMouseCtrl::StopAllMotors(){
    for (int i=0;i<13;i++) {
        ProcessSpine(SetMotorOff, MotorID[i], 0, 0);
        usleep(10000);
    }
}

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
        TrottArray[i][A_TIMESTAMP] = i;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpLeg.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = 180;
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

}

//Walking++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
    case right:
        tmpSpine = Spine.moveStepLeft(motionlength);
        break;
    case left:
        tmpSpine = Spine.moveStepRight(motionlength);
        break;
    }

    //Setting Goals starting with Right leg forward
    LHindLeft.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        if (tail) {TrottArray[i][A_TAIL] = Spine.moveTailLeft(halfMotion);}
        else {TrottArray[i][A_TAIL] = tmpSpine.tail;}
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        if (tail) {TrottArray[i][A_TAIL] = Spine.moveTailRight(motionlength-halfMotion);}
        else {TrottArray[i][A_TAIL] = tmpSpine.tail;}
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
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
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
    }

}

void CMouseCtrl::Bound(int motionlength) //calculates trott gait
{
    // Bound Gait Motion
    /* From init position
     * Move Forepaws down
     * Push Up
     * Swing forward while hindlegs push backward
     * set down and move down
     * move backwards pull hindlegs forward
     *
     * repeat
     *
     *Foreleg positions:
     *Fwd Up (uFrontLegStart,FLift)
     *half down/Bkwd ((uStepLengthF+uFrontLegStart)/2,FLift/2)
     *Backward down (uStepLengthF+uFrontLegStart, uFWalkLevel)
     *Backward up (uStepLengthF+uFrontLegStart, FLift)
     *
     * Hindleg Positions:
     * Fwd Down (uHindLegStart,uHWalkLevel)
     * half bkwdDwon ((uStepLengthH+uHindLegStart)/2, uHWalkLevel)
     * bwkdDown ((uStepLengthH+uHindLegStart), uHWalkLevel)
     * half fwdup ((uStepLengthH+uHindLegStart)/2, HLift)
     */

    //Variables
    int PointDuration = (int)round(motionlength/4);
    int Point1 = PointDuration;
    int Point2 = Point1*2;
    int Point3 = Point1*3;

    int i;
    CLegPos tmp;
    CSpinePos tmpSpine;

    tmpSpine = Spine.centre();

    // POINT1 -----------------------------------------------------------------------

    //Setting Goals starting with Right leg forward
    //Hind
    LHindLeft.StartLeg(uHindLegStart, uHWalkLevel, PointDuration, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, uHWalkLevel, PointDuration, CMouseLeg::Stance);
    //Fore
    LForeLeft.StartLeg(uFrontLegStart-10, uFWalkLevel, PointDuration, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart-10, uFWalkLevel, PointDuration, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<Point1; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.crouch(); //Spine crouched until hindlegs move
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

    // POINT2 -----------------------------------------------------------------------

    // Setting Leg Goals starting with Left leg forward
    //Hind
    LHindLeft.StartLeg((uHindLegStart), uHWalkLevel, PointDuration, CMouseLeg::Stance);
    LHindRight.StartLeg((uHindLegStart), uHWalkLevel, PointDuration, CMouseLeg::Stance);
    //Fore
    LForeLeft.StartLeg((uStepLengthF+uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Stance);
    LForeRight.StartLeg((uStepLengthF+uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=Point1; i<Point2; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        /*tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;*/
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.crouch();
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

    // POINT3 -----------------------------------------------------------------------
    // Setting Leg Goals starting with Left leg forward
    //Hind
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, PointDuration, CMouseLeg::Stance);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, PointDuration, CMouseLeg::Stance);
    //Frre
    LForeLeft.StartLeg((uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Swing);
    LForeRight.StartLeg((uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=Point2; i<Point3; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.crouch();
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

    // POINT4 (end)-----------------------------------------------------------------------

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg((uHindLegStart), HLift, PointDuration, CMouseLeg::Swing);
    LHindRight.StartLeg((uHindLegStart), HLift, PointDuration, CMouseLeg::Swing);
    LForeLeft.StartLeg((uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Stance);
    LForeRight.StartLeg((uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=Point3; i<motionlength; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.crouch();
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }
}


void CMouseCtrl::Bound2(int motionlength) //calculates trott gait
{
    // Bound Gait Motion
    /* From init position
     * Move Forepaws down
     * Push Up
     * Swing forward while hindlegs push backward
     * set down and move down
     * move backwards pull hindlegs forward
     *
     * repeat
     *
     *Foreleg positions:
     *Fwd Up (uFrontLegStart,FLift)
     *half down/Bkwd ((uStepLengthF+uFrontLegStart)/2,FLift/2)
     *Backward down (uStepLengthF+uFrontLegStart, uFWalkLevel)
     *Backward up (uStepLengthF+uFrontLegStart, FLift)
     *
     * Hindleg Positions:
     * Fwd Down (uHindLegStart,uHWalkLevel)
     * half bkwdDwon ((uStepLengthH+uHindLegStart)/2, uHWalkLevel)
     * bwkdDown ((uStepLengthH+uHindLegStart), uHWalkLevel)
     * half fwdup ((uStepLengthH+uHindLegStart)/2, HLift)
     */

    //Variables
    int PointDuration = (int)round(motionlength/2);

    int i;
    CLegPos tmp;
    CSpinePos tmpSpine;

    tmpSpine = Spine.centre();

    // POINT1 -----------------------------------------------------------------------

    //Setting Goals starting with Right leg forward
    //Hind
    LHindLeft.StartLeg(uHindLegStart, uHWalkLevel, PointDuration, CMouseLeg::Swing);
    LHindRight.StartLeg(uHindLegStart, uHWalkLevel, PointDuration, CMouseLeg::Swing);
    //Fore
    LForeLeft.StartLeg((uStepLengthF+uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Stance);
    LForeRight.StartLeg((uStepLengthF+uFrontLegStart), uFWalkLevel, PointDuration, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<PointDuration; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.crouch(); //Spine crouched until hindlegs move
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

    // POINT2 -----------------------------------------------------------------------

    // Setting Leg Goals starting with Left leg forward
    //Hind
    LHindLeft.StartLeg((uHindLegStart+uStepLengthH), uHWalkLevel, PointDuration, CMouseLeg::Stance);
    LHindRight.StartLeg((uHindLegStart+uStepLengthH), uHWalkLevel, PointDuration, CMouseLeg::Stance);
    //Fore
    LForeLeft.StartLeg((uFrontLegStart-10), uFWalkLevel, PointDuration, CMouseLeg::Swing);
    LForeRight.StartLeg((uFrontLegStart-10), uFWalkLevel, PointDuration, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=PointDuration; i<motionlength; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.crouch();
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }

}

//Sitting++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CMouseCtrl::SitUp(int length) //initalizes all legs to zero position
{
    int i, leng_init, leng_to_3,leng_to_4;
    CLegPos tmpHL, tmpHR, tmpFL;
    CSpinePos tmpSpine;

    //caculate array segmentation
    // Motion is as follows:
    // 1)Move Forlegs into backward position one by one
    // 2)Move Hindlegs forward and up while Forelegs push back
    // 3)Move Forlegs backward to rise body
    // 4)lift spine up
    // left leg to start - right leg to start - Spine to fix + Sit up
    leng_init = (int)round(length * 0.25);
    leng_to_3 = leng_init+leng_init;
    leng_to_4 = leng_to_3 + leng_init;
    //leng_up = length - l2;

    clearArr();

    //Spine positions
    tmpSpine = Spine.centre();

    //Forelg init pos:
    LForeLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    tmpFL = LForeLeft.GetNext();
    double foreLeftInitL = tmpFL.leg;
    double foreLeftInitC = tmpFL.coil;
    //LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, 1, CMouseLeg::Stance);
    //tmpFL = LForeRight.GetNext();
    //double foreRightInitL = tmpFL.leg;
    //double foreRightInitC = tmpFL.coil;

    //1)Move Forlegs into backward position one by one--------------------------------------------------------

    //Forward Goal backward pos
    LForeLeft.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);
    //Hindleg Goal init pose
    LHindLeft.StartLeg(uHindLegStart, 0, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, 0, leng_init, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<leng_init; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //move Forelegs to forward pose
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        if (i>(leng_init/2)){
            tmpFL = LForeLeft.GetNext();
            TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
            TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        }else {
            TrottArray[i][A_FORELEFT_HIP] = foreLeftInitL;
            TrottArray[i][A_FORELEFT_KNEE] = foreLeftInitC;
        }
        tmpHL = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = 180;
        TrottArray[i][A_HINDLEFT_KNEE] = 180;
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = 180;
        TrottArray[i][A_HINDRIGHT_KNEE] = 180;
    }

    // 2)Move Hindlegs forward and up while Forelegs push back----------------------------------

    //Forleg Goal forward Pos
    LForeLeft.StartLeg(uFrontLegStart, 0, leng_init, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart, 0, leng_init, CMouseLeg::Stance);
    //Hindleg Goal most forward pose
    LHindLeft.StartLeg(uSitting_x, uSitting_y, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(uSitting_x, uSitting_y, leng_init, CMouseLeg::Stance);

    for (i=leng_init; i<leng_to_3; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Forelegs push back
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        tmpFL = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        //Move Hindlegs forward
        tmpHL = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpHL.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpHL.coil;
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpHR.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpHR.coil;
    }

    // 3)Move Forlegs backward to rise body-------------------------------------------------------

    //Forward Goal backward pos
    LForeLeft.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);

    for (i=leng_to_3; i<leng_to_4; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Forelegs push back
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        tmpFL = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        //Move Hindlegs forward
        TrottArray[i][A_HINDLEFT_HIP] = TrottArray[i-1][A_HINDLEFT_HIP];
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        TrottArray[i][A_HINDRIGHT_HIP] = TrottArray[i-1][A_HINDRIGHT_HIP];
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
    }

    //4)-Lift Spine Up-----------------------------------------------------------------------

    LHindLeft.StartLeg(40, 0, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(40, 0, leng_init, CMouseLeg::Stance);

    double posSpineSit = 140;
    double posSpineStart = 180;
    double spineCurr = posSpineStart;
    double spineStep = (posSpineStart - posSpineSit)/(double)(length-leng_to_4); //steplength for rising spine


    for (i=leng_to_4; i<length; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //move both hindlegs simultaniously to sit up body
        tmpHL = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpHL.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpHR.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
        //keep forelegs in position
        TrottArray[i][A_FORELEFT_HIP] = TrottArray[i-1][A_FORELEFT_HIP];
        TrottArray[i][A_FORELEFT_KNEE] = TrottArray[i-1][A_FORELEFT_KNEE];
        TrottArray[i][A_FORERIGHT_HIP] = TrottArray[i-1][A_FORERIGHT_HIP];
        TrottArray[i][A_FORERIGHT_KNEE] = TrottArray[i-1][A_FORERIGHT_KNEE];

        //iterate spine to stretch to move COG backward when sitting
        spineCurr = spineCurr - spineStep;
        TrottArray[i][A_SPINE_FLEX] = spineCurr;
    }

}

void CMouseCtrl::SitDown(int length){
    CLegPos tmpFL, tmpFR ,tmpHL, tmpHR;
    double spineCurr = sitPosSpineFlex;
    double spineStep = (uPosSpineFlexRelax - sitPosSpineFlex)/(double)(length);
    //Forelegs
    LForeLeft.StartLeg(sitPosFL, 0, 1, CMouseLeg::Stance);
    LForeRight.StartLeg(sitPosFL, 0, 1, CMouseLeg::Stance);
    tmpFL = LForeLeft.GetNext();
    tmpFR = LForeRight.GetNext();
    //Hindlegs
    LHindLeft.StartLeg(uHindLegStart, 0, length, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, 0, length, CMouseLeg::Stance);
    tmpHL = LHindLeft.GetNext();
    tmpHR = LHindRight.GetNext();
    //first Array index for following values
    TrottArray[0][A_TIMESTAMP] = 0;
    //Keep All Motors in position
    TrottArray[0][A_HINDLEFT_HIP] = tmpHL.leg;
    TrottArray[0][A_HINDLEFT_KNEE] = tmpHL.coil;
    TrottArray[0][A_HINDRIGHT_HIP] = tmpHR.leg;
    TrottArray[0][A_HINDRIGHT_KNEE] = tmpHR.coil;
    TrottArray[0][A_FORERIGHT_HIP] = tmpFR.leg;
    TrottArray[0][A_FORERIGHT_KNEE] = tmpFR.coil;
    TrottArray[0][A_FORELEFT_HIP] = tmpFL.leg;
    TrottArray[0][A_FORELEFT_KNEE] = tmpFL.coil;
    TrottArray[0][A_SPINE_FLEX] = sitPosSpineFlex;
    TrottArray[0][A_TAIL] = sitPosTail;
    TrottArray[0][A_SPINE] = sitPosSpine;
    TrottArray[0][A_HEAD_PAN] = sitPosHeadPan;
    TrottArray[0][A_HEAD_TILT] = sitPosHeadTilt;

    //create motion array
    for (int i=1; i<length; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Keep All Motors in position
        TrottArray[i][A_TAIL] = TrottArray[i-1][A_TAIL];
        TrottArray[i][A_SPINE] = TrottArray[i-1][A_SPINE];
        TrottArray[i][A_HEAD_PAN] = TrottArray[i-1][A_HEAD_PAN];
        TrottArray[i][A_HEAD_TILT] = TrottArray[i-1][A_HEAD_TILT];
        TrottArray[i][A_FORERIGHT_HIP] = TrottArray[i-1][A_FORERIGHT_HIP];
        TrottArray[i][A_FORERIGHT_KNEE] = TrottArray[i-1][A_FORERIGHT_KNEE];
        TrottArray[i][A_FORELEFT_HIP] = TrottArray[i-1][A_FORELEFT_HIP];
        TrottArray[i][A_FORELEFT_KNEE] = TrottArray[i-1][A_FORELEFT_KNEE];
        //Hindlegs lower
        tmpHL = LHindLeft.GetNext();
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpHL.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpHL.coil;
        TrottArray[i][A_HINDRIGHT_HIP] = tmpHR.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpHR.coil;
        //Spine relax
        spineCurr = spineCurr + spineStep;
        TrottArray[i][A_SPINE_FLEX] = spineCurr;
    }
}

void CMouseCtrl::PushBothHands(int length){
    CLegPos tmpFL, tmpFR ,tmpHL, tmpHR;
    //Forelegs
    LForeLeft.StartLeg(sitPushPosFL, sitStrechPosFL, length, CMouseLeg::Stance);
    LForeRight.StartLeg(sitPushPosFL, sitStrechPosFL, length, CMouseLeg::Stance);
    tmpFL = LForeLeft.GetNext();
    tmpFR = LForeRight.GetNext();
    //Hindlegs
    LHindLeft.StartLeg(sitPosHL, 0, 1, CMouseLeg::Stance);
    LHindRight.StartLeg(sitPosHL, 0, 1, CMouseLeg::Stance);
    tmpHL = LHindLeft.GetNext();
    tmpHR = LHindRight.GetNext();
    //first Array index for following values
    TrottArray[0][A_TIMESTAMP] = 0;
    //Keep All Motors in position
    TrottArray[0][A_HINDLEFT_HIP] = tmpHL.leg;
    TrottArray[0][A_HINDLEFT_KNEE] = tmpHL.coil;
    TrottArray[0][A_HINDRIGHT_HIP] = tmpHR.leg;
    TrottArray[0][A_HINDRIGHT_KNEE] = tmpHR.coil;
    TrottArray[0][A_FORERIGHT_HIP] = tmpFR.leg;
    TrottArray[0][A_FORERIGHT_KNEE] = tmpFR.coil;
    TrottArray[0][A_FORELEFT_HIP] = tmpFL.leg;
    TrottArray[0][A_FORELEFT_KNEE] = tmpFL.coil;
    TrottArray[0][A_SPINE_FLEX] = sitPosSpineFlex;
    TrottArray[0][A_TAIL] = sitPosTail;
    TrottArray[0][A_SPINE] = sitPosSpine;
    TrottArray[0][A_HEAD_PAN] = sitPosHeadPan;
    TrottArray[0][A_HEAD_TILT] = sitPosHeadTilt;

    //create motion array
    for (int i=1; i<length; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Keep All Motors in position
        TrottArray[i][A_HINDLEFT_HIP] = TrottArray[i-1][A_HINDLEFT_HIP];
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        TrottArray[i][A_HINDRIGHT_HIP] = TrottArray[i-1][A_HINDRIGHT_HIP];
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
        TrottArray[i][A_SPINE_FLEX] = TrottArray[i-1][A_SPINE_FLEX];
        TrottArray[i][A_TAIL] = TrottArray[i-1][A_TAIL];
        TrottArray[i][A_SPINE] = TrottArray[i-1][A_SPINE];
        TrottArray[i][A_HEAD_PAN] = TrottArray[i-1][A_HEAD_PAN];
        TrottArray[i][A_HEAD_TILT] = TrottArray[i-1][A_HEAD_TILT];

        //keep forelegs in position
        tmpFL = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;

    }
}

void CMouseCtrl::LiftBothHands(int length){
    CLegPos tmpFL, tmpFR ,tmpHL, tmpHR;
    //Forelegs
    LForeLeft.StartLeg(sitLiftPosFL, sitStrechPosFL, length, CMouseLeg::Stance);
    LForeRight.StartLeg(sitLiftPosFL, sitStrechPosFL, length, CMouseLeg::Stance);
    tmpFL = LForeLeft.GetNext();
    tmpFR = LForeRight.GetNext();
    //Hindlegs
    LHindLeft.StartLeg(sitPosHL, 0, 1, CMouseLeg::Stance);
    LHindRight.StartLeg(sitPosHL, 0, 1, CMouseLeg::Stance);
    tmpHL = LHindLeft.GetNext();
    tmpHR = LHindRight.GetNext();
    //first Array index for following values
    TrottArray[0][A_TIMESTAMP] = 0;
    //Keep All Motors in position
    TrottArray[0][A_HINDLEFT_HIP] = tmpHL.leg;
    TrottArray[0][A_HINDLEFT_KNEE] = tmpHL.coil;
    TrottArray[0][A_HINDRIGHT_HIP] = tmpHR.leg;
    TrottArray[0][A_HINDRIGHT_KNEE] = tmpHR.coil;
    TrottArray[0][A_FORERIGHT_HIP] = tmpFR.leg;
    TrottArray[0][A_FORERIGHT_KNEE] = tmpFR.coil;
    TrottArray[0][A_FORELEFT_HIP] = tmpFL.leg;
    TrottArray[0][A_FORELEFT_KNEE] = tmpFL.coil;
    TrottArray[0][A_SPINE_FLEX] = sitPosSpineFlex;
    TrottArray[0][A_TAIL] = sitPosTail;
    TrottArray[0][A_SPINE] = sitPosSpine;
    TrottArray[0][A_HEAD_PAN] = sitPosHeadPan;
    TrottArray[0][A_HEAD_TILT] = sitPosHeadTilt;


    for (int i=1; i<length; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Keep All Motors in position
        TrottArray[i][A_HINDLEFT_HIP] = TrottArray[i-1][A_HINDLEFT_HIP];
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        TrottArray[i][A_HINDRIGHT_HIP] = TrottArray[i-1][A_HINDRIGHT_HIP];
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
        TrottArray[i][A_SPINE_FLEX] = TrottArray[i-1][A_SPINE_FLEX];
        TrottArray[i][A_TAIL] = TrottArray[i-1][A_TAIL];
        TrottArray[i][A_SPINE] = TrottArray[i-1][A_SPINE];
        TrottArray[i][A_HEAD_PAN] = TrottArray[i-1][A_HEAD_PAN];
        TrottArray[i][A_HEAD_TILT] = TrottArray[i-1][A_HEAD_TILT];

        //keep forelegs in position
        tmpFL = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;

    }
}

void CMouseCtrl::ReleaseLever(int length, char side){

    CLegPos tmpFL, tmpFR ,tmpHL, tmpHR;
    int half = length/2;
    //Forelegs
    if (side == 'l'){
        LForeLeft.StartLeg(sitLiftPosFL, sitStrechPosFL, half, CMouseLeg::Stance);
        LForeRight.StartLeg(sitPushPosFL, sitStrechPosFL, 1, CMouseLeg::Stance);
    }else if (side == 'r') {
        LForeRight.StartLeg(sitLiftPosFL, sitStrechPosFL, half, CMouseLeg::Stance);
        LForeLeft.StartLeg(sitPushPosFL, sitStrechPosFL, 1, CMouseLeg::Stance);
    }
    tmpFL = LForeLeft.GetNext();
    tmpFR = LForeRight.GetNext();
    //Hindlegs
    LHindLeft.StartLeg(sitPosHL, 0, 1, CMouseLeg::Stance);
    LHindRight.StartLeg(sitPosHL, 0, 1, CMouseLeg::Stance);
    tmpHL = LHindLeft.GetNext();
    tmpHR = LHindRight.GetNext();
    //first Array index for following values
    TrottArray[0][A_TIMESTAMP] = 0;
    //Keep All Motors in position
    TrottArray[0][A_HINDLEFT_HIP] = tmpHL.leg;
    TrottArray[0][A_HINDLEFT_KNEE] = tmpHL.coil;
    TrottArray[0][A_HINDRIGHT_HIP] = tmpHR.leg;
    TrottArray[0][A_HINDRIGHT_KNEE] = tmpHR.coil;
    TrottArray[0][A_FORERIGHT_HIP] = tmpFR.leg;
    TrottArray[0][A_FORERIGHT_KNEE] = tmpFR.coil;
    TrottArray[0][A_FORELEFT_HIP] = tmpFL.leg;
    TrottArray[0][A_FORELEFT_KNEE] = tmpFL.coil;
    TrottArray[0][A_SPINE_FLEX] = sitPosSpineFlex;
    TrottArray[0][A_TAIL] = sitPosTail;
    TrottArray[0][A_SPINE] = sitPosSpine;
    TrottArray[0][A_HEAD_PAN] = sitPosHeadPan;
    TrottArray[0][A_HEAD_TILT] = sitPosHeadTilt;

    //push hand down
    for (int i=1; i<half; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Keep All Motors in position
        TrottArray[i][A_HINDLEFT_HIP] = TrottArray[i-1][A_HINDLEFT_HIP];
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        TrottArray[i][A_HINDRIGHT_HIP] = TrottArray[i-1][A_HINDRIGHT_HIP];
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
        TrottArray[i][A_SPINE_FLEX] = TrottArray[i-1][A_SPINE_FLEX];
        TrottArray[i][A_TAIL] = TrottArray[i-1][A_TAIL];
        TrottArray[i][A_SPINE] = TrottArray[i-1][A_SPINE];
        TrottArray[i][A_HEAD_PAN] = TrottArray[i-1][A_HEAD_PAN];
        TrottArray[i][A_HEAD_TILT] = TrottArray[i-1][A_HEAD_TILT];

        //keep forelegs in position
        if (side == 'l'){
            TrottArray[i][A_FORERIGHT_HIP] = TrottArray[i-1][A_FORERIGHT_HIP];
            TrottArray[i][A_FORERIGHT_KNEE] = TrottArray[i-1][A_FORERIGHT_KNEE];
            tmpFL = LForeLeft.GetNext();
            TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
            TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        }else if (side == 'r') {
            TrottArray[i][A_FORELEFT_HIP] = TrottArray[i-1][A_FORELEFT_HIP];
            TrottArray[i][A_FORELEFT_KNEE] = TrottArray[i-1][A_FORELEFT_KNEE];
            tmpFL = LForeRight.GetNext();
            TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
            TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        }
    }

    //Lift hand up again
    if (side == 'l'){
        LForeLeft.StartLeg(sitPushPosFL, sitStrechPosFL, half, CMouseLeg::Stance);
        LForeRight.StartLeg(sitPushPosFL, sitStrechPosFL, 1, CMouseLeg::Stance);
    }else if (side == 'r') {
        LForeRight.StartLeg(sitPushPosFL, sitStrechPosFL, half, CMouseLeg::Stance);
        LForeLeft.StartLeg(sitPushPosFL, sitStrechPosFL, 1, CMouseLeg::Stance);
    }

    for (int i=half; i<length; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Keep All Motors in position
        TrottArray[i][A_HINDLEFT_HIP] = TrottArray[i-1][A_HINDLEFT_HIP];
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        TrottArray[i][A_HINDRIGHT_HIP] = TrottArray[i-1][A_HINDRIGHT_HIP];
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
        TrottArray[i][A_SPINE_FLEX] = TrottArray[i-1][A_SPINE_FLEX];
        TrottArray[i][A_TAIL] = TrottArray[i-1][A_TAIL];
        TrottArray[i][A_SPINE] = TrottArray[i-1][A_SPINE];
        TrottArray[i][A_HEAD_PAN] = TrottArray[i-1][A_HEAD_PAN];
        TrottArray[i][A_HEAD_TILT] = TrottArray[i-1][A_HEAD_TILT];

        //keep forelegs in position
        if (side == 'l'){
            TrottArray[i][A_FORERIGHT_HIP] = TrottArray[i-1][A_FORERIGHT_HIP];
            TrottArray[i][A_FORERIGHT_KNEE] = TrottArray[i-1][A_FORERIGHT_KNEE];
            tmpFL = LForeLeft.GetNext();
            TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
            TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        }else if (side == 'r') {
            TrottArray[i][A_FORELEFT_HIP] = TrottArray[i-1][A_FORELEFT_HIP];
            TrottArray[i][A_FORELEFT_KNEE] = TrottArray[i-1][A_FORELEFT_KNEE];
            tmpFL = LForeRight.GetNext();
            TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
            TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        }
    }
}

//##################################################################################################
//Array functions

void CMouseCtrl::clearStoreArr(){

    for(int i=0;i<storageBuffer;i++)
    {
        StoreArray[i][A_TIMESTAMP] = 0;
        StoreArray[i][A_FORELEFT_HIP] = 0;
        StoreArray[i][A_FORELEFT_KNEE] = 0;
        StoreArray[i][A_FORERIGHT_HIP] = 0;
        StoreArray[i][A_FORERIGHT_KNEE] = 0;
        StoreArray[i][A_HINDLEFT_HIP] = 0;
        StoreArray[i][A_HINDLEFT_KNEE] = 0;
        StoreArray[i][A_HINDRIGHT_HIP] = 0;
        StoreArray[i][A_HINDRIGHT_KNEE] = 0;
        StoreArray[i][A_SPINE] = 0;
        StoreArray[i][A_TAIL] = 0;
        StoreArray[i][A_SPINE_FLEX] = 0;
        StoreArray[i][A_HEAD_PAN] = 0;
        StoreArray[i][A_HEAD_PAN] = 0;
        StoreArray[i][14] = 0;
    }
}

void CMouseCtrl::clearArr(){    //clear the TrottArray

    CLegPos tmpLeg;
    int centrePos = 180;

    //initalize Leg motion with Right leg forward
    LHindLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LHindRight.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LForeLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LForeRight.StartLeg(0, 0, 1, CMouseLeg::Stance);

    for(int i=0;i<ArrayBuffer;i++)
    {
        TrottArray[i][A_TIMESTAMP] = 0 ;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpLeg.coil;
        TrottArray[i][A_SPINE] = centrePos;
        TrottArray[i][A_TAIL] = centrePos;
        TrottArray[i][A_SPINE_FLEX] = centrePos;
        TrottArray[i][A_HEAD_PAN] = uPosHeadPan;
        TrottArray[i][A_HEAD_TILT] = uPosHeadTilt;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// leg machine methods

//initalizes trajectory
void CMouseLeg::StartLeg(double x, double y, int length, typPhase phase)
{
    //length is the length of the array to be filled = number of steps
    stepcount = length;
    currPhase = phase;
    if (phase == Swing){
        risetime = (int)round((double)length/5);
        riseStep = pawLift/risetime;
    }
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
    if ((currPhase == Swing) && (step > 0) && (step<(stepcount-1)) && leg == 'h'){
        if (step < risetime){                       //leg rises
            Y += riseStep*step;                     //rise stepwise with time
        }else if (step > (stepcount-risetime)) {    //leg is set down
            Y += riseStep*(stepcount - step);       //set down with time
        }else {
            Y += pawLift;  // else keep height
        }
    }
    //    if ((currPhase == Swing) && (step > 0) && (step<stepcount-1)){
    //            Y += pawLift;  // else keep height
    //    }
    return (leg == 'f') ? ikforeleg(X, Y, side)
                        : ikhindleg(X, Y, side);
}

// returns the servo values for the leg and sets current internal positios
CLegPos CMouseLeg::SetPosition(CLegPos ang)
{
    docu[step] = ptLeg; //documentation for debugging
    if ((currPhase == Swing) && (step > 0) && (step<(stepcount-1)) && leg == 'f'){
        (side == 'l') ? ang.coil = ang.coil-(pawLift*2)
                : ang.coil = ang.coil+(pawLift*2);
    }
    ptLeg.x += vx;  ptLeg.y += vy;      // Vektor auf letzten Punkt addieren
    return ang; //output zurückgeben
}

//calculates distance between two points
float CMouseLeg::Distance(float x1, float y1, float x2, float y2)
{
    return std::hypot((x2-x1),(y2-y1));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    curSP = curSP + spineStep;
    if (curSP > posFarRight){
        curSP = posFarRight;
    }
    return CSpinePos(curSP, curTL);
}

//centering the Spine
CSpinePos CSpine::centre()
{
    return CSpinePos((posCentre+cOffsetSpine), (posCentre+cOffsetTail));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS CLASS
#if defined ROS
CMouseRos::CMouseRos()
{
    //msgarr.data.resize(14); //need to declare the size, else it wont work
    //pub = n.advertise<std_msgs::Float64MultiArray>("nrpmouse_servotopic", 512);
    n.setParam("length", 50);
    //ros::Rate loop_rate(10); //run at 10Hz, not sleep time
    LElbow_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_L_Elbow/cmd_pos", 512);
    LShoulder_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_L_Shoulder/cmd_pos", 512);
    RElbow_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_R_Elbow/cmd_pos", 512);
    RShoulder_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_R_Shoulder/cmd_pos", 512);
    LHip_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_L_Hip/cmd_pos", 512);
    LKnee_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_L_Knee/cmd_pos", 512);
    RHip_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_R_Hip/cmd_pos", 512);
    RKnee_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_R_Knee/cmd_pos", 512);
    LBodyH10_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_Body10/cmd_pos", 512);
    LBodyH1_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_Body1/cmd_pos", 512);
    LBodyH7_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_Body7/cmd_pos", 512);

    LTail21_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_Tail21/cmd_pos", 512);
    LTail15_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_Tail15/cmd_pos", 512);
    LTail9_pub = n.advertise<std_msgs::Float64>("/Nermo/joint_Tail9/cmd_pos", 512);

    std::cout << "Publisher initalized!" << std::endl;

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
    int cmd;
    bool OK = true;
    //Amount of Waypoints generated per motion:
    int motionlength = 50;
    int boundlength = 20;
    int situpDownTime = 80;
    int switchingTime = 20;
    int pushingTime = 20;
    int initTime = 1;

    //get Programm starting time
    StartTime = GetCurTime().count();

    //set everything to 180 deg(neutral position) to avoid damage.
    clearArr();
    //print greeting message
    Greeting();

    while (OK)
    {
        cmd = messages; //just one access to messages per run - not yet atomic!!
        //cmd = 'a';
        if (cmd != state && cmd != 0){
            state = cmd;
        }
        switch (state) {
        case 'i':   //initalize pose
            dir = stop;
            std::cout << "init" << std::endl;
            Init(initTime);
            Publish(initTime);
            messages = 0;
            state = 'h';
            break;
        case 'w': //walk forward+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            dir = Fwd;
            std::cout << "Straight ahead" << std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 's': //Stop Motors
            dir = Bkwd;
            std::cout<<"Stop Motors"<<std::endl;
            StopAllMotors();
            messages = 0;
            state = 'h';
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
        case 'b':   //Bound Gait+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            std::cout<<"Bound"<<std::endl;
            Bound(boundlength);
            Publish(boundlength);
            messages = 0;
            state = '.';
            break;
        case 'n':   //Bound2 Gait
            std::cout<<"Bound2"<<std::endl;
            Bound2(boundlength);
            Publish(boundlength);
            messages = 0;
            state = '.';
            break;
        case 'y':   //Sitting+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            std::cout<<"Sitting"<<std::endl;
            SitUp(situpDownTime);
            Publish(situpDownTime);
            messages = 0;
            state = 'h';
            break;
        case 'f':   //press both paws
            std::cout<<"press both paws"<<std::endl;
            PushBothHands(switchingTime);
            Publish(switchingTime);
            messages = 0;
            state = 'h';
            break;
        case 't':   //lift both paws
            std::cout<<"lift both paws"<<std::endl;
            LiftBothHands(switchingTime);
            Publish(switchingTime);
            messages = 0;
            state = 'h';
            break;
        case 'e':   //push left paw
            std::cout<<"push left paw"<<std::endl;
            ReleaseLever(pushingTime, 'l');
            Publish(pushingTime);
            messages = 0;
            state = 'h';
            break;
        case 'r':   //push right paw
            std::cout<<"push right paw"<<std::endl;
            ReleaseLever(pushingTime, 'r');
            Publish(pushingTime);
            messages = 0;
            state = 'h';
            break;
        case 'x':   //sit down
            std::cout<<"sit down"<<std::endl;
            SitDown(situpDownTime);
            Publish(situpDownTime);
            messages = 0;
            state = 'h';
            break;
        case '+':   //increase speed+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            CommandDelay = CommandDelay + 5000;
            std::cout<<"new Speed: "<<CommandDelay<<"\n";
            messages = 0;
            state = 'h';
            break;
        case '-':   //decrease speed
            CommandDelay = CommandDelay - 5000;
            std::cout<<"new Speed: "<<CommandDelay<<"\n";
            messages = 0;
            state = 'h';
            break;
        case 'm':   //publish motions to uart
            Publish(motionlength);
            break;
        case '.':   //publish motions to uart
            Publish(boundlength);
            break;
        case 'h':   //idle-hold
            dir = stop;
            usleep(90);
            break;
        case 'p':   //save motion data++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            StoreFile();
            messages = 0;
            state = 'h';
            break;
        case 'o':   //read motion data
            std::cout<<"reading data"<<std::endl;
            ReadFile();
            messages = 0;
            state = 'h';
            break;
        case 'l':   //playback motion data once
            PlayFile();
            messages = 0;
            state = 'h';
            break;
        case ',':   //playback motion data reverse once
            PlayFileReverse();
            messages = 0;
            state = 'h';
            break;
        case 'k':   //playback motion data continous
            PlayFile();
            break;
        case 'j':   //playback motion data reverse continous
            PlayFileReverse();
            break;
        case 'q':   //quit programm
            std::cout<<"Quitting"<<std::endl;
            StopAllMotors();
            return;
        }
    }
}

void CMouseRos::Publish(int length)
{
    ros::Rate loop_rate(20); //run at 20Hz, not sleep time
    for(int i=0;i<length;i++){

        LShoulder.data=(degToRad(TrottArray[i][FORELEFT_HIP]))-1.4;
        LElbow.data=-((degToRad(TrottArray[i][FORELEFT_KNEE])*0.4)-0.6);
        RShoulder.data=(degToRad(TrottArray[i][FORERIGHT_HIP]))-1.4;
        RElbow.data=-((degToRad(TrottArray[i][FORERIGHT_KNEE])*0.4)-0.6);
        LHip.data=(degToRad(TrottArray[i][HINDLEFT_HIP]))-0.2;
        RHip.data=(degToRad(TrottArray[i][HINDRIGHT_HIP]))-0.2;
        LKnee.data=(-degToRad(TrottArray[i][HINDLEFT_KNEE])/2)+0.7;
        RKnee.data=(-degToRad(TrottArray[i][HINDRIGHT_KNEE])/2)+0.7;
        LBodyH.data=(degToRad(TrottArray[i][SPINE]))-1.309;
        LBodyH2.data=-((degToRad(TrottArray[i][SPINE]))-1.309);
        //LBodyH3.data=-((degToRad(TrottArray[i][SPINE]))-1.309);

        LTail.data=(degToRad(TrottArray[i][TAIL])*0.4)-0.6;

        LElbow_pub.publish(LElbow);
        LShoulder_pub.publish(LShoulder);
        RElbow_pub.publish(RElbow);
        RShoulder_pub.publish(RShoulder);
        LHip_pub.publish(LHip);
        LKnee_pub.publish(LKnee);
        RHip_pub.publish(RHip);
        RKnee_pub.publish(RKnee);
        LBodyH10_pub.publish(LBodyH);
        LBodyH1_pub.publish(LBodyH2);
        LBodyH7_pub.publish(LBodyH2);

        //LTail21_pub.publish(LTail);
        //LTail15_pub.publish(LTail);
        //LTail9_pub.publish(LTail);


        ros::spinOnce();
        loop_rate.sleep();

    }
}

float CMouseRos::degToRad(double l)
{
    return l*halfC;
}

#endif



