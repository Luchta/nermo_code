#ifndef KINEMATICS_H
#define KINEMATICS_H
/* The class CKinematics calculates the kinematics for the legs of the NRP Mouse Version 4
 *
 * Usage:
 * Input are x and y coordinates and a 'l' or 'r' regarding the side, respectively for fore and hind legs
 * Output is a CLegPos class including angle values for the motors in Degree (negatives are possible)
 * x=y=0 results in the initial stance position of the Mouse
 */

#define NRPServos //this defines the use of the custom build NRP Servos which changes the centre positions for optimal use

class CLegPos
{
public:
    CLegPos() { leg=coil=0; }
    CLegPos(double l, double c) { leg=l; coil=c; }
    double leg;
    double coil;
};

class CKoord
{
public:
    CKoord() { x=y=0; }
    CKoord(double px, double py) { x=px; y=py; }
    double x;
    double y;
};


class CKinematics
{
public:
    CKinematics() {
        /*rp1=6.75;        //radius of the pulley

        //Foreleg
        fl1=31.0;        //l_humerus length upper arm
        fl2=26.0;        //l_ulna length forearm
        fl3=13.0;        //l_hand length hand
        ft1i=54.24;      //180° - init angle shoulder
        ft2i=87.65;      //180° - init angle elbow
        ft3i=155.0;      //init angle hand

        //Foreleg_Servo_Positions
        flsi=48.332; 	//flsi=41.198+(fahip*pi/180*frhip); //initial calculated length
        fltibia=13.914;  //string attachment point on tibia
        frhip=3.5;       //effective radius of the pulley
        fahip=116.79;    //initial angle of string routed on the hip
        //fksainit=80;

        //Hindleg
        hl1=35.0;        //l_femur 	(thigh Oberschenkel)
        hl2=40.0;        //l_tibia 	(lower leg Unterschenkel)
        hl3=19.0;        //l_toe		(toe Zeh)
        ht1i=7.49;       //a_femur	(init angle femur)
        ht2i=108.93;     //a_knee	(180 - init angle knee)
        ht3i=110.0;      //a_toe		(init angle toe)

        //Hindleg_Servo_Positions
        hlsi=47.477;     //hlsi=37.687+(hahip*pi/180*hrhip); //initial total length
        hltibia=16.037;  //string attachment point on tibia
        hrhip=3.5;       //effective radius of the pulley
        hahip=160.26;    //initial angle of string routed on the hip
        //hksainit=40;*/
    }

    //void createWaypoints(int Ax, int Ay, int Bx, int By, int *waypoints, int lengths);
    CLegPos ikhindleg(double hx, double hy, char side);
    CLegPos ikforeleg(double fx, double fy, char side);

    void getHindLegDims(double& h1, double& h2, double& h3, double& h3i) { h1=hl1; h2=hl2; h3=hl3; h3i=ht3i; }
    void getForeLegDims(double& h1, double& h2, double& h3, double& h3i) { h1=fl1; h2=fl2; h3=fl3; h3i=ft3i; }
private:

    //Variables

    //foreleg and hindleg position matrix for trot and walk trajectory
    //double fpos_trot[5][2]={{0,0},{25,0},{50,0},{40,25},{12,20}}; 	//refined values
    //double hpos_trot[5][2]={{0,5},{25,5},{50,0},{45,35},{20,30}};	//refined values
    //double fpos_trotinit[1][2]={28,55.15};	//refined values
    //double hpos_trotinit[1][2]={20,59.15};	//refined values

    //Kinematic Valuess////////////////////////////////////////////////////////////////////
    const double rp1 = 7.5;//=6.75; //radius of the pulley

    //Foreleg---------------------------------------------------------------------------
    const double fl1=27.8;//31.0;   //l_humerus length upper arm
    const double fl2=26.4;//26.0;   //l_ulna length forearm
    const double fl3=14.2;//13.0;   //l_hand length hand
    //initial Angles at x=y=0
    const double ft1i=54.24;        //180° - init angle shoulder
    const double ft2i=87.65;        //180° - init angle elbow
    const double ft3i=155.0;        //init angle hand

    //Foreleg_String_Calculation
    const double fltibia=10;        //string attachment point on tibia
    const double frhip=4;//3.5;     //effective radius of the pulley
    const double fahip=116.79;      //initial angle of string routed on the hip
    const double flipfree=41.198; 	//inital length of free string Attachment-Hip
    const double flsi=flipfree+(fahip*3.1415/180*frhip); //initial calculated length
    //const double flsi=48.332; 	//flsi=41.198+(fahip*pi/180*frhip); //initial calculated length

    //Hindleg---------------------------------------------------------------------------
    const double hl1=35.0;          //l_femur 	(thigh Oberschenkel)
    const double hl2=39.4;//40.0;   //l_tibia 	(lower leg Unterschenkel)
    const double hl3=20;//19.0;     //l_toe		(toe Zeh)
    //initial Angles at x=y=0
    const double ht1i=7.49;         //a_femur	(init angle femur)
    const double ht2i=108.93;       //a_knee	(180 - init angle knee)
    const double ht3i=110.0;        //a_toe		(init angle toe)

    //Hindleg_String_Calculation
    const double hltibia=17.4;      //string attachment point on tibia
    const double hrhip=4;//3.5;     //effective radius of the pulley
    const double hahip=160.26;      //initial angle of string routed on the hip
    const double hlhipfree=37.687;  //initial length of string hip-attachment
    const double hlsi=hlhipfree+(hahip*3.1415/180*hrhip); //initial total length
    //const double hlsi=47.477;     //hlsi=37.687+(hahip*pi/180*hrhip); //initial total length

    //Servo Specific Setup////////////////////////////////////////////////////////////////////
    // in degree
    //double fksainit;//=80;
    //double hksainit;//=40;
    //initial (centre) position of servos for optimal usage of servo range
#ifdef NRPServos
    //Hindlegs---------------------------------------------------------------------------
    const double lhlHipInit = 180;//150;  //hind left hip
    const double rhlHipInit = 180;//30;   //hind right hip
    const double lhlCoilInit = 180;//40;  //hind left Coil
    const double rhlCoilInit = 180;//40;  //hind right Coil
    //Forelegs---------------------------------------------------------------------------
    const double lflHipInit = 180;//90;   //fore left Hip
    const double rflHipInit = 180;//90;   //fore right Hip
    const double lflCoilInit = 180;//80;  //fore left Coil
    const double rflCoilInit = 180;//80;  //fore right coil
    //calibration offsets----------------------------------------------------------------
    const double flOffset=0;//5;        //offset value for foreleft hip servo
    const double frOffset=0;        //offset value for foreright hip servo
    const double hlOffset=0;        //offset value for hindleft hip servo
    const double hrOffset=0;        //offset value for hindright hip servo
#else
    //Hindlegs---------------------------------------------------------------------------
    const double lhlHipInit = 150;  //hind left hip
    const double rhlHipInit = 30;   //hind right hip
    const double lhlCoilInit = 40;  //hind left Coil
    const double rhlCoilInit = 40;  //hind right Coil
    //Forelegs---------------------------------------------------------------------------
    const double lflHipInit = 90;   //fore left Hip
    const double rflHipInit = 90;   //fore right Hip
    const double lflCoilInit = 80;  //fore left Coil
    const double rflCoilInit = 80;  //fore right coil
    //calibration offsets----------------------------------------------------------------
    const double flOffset=5;        //offset value for foreleft hip servo
    const double frOffset=0;        //offset value for foreright hip servo
    const double hlOffset=0;        //offset value for hindleft hip servo
    const double hrOffset=0;        //offset value for hindright hip servo
#endif

    //Functions////////////////////////////////////////////////////////////////////
    //Hindleg
    double hindKneeServoAngle(double ht1,double ht2, char side);
    double hindHipServoAngle(double ht1, char side);

    //Foreleg
    double foreKneeServoAngle(double ht1,double ht2, char side);
    double foreHipServoAngle(double ht1, char side);

    //
    double kneeservoangle(double l);
};

#endif // KINEMATICS_H
