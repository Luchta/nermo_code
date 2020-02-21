//////////////////////////////////////////////////////////////////////////////////////////////////////////
// kinematics.cpp

#include "Kinematics.h"
#define _USE_MATH_DEFINES
#include <math.h>   //cos sin and all


//Foreleg Functions////////////////////////////////////////////////////////////////////////////////////////////////
CLegPos CKinematics::ikforeleg(double fx, double fy, char side)
{
    //side can be l( left leg ) and r (right leg)
    //ik of the fore leg, returns the servoangles required for IK
    //theta3 (hand angle) is assumed to be a constant to prevent complex calculations
    // Assuming the kinematics only, no ground contact forces

    //---- setup Values -----------------------------------
    //Initial position of leg in taskspace
    //(The Toe is located 28 in Y Direction and 55.15 in X DIrection)
    double fpos_init[1][2]={28,55.15};	//corresponding to trot gait

    //---- intermediate Values -----------------------
    double ft1=0;	//shoulder angle
    double ft2=0;	//elbow angle
    double flb=0;	//length elbow-fingertip
    double ftb=0;	//angle upper_arm - flb

    //shifting the goal coordinate by the initial toe coordinate
    fx=fx-fpos_init[0][0];         //origin position of the leg (easier for trajectory planning)(relative to center point of trajectory circle)
    fy=fy-fpos_init[0][1];

    //---- IK Calculations -----------------------
    flb=sqrt(pow(fl3,2)+pow(fl2,2)-(2*fl2*fl3*cos(ft3i*M_PI/180)));
    ftb=acos(((pow(fx,2)+pow(fy,2))-(pow(fl1,2)+pow(flb,2)))/(-2*fl1*flb));
    ft2=ftb+asin(fl3*sin(ft3i*M_PI/180)/flb);

    // differentiate between X coordinate -> passing of horizontal line for Shoulder Angle
    if (fx<0)
    {ft1=M_PI-atan(fy/fx)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    else if (fx>0)
    {ft1=-atan(fy/fx)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    else //if (fx==0) only 0 remains
    {ft1=(90*M_PI/180)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}

    // Rückrechnung in Grad
    ft1=ft1*180/M_PI;
    ft2=ft2*180/M_PI;

    //calculation of servo positions from leg angles (side differentiation)
    //goal->leg = foreHipServoAngle(ft1, side);
    //goal->coil = foreKneeServoAngle(ft1, ft2, side);
#ifdef _WINDOWS
    return CLegPos(ft1*10, ft2*10);    // Simulation: Zehntel-Winkel zur Anzeige direkt
#else
    return CLegPos(foreHipServoAngle(ft1, side), foreKneeServoAngle(ft1, ft2, side));
#endif
}

double CKinematics::foreKneeServoAngle(double ft1,double ft2, char side)
{
    //calculates the servo angle for the knee servo for foreleg
    //hip pulley is 8mm diameter, 1mm string diameter sweep, 3.5mm effective radius
    //distance from string attachment point to knee geometric center=13.914mm
    //initial string length=41.198mm (geometric length)(From Solidworks)
    double lknee=0,fls=0, centerdist=0, lhip=0;

    //calcuate length of string at the hip turn
    if (ft1>ft1i)         											//when leg is moved forward
    {lhip=(fahip-(ft1-ft1i))*M_PI/180*3.7;} 						//the string length at the hip pulley,
    else    														//when leg is moved backward
    {lhip=(fahip+(ft1i-ft1))*M_PI/180*3.7;}

    //calculate length of string from hip to attachment point at leg
    centerdist=sqrt(pow(fl1,2)+pow(fltibia,2)-(2*fl1*fltibia*cos(ft2*M_PI/180))); //length from attachment point to hip centre
    lknee=sqrt(pow(centerdist,2)-pow(frhip,2)); //length from hip tanget to attachment point

    fls=lhip+lknee;     											//total calculated length

    //calculation for left/right side
    // *180/M_PI is for change from radians to degrees
    if (side == 'l')
    {                                                                   //left side-----------------
        if (fls>flsi)        											//need to release string, turn cw (+)
        {return lflCoilInit+(kneeservoangle(fls-flsi))*180/M_PI;}
        else                  											//need to tension string
        {return lflCoilInit-(kneeservoangle(flsi-fls))*180/M_PI;}
    }else {                                                             //right side-------------------
        if (fls>flsi)         											//need to release string, turn anticw(-)
        {return rflCoilInit-(kneeservoangle(fls-flsi))*180/M_PI;}
        else                  											//need to tension string, turn cw (+)
        {return rflCoilInit+(kneeservoangle(flsi-fls))*180/M_PI;}
    }


}

double CKinematics::foreHipServoAngle(double ft1, char side)
{
    //calculates the servoangle for the foreleg hip servos
    //Forumlar: ServoCentrePosition +- (abs(initialKinematicAngle - newKinematicAngle))
    // Servo Centre Position depends on servo range, as the range as to aviod running into unavailaple positions
    //double ft1i=53; already defined
    //double fhsainit=90;	// now lflHipInit and rflHipInit//like the midpointposition of the servo
    //double floffset=5;	//offset for calibration of servo position (manufacturing/assembly problem from the servo)
    if (side == 'l')
    {
        if (ft1>ft1i)       //move leg forward, turn cw (+)
        {return lflHipInit+(ft1-ft1i)+flOffset;}
        else                //move leg back
        {return lflHipInit-(ft1i-ft1)+flOffset;}
    }
    else {
        if (ft1>ft1i)       //move leg forward, turn anticw (-)
        {return rflHipInit-(ft1-ft1i)+frOffset;}
        else                //move leg back,
        {return rflHipInit+(ft1i-ft1)+frOffset;}
    }
}

//Hindleg Functions////////////////////////////////////////////////////////////////////////////////////////
CLegPos CKinematics::ikhindleg(double hx, double hy, char side)
{
    //side can be l( left leg ) and r (right leg)
    //ik of the hind leg, returns the servoangles required for IK
    //theta3 (foot angle) is assumed to be a constant to prevent complex calculations
    //The string will be routed on the tibia for now.
    //Assuming the kinematics only, no ground contact forces


    //---- setup Values -----
    //Initial position of leg in taskspace
    //(The Toe is located 20mm in Y Direction and 59.15mm in X DIrection)
    double hpos_init[1][2]={20,59.15};

    /*
                   [0.0] centre point
                ht1(/
                   /
                  /
                  \ )ht2
                   \
                    \
                     \
                ------

    */

    //---- intermediate Values -----

    double ht1=0, //Hip Angle - theta 1
            ht2=0,	//Knee Angle - theta 2
            hlb=0, //length knee-toe
            htb=0;	// angle oberschenkel-knee-toe

    //shifting the goal coordinate by the initial toe coordinate
    hx = hx-hpos_init[0][0];
    hy = hy-hpos_init[0][1];

    //---- IK Calculations -----------------------
    hlb = sqrt(pow(hl3,2)+pow(hl2,2)-(2*hl2*hl3*cos(ht3i*M_PI/180)));
    htb = acos(((pow(hx,2)+pow(hy,2))-(pow(hl1,2)+pow(hlb,2)))/(-2*hl1*hlb));
    ht2 = htb-asin(hl3*sin(ht3i*M_PI/180)/hlb);

    // differentiate between X coordinate -> passing of horizontal line for Hip Angle
    if (hx>0)
    {
        ht1=M_PI+atan(hy/hx)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));
    }
    else if (hx<0)
    {
        ht1=atan(hy/hx)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));
    }
    else //if (hx==0) only 0 remains
    {
        ht1=(90*M_PI/180)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));
    }

    //
    ht1=ht1*180/M_PI;
    ht2=ht2*180/M_PI;

    //calculation of servo positions from leg angles (side differentiation)
    //goal->leg = hindHipServoAngle(ht1, side);
    //goal->coil = hindKneeServoAngle(ht1,ht2, side);
#ifdef _WINDOWS
    return CLegPos(ht1*10, ht2*10);  // Simulation: Zehntel-Winkel zur Anzeige direkt
#else
    return CLegPos(hindHipServoAngle(ht1, side), hindKneeServoAngle(ht1, ht2, side));
#endif
}

double CKinematics::hindKneeServoAngle(double ht1, double ht2, char side)
{
    //calculates the servo angle for the knee servo for hingleg
    //hip pulley is 8mm diameter, 1mm string diameter sweep, 3.5mm effective radius
    //distance from string attachment point to knee geometric center=16.037mm
    //initial string length=37.687mm (geometric length)(From Solidworks)
    double lknee=0, hls=0, centerdist=0, lhip=0;

    //calcuate length of string at the hip turn
    if (ht1>ht1i)           										//when leg is moved backward
    {lhip=(hahip-(ht1-ht1i))*M_PI/180*3.7;} 						//the string length at the hip pulley,
    else                    										//when leg is moved forward
    {lhip=(hahip+(ht1i-ht1))*M_PI/180*3.7;}

    //calculate length of string from hip to attachment point at leg
    centerdist=sqrt(pow(hl1,2)+pow(hltibia,2)-(2*hl1*hltibia*cos(ht2*M_PI/180)));//length from attachment point to hip centre
    lknee=sqrt(pow(centerdist,2)-pow(hrhip,2));//length from hip tanget to attachment point

    hls=lhip+lknee;     											//total calculated length

    //calculation for left/right side
    // *180/M_PI is for change from radians to degrees
    if (side == 'l')
    {                                                                   //left side-----------------
        if (hls>hlsi)         											//need to release string, turn anticw (-)
        {return lhlCoilInit-(kneeservoangle(hls-hlsi))*180/M_PI;}
        else                											//need to tension string
        {return lhlCoilInit+(kneeservoangle(hlsi-hls))*180/M_PI;} 		//returns the final servo angle value in (degrees)
    }
    else //if (side == 'r')
    {                                                                   //right side-----------------
        if (hls>hlsi)         											//need to release string, turn cw(+)
        {return rhlCoilInit+(kneeservoangle(hls-hlsi))*180/M_PI;}
        else                											//need to tension string
        {return rhlCoilInit-(kneeservoangle(hlsi-hls))*180/M_PI;} 		//returns the final servo angle value in (degrees)
    }
    /*else {
        return -1;
    }*/

}

double CKinematics::hindHipServoAngle(double ht1, char side)
{
    //calculates the servoangle for the hindleg hip servos
    //Forumlar: ServoCentrePosition +- (abs(initialKinematicAngle - newKinematicAngle))
    // Servo Centre Position depends on servo range, as the range as to aviod running into unavailaple positions
    if (side == 'l')
    {
        //double hhsainit=150;//now lhlHipInit //like the midpointposition of the servo
        if (ht1>ht1i)       //move leg back,  turn anti cw (-)
        {return lhlHipInit-(ht1-ht1i)+hlOffset;}
        else                //move leg forward
        {return lhlHipInit+(ht1i-ht1)+hlOffset;}
    }
    else {
        //double hhsainit=30;//now rhlHipInit //like the midpointposition of the servo
        if (ht1>ht1i)       //move leg back,  turn cw (+)
        {return rhlHipInit+(ht1-ht1i)+hrOffset;}
        else                //move leg forward
        {return rhlHipInit-(ht1i-ht1)+hrOffset;}
    }
}

//------General Functions-----------
double CKinematics::kneeservoangle(double l)
{
    //calculates the amount of angle required for the pulley to wind a length of string
    return l/rp1;          //in radians
}
