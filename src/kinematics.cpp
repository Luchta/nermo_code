//////////////////////////////////////////////////////////////////////////////////////////////////////////
// kinematics.cpp

#include "kinematics.h"
#define _USE_MATH_DEFINES
#include <math.h>   //cos sin and all


//------Foreleg Functions-----------
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
    double ft1;	//shoulder angle
    double ft2;	//elbow angle
    double flb;	//length elbow-fingertip
    double ftb;	//angle upper_arm - flb

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
    else if (fx==0)
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
    double lknee;
    double fls;
    double centerdist;
    double lhip;

    if (ft1>ft1i)         											//when leg is moved forward
    {lhip=(fahip-(ft1-ft1i))*M_PI/180*3.7;} 						//the string length at the hip pulley,
    else    														//when leg is moved backward
    {lhip=(fahip+(ft1i-ft1))*M_PI/180*3.7;}

    centerdist=sqrt(pow(fl1,2)+pow(fltibia,2)-(2*fl1*fltibia*cos(ft2*M_PI/180)));
    lknee=sqrt(pow(centerdist,2)-pow(frhip,2));

    fls=lhip+lknee;     											//total calculated length

    if (side == 'l')
    {
        if (fls>flsi)        											//need to release string, turn cw (+)
        {return fksainit+(kneeservoangle(fls-flsi))*180/M_PI;}
        else                  											//need to tension string
        {return fksainit-(kneeservoangle(flsi-fls))*180/M_PI;}
    }else {
        if (fls>flsi)         											//need to release string, turn anticw(-)
        {return fksainit-(kneeservoangle(fls-flsi))*180/M_PI;}
        else                  											//need to tension string, turn cw (+)
        {return fksainit+(kneeservoangle(flsi-fls))*180/M_PI;}
    }


}

double CKinematics::foreHipServoAngle(double ft1, char side)
{
    //calculates the servoangle for hip servo for foreleg
    double ft1i=53;
    double fhsainit=90;	//like the midpointposition of the servo
    double floffset=5;	//offset for calibration of servo position (manufacturing/assembly problem from the servo)
    if (side == 'l')
    {
        if (ft1>ft1i)       //move leg forward, turn cw (+)
        {return fhsainit+(ft1-ft1i)+floffset;}
        else                //move leg back
        {return fhsainit-(ft1i-ft1)+floffset;}
    }
    else {
        if (ft1>ft1i)       //move leg forward, turn anticw (-)
        {return fhsainit-(ft1-ft1i);}
        else                //move leg back,
        {return fhsainit+(ft1i-ft1);}
    }
}

//------Hindleg Functions-----------
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

    double ht1; //Hip Angle - theta 1
    double ht2;	//Knee Angle - theta 2
    double hlb; //length knee-toe
    double htb;	// angle oberschenkel-knee-toe

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
    else if (hx==0)
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
    double lknee;
    double hls;
    double centerdist;
    double lhip;

    if (ht1>ht1i)           										//when leg is moved backward
    {lhip=(hahip-(ht1-ht1i))*M_PI/180*3.7;} 						//the string length at the hip pulley,
    else                    										//when leg is moved forward
    {lhip=(hahip+(ht1i-ht1))*M_PI/180*3.7;}

    //
    centerdist=sqrt(pow(hl1,2)+pow(hltibia,2)-(2*hl1*hltibia*cos(ht2*M_PI/180)));
    lknee=sqrt(pow(centerdist,2)-pow(hrhip,2));

    hls=lhip+lknee;     											//total calculated length

    //calculation for left side
    if (side == 'l')
    {
        if (hls>hlsi)         											//need to release string, turn anticw (-)
        {return hksainit-(kneeservoangle(hls-hlsi))*180/M_PI;}
        else                											//need to tension string
        {return hksainit+(kneeservoangle(hlsi-hls))*180/M_PI;} 		//returns the final servo angle value in (degrees)
    }
    else if (side == 'r')//calculation for right side
    {
        if (hls>hlsi)         											//need to release string, turn cw(+)
        {return hksainit+(kneeservoangle(hls-hlsi))*180/M_PI;}
        else                											//need to tension string
        {return hksainit-(kneeservoangle(hlsi-hls))*180/M_PI;} 		//returns the final servo angle value in (degrees)
    }
    else {
        return -1;
    }

}

double CKinematics::hindHipServoAngle(double ht1, char side)
{
    //calculates the servoangle for hip servo for hindleg
    double ht1i=8;
    if (side == 'l')
    {
        double hhsainit=150;//like the midpointposition of the servo
        if (ht1>ht1i)       //move leg back,  turn anti cw (-)
        {return hhsainit-(ht1-ht1i);}
        else                //move leg forward
        {return hhsainit+(ht1i-ht1);}
    }
    else {
        double hhsainit=30;//like the midpointposition of the servo
        if (ht1>ht1i)       //move leg back,  turn cw (+)
        {return hhsainit+(ht1-ht1i);}
        else                //move leg forward
        {return hhsainit-(ht1i-ht1);}
    }
}

//------General Functions-----------
double CKinematics::kneeservoangle(double l)
{
    //calculates the amount of angle required for the pulley to wind a length of string
    return l/rp1;          //in radians
}
