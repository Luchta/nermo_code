
#ifndef PHYSIOLOGY_H
#define PHYSIOLOGY_H

#include <iostream>

//Safe Position Defaults
//Left Hindfoot
#define DEFAULT_HINDLEFT_HIP 240
#define DEFAULT_HINDLEFT_FOOT 0
//Right Hindfoot
#define DEFAULT_HINDRIGHT_HIP 40
#define DEFAULT_HINDRIGHT_FOOT 240
//Left Frontfoot
#define DEFAULT_FORELEFT_HIP 40
#define DEFAULT_FORELEFT_FOOT 240
//Right Frontfoot
#define DEFAULT_FORERIGHT_HIP 240
#define DEFAULT_FORERIGHT_FOOT 0

enum legservo : uint8_t
{
    HIND_LEFT_HIP,
    HIND_LEFT_FOOT,
    FORE_LEFT_HIP,
    FORE_LEFT_FOOT,
    FORE_RIGHT_FOOT,
    FORE_RIGHT_HIP,
    HIND_RIGHT_FOOT,
    HIND_RIGHT_HIP,
};

enum leg : int
{
    HIND_LEFT,
    FORE_LEFT,
    FORE_RIGHT,
    HIND_RIGHT
};


//++               *               ++
//++              ***              ++
//++               *               ++
//++ <270-0>     2 * 5     <0-270> ++
//++ <0-270>     3 * 4     <270-0> ++
//++              ***              ++
//++              ***              ++
//++              ***              ++
//++ <270-0>     1 * 6     <0-270> ++
//++ <0-270>     0 * 7     <270-0> ++
//++               *               ++
//++               *               ++
//++               *               ++
//++               *               ++
//++               *               ++
//++               *               ++
//+++++++++++++++++++++++++++++++++++

//Variable Coding: POS_<FORE/HIND>_<LT/RT>_<HIP/FOOT>_<FWD/BKWD/CENTER/TURN>/</UP/DOWN>

/*
    uint16_t POS_HIND_LT_HIP_FWD     = 260; //notizen 270
    uint16_t POS_HIND_LT_HIP_CENTER  = 230;
    uint16_t POS_HIND_LT_HIP_BKWD    = 210; //notizen 190
    uint16_t POS_HIND_LT_FOOT_UP     = 210; //notizen 110
    uint16_t POS_HIND_LT_FOOT_DOWN   = 0;   //notizen 0
 */
/*
//Left Hindfoot Positions
#define POS_HIND_LT_HIP_FWD     10//inv260 //
#define POS_HIND_LT_HIP_CENTER  40//inv230 //
#define POS_HIND_LT_HIP_TURN    40//inv230 //instead of bkwd
#define POS_HIND_LT_HIP_BKWD    60//inv210 //

#define POS_HIND_LT_FOOT_UP     230
#define POS_HIND_LT_FOOT_DOWN   70 //35

//Right Hindfoot Positions
#define POS_HIND_RT_HIP_FWD     15  //20
#define POS_HIND_RT_HIP_CENTER  40  //
#define POS_HIND_RT_HIP_TURN    35  // 45 instead of bkwd
#define POS_HIND_RT_HIP_BKWD    65  //

#define POS_HIND_RT_FOOT_UP     230//inv40
#define POS_HIND_RT_FOOT_DOWN   70//inv200

//Left Frontfoot Positions
#define POS_FORE_LT_HIP_FWD     65 //55,60,80
#define POS_FORE_LT_HIP_TURN    25 // 35 instead of fwd
#define POS_FORE_LT_HIP_CENTER  40 //
#define POS_FORE_LT_HIP_BKWD    18 //0,10
#define POS_FORE_LT_HIP_LIFT    8 //0,10

#define POS_FORE_LT_FOOT_UP     145//inv125
#define POS_FORE_LT_FOOT_DOWN   45//inv225

//Right Frontfoot Positions
#define POS_FORE_RT_HIP_FWD     70//inv200
#define POS_FORE_RT_HIP_TURN    25//inv245 instead of fwd
#define POS_FORE_RT_HIP_CENTER  40//inv220 //
#define POS_FORE_RT_HIP_BKWD    22//inv248 //
#define POS_FORE_RT_HIP_LIFT    13//inv257 //

#define POS_FORE_RT_FOOT_UP     125
#define POS_FORE_RT_FOOT_DOWN   30 //
*/

//Left Hindfoot Positions
#define POS_HIND_LT_HIP_FWD     260 //246
#define POS_HIND_LT_HIP_CENTER  245 //230 //230
#define POS_HIND_LT_HIP_TURN    230 // 220 instead of bkwd
#define POS_HIND_LT_HIP_BKWD    210 //200
#define POS_HIND_LT_FOOT_UP     230
#define POS_HIND_LT_FOOT_DOWN   70 //35

//Right Hindfoot Positions
#define POS_HIND_RT_HIP_FWD     15  //20
#define POS_HIND_RT_HIP_CENTER  33//40  //
#define POS_HIND_RT_HIP_TURN    35  // 45 instead of bkwd
#define POS_HIND_RT_HIP_BKWD    65  //
#define POS_HIND_RT_FOOT_UP     40 //20
#define POS_HIND_RT_FOOT_DOWN   200 //230

//Left Frontfoot Positions
#define POS_FORE_LT_HIP_FWD     70//65 //55,60,80
#define POS_FORE_LT_HIP_TURN    25 // 35 instead of fwd
#define POS_FORE_LT_HIP_CENTER  40 //
#define POS_FORE_LT_HIP_BKWD    15//18 //0,10
#define POS_FORE_LT_HIP_LIFT    8 //0,10

#define POS_FORE_LT_FOOT_UP     125
#define POS_FORE_LT_FOOT_DOWN   225 //240

//Right Frontfoot Positions
#define POS_FORE_RT_HIP_FWD     195//200 //215,210,190
#define POS_FORE_RT_HIP_TURN    245 // 235 instead of fwd
#define POS_FORE_RT_HIP_CENTER  230 //220 //
#define POS_FORE_RT_HIP_BKWD    255//248 //260,250
#define POS_FORE_RT_HIP_LIFT    257 //260,250
#define POS_FORE_RT_FOOT_UP     125
#define POS_FORE_RT_FOOT_DOWN   30 //


#endif
