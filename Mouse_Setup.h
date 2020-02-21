/*Simple terminal based mouse control programm for Sero setup, pin control and walking tests
 *
 * Author: Peer Lucas
 * Email: peer.lucas@tum.de
 *
 * Last edited: 24.02.2017
 *
 */


//standard includes
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <string>

#include "mraa.hpp"
//mouse includes
#include "NRPMouse.h"

//Constants
#define ALLLEGS 0
#define FORELEGS 1
#define HINDLEGS 2
#define LEFT_FORELEG 3
#define RIGHT_FORELEG 4
#define LEFT_HINDLEG 5
#define RIGHT_HINDLEG 6

class MouseSetup
{

public:

// Constructor and destructor
MouseSetup();
~MouseSetup(){} //

void mainMenu(void);                    //main Menu starter

//settings
void init(void);                        //initialization of servo mode, limits and positions
void readInit(void);                    //read position init file


private:
//GPIO Control
void gpio(void);                        //menu
void sense(void);

//walking test and Control
void walking(void);                     //menu
void walkFwd(int side);
void legCtrl(int pos);
void legSetup(void);

//Servo Control
void servos(void);                      //menu
void setServoAngle(void);               //setting servo angles (servo setting in the menu)
void setServoDutycycle(void);           //setting of the servo dutycycle

legservo intToServo(int pin);
leg intToLeg(int pin);

};
