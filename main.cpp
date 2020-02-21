#include <iostream>
#include "MouseCtrl.h"
#include <unistd.h>

using namespace std;

int main()
{
    cout << "Hello World!" << endl;
    CMouseCtrl Mouse;// = CMouseCtrl();
    //CMouseCtrl Mouse = CMouseCtrl();
    CMouseUI UI = CMouseUI(Mouse.messages);

    Mouse.startCtrlThread();

    Mouse.sendNL();
    usleep(10000);

    Mouse.MotorP = 8;
    Mouse.MotorI = 1;
    Mouse.MotorD = 21;//h15

    /*old sets:
    *9-0-25
    *16-0-37
    *17-0-35
    */
    Mouse.MotorSetup();

    //usleep(200000);
    UI.process();

    return 0;
}
