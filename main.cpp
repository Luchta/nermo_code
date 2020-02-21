#include "Mouse_Setup.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <string>

#include "mraa.hpp"

MouseSetup* mousetest = new MouseSetup();

int main(void) {
    std::cout << "Hello Edison!\n";
    std::cout << "Starting Init"<<std::endl;
    mousetest->init();

    mousetest->mainMenu();

    return MRAA_SUCCESS;
}
