#include "MouseCom.h"

#include <unistd.h>	//UART and usleep
#include <fcntl.h>	//UART
#include <termios.h>//UART
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <stdio.h> //strtok
#include <iostream>

//Parsing defines for command lengths
#define MAX_RECIEVE_LENGTH 255
#define MAX_ARG_LENGTH 50
#define MAX_ARG_PER_LINE 10

//Debugging Mode
#define DEBUG false

//PUBLIC/////////////////////////////////////////////////////////////////////////////////////////////

CMouseCom::CMouseCom()
{
    if(DEBUG){std::cout << "Mouse Com in Debug Mode\n";}
    else {std::cout << "Mouse Com in Standard Mode\n";}
    //InitRPI();
}

CMouseCom::~CMouseCom()
{
    //----- CLOSE THE UART -----
    close(uart0_readstream);
    close(uart0_sendstream);
}

//starting the loop thread
void CMouseCom::startThread() {
    std::cout << "starting UART and MOUSE thread"<<std::endl;
    t1 = std::thread ([=] { MouseInputLoop(); });
    t1.detach();
    std::cout << "Thread detached"<<std::endl;
}

void CMouseCom::startUART()
{
    if(DEBUG){std::cout << "Setting up UART Interface\n";}
    setup_uart_read();
    usleep(100000);
    setup_uart_send();
    usleep(100000);
    sendNL();//flush the buffer, all new
    usleep(10000);
    MotorSetup();
}

void CMouseCom::setConsoleCcmnd(typCmd cmd, int val1, int val2, int val3){
    Ccmnd tmp;

    if (DEBUG){std::cout << "storing new console cmd!\n";}
    tmp.command = cmd;
    tmp.val1 = val1;
    tmp.val2 = val2;
    tmp.val3 = val3;
    tmp.valid = true;

    //consoleCmnd.exchange(tmp);
    //consoleCmnd = cmd;
    consoleCmnd.store(tmp);
}

//PROTECTED/////////////////////////////////////////////////////////////////////////////////////////////

//UART-Schnittstelle zu CRPI
void CMouseCom::ProcessSpine(CMouseCom::typCmd cmd, int val1, int val2, int val3)
{
    switch (cmd)
    {
    case SetMotorPos:
        CMouseCom::sendMotor_Serial(val1, val2, val3); //takes ID, Pos, Duration
        break;
    case GetSensorValue:
        CMouseCom::sendSensorRequest(val1); //only takes ID
        break;
    case SetLed:
        CMouseCom::setMotorLed(val1, val2);
        break;
    case SetMotorOff:
        setMotorOFF(val1);
        break;
    case MPwrOff:
        setMotorPwrOFF();
        break;
    default:
        //NOOOOOOO!
        std::cerr << "Error - Unknwon UART Send Request!" << std::endl;
        break;
    }
}

//PRIVATE/////////////////////////////////////////////////////////////////////////////////////////////
//Motor Setup---------------------------------------------------------------------------------------
void CMouseCom::MotorSetup()
{
    int l;
    char buffer [18];

	for (int i=0;i<13;i++) {
		//Set Motors Silent
		l = sprintf (buffer, ":%02d!U-\n", Motors[i]);
		sendUartMessage(buffer,l);
		usleep(10000); //safety delay to avoid collisions
	}
	//setMotorPID();
}


//Sending-------------------------------------------------------------------------------------------------

bool CMouseCom::sendMotor_Serial(int id, int pos, int speed)
{
    //------ Calculate adequate centred Value ------
    //FIXME!!
    //int CentreOffset = 1800;
    //pos = pos + CentreOffset;
    //------ form string ------
    int l;
    char buffer [18];
    //string: "A ID POS SPEED" Pos and Speed are 4digit hex numbers
    //l = sprintf (buffer, "A %02d %04x %04x\n", id, pos, speed);

    //old command structure:
    l = sprintf (buffer, ":%02d!P=%04x\n", id, pos);


    if (DEBUG){
        std::cout.write(&buffer[0], l);
        std::cout << "\n";
    }
    //send and return
    return sendUartMessage(buffer, l);

}

void CMouseCom::setMotorLed(int id, int state)
{
    //------ form string------
    int l;
    char buffer [18];
    //string: ":ID!L=STATE"
    if (state == 1) {
        //switch on
        l = sprintf (buffer, ":%02d!L+\n", id);
    }else if (state == 0){
        //set off
        l = sprintf (buffer, ":%02d!L-\n", id);
    }else {
        //set frequency
        l = sprintf (buffer, ":%02d!L=%04x\n", id, state);
    }
    //send
    sendUartMessage(buffer,l);
}

void CMouseCom::setMotorOFF(int id)
{
    //------ form string------
    int l;
    char buffer [18];
    //string: ":ID!L=STATE"
    l = sprintf (buffer, ":%02d!P=OFF\n", id);
    //send
    sendUartMessage(buffer,l);
}

void CMouseCom::sendNL()
{
    //------ form string------
    int l;
    char buffer [18];
    //string: ":ID!L=STATE"
    l = sprintf (buffer, "\n");
    //send
    sendUartMessage(buffer,l);
}

void CMouseCom::setMotorSilent(int id, int val1)
{
    //------ form string------
    int l=0;
    char buffer [18];

    if (val1 == 1) {
        //switch on
        l = sprintf (buffer, ":%02d!U+\n", id);
    }else if (val1 == 0){
        //switch off
        l = sprintf (buffer, ":%02d!U-\n", id);
    }
    //send
    sendUartMessage(buffer,l);
}

void CMouseCom::setMotorPID()
{
    int l;
    char buffer [18];

	for (int i=0;i<13;i++) {
		//Set PID Values
		// P
		l = sprintf (buffer, ":%02d!CP=%03x0\n", Motors[i], MotorP);
		sendUartMessage(buffer,l);
		usleep(5000); //safety delay to avoid collisions
		// I
		l = sprintf (buffer, ":%02d!CI=%03x0\n", Motors[i], MotorI);
		sendUartMessage(buffer,l);
		usleep(5000); //safety delay to avoid collisions
		// D
		l = sprintf (buffer, ":%02d!CD=%03x0\n", Motors[i], MotorD);
		sendUartMessage(buffer,l);
		usleep(5000); //safety delay to avoid collisions
	}
}


void CMouseCom::setMotorPwrOFF()
{
    //------ form string------
    int l;
    char buffer [18];
    //string: ":ID!L=STATE"
    l = sprintf (buffer, "!PS-\n");
    //send
    sendUartMessage(buffer,l);
}

void CMouseCom::MotorPwrCycle()
{
    //------ form string------
    int l;
    char buffer [18];
    //string: ":ID!L=STATE"
    l = sprintf (buffer, "!PS-\n");
    //send
    sendUartMessage(buffer,l);
    usleep(500000);
    l = sprintf (buffer, "!PS+\n");
    //send
    sendUartMessage(buffer,l);
}

bool CMouseCom::sendStreamRequest(int id, int frequency, int amount)
{
    std::cerr << "Error - sendStreamRequest not implemented!" << std::endl;
    return false;
}

bool CMouseCom::sendSensorRequest(int id)
{
    //------ form String ------
    int l;
    char buffer [18];
    //String: "S ID"
    l = sprintf (buffer, "S %02d\n", id);
    //------ Send and Return ------
    return sendUartMessage(buffer, l);
}

//transmit function
bool CMouseCom::sendUartMessage(char buffer[], int l){
    //----- TX BYTES -----

    if (uart0_sendstream != -1)
    {
        int count = write(uart0_sendstream, &buffer[0], l);		//Filestream, bytes to write, number of bytes to write
        //int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0])); //Filestream, bytes to write, number of bytes to write
        //FIXME!
        //usleep(10000); //safety delay to avoid collisions
        if (count < 0)
        {
            //printf("UART TX error\n");
            std::cerr << "Error - UART TX ERROR" << std::endl;
            return false;
        }
    }
    else {
        //printf("Error: UART not opened\n");
        std::cerr << "Error - UART Sendstream not opened!" << std::endl;
        return false;
    }
    return true;
}


//Recieving-------------------------------------------------------------------------------------------------

//Waiting loop
void CMouseCom::MouseInputLoop()
{
    bool OK = true; //for later Break criteria

    usleep(600000);

    while(OK)
    {

        //handle UART input
        recieveData();

        sleep(10); //waiting for data
    }
}

//Console recieve function
void CMouseCom::checkComndConsole()
{
    //check for Console Commands
    //load atomic struct
    if (DEBUG){std::cout << "loading Console Cmd\n";}
    Ccmnd tmp = consoleCmnd.load();
    //check for new commands
    if(tmp.valid)
    {
        if (DEBUG){std::cout << "new Console Cmd read: " << (typCmd)tmp.command << ";" << tmp.val1 << ";" << tmp.val2 << ";" << tmp.val3 <<"\n";}
        //set false and store
        tmp.valid = false;
        consoleCmnd.store(tmp);

        //send Message
        ReceiveMsg((typCmd)tmp.command, tmp.val1, tmp.val2, tmp.val3);
    }
}

//UART recieve function
int CMouseCom::recieveData()
{
    //parsing variables
    char const separator = ' ';
    char newArg[MAX_ARG_PER_LINE][MAX_ARG_LENGTH];
    int arguments[MAX_ARG_PER_LINE] = {0};
    int i = 0;
    int count = 0;
    int rx_length = -1;
    // Read up to 255 characters from the port if they are there
    char rx_buffer[256];
    //int numArgs = 0;

    //----- CHECK FOR ANY RX BYTES -----
    if (uart0_readstream != -1)
    {
        // Read input while new data available
        while ((rx_length = read(uart0_readstream, (void*)rx_buffer, 255)) != 0)
        {
            //rx_length = read(uart0_readstream, (void*)rx_buffer, MAX_RECIEVE_LENGTH);		//Filestream, buffer to store in, number of bytes to read (max)
            if (rx_length < 0)
            {
                //An error occured (will occur if there are no bytes)
                std::cerr << "Error: UART Read returned no bytes!" << std::endl;
                return -1;
            }
            else if (rx_length == 0)
            {
                //No data waiting
                checkComndConsole(); //see if anything came through the console
                return 0;

            }
            else
            {
                //Bytes received
                rx_buffer[rx_length] = '\0';
                //DEBUGGIN-Comment OUT!
                if (DEBUG){printf("RX - %i bytes read : %s\n", rx_length, rx_buffer);}
                

                //parse to arguments
                //reset i and count
                i = 0;
                count = 0;
                //seperate string
                char *token = strtok(rx_buffer, &separator);
                while ((i < MAX_ARG_PER_LINE) && (token))
                {
                    strcpy(newArg[i++], token);
                    token = strtok(nullptr, &separator);
                    count++;
                }
                //convert to int and store
                if (count == 0 || count < 0)
                {
                    //no Arguments read || Error - should never be reached
                    std::cerr << "This should not have happend, empty RX count!\n";
                }else {
                    arguments[0] = (int)newArg[0][0]; //Msg Type is char
                    arguments[1] = atoi(newArg[1]); //ID is int
                    for (i=2;i<count;i++) {
                        //remaining arguments are (4 digit) hex values
                        arguments[i] = (int)strtol (newArg[i],nullptr,16); //read hex string to int
                    }
                }
                //DEBUG output converted inputs
                if (DEBUG){
                    printf("RX - Recieved %d Arguments:",count);
                    for (int k = 0; k < count; ++k) {
                        printf(" val%d: %d",k, arguments[k]);
                    }
                    printf(" - RX: Msg %c, Id: %d\n", arguments[0], arguments[1]);
                }
                //check for Msg Type

                switch (arguments[0]) {
                case 'A':
                    //Motor reached pos: IDs chain 1-4 ;servos 0-4
                    if (arguments[2] == 1){ std::cerr << "Motor Error State!\n";}
                    ReceiveMsg(PosReached, arguments[1], arguments[2]);
                    break;
                case 'S':
                    //Sensor: IDs chain 1-4; sensor 1
                case 'P':
                    //Motor Position: IDs chain 1-4; sensor 1-5
                    ReceiveMsg(SensorValue, arguments[1], arguments[2]);
                    break;
                case 'E':
                    //EVENT: IDs chain 6; sensors 1-8
                    break;
                default:
                    std::cerr << "unknwon msg type recieved\n";
                    printf("RX - %i bytes read : %s\n", rx_length, rx_buffer);
                    break;

                }

                //return 1;
            }

            checkComndConsole(); //see if anything came through the console
            //clear array
            for (i=0;i<count;i++) {
                arguments[i] = 0;
            }

        }
        return 0;
    } else {
        std::cerr << "Error: UART Readstream not opened!" << std::endl;
    }
    checkComndConsole(); //see if anything came through the console

    return -1;
}

//Setup-------------------------------------------------------------------------------------------------

void CMouseCom::setup_uart_send()
{
    //This code was taken from https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart

    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively


    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    uart0_sendstream = open("/dev/ttyAMA0", O_WRONLY | O_NOCTTY);		//Open in non blocking read/write mode
    if (uart0_sendstream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        std::cerr << "Error - Unable to open UART for Transmission!" << std::endl;
        //printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_sendstream, &options);
    options.c_cflag = B1000000 | CS8 | CLOCAL | CREAD | CRTSCTS;;		//<Set baud rate
    options.c_iflag = IGNPAR | ICRNL;
    options.c_oflag = 0;
    // options.c_lflag = 0;
    options.c_lflag = ICANON;
    tcflush(uart0_sendstream, TCIFLUSH);
    tcsetattr(uart0_sendstream, TCSANOW, &options);


}

void CMouseCom::setup_uart_read()
{
    //This code was taken from https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart

    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively


    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    //uart0_readstream = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    uart0_readstream = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY);		//Open in blocking read mode

    if (uart0_readstream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        std::cerr << "Error - Unable to open UART for Recieving!" << std::endl;
        //printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_readstream, &options);
    options.c_cflag = B1000000 | CS8 | CLOCAL | CREAD | CRTSCTS;;		//<Set baud rate
    options.c_iflag = IGNPAR | ICRNL; //ICRNL needed for putty and minicom else RPI will not recieve \n, do not know why!!!!
    options.c_oflag = 0;
    //options.c_lflag = 0;
    options.c_lflag = ICANON;
    tcflush(uart0_readstream, TCIFLUSH);
    tcsetattr(uart0_readstream, TCSANOW, &options);


}

