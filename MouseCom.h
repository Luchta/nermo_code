#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <thread>
#include <atomic>


struct Ccmnd
{
    bool valid = false;
    int val1 = 0, val2 = 0, val3 = 0;
    int command = 7; //FIXIT for TypCmd
};


class CMousCtrlSet;


class CMouseCom
{
public:
    CMouseCom();
    virtual ~CMouseCom();

    typedef enum Commmands{ SetMotorPos, PosReached, GetSensorValue, SensorValue, SetLed, SetMotorOff, MPwrOff,   // commands spine
                            MoveLeg, StepLeg, StepDone,                                // commands rpi internal
                            InitMouse, Trott, StopAll} typCmd;                        // commands from shell
    std::atomic<Ccmnd> consoleCmnd;// for thread communication

	// setup values
	int Motors[13] = {00,01,02,03,10,11,12,13,20,21,22,23,24};
	int MotorP = 17;
	int MotorI = 0;
	int MotorD = 35;


    //Functions
    void startThread();
    void startUART();
    void setConsoleCcmnd(typCmd cmd, int val1=0, int val2=0, int val3=0);
    void ProcessSpine(typCmd cmd, int val1, int val2=0, int val3=0);


    //OLD COMMUNICATION
    void setMotorOFF(int id);
    void setMotorPwrOFF();
    void sendNL();

    void MotorPwrCycle();
    void setMotorPID();
    void setMotorSilent(int id, int val1);
    void MotorSetup();
protected:
    //virtual void ProcessSpine(typCmd cmd, int val1, int val2, int val3);
    virtual CMousCtrlSet& InitRPI(){}
    virtual void ReceiveMsg(typCmd cmd, int val1=0, int val2=0, int val3=0) {}

private:

    std::thread t1; //waiting loop thread
    // UART STREAMS
    int uart0_readstream = -1;
    int uart0_sendstream = -1;
    //FUNCTIONS
    void setup_uart_send(); //sets up uart for read and write
    void setup_uart_read(); //sets up uart for read and write
    void MouseInputLoop();

    //sending funtions return true on success
    bool sendMotor_Serial(int id, int pos, int speed);
    void setMotorLed(int id, int state); //0,1,2
    bool sendStreamRequest(int id, int frequency, int amount);
    bool sendSensorRequest(int id);

    //recieve returns amount of arguments, 0 for nothing to read, -1 for error
    int recieveData(); // give array with MAX_ARG_LENGTH
    void checkComndConsole();
    bool sendUartMessage(char buffer[], int l);
};



#endif // SERIAL_COM_H
