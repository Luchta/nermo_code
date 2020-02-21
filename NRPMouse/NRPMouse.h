#ifndef NRPMouse_h
#define NRPMouse_h

#include "HwInterface.h"
#include "Anatomy.h"


class NRPMouse
{

public:
    //get instance of only mouse instance
    static NRPMouse& Instance();

    void moveLeg(legservo servo, uint16_t angle, uint16_t duration = 1); //move leg servos
    void legStatus(leg ch, float *status); //access pressure sensor (ch 0,1) (currently not working)
    void squeak(int beeps, int speedms); //access speaker (currently not attached)

    //request direct hardware access DO NOT USE!
    //for testing only
    HwInterface* request_hw_accesss(){
        return _hw;
    }


protected:
    NRPMouse();
    //Copy-constructor: prohibit object copy (protected)
    NRPMouse(const NRPMouse& other) { }

private:
    HwInterface * _hw = NULL;

};

//simple object access
inline NRPMouse& getNrpMouseInstance() { return NRPMouse::Instance(); }

#endif
