
#include "Servo.h"

//**********************************************************************************************
//public functions
//**********************************************************************************************

Servo::Servo(HwInterface *hw)
{
    //set default values for speed and position
    hindleft_hip[POSITION].store(DEFAULT_HINDLEFT_HIP);
    hindleft_hip[DURATION].store(MINDURATION);
    hindleft_foot[POSITION].store(DEFAULT_HINDLEFT_FOOT);
    hindleft_foot[DURATION].store(MINDURATION);
    foreleft_hip[POSITION].store(DEFAULT_FORELEFT_HIP);
    foreleft_hip[DURATION].store(MINDURATION);
    foreleft_foot[POSITION].store(DEFAULT_FORELEFT_FOOT);
    foreleft_foot[DURATION].store(MINDURATION);
    foreright_foot[POSITION].store(DEFAULT_FORERIGHT_FOOT);
    foreright_foot[DURATION].store(MINDURATION);
    foreright_hip[POSITION].store(DEFAULT_FORERIGHT_HIP);
    foreright_hip[DURATION].store(MINDURATION);
    hindright_foot[POSITION].store(DEFAULT_HINDRIGHT_FOOT);
    hindright_foot[DURATION].store(MINDURATION);
    hindright_hip[POSITION].store(DEFAULT_HINDRIGHT_HIP);
    hindright_hip[DURATION].store(MINDURATION);

    //start 8 servos
    Servo::startServo(hindleft_hip, hw, HIND_LEFT_HIP, true);
    std::cout << "Servo "<<HIND_LEFT_HIP<<" started"<<std::endl;
    Servo::startServo(hindleft_foot, hw, HIND_LEFT_FOOT, false);
    std::cout << "Servo "<<HIND_LEFT_FOOT<<" started"<<std::endl;
    Servo::startServo(hindright_hip, hw, HIND_RIGHT_HIP, false);
    std::cout << "Servo "<<HIND_RIGHT_HIP<<" started"<<std::endl;
    Servo::startServo(hindright_foot, hw, HIND_RIGHT_FOOT, true);
    std::cout << "Servo "<<HIND_RIGHT_FOOT<<" started"<<std::endl;
    Servo::startServo(foreleft_hip, hw, FORE_LEFT_HIP, false);
    std::cout << "Servo "<<FORE_LEFT_HIP<<" started"<<std::endl;
    Servo::startServo(foreleft_foot, hw, FORE_LEFT_FOOT, true);
    std::cout << "Servo "<<FORE_LEFT_FOOT<<" started"<<std::endl;
    Servo::startServo(foreright_hip, hw, FORE_RIGHT_HIP, true);
    std::cout << "Servo "<<FORE_RIGHT_HIP<<" started"<<std::endl;
    Servo::startServo(foreright_foot, hw, FORE_RIGHT_FOOT, false);
    std::cout << "Servo "<<FORE_RIGHT_FOOT<<" started"<<std::endl;

}

Servo::~Servo()
{
    //send closing signal to all servo threads
    hindleft_hip[POSITION].store(CLOSETHREAD);
    hindleft_foot[POSITION].store(CLOSETHREAD);
    foreleft_hip[POSITION].store(CLOSETHREAD);
    foreleft_foot[POSITION].store(CLOSETHREAD);
    foreright_foot[POSITION].store(CLOSETHREAD);
    foreright_hip[POSITION].store(CLOSETHREAD);
    hindright_foot[POSITION].store(CLOSETHREAD);
    hindright_hip[POSITION].store(CLOSETHREAD);
}




void Servo::moveLeg(legservo servo, uint16_t pos, uint16_t duration)
{
    //speed value guard, position guard is done within hardware
    if((duration > 0) && (duration <= MAXDURATION))
    {
        switch (servo) {
        case HIND_LEFT_HIP:
            hindleft_hip[POSITION].store(pos);
            hindleft_hip[DURATION].store(duration);
            break;
        case HIND_LEFT_FOOT:
            hindleft_foot[POSITION].store(pos);
            hindleft_foot[DURATION].store(duration);
            break;
        case FORE_LEFT_HIP:
            foreleft_hip[POSITION].store(pos);
            foreleft_hip[DURATION].store(duration);
            break;
        case FORE_LEFT_FOOT:
            foreleft_foot[POSITION].store(pos);
            foreleft_foot[DURATION].store(duration);
            break;
        case FORE_RIGHT_FOOT:
            foreright_foot[POSITION].store(pos);
            foreright_foot[DURATION].store(duration);
            break;
        case FORE_RIGHT_HIP:
            foreright_hip[POSITION].store(pos);
            foreright_hip[DURATION].store(duration);
            break;
        case HIND_RIGHT_FOOT:
            hindright_foot[POSITION].store(pos);
            hindright_foot[DURATION].store(duration);
            break;
        case HIND_RIGHT_HIP:
            hindright_hip[POSITION].store(pos);
            hindright_hip[DURATION].store(duration);
            break;

        default:
            std::cerr<<"servo not known in servo::moveLeg"<<std::endl;
            break;
        }
    }
    else
        std::cerr<<"speed out of range in servo::moveLeg"<<std::endl;

    return;
}

//**********************************************************************************************
//private functions
//**********************************************************************************************

void Servo::move(std::atomic<int> *ctrl, HwInterface *hw, legservo pin, bool _inverted)
{
    /*It is to note, that the speed of the current servos is 0,1s per 60° in an optimal world
     * meaning 100ms per 60degree movement ~ 1,7 ms/deg
     * difference of angle is in degree
     *
     * currently in this function the speed of 1step is fixed on 1ms no matter the actual step length
     *
     * an update would take the servo speed into account and use it in this calculation
     *
     *
     * time needed ms = difference° * speed ms/°
     *
     *
     *
     */


    //init all variables for thread

    std::cout << "Servo "<<pin<<" started"<<std::endl;

    bool active = true; //thread termination bool
    bool done;  //bool for stepping closure
    //bool _inverted = false;

    //int _offset = 0;
    int _angle; //current angle position of servo
    double _step_angle; //angle per timestep
    double _cur_angle;  //current angle in stepping procedure
    int _step;  //number of current step
    int _steps; //steps needed till goal


    //init current servo position
    int _to_angle = ctrl[POSITION];
    int _speed = ctrl[DURATION];
    //write init positions to servos
    hw->setServoAngle(pin,static_cast<uint16_t>(_to_angle));
    //set current angle
    _angle = _to_angle;

    //start thread loop
    do
    {
        //read atomic ctrl variables
        _to_angle = ctrl[POSITION];
        _speed = ctrl[DURATION];

        //check for end signal
        if(_to_angle == CLOSETHREAD)
        {
            active = false;
            continue;
        }

        // calculate the absolute value of the angle difference
        int angle_diff = _to_angle - _angle;
        if (angle_diff < 0)
            angle_diff *= -1;

        //if no difference -> no movement required
        if (angle_diff == 0)
        {
            _step = 0;
            _steps = 0;
            usleep(10000); //sleep in busy loop
            continue;
        }


        // number of degrees to move with each 1ms step
        //here it is assumed, that every step takes 1ms no matter the length,
        //as there is a 1ms delay between position updates.
        _step_angle = (double)angle_diff / _speed;

        _cur_angle = _angle;

        // stepping
        _step = 0;
        _steps = angle_diff / _step_angle;

        // 180->0 = count downwards
        if (_to_angle < _angle)
            _step_angle *= -1;


        //stepping to position within calculated time

        do {
            done = true;

            if (_step < _steps)
            {
                _cur_angle += _step_angle;
                 //write current step to hardware
                //offset and inverted
                //if (_inverted) _cur_angle = 270 - _cur_angle; //270 = default max angle of hw_edison
                //_cur_angle += (_inverted ? -_offset : _offset);
                //offset and inverted end
                hw->setServoAngle(pin,static_cast<uint16_t>(_cur_angle));
                _angle = _cur_angle;
                _step++;
            }

            done &= (_step >= _steps);
            usleep(10000);//1ms steps
        }
        while(!done);

    }while(active);

    std::cout<<"servo: "<<pin<<" stopped"<<std::endl;

}







