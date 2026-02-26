#include "mbed.h"
#include "sensors_actuators.h"
#include "ControllerLoop.h"
#include "minicube_parametermap.h"
#include "choreography.h"
#include <iterator>
#include <vector>

#define PI 3.1415927

#define INIT 0
#define IDLE 99
#define FLAT 1
#define BALANCE 2
#define WIGGLE_SLOW 3
#define WIGGLE_FAST 4
#define WALK_LEFT 5
#define WALK_RIGHT 6
#define WALK_LEFT_FAST 7
#define WALK_RIGHT_FAST 8
#define FLAT_L 9
#define FLAT_R 10
#define DOWN 51
#define FREEFALL 70
#define MAX_BARS 81
#define FINISH 77

#define AY_LIMIT 2.0
#define AX_LIMIT 8.0


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,ControllerLoop *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    uint8_t C_SS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer lti;
    Timer gti;
    float Ts;
    void sendSignal();
    sensors_actuators *m_sa;
    ControllerLoop *m_loop;
    bool detect_on_edge();
    float phi_bd_des_target;
    uint8_t get_curr_bar(void);
    uint8_t curr_bar;
    bool use_choreo;
    float saturate(float,float,float);
};