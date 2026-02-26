#include "state_machine.h"
using namespace std;
/* 
state_machine.cpp defines the individual sequences of cuboids
currently the states "walk_LEFT_FAST/RIGHT" are not used.
the speed of the state change is based on the 126BPM music tune Medina by "no Jazz"
*/

// contructor for state_machine
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096*2)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    use_choreo = false;
    gti.reset();
    gti.start();
    lti.start();
    lti.reset();
    curr_bar = 0;
    m_loop->phi_bd_des = phi_bd_des_target = -PI/4;
}

state_machine::~state_machine() {}
// ----------------------------------------------------------------------------
void state_machine::loop(void){
    float down_speed = 1.1;
    float bar2sec = 60.0f/126.0f*4.0f;           // seconds per bar
    float sec2bar = 1.0f/bar2sec;
    float dphi = 0;
    float om_slow = 2*PI*sec2bar * 2;
    float om_fast = 2*PI*sec2bar * 4;
    float t_offset = 0.0;
    float del_t = 0.0;
    float max_ax=0;
    float max_ay=0;
    float max_v=0;
    uint8_t old_state;
    uint8_t old_bar;
    float time_for_half_mode = bar2sec;
   // std::vector<int>::iterator itrtr = CHOREOGRAPHY.begin();
    //volatile int choreoState = *itrtr;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        float gt = gti.read()-.05;
        float lt = lti.read();
        if(use_choreo)
            {
            old_state = CS;
            old_bar = curr_bar;
            curr_bar = floor(gt*sec2bar)+2; // use one bar offset
            if(curr_bar>MAX_BARS)
                curr_bar = MAX_BARS;
            CS = CHOREOGRAPHY[curr_bar-1];
            if(fabs(m_sa->get_vphi_fw())>max_v)
                max_v = fabs(m_sa->get_vphi_fw());
            if(fabs(m_sa->ay_fil)>max_ay)
                max_ay = fabs(m_sa->ay_fil);
            }
        if(old_state != CS && use_choreo)
            {
            //printf("old: %d, CS: %d\r\n",old_state,CS);
            switch(CS)
                {
                case FLAT_L:
                case FLAT_R:
                case FLAT:
                    if(detect_on_edge())
                        {
                        if(CS == FLAT_R)
                            {
                            dphi = -.78/.6;    
                            phi_bd_des_target -= PI/4;
                            }
                        else
                            {
                            dphi = .78/.6;
                            phi_bd_des_target += PI/4;
                            }
                        C_SS = DOWN;
                        lti.reset();
                        lt = 0;
                        }
                    else {
                        C_SS = FLAT;
                        m_loop->enable_vel_cntrl();
                        }
                    break;
                case WIGGLE_FAST:
                case WIGGLE_SLOW:
                case BALANCE:
                    if(!detect_on_edge())
                        {
                        if(old_state == WALK_LEFT || old_state == WALK_LEFT_FAST)
                            phi_bd_des_target += PI/4;
                        else if(old_state == WALK_RIGHT || old_state == WALK_RIGHT_FAST)
                            phi_bd_des_target -= PI/4;
                        else
                            phi_bd_des_target += PI/4;;
                        m_loop->phi_bd_des = phi_bd_des_target;
                        }   
                    m_loop->enable_bal_cntrl();
                    break;
                case WALK_LEFT:
                case WALK_RIGHT:
                case WALK_LEFT_FAST:
                case WALK_RIGHT_FAST:
                    if(detect_on_edge())
                        {
                        lti.reset();
                        lt = 0;
                        del_t = 0.0;
                        C_SS = DOWN;
                        printf("New: go DOWN!\r\n");
                        t_offset = 0;
                        if(CS == WALK_LEFT || CS ==WALK_LEFT_FAST)
                            {
                            phi_bd_des_target += PI/4;
                            dphi = .78/.6;
                            }
                        else
                            {
                            phi_bd_des_target -= PI/4;
                            dphi = -.78/.6;
                            }                           
                        printf("started on edge\r\n");
                        }
                    else
                        {
                        printf("start flat\r\n");
                        lti.reset();
                        lt = 0;
                        t_offset = -bar2sec;
                        C_SS = FLAT;
                        }
                    if(CS == WALK_LEFT || CS == WALK_RIGHT)
                        time_for_half_mode = bar2sec;
                    else
                        time_for_half_mode = 0.5f*bar2sec;  
                    break;
                }
            }
            if(old_bar != curr_bar)
            {
            printf("lti: %f bar: %d CS: %d, C_SS: %d, bd_des: %f, bd_act: %f, des v: %f\r\n",lt,curr_bar,CS,C_SS,m_loop->phi_bd_des,m_sa->get_phi_bd() ,max_v);
            //printf("max: %f, may: %f\r\n",max_ax,max_ay);
            max_ax = max_ay = 0;
            }
        switch(CS)
            {
            case INIT:
                m_sa->disable_escon();
                m_sa->force_curr(0);
                if(gt>2)
                    CS = IDLE;
                break;
            case IDLE:
                if(m_sa->get_but())//ax_fil > AX_LIMIT)
                    {
                    gti.reset();
                    m_sa->force_curr(0);
                    m_sa->enable_escon();
                    printf("STARTED\r\n");
                    use_choreo = true;
                    }
                break;
            case FLAT_L:
            case FLAT_R:
            case FLAT:
                switch(C_SS)
                    {
                    case DOWN:
                        m_loop->phi_bd_des += dphi * Ts;
                        if (lt > 0.8)
                            {
                            C_SS = FREEFALL;
                            m_loop->phi_bd_des = phi_bd_des_target;
                            m_loop->disable_all_cntrl();
                            }
                        break;
                    case FREEFALL:
                        if (lt > 1)
                            {
                            C_SS = FLAT;
                            m_loop->enable_vel_cntrl();
                            }
                        break;
                    case FLAT:
                        break;
                    }
                break;
            case WALK_LEFT:
            case WALK_RIGHT:
                switch(C_SS)
                    {
                    case BALANCE:
                        if (lt-t_offset >= 2*time_for_half_mode)
                            {
                            printf("lti: %f BAL --> DOWN! des: %f\r\n",lt,m_loop->phi_bd_des);
                            C_SS = DOWN;
                            t_offset = lt;
                            if(CS == WALK_LEFT)
                                {
                                phi_bd_des_target += PI/4;
                                dphi = .78/.6;
                                }
                            else
                                {
                                phi_bd_des_target -= PI/4;
                                dphi = -.78/.6;
                                }
                            }
                        break;
                    case DOWN:
                        m_loop->phi_bd_des += dphi * Ts;
                        if (lt - t_offset > 0.6)
                            {
                            printf("lti: %f DOWN --> FREEFALL, des: %f\r\n",lt,m_loop->phi_bd_des);
                            C_SS = FREEFALL;
                            m_loop->phi_bd_des = phi_bd_des_target;
                            m_loop->disable_all_cntrl();
                            }
                        break;
                    case FREEFALL:
                        if (lt -t_offset > .9)
                            {
                            C_SS = FLAT;
                            printf("lti: %f FREEFALL --> FLAT! des: %f\r\n",lt,m_loop->phi_bd_des);
                            m_loop->enable_vel_cntrl();
                            }
                        break;
                    case FLAT:
                        if(lt-t_offset >=  time_for_half_mode)
                            {
                                C_SS = BALANCE;
                                printf("lti: %f FLAT --> BAL! des: %f\r\n",lt,m_loop->phi_bd_des);
                                m_loop->enable_bal_cntrl();
                                if(CS == WALK_LEFT || CS == WALK_LEFT_FAST)
                                    phi_bd_des_target += PI/4;
                                else
                                    phi_bd_des_target -= PI/4;
                                m_loop->phi_bd_des = phi_bd_des_target;        
                                printf("FLAT --> BAL! bd_des: %f\r\n",phi_bd_des_target);
                                lti.reset();
                                lt = 0;
                                t_offset = 0;
                            }
                        break;
                    }
                break;
            case WALK_LEFT_FAST:
            case WALK_RIGHT_FAST:
                switch(C_SS)
                    {
                    case BALANCE:
                        if (lt >= time_for_half_mode)
                            {
                            lti.reset();
                            lt = 0;
                            C_SS = DOWN;
                            if(CS == WALK_LEFT_FAST)
                                {
                                phi_bd_des_target += PI/4;
                                m_loop->disable_all_cntrl();
                                m_sa->force_curr(-6);
                                }
                            else
                                {
                                phi_bd_des_target -= PI/4;
                                m_loop->disable_all_cntrl();
                                m_sa->force_curr(6);
                                }
                            }
                        break;
                    case DOWN:
                        if (lt > 0.05)
                            {
                            C_SS = FREEFALL;
                            m_sa->force_curr(0);
                            }
                        break;
                    case FREEFALL:
                        if (fabs(m_sa->get_phi_bd()-phi_bd_des_target)<.05)
                            {
                            printf("switch to BALANCE\r\n");
                            C_SS = BALANCE;
                            m_loop->enable_bal_cntrl();
                            if(CS == WALK_LEFT || CS == WALK_LEFT_FAST)
                                phi_bd_des_target += PI/4;
                            else
                                phi_bd_des_target -= PI/4;
                            m_loop->phi_bd_des = phi_bd_des_target;        
                            }
                        break;
                    }
                break;
            case WIGGLE_SLOW:
                m_loop->phi_bd_des = phi_bd_des_target + saturate(lt,0,1)*0.11*sin(om_slow*(gt));
                break;
            case WIGGLE_FAST:
                m_loop->phi_bd_des = phi_bd_des_target + 0.04*sin(om_fast*gt);
                break;
            case FINISH:
                m_sa->disable_escon();
                m_loop->disable_all_cntrl();
            break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}
bool state_machine::detect_on_edge(void)
{
    int ne = round(m_sa->get_phi_bd()/1.570796327f);
    float re = fabs(m_sa->get_phi_bd()-(float)ne*1.570796327f);
    if(re <.25)
        {
        printf("re = %f, edge detected\r\n",re);
        return true;
        }
    else
        {
        printf("re = %f, flat detected\r\n",re);
        return false;
        }
}
float state_machine::saturate(float u, float uMin, float uMax)
{
    if(u > uMax) {
        u = uMax;
    } else if(u < uMin) {
        u = uMin;
    }
    return u;
}
/* for printout:
 case IDLE:
                if(fabs(m_sa->ax_fil) >max_ax)
                    max_ax = fabs(m_sa->ax_fil);
                if(fabs(m_sa->ay_fil) >max_ay)
                    max_ay = fabs(m_sa->ay_fil);
                if(m_sa->ay_fil > AY_LIMIT || gti.read()>1)
                    {
                    printf("max: %f, may %f\r\n",max_ax,max_ay);
                    max_ax = max_ay = 0;
                    if(gti.read()>1)
                        gti.reset();
                    //gti.reset();
                    //m_sa->force_curr(0);
                    //m_sa->enable_escon();
                    //printf("STARTED\r\n");
                    //use_choreo = true;
                    }
*/