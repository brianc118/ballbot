/* Library for my ballbot, found at http://onewaytobalance.blogspot.com
 * 
 * Open sourced under the MIT License (see LICENSE.txt)
 * (c) Brian Chen 2015
 *
 * Balancing method 1
 *     Target roll/pitch based on position
 *     PID loop on position calculates target roll/pitch
 *     Target roll/pitch passed as input into P controller
 *     for motor output.
 * 
 * Balancing method 2 (allows remote control)
 *     Target velocity based on position
 *     PD loop on position calculates target velocity
 *     Tarvet velocity passed as input into PID controller
 *     for target roll/pitch
 *     Target roll/pitch passed as input into P controller
 *     for motor output
 *
 */

#define SIGN(a) (a < 0 ? -1 : 1)

#define SINDEG(a)(sin(a*PI/180))
#define COSDEG(a)(cos(a*PI/180))

class BalanceController{
public:
    float kp_theta, ki_theta, kd_theta;
    float e_theta_x, e_theta_y;
    float d_theta_x, d_theta_y;
    float d_theta_x_prev, d_theta_y_prev;
    float int_theta_x, int_theta_y;

    float kp_v, ki_v, kd_v;
    float e_v_x, e_v_y;
    float d_v_x, d_v_y;
    float int_v_x, int_v_y;

    float kp_pos, ki_pos, kd_pos;
    float e_pos_x, e_pos_y;
    float d_pos_x, d_pos_y;
    float int_pos_x, int_pos_y;
    
    BalanceController(float *_v_x, float *_v_y, 
                      float *_pos_x, float *_pos_y, float *_theta, 
                      float *_r, float *_p){
        v_x = _v_x;
        v_y = _v_y;

        pos_x = _pos_x;
        pos_y = _pos_y;

        theta = _theta;

        r = _r; // roll
        p = _p; // pitch
    }
    void setTunings(float _kp_theta, float _ki_theta, float _kd_theta,
                    float _kp_v, float _ki_v, float _kd_v,
                    float _kp_pos, float _ki_pos, float _kd_pos){
        kp_theta = _kp_theta;
        ki_theta = _ki_theta;
        kd_theta = _kd_theta;

        kp_v = _kp_v;
        ki_v = _ki_v;
        kd_v = _kd_v;

        kp_pos = _kp_pos;
        ki_pos = _ki_pos;
        kd_pos = _kd_pos;
    }
    void setTipLimit(int _tip_limit){
        tip_limit = _tip_limit;
    }
    void setOffset(float _targ_theta_x, float _targ_theta_y,
                   float _targ_v_x, float _targ_v_y,
                   float _targ_pos_x, float _targ_pos_y){
        targ_theta_x = _targ_theta_x;
        targ_theta_y = _targ_theta_y;

        targ_v_x = _targ_v_x;
        targ_v_y = _targ_v_y;

        targ_pos_x = _targ_pos_x;
        targ_pos_y = _targ_pos_y;
    }

    void setTargPos(float _targ_pos_x, float _targ_pos_y){
        targ_pos_x = _targ_pos_x;
        targ_pos_y = _targ_pos_y;
    }

    bool update(){
        unsigned long dt = deltaT;
        deltaT = 0;

        *v_x = 0;
        *v_y = 0;

        if (pos_cor){
            e_pos_x = targ_pos_x - *pos_x;
            e_pos_y = targ_pos_y - *pos_y;

            d_pos_x = (*pos_x - pos_x_prev)*1000000/dt;
            d_pos_y = (*pos_y - pos_y_prev)*1000000/dt;

            pos_x_prev = *pos_x;
            pos_y_prev = *pos_y;

            int_pos_x += ki_pos * e_pos_x * dt / 1000000;
            int_pos_y += ki_pos * e_pos_y * dt / 1000000;

            // saturate integral term
            if (int_pos_x > 10) int_pos_x = 10 * SIGN(int_pos_x);
            if (int_pos_y > 10) int_pos_y = 10 * SIGN(int_pos_y);

            float v_x_abs = kp_pos * e_pos_x + int_pos_x - kd_pos * d_pos_x;
            float v_y_abs = kp_pos * e_pos_y + int_pos_y - kd_pos * d_pos_y;

            // note that v_x and v_y are relative to robot. Thus we need to
            // convert absolute coordinates
            /*
            [v_x] = [cos(alpha) -sin(alpha)][v_x_rel]
            [v_y]   [sin(alpha)  cos(alpha)][v_y_rel]
            */
            float temp_x = COSDEG(-*theta) * v_x_abs - SINDEG(-*theta) * v_y_abs;
            float temp_y = SINDEG(-*theta) * v_x_abs + COSDEG(-*theta) * v_y_abs;

            if (flip_pos_cor){
                temp_x *= -1;
                temp_y *= -1;
            }

            if (pos_cor_angular_mode){
                // assign target angle based on position
            	targ_theta_x = temp_x;
	            targ_theta_y = temp_y;

	            if (abs(targ_theta_x) > 15){
	                targ_theta_x = SIGN(targ_theta_x) * 15;
	            }
	            if (abs(targ_theta_y) > 15){
	                targ_theta_y = SIGN(targ_theta_y) * 15;
	            }
            }
            else{
                // assign target velocity based on position
            	if (abs(temp_x) > 255) temp_x = 255  * SIGN(temp_x);
            	if (abs(temp_y) > 255) temp_y = 255  * SIGN(temp_y);

                targ_v_x += temp_x;
            	targ_v_y += temp_y;
            }
        }

        if (balance){
            // first comes the velocity controller
            d_v_x = (d_theta_x - d_theta_x_prev)*1000000/dt;
            d_v_y = (d_theta_y - d_theta_y_prev)*1000000/dt;

            d_theta_x_prev = d_theta_x;
            d_theta_y_prev = d_theta_y;

            e_v_x = targ_v_x - d_theta_x;
            e_v_y = targ_v_y - d_theta_y;

            int_v_x += ki_v * e_v_x * dt / 1000000;
            int_v_y += ki_v * e_v_y * dt / 1000000;

            if (pos_cor_angular_mode){
                // do nothing
            }
            else{
                // assign target roll/pitch
                targ_theta_x = kp_v * e_v_x + int_v_x - kd_v * d_v_x;
                targ_theta_y = kp_v * e_v_y + int_v_y - kd_v * d_v_y;
            }       

            // then comes the roll/pitch controller
            d_theta_x = (*p - p_prev)*1000000/dt;
            d_theta_y = (*r - r_prev)*1000000/dt;

            p_prev = *p;
            r_prev = *r;
            
            e_theta_x = targ_theta_x - *p;
            e_theta_y = targ_theta_y - *r;            

            int_theta_x += ki_theta * e_theta_x * dt / 1000000;
            int_theta_y += ki_theta * e_theta_y * dt / 1000000;

            *v_x += kp_theta * e_theta_x + int_theta_x + kd_theta * d_theta_x;
            *v_y += kp_theta * e_theta_y + int_theta_y + kd_theta * d_theta_y;
        }

        if (abs(*v_x) > tip_limit || abs(*v_y) > tip_limit){
            if (abs(*v_x) > tip_limit) *v_x = tip_limit * SIGN(*v_x);
            if (abs(*v_y) > tip_limit) *v_y = tip_limit * SIGN(*v_y);
            return false;
        }

        return true;
    }
    void resetPosCorrection(){
        pos_x_prev = 0;
        pos_y_prev = 0;

        int_pos_x = 0;
        int_pos_y = 0;

        *v_x = 0;
        *v_y = 0;
    }
    void enablePosCorrection(){
        pos_cor = true;
        resetPosCorrection();
    }
    void disablePosCorrection(){
        pos_cor = false;
        resetPosCorrection();        
    }
    void resetBalance(){
        p_prev = 0;
        r_prev = 0;

        int_theta_x = 0;
        int_theta_y = 0;

        int_v_x = 0;
        int_v_y = 0;

        int_pos_x = 0;
        int_pos_y = 0;


        *v_x = 0;
        *v_y = 0;
    }
    void enableBalance(){
        balance = true;
        resetBalance();
    }
    void disableBalance(){
        balance = false;
        resetBalance();
    }
    void enablePosCorFlip(){
        flip_pos_cor = true;
    }
    void disablePosCorFlip(){
        flip_pos_cor = false;
    }
    void disable(){
        disableBalance();
        disablePosCorrection();
    }
    void enable(){
        enableBalance();
        enablePosCorrection();
    }
    bool balanceEnabled(){
        return balance;
    }
    bool posCorEnabled(){
        return pos_cor;
    }
private:
    bool enabled = true;
    elapsedMicros deltaT;

    int tip_limit = 150;

    float *v_x, *v_y;

    float *r, *p;
    float r_prev, p_prev;

    float *theta;
    float *pos_x, *pos_y;
    float pos_x_prev, pos_y_prev;

    /* balancing var */
    bool balance = true;
    
    float targ_theta_x, targ_theta_y;

    /* dtheta var */
    float targ_v_x, targ_v_y;

    /* pos correction */
    bool pos_cor = true;
    bool pos_cor_angular_mode = true;
    bool flip_pos_cor = false;

    
    float targ_pos_x, targ_pos_y;
};