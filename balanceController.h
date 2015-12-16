#define SIGN(a) (a < 0 ? -1 : 1)

#define SINDEG(a)(sin(a*PI/180))
#define COSDEG(a)(cos(a*PI/180))

class BalanceController{
public:
	float kp_theta, ki_theta, kd_theta;
	float e_theta_x, e_theta_y;
	float d_theta_x, d_theta_y;
	float int_theta_x, int_theta_y;
	
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
					float _kp_pos, float _ki_pos, float _kd_pos){
		kp_theta = _kp_theta;
		ki_theta = _ki_theta;
		kd_theta = _kd_theta;

		kp_pos = _kp_pos;
		ki_pos = _ki_pos;
		kd_pos = _kd_pos;
	}
	void setOffset(float _targ_theta_x, float _targ_theta_y,
				   float _targ_pos_x, float _targ_pos_y){
		targ_theta_x = _targ_theta_x;
		targ_theta_y = _targ_theta_y;

		targ_pos_x = _targ_pos_x;
		targ_pos_y = _targ_pos_y;
	}

	bool update(){
		unsigned long dt = deltaT;
		deltaT = 0;

		*v_x = 0;
		*v_y = 0;

		if (balance){
			d_theta_x = (*p - p_prev)/dt;
			d_theta_y = (*r - r_prev)/dt;

			p_prev = *p;
			r_prev = *r;

			if (balanceModeDerivative){
				// this mode means that the derivative is also counted into
				// error (as we ideally want the derivative to be 0 as well)
				e_theta_x = (targ_theta_x - *p) + 200*d_theta_x;  
				e_theta_y = (targ_theta_y - *r) + 200*d_theta_y;  
			}
			else{
				e_theta_x = targ_theta_x - *p;
				e_theta_y = targ_theta_y - *r;
			}

			int_theta_x += ki_theta * e_theta_x * dt;
			int_theta_y += ki_theta * e_theta_y * dt;

			*v_x += kp_theta * e_theta_x + int_theta_x + kd_theta * d_theta_x;
			*v_y += kp_theta * e_theta_y + int_theta_y + kd_theta * d_theta_y;
		}

		if (pos_cor){
			e_pos_x = targ_pos_x - *pos_x;
			e_pos_y = targ_pos_y - *pos_y;

			d_pos_x = (*pos_x - pos_x_prev)/dt;
			d_pos_y = (*pos_y - pos_y_prev)/dt;

			pos_x_prev = *pos_x;
			pos_y_prev = *pos_y;

			int_pos_x += ki_pos * e_pos_x * dt;
			int_pos_y += ki_pos * e_pos_y * dt;

			float v_x_abs = kp_pos * e_pos_x + int_pos_x + kd_pos * d_pos_x;
			float v_y_abs = kp_pos * e_pos_y + int_pos_y + kd_pos * d_pos_y;

			// note that v_x and v_y are relative to robot. Thus we need to
			// convert absolute coordinates
			/*
			[v_x] = [cos(alpha) -sin(alpha)][v_x_rel]
			[v_y]   [sin(alpha)  cos(alpha)][v_y_rel]
			*/
			float temp_x = COSDEG(-*theta) * v_x_abs - SINDEG(-*theta) * v_y_abs;
			float temp_y = SINDEG(-*theta) * v_x_abs + COSDEG(-*theta) * v_y_abs;

			if (abs(temp_x) > 50) temp_x = 50  * SIGN(temp_x);
			if (abs(temp_y) > 50) temp_y = 50  * SIGN(temp_y);

			if (flip_pos_cor){
				temp_x *= -1;
				temp_y *= -1;
			}

			*v_x += temp_x;
			*v_y += temp_y;
		}
		if (abs(*v_x) > 150 || abs(*v_y) > 150){
			if (abs(*v_x) > 150) *v_x = 150 * SIGN(*v_x);
			if (abs(*v_y) > 150) *v_y = 150 * SIGN(*v_y);
			return false;
		}

		return true;
	}
	void enablePosCorrection(){
		pos_cor = true;
	}
	void disablePosCorrection(){
		pos_cor = false;

		pos_x_prev = 0;
		pos_y_prev = 0;

		int_pos_x = 0;
		int_pos_y = 0;
	}
	void enableBalance(){
		balance = true;
	}
	void disableBalance(){
		balance = false;

		p_prev = 0;
		r_prev = 0;

		int_theta_x = 0;
		int_theta_y = 0;
	}
	void enablePosCorFlip(){
		flip_pos_cor = true;
	}
	void disablePosCorFlip(){
		flip_pos_cor = false;
	}
	void setBalanceMode(bool mode){
		balanceModeDerivative = mode;
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

	float *v_x, *v_y;

	float *r, *p;
	float r_prev, p_prev;

	float *theta;
	float *pos_x, *pos_y;
	float pos_x_prev, pos_y_prev;

	/* balancing var */
	bool balance = true;
	bool balanceModeDerivative = false;
	
	float targ_theta_x, targ_theta_y;

	/* pos correction */
	bool pos_cor = true;
	bool flip_pos_cor = false;

	float kp_pos, ki_pos, kd_pos;
	float e_pos_x, e_pos_y;
	float d_pos_x, d_pos_y;
	float int_pos_x, int_pos_y;
	float targ_pos_x, targ_pos_y;
};