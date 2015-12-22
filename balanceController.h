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

	float kp_dtheta, ki_dtheta, kd_dtheta;
	float e_dtheta_x, e_dtheta_y;
	float d_dtheta_x, d_dtheta_y;
	float int_dtheta_x, int_dtheta_y;


	float kp_pos, ki_pos, kd_pos;
	
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
					float _kp_dtheta, float _ki_dtheta, float _kd_dtheta,
					float _kp_pos, float _ki_pos, float _kd_pos){
		kp_theta = _kp_theta;
		ki_theta = _ki_theta;
		kd_theta = _kd_theta;

		kp_dtheta = _kp_dtheta;
		ki_dtheta = _ki_dtheta;
		kd_dtheta = _kd_dtheta;

		kp_pos = _kp_pos;
		ki_pos = _ki_pos;
		kd_pos = _kd_pos;
	}
	void setTipLimit(int _tip_limit){
		tip_limit = _tip_limit;
	}
	void setOffset(float _targ_theta_x, float _targ_theta_y,
				   float _targ_dtheta_x, float _targ_dtheta_y,
				   float _targ_pos_x, float _targ_pos_y){
		targ_theta_x = _targ_theta_x;
		targ_theta_y = _targ_theta_y;

		targ_dtheta_x = _targ_dtheta_x;
		targ_dtheta_y = _targ_dtheta_y;

		targ_pos_x = _targ_pos_x;
		targ_pos_y = _targ_pos_y;
	}

	bool update(){
		unsigned long dt = deltaT;
		deltaT = 0;

		*v_x = 0;
		*v_y = 0;

		if (balance){
			d_theta_x = (*p - p_prev)*1000000/dt;
			d_theta_y = (*r - r_prev)*1000000/dt;

			p_prev = *p;
			r_prev = *r;
			
			e_theta_x = targ_theta_x - *p;
			e_theta_y = targ_theta_y - *r;			

			int_theta_x += ki_theta * e_theta_x * dt / 1000000;
			int_theta_y += ki_theta * e_theta_y * dt / 1000000;



			// doesn't work well (at least with first try)
			// targ_theta_x -= int_theta_x * 0.0001;
			// targ_theta_y -= int_theta_y * 0.0001;

			d_dtheta_x = (d_theta_x - d_theta_x_prev)*1000000/dt;
			d_dtheta_y = (d_theta_y - d_theta_y_prev)*1000000/dt;

			d_theta_x_prev = d_theta_x;
			d_theta_y_prev = d_theta_y;

			e_dtheta_x = targ_dtheta_x - d_theta_x;
			e_dtheta_y = targ_dtheta_y - d_theta_y;

			int_dtheta_x += ki_dtheta * e_dtheta_x * dt / 1000000;
			int_dtheta_y += ki_dtheta * e_dtheta_y * dt / 1000000;

			// *v_x += kp_dtheta * e_dtheta_x + int_dtheta_x + kd_dtheta * d_dtheta_x;
			// *v_y += kp_dtheta * e_dtheta_y + int_dtheta_y + kd_dtheta * d_dtheta_y;
			// *v_x += 1;
			// *v_y += 1;

			*v_x = kp_theta * e_theta_x + int_theta_x + kd_theta * d_theta_x 
				  + kp_dtheta * e_dtheta_x + int_dtheta_x + kd_dtheta * d_dtheta_x;
			*v_y = kp_theta * e_theta_y + int_theta_y + kd_theta * d_theta_y 
			      + kp_dtheta * e_dtheta_y + int_dtheta_y + kd_dtheta * d_dtheta_y;

			// *v_x = kp_theta * e_theta_x + int_theta_x + kd_theta * d_theta_x 
			// 	  + kp_dtheta * e_dtheta_x;
			// *v_y = kp_theta * e_theta_y + int_theta_y + kd_theta * d_theta_y 
			//       + kp_dtheta * e_dtheta_y;
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

	int tip_limit = 150;

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

	/* dtheta var */
	float targ_dtheta_x, targ_dtheta_y;

	/* pos correction */
	bool pos_cor = true;
	bool flip_pos_cor = false;

	float e_pos_x, e_pos_y;
	float d_pos_x, d_pos_y;
	float int_pos_x, int_pos_y;
	float targ_pos_x, targ_pos_y;
};