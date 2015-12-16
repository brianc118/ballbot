

/*

Solve linear system for [v_x_rel  v_y_rel  R_w]^T

[v1]   [ -sin(theta_a)  cos(theta_a)  1] [v_x_rel]
[v2] = [ -sin(theta_b)  cos(theta_b)  1] [v_y_rel]
[v3]   [ -sin(theta_c)  cos(theta_c)  1] [R_w    ]

[v1]   [ -s_a  c_a  1] [v_x_rel]
[v2] = [ -s_c  c_b  1] [v_y_rel]
[v3]   [ -s_b  c_c  1] [R_w    ]

[v_x_rel]   [ -s_a  c_a  1]^-1 [v1]
[v_y_rel] = [ -s_c  c_b  1]    [v2]
[R_w    ]   [ -s_b  c_c  1]    [v3]

[v_x_rel]                                                                   [ (c_b - c_c)          (c_c - c_a)          (c_a - c_b)         ] [v1]
[v_y_rel] =   1/(c_c*s_a - c_b*s_a + c_a*s_b - c_c*s_b + c_b*s_c - c_a*s_c) [ (s_b - s_c)          (s_c - s_a)          (s_a - s_b)         ] [v2]
[R*w    ]                                                                   [ (c_b*s_c - c_c*s_b)  (c_c*s_a - c_a*s_c)  (c_a*s_b - c_b*s_a) ] [v3]

Transformation of (v_x_rel, v_y_rel) to (v_x, v_y) -- real coordinates

[v_x] = [cos(alpha) -sin(alpha)][v_x_rel]
[v_y]   [sin(alpha)  cos(alpha)][v_y_rel]

where alpha = 

*/

#define SINDEG(a)(sin(a*PI/180))
#define COSDEG(a)(cos(a*PI/180))

class LocationCalculator{
public:
	float v_x, v_y;
	float v_x_rel, v_y_rel;
	float w;
	
	float x, y;
	float theta; // current angle

	LocationCalculator(float _R, float _theta_a, float _theta_b, float _theta_c,
					   float *_v_a, float *_v_b, float *_v_c){
		R = _R;

		theta_a = _theta_a;
		theta_b = _theta_b;
		theta_c = _theta_c;

		v_a = _v_a;
		v_b = _v_b;
		v_c = _v_c;

		s_a = SINDEG(theta_a);  c_a = COSDEG(theta_a);
		s_b = SINDEG(theta_b);  c_b = COSDEG(theta_b);
		s_c = SINDEG(theta_c);  c_c = COSDEG(theta_c);

		/* matrix stuff */

		det = (c_c*s_a - c_b*s_a + c_a*s_b - c_c*s_b + c_b*s_c - c_a*s_c);
	}
	void update(unsigned long _dt){
		v_x_rel = (c_b - c_c) * (*v_a) 
		   		+ (c_c - c_a) * (*v_b) 
		   		+ (c_a - c_b) * (*v_c);

		v_x_rel /= det;

		v_y_rel = (s_b - s_c) * (*v_a) 
		    	+ (s_c - s_a) * (*v_b) 
		    	+ (s_a - s_b) * (*v_c);

		v_y_rel /= det;

		w = (c_b*s_c - c_c*s_b) * (*v_a)
		  + (c_c*s_a - c_a*s_c) * (*v_b)
		  + (c_a*s_b - c_b*s_a) * (*v_c);

		w = w / det / R;  // in rad
		w = w * 180 / PI; // in deg

		// linear equations solved. Now integrate.
		
		theta += w * _dt / 1000000;
		if (theta >= 360) theta -= 360;

		average_theta = theta - w * _dt / 1000000 / 2;
		alpha = -average_theta;

		// first transform velocities to real coordinates.
		v_x = COSDEG(alpha) * v_x_rel - SINDEG(alpha) * v_y_rel;
		v_y = SINDEG(alpha) * v_x_rel + COSDEG(alpha) * v_y_rel;

		// now integrate
		x += v_x * _dt / 1000000;
		y += v_y * _dt / 1000000;
	}
	void update(){
		dt = deltaT;
		deltaT = 0;
		//Serial.println(dt);
		update(dt);
	}
private:	
	elapsedMicros deltaT;
	unsigned long dt; // in us

	float theta_a, theta_b, theta_c;
	float *v_a, *v_b, *v_c;
	float R;
	

	float s_a;   float c_a;
	float s_b;   float c_b;
	float s_c;   float c_c;

	float det;

	float average_theta;
	float alpha;
};