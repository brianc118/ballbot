/* Library for my ballbot, found at http://onewaytobalance.blogspot.com
 * 
 * For a good explanation of the maths here, see
 * http://onewaytobalance.blogspot.com.au/2015/12/omnidirectional-control.html
 *
 * Open sourced under the MIT License (see LICENSE.txt)
 * (c) Brian Chen 2015
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
    void update(unsigned long _dt, float rot){
        v_x_rel = (c_b - c_c) * (*v_a) 
                + (c_c - c_a) * (*v_b) 
                + (c_a - c_b) * (*v_c);

        v_x_rel /= det;

        v_y_rel = (s_b - s_c) * (*v_a) 
                + (s_c - s_a) * (*v_b) 
                + (s_a - s_b) * (*v_c);

        v_y_rel /= det;

        // first transform velocities to real coordinates.
        v_x = COSDEG(rot) * v_x_rel - SINDEG(rot) * v_y_rel;
        v_y = SINDEG(rot) * v_x_rel + COSDEG(rot) * v_y_rel;

        // now integrate
        x += v_x * _dt / 1000000;
        y += v_y * _dt / 1000000;
    }
    void update(unsigned long _dt){
    	float alpha = getRot(_dt);
    	update(_dt, alpha);
    }
    void update(float alpha){
        unsigned long _dt = getDt();
        update(_dt, alpha);
    }
    void update(){
    	unsigned long _dt = getDt();
    	float alpha = getRot(_dt);
    	update(_dt, alpha);
    }

    void zero(){
        x = 0;
        y = 0;
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

    unsigned long getDt(){
    	dt = deltaT;
        deltaT = 0;
        return dt;
    }

    float getRot(unsigned long _dt){
    	w = (c_b*s_c - c_c*s_b) * (*v_a)
          + (c_c*s_a - c_a*s_c) * (*v_b)
          + (c_a*s_b - c_b*s_a) * (*v_c);

        w = w / det / R;  // in rad
        w = w * 180 / PI; // in deg

        // linear equations solved. Now integrate.
        
        theta += w * _dt / 1000000;
        if (theta >= 360) theta -= 360;

        average_theta = theta - w * _dt / 1000000 / 2;
        return -average_theta;
    }
};