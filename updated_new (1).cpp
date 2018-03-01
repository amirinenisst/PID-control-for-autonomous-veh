#include <math.h> // for math functions
#include <vector> // for array functions
#include <cmath>
#define _USE_MATH_DEFINES
#include <tuple>
#include <string>

using namespace std;

float Kpv = 1.0; // speed propotional gain
float Kpe = 1.0;
float Kie = 0.1;
float Kde = 0;
float dt = 0.1;  // [s]


bool show_animation = true;
// state class
class State {
    public:
        // instance variables
        float x1,y1,yaw1,v1;
    public:
        // Constructor
        State(){
             x1 = 0.0;
             y1 = 0.0;
             yaw1 = 0.0;
             v1 = 0.0;
        }
};

class PID {
	public:

		double p_error;
		double i_error;
		double d_error;

		double Kp;
		double Ki;
		double Kd;

		void Init(double Kp1, double Ki1, double Kd1){
			
			Kp = Kp1;
			Ki = Ki1;
			Kd = Kd1;

		}
		
		void UpdateError(double ce, int sign){

			d_error = (sign*ce - p_error);
			p_error = sign*ce;
			i_error += sign*ce;

		}
		
		double TotalError(){
			
			return -Kp * p_error - Kd * d_error - Ki * i_error;
		}
};

class Algorithm {
    public:
		float tx;
		float alpha;
		float ty;
		float Lf;
		float delta;
		float k = 0.1;  // look forward gain
        float Lfc = 1.0;  // look-ahead distance
        float L = 3.0;  // [m] wheel base of vehicle
		
		Algorithm(){
			delta = 0.0;
		}
        
        // function that return state object
        State update(State& state,float a,float delta){
        
            state.x1 = state.x1 + state.v1 * cos(state.yaw1) * dt;
            state.y1 = state.y1 + state.v1 * sin(state.yaw1) * dt;
            state.yaw1 = state.yaw1 + state.v1 / L * tan(delta) * dt;
            state.v1 = state.v1 + a * dt;
        
            return state;
        
        }
        
        float PIDControl(float target, float current){
            float a = Kpv * (target - current);
        
            return a;
        }

        
        float pure_pursuit_control(State& state, vector<float> cx, vector<float> cy, int pind, PID& pid){

            // there are no declaration for this function
            pair<int, int> indices = calc_target_index(state, cx, cy);
            int ind = indices.first;
            int near_ind = indices.second;
			int sign = 1;
			float error;

            if (pind >= ind){
                ind = pind;
            }

            if (ind < cx.size()) {
                tx = cx[ind];
                ty = cy[ind];
            }
            else {
                tx = cx.back();
                ty = cy.back();
                ind = cx.size() - 1;
            }

            alpha = atan2(ty - state.y1, tx - state.x1) - state.yaw1;

            if (state.v1 < 0) {
                alpha = M_PI - alpha;
				sign = -1;
            }

            Lf = k * state.v1 + Lfc;
			
			error = sqrt(abs(sqrt(pow((state.x1 - cx[near_ind]),2) + pow((state.y1 - cy[near_ind]),2))));
			
			pid.UpdateError(error, sign);

            delta = atan2(2.0 * L * sin(alpha) / Lf, 1.0) + pid.TotalError();

            return delta;
        }
        
        pair<int, int> calc_target_index(State& state, vector<float> cx, vector<float> cy){

            // search nearest point index
            vector<float> dx;
            vector<float> dy;
            vector<float> d;
            for (unsigned int i = 0; i < cx.size();i++){
                dx.push_back(state.x1 - cx[i]);
            }

            for (unsigned int i = 0; i < cy.size();i++){
                dy.push_back(state.y1 - cy[i]);
            }

            for (unsigned int i = 0; i < dx.size();i++){
                d.push_back(abs(sqrt(pow(dx[i],2) + pow(dy[i],2))));
            }


            unsigned int ind = FindMinElementIndex(d);
            unsigned int nearest_ind = ind;
            L = 0.0;

            Lf = k * state.v1 + Lfc;

            // search look ahead target point index
            while (Lf > L and (ind + 1) < cx.size()) {
                dx[ind] = cx[ind + 1] - cx[ind];
                dy[ind] = cx[ind + 1] - cx[ind];
                L += sqrt(pow(dx[ind], 2) + pow(dy[ind], 2));
                ind += 1;
            }
            pair <int, int> indices;
            indices.first = ind;
            indices.second = nearest_ind;
            return indices;
        }
    private:
                
        unsigned int FindMinElementIndex(vector<float> a){
            int min = 1000*1000,ind = 0;
            for (int i = 0; i < a.size();i++){
                if (a[i] < min){
                    min = a[i];
                    ind = i;
                }
            }
            return ind;
        }
};

int main(){

    //  target course
    vector<float> cx;
    vector<float> cy;
    int j;
    for (j = 0; j < 50;j++){
        cx.push_back(j*0.1);
    }

    for (unsigned int i = 0; i < cx.size();i++){
        cy.push_back(sin(cx[i] / 5.0) * cx[i] / 2.0);
    }

    float target_speed = 10.0 / 3.6;  // [m/s]

    float T = 100.0 ; // max simulation time
	PID pid;
	pid.Init(Kpe, Kie, Kde);

    // initial state
    State state;
    state.x1 = -0.0;
    state.y1 = -3.0;
    state.yaw1 = 0.0;
    state.v1 = 0.0;
    
    Algorithm alg;
	
	float time_e;
    
    unsigned int lastIndex = cx.size() - 1;

    vector<float> x;
    x.push_back(state.x1);
    vector<float> y;
    y.push_back(state.y1);
    vector<float> yaw;
    yaw.push_back(state.yaw1);
    vector<float> v;
    v.push_back(state.v1);
    vector<float> t;
    t.push_back(0.0);
    pair<int, int> indices = alg.calc_target_index(state, cx, cy);
    
    int target_ind = indices.first;

    while (T >= time_e and lastIndex > target_ind){

        float ai = alg.PIDControl(target_speed, state.v1);
        float di = alg.pure_pursuit_control(state, cx, cy, target_ind, pid);

        state = alg.update(state, ai, di);

        time_e = time_e + dt;

        x.push_back(state.x1);
        y.push_back(state.y1);
        yaw.push_back(state.yaw1);
        v.push_back(state.v1);
        t.push_back(time_e);
    }
}