#include "constants.h"

using namespace std;

// convert coordinates from global to car (the car is always at 0,0).
vector<double> global_to_car(double x, double y, double ref_x, double ref_y, double ref_yaw) {
     double shift_x = x - ref_x;
     double shift_y = y - ref_y;

     double x_car = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
     double y_car = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);

     return {x_car, y_car};
}

// convert car coordinates to global.
vector<double> car_to_global(double x, double y, double ref_x, double ref_y, double ref_yaw) {
     double x_global = x * cos(ref_yaw) - y * sin(ref_yaw);
     double y_global = x * sin(ref_yaw) + y * cos(ref_yaw);

     x_global += ref_x;
     y_global += ref_y;

    return {x_global, y_global};
}


int d_to_lane(double d) {
    return floor(d / LANEWIDTH);
}

double lane_to_d(int lane) {
    return lane*LANEWIDTH+LANEWIDTH/2;
}


// represents the action to take.
// data is used for trajectory design.
struct action {
    int lane;
    double velocity;
};


// returns the speed of the nearest car in the specified lane in a given space.
// returns MAXVAL if there is no other car there.
double lane_is_busy(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size,
                  double dist_behind, double dist_ahead) {

    double min_dist = MAXVAL;
    double speed_next = MAXVAL;

    for (int i = 0; i < sensor_fusion.size(); i++) {
        int lane_other = d_to_lane(sensor_fusion[i][6]);
        if (lane_other == lane) {
            double vx_other = sensor_fusion[i][3];
            double vy_other = sensor_fusion[i][4];
            double v_other = sqrt(vx_other*vx_other+vy_other*vy_other);

            double s_other = sensor_fusion[i][5];
            s_other += ((double)prev_size)*POINTSPEED*v_other;

            double distance = s_other - car_s;
            double other_speed = MS2MPH(v_other);

            if (distance > dist_behind && distance < dist_ahead) {
                if (min_dist > abs(distance)) {
                    min_dist = abs(distance);
                    speed_next = other_speed;
                }
            }
        }
    }
    // if (min_dist < MAXVAL)
    //     cout << "lane: " << lane << " | speed: " << speed_next << ", dist: " << min_dist << endl;
    return speed_next;
}


// returns the speed of the nearest car ahead.
// returns MAXVAL if there is no other car.
double lane_is_busy_ahead(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size) {
    return lane_is_busy(sensor_fusion, lane, car_s, prev_size, BUSYAHEADMIN, BUSYAHEADMAX);
}

// cost for keeping the same lane.
double cost_KEEP_LANE(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size, bool busy_ahead) {
    double cost = 0;
    if (busy_ahead) cost += .4;         // penalty to keep lane if there's a car ahead.
    if (car_lane != 1) cost += .3;      // prefer center lane.
    return cost;
}

// cost for changing to the lane at the left.
double cost_LANE_CHANGE_LEFT(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size, bool busy_ahead) {
    double cost = 0.1;                  // basic penalty for turning
    if (car_lane == 0) cost += 1;       // don't go left if in leftmost lane
    bool lbusy = lane_is_busy(sensor_fusion, car_lane-1, car_s, prev_size, BUSYCHANGEMIN, BUSYCHANGEMAX) < MAXVAL;
    if (lbusy) cost += 1;               // don't turn left if occupied
    return cost;
}

// cost for changing to the lane at the right.
double cost_LANE_CHANGE_RIGHT(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size, bool busy_ahead) {
    double cost = 0.2;                  // basic penalty for turning right. prefers overtake through the left.
    if (car_lane == 2) cost += 1;       // don't go left if in right-most lane
    bool lbusy = lane_is_busy(sensor_fusion, car_lane+1, car_s, prev_size, BUSYCHANGEMIN, BUSYCHANGEMAX) < MAXVAL;
    if (lbusy) cost += 1;               // don't turn right if occupied
    return cost;
}

typedef double (*cost_type)(vector<vector<double>>, int, double, int, bool);

cost_type cost_functions[] = {
        cost_KEEP_LANE,
        cost_LANE_CHANGE_LEFT,
        cost_LANE_CHANGE_RIGHT
};

string state_name(int state) {
    string names[] = { "KEEP LANE", "LANE CHANGE LEFT", "LANE CHANGE RIGHT" };
    return names[state];
}

// Simple Finite State Machine.
int get_next_state(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size, bool busy_ahead) {

    int next_state = KEEP_LANE;
    double lowest_cost = 10;
    for (int i = 0; i < 3; i++) {
        double newcost = cost_functions[i](sensor_fusion, car_lane, car_s, prev_size, busy_ahead);
        // cout << "state: " << state_name(i) << "  cost: " << newcost << " | ";
        if (newcost < lowest_cost) {
            lowest_cost = newcost;
            next_state = i;
        }
    }
    // cout << endl;
    return next_state;
}

// get the next car action (lane and velocity)
action next_action(vector<vector<double>> sensor_fusion,
                   double car_s, int prev_size, int lane_mine, double velocity) {

    // cache some processing
    double speed_ahead = lane_is_busy_ahead(sensor_fusion, lane_mine, car_s, prev_size);
    bool busy_ahead = speed_ahead < MAXVAL;

    // FSM
    int next_state = get_next_state(sensor_fusion, lane_mine, car_s, prev_size, busy_ahead);

    switch (next_state) {
        case KEEP_LANE:
            if (busy_ahead && velocity > (speed_ahead * SAFETYMARGIN)) {
                // match the speed of car ahead
                velocity -= NOJERKACC;
            } else if (velocity < MAXVELOCITY) {
                velocity += NOJERKACC;
            }
            break;
        case LANE_CHANGE_RIGHT:
            lane_mine += 1;
            break;
        case LANE_CHANGE_LEFT:
            lane_mine -= 1;
            break;
    }

    return { lane_mine, velocity };
}

// interpolate using the spline and convert back to global coordinates.
void interpolate_next_vals(vector<double> & next_x_vals, vector<double> &next_y_vals, tk::spline s,
                           int prev_size, double ref_x, double ref_y, double ref_yaw, double ref_vel) {
    
    double x_target = SPLINESTEP;
    double y_target = s(x_target);
    double path = sqrt((x_target*x_target)  + (y_target*y_target));
    
    double x_addon = 0;

    for (int k = 1; k < PATHPOINTS - prev_size; ++k) {
        
        double N = path / (POINTSPEED * MPH2MS(ref_vel));
        double x_point = x_addon + (x_target/N);
        double y_point = s(x_point);
        
        x_addon = x_point ;
        
        vector<double> xy = car_to_global(x_point, y_point, ref_x, ref_y, ref_yaw);
        
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
        
    }
}

