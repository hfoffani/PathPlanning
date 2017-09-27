

vector<double> global_to_car(double x, double y, double ref_x, double ref_y, double ref_yaw) {
     double shift_x = x - ref_x;
     double shift_y = y - ref_y;

     double x_car = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
     double y_car = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);

     return {x_car, y_car};
}

vector<double> car_to_global(double x, double y, double ref_x, double ref_y, double ref_yaw) {
     double x_global = x * cos(ref_yaw) - y * sin(ref_yaw);
     double y_global = x * sin(ref_yaw) + y * cos(ref_yaw);

     x_global += ref_x;
     y_global += ref_y;

    return {x_global, y_global};
}


int d_to_lane(double d) {
    return floor(d / 4);
}

double lane_to_d(int lane) {
    return lane*4+2;
}


struct action {
    int lane;
    double velocity;
};


bool lane_is_busy(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size,
                  double dist_behind, double dist_ahead) {
    bool is_busy = false;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        int lane_other = d_to_lane(sensor_fusion[i][6]);
        if (lane_other == lane) {
            double vx_other = sensor_fusion[i][3];
            double vy_other = sensor_fusion[i][4];
            double v_other = sqrt(vx_other*vx_other+vy_other*vy_other);

            double s_other = sensor_fusion[i][5];
            s_other += ((double)prev_size)*.02*v_other;

            double distance = s_other - car_s;
            if (distance > dist_behind && distance < dist_ahead) {
                is_busy = true;
            }
        }
    }
    return is_busy;
}

bool lane_is_busy_ahead(vector<vector<double>> sensor_fusion, int lane, double car_s, int prev_size) {
    return lane_is_busy(sensor_fusion, lane, car_s, prev_size, 0, 30);
}

#define KEEPLANE 0
#define TURNLEFT 1
#define TURNRIGHT 2

double cost_KEEPLANE(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size) {
    double cost = 0;
    if (lane_is_busy_ahead(sensor_fusion, car_lane, car_s, prev_size)) cost += .5;
    return cost;
}
double cost_TURNLEFT(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size) {
    double cost = 0;
    if (! lane_is_busy_ahead(sensor_fusion, car_lane, car_s, prev_size)) cost += .7;
    if (car_lane == 0) cost += 1;
    if (lane_is_busy(sensor_fusion, car_lane-1, car_s, prev_size, -20, 30)) cost += 1;
    return cost;
}
double cost_TURNRIGHT(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size) {
    double cost = 0;
    if (! lane_is_busy_ahead(sensor_fusion, car_lane, car_s, prev_size)) cost += .7;
    if (car_lane == 2) cost += 1;
    if (lane_is_busy(sensor_fusion, car_lane+1, car_s, prev_size, -20, 30)) cost += 1;
    return cost;
}

typedef double (*cost_type)(vector<vector<double>>, int, double, int);

cost_type cost_functions[] = {
        cost_KEEPLANE,
        cost_TURNLEFT,
        cost_TURNRIGHT
};

int get_next_state(vector<vector<double>> sensor_fusion, int car_lane, double car_s, int prev_size) {

    int next_state = KEEPLANE;
    double lowest_cost = 10;
    for (int i = 0; i < 3; i++) {
        double newcost = cost_functions[i](sensor_fusion, car_lane, car_s, prev_size);
        // cout << "state: " << i << "  cost: " << newcost << " | ";
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

    int next_state = get_next_state(sensor_fusion, lane_mine, car_s, prev_size);
    switch (next_state) {
        case KEEPLANE:
            if ( lane_is_busy_ahead(sensor_fusion, lane_mine, car_s, prev_size) ) {
                velocity -= .224; // 5m/s
            } else if (velocity < 49.5) {
                velocity += .224; // 5m/s
            }
            break;
        case TURNRIGHT:
            lane_mine += 1;
            break;
        case TURNLEFT:
            lane_mine -= 1;
            break;
    }

    return { lane_mine, velocity };
}

// interpolate using the spline and convert back to global coordinates.
void interpolate_next_vals(vector<double> & next_x_vals, vector<double> &next_y_vals, tk::spline s,
                           int prev_size, double ref_x, double ref_y, double ref_yaw, double ref_vel) {
    
    double x_target = 30.0;
    double y_target = s(x_target);
    double path = sqrt((x_target*x_target)  + (y_target*y_target));
    
    double x_addon = 0;

    for (int k = 1; k < 50 - prev_size; ++k) {
        
        double N = 2.24* path / (0.02 * ref_vel);
        double x_point = x_addon + (x_target/N);
        double y_point = s(x_point);
        
        x_addon = x_point ;
        
        vector<double> xy = car_to_global(x_point, y_point, ref_x, ref_y, ref_yaw);
        
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
        
    }
}

