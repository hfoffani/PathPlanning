
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

// get the next action (lane and velocity)
action next_action(vector<vector<double>> sensor_fusion,
                           double car_s, int prev_size, int lane_mine, double velocity) {

    bool too_close = false;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        int lane_other = d_to_lane(sensor_fusion[i][6]);
        if (lane_other == lane_mine) {
            double vx_other = sensor_fusion[i][3];
            double vy_other = sensor_fusion[i][4];
            double v_other = sqrt(vx_other*vx_other+vy_other*vy_other);

            double s_other = sensor_fusion[i][5];
            s_other += ((double)prev_size)*.02*v_other;
            if (s_other > car_s && (s_other - car_s) < 30) {
                too_close = true;
                if (lane_mine > 0)
                    lane_mine = 0;
            }
        }
    }

    if (too_close) {
        velocity -= .224; // 5m/s
    } else if (velocity < 49.5) {
        velocity += .224; // 5m/s
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

