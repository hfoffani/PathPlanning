
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

