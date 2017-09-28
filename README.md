# Path Planning

A C++ implementation of a path planner for self-driving cars.


This project implements a Path Planner to safely navigate a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
It uses the Udacity simulator which is available in [Github](https://github.com/udacity/self-driving-car-sim/releases).


### The Model




---

        constants.h:#define POINTSPEED 0.02             // car moves to a new waypoint every 20ms
        constants.h:#define NOJERKACC (0.224*1.5)       // 7.5m/s
        constants.h:#define SAFETYMARGIN .9             // car ahead speed limit befere braking
        constants.h:#define MPH2MS(x) (x/2.24)          // mph to m/s
        constants.h:#define MS2MPH(x) (x*2.24)          // m/s to mph
        helper.h:// convert coordinates from global to car (the car is always at 0,0).
        helper.h:// convert car coordinates to global.
        helper.h:// represents the action to take.
        helper.h:// data is used for trajectory design.
        helper.h:// returns the speed of the nearest car in the specified lane in a given space.
        helper.h:// returns MAXVAL if there is no other car there.
        helper.h:    // if (min_dist < MAXVAL)
        helper.h:    //     cout << "lane: " << lane << " | speed: " << speed_next << ", dist: " << min_dist << endl;
        helper.h:// returns the speed of the nearest car ahead.
        helper.h:// returns MAXVAL if there is no other car.
        helper.h:// cost for keeping the same lane.
        helper.h:    if (busy_ahead) cost += .4;         // penalty to keep lane if there's a car ahead.
        helper.h:    if (car_lane != 1) cost += .3;      // prefer center lane.
        helper.h:// cost for changing to the lane at the left.
        helper.h:    double cost = 0.1;                  // basic penalty for turning
        helper.h:    if (car_lane == 0) cost += 1;       // don't go left if in leftmost lane
        helper.h:    if (lbusy) cost += 1;               // don't turn left if occupied
        helper.h:// cost for changing to the lane at the right.
        helper.h:    double cost = 0.2;                  // basic penalty for turning right. prefers overtake through the left.
        helper.h:    if (car_lane == 2) cost += 1;       // don't go left if in right-most lane
        helper.h:    if (lbusy) cost += 1;               // don't turn right if occupied
        helper.h:// Simple Finite State Machine.
        helper.h:        // cout << "state: " << state_name(i) << "  cost: " << newcost << " | ";
        helper.h:    // cout << endl;
        helper.h:// get the next car action (lane and velocity)
        helper.h:    // cache some processing
        helper.h:    // FSM
        helper.h:                // match the speed of car ahead
        helper.h:// interpolate using the spline and convert back to global coordinates.
        main.cpp:                    // obtain the next action for the ego car. (the best lane and velocity)
        main.cpp:                    // sparsed points for spline to fill.
        main.cpp:                    // get the two previous points consumed by the car so the trajectory for
        main.cpp:                    // the future path fits more smoothly.
        main.cpp:                    // three future points sparsed SPLINESPTEP distance between them.
        main.cpp:                    // cout << "car_s: " << car_s << ", d: " << next_d << endl;
        main.cpp:                    // cout << "sparse points: " << ptsx.size() << endl;
        main.cpp:                    // change x,y to car reference 0,0
        main.cpp:                    // fit a spline with the given five points
        main.cpp:                    // interpolate all the required values from the spline, convert them to global coordinates
        main.cpp:                    // and put them into the next_x_vals and next_y_vals vectors.
        main.cpp:                    // cout << " ---- " << endl;
        main.cpp:                    // ends my code.
---


---

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


---


### License

This project is published under the [Apache License](http://www.apache.org/licenses/LICENSE-2.0).


### Contributions

I gratefully honor Pull Requests.
Please, consider formatting the code with K&R style and four spaces tabs.


### Who do I talk to?

For questions or requests post an issue here or tweet me at
[@herchu](http://twitter.com/herchu)


