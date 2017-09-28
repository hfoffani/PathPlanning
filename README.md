# Path Planning

A C++ implementation of a path planner for self-driving cars.

This project implements a Path Planner to safely navigate a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
It uses the Udacity simulator which is available in [Github](https://github.com/udacity/self-driving-car-sim/releases).


### The Model

The model consists in two main parts. The Behavior Planning and the Trajectory Generation,

#### Behavior Planning

The Behavior Planning is implemented using a simple Finite State Machine with three states:
`KEEP_LANE`, `LANE_CHANGE_LEFT`, `LANE_CHANGE_RIGHT`.

The manouver preferences are tuned using a cost function for each action.

1. If the lane is free ahead it prefers to stay in it.
1. If the lane is busy try to see if the closest lanes are free.
1. Only overtake if there is space to gain in the new lane.
1. Prefer the left lane over the right lane to overtake a car.
1. Do not turn left/right if the car is in the left-most/right-most lane.
1. Prefer the middle lane over the left and right lanes.

If the car cannot change lanes because they are busy, try to mimic the current
velocity of the car ahead keeping a safe distance.

The output of this step is the lane and the speed that the car should target.


#### Trajectory Generation

With the next optimal lane and speed the process generates a trajectory
that smoothly respects the highway map and the desired action.

Using geometry from the map waypoints data, the current position of the
car and the desired destination I create 3 points spread along the next 90mts.
These three points plus two more from the previous trajectory conforms
the input dataset.

A spline algorithm fed by the input dataset provides a function that allows
us to interpolate the rest of the points spread by 20ms each.

The interpolated points are converted to global coordinates before sending
them to the simulator.


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


