#define SPLINESTEP 30

#define KEEPLANE 0
#define TURNLEFT 1
#define TURNRIGHT 2

#define LANEWIDTH 4

#define BUSYAHEADMIN 0
#define BUSYAHEADMAX 30

#define BUSYCHANGEMIN -25
#define BUSYCHANGEMAX 35

#define PATHPOINTS 50

#define POINTSPEED 0.02     // car moves to a new waypoint every 20ms
#define NOJERKACC 0.224     // 5m/s per ??
#define SAFETYMARGIN .9     // car ahead speed limit befere braking

#define MPH2MS(x) (x/2.24)  // mph to m/s
#define MS2MPH(x) (x*2.24)  // m/s to mph

#define MAXVELOCITY 49.70

#define MAXVAL 100000.0

