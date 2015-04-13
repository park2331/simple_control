1
2 //***********************************************************************************************************************
3 //Last updated 5/2/2012
4 //Josh's targeted control algorithm
5 //These are used by simple_control(), steerToTarget(), landingRoutine(), and landingFlare()
6
7
8 //Global variables:
9 float headingAct[2] = {0,0}; //actual heading, initialize {0,0} here; updated in loop
10 float servoOutPrcnt[2] = {0,0}; //intialize {0,0}; corrective steering command to servo without amplification scaling, a fn of headingDev
11 float desiredYawRate[2] = {0,0}; //intialize {0,0}; minimum desired yaw rate, a fn of headingDev, degrees/s, must be L ESS THAN 180 deg/s or steering routine WILL FAIL
12 int steeringActive = 0; //intialize 0; 1 if steering occured previously this loop, 0 if not
13 float steeringScale[2] = {0,0}; //intialize {0,0}; servo scaling factor to achieve desired yaw
14 float achievedYawRate[2] = {0.0}; //intialize {0,0}; yaw rate achieved by previous turn command, degrees/s
15 int steerDelayCounter = 0; //intialize 0; counter for initial steering input lag delay
16 int turnedLeft[2] = {0,0}; //intialize {0,0}; flag -- if true, system turned left last loop (but NOT as a result of negat
ive servo value)
17 int turnedRight[2] = {0,0}; //intialize {0,0}; flag -- if true, system turned right last loop (but NOT as a result of neg
ative servo value)
18 double headingTime[2] = {0,0}; //initialize {0,0}; holds GPS time-of-week (TOW) in milliseconds 19
20 //Function prototypes:
21 void simple_control(void);
22 void steerToTarget(float headingDes);
23 void landingRoutine(float headingDes);
24 void landingFlare(void);
25 26
27 //Landing target parameters:
28 #define TARGETLAT 37.2862100 //target latitude in decimal degrees
29 #define TARGETLONG -121.8517000 //target longitude in decimal degrees
30 #define TARGET_ALTITUDE 72.1550 //elevation of landing target coordinates above sea level, meters
31 32
33 //Distance and heading calculation constants:
34 #define EARTHRAD 6371000 //radius of the Earth, average, m
35 36
37 //Steering algorithm tuning parameters:
38 #define HEADING_DEADBAND 2.5 //deadband +/- degrees deviation between actual and desired; within this range no servo upda
tes will occur this loop
39 #define DESIRED_YAW_DEADBAND 1 //yaw rate deadband +/- degrees/sec for no yaw rate adjustment in next loop (holds servo v
alues constant)
40 #define NUM_LOOPS_BEFORE_SCALING_TURN 1 //number of loops to hold steering value before stepping to a new servo pull value; value of 1 updates every loop, 2 every 2 loops, etc.
41 #define FINE_SCALING_CUTOFF_DEG_SEC 2.5 //if yaw rate deviation from actual to desired (+/-) is less than this value, st
eering gain stepping uses fine increments
42 #define FINE_SCALING_STEP_PERCENT 0.010 //range 0 to 1; servo pull percent change each update (10 times/sec) to achieve t
arget yaw rate when yaw rate deviation < FINE_SCALING_CUTOFF_DEG_SEC
43 #define FINE_SCALING_UNWIND_GAIN 1 //damping factor to increase rate of toggle line unwind; prevents underdamped overshoo
t of yaw rate and desired heading at low deviation angles
44 //usage example: 1.5 means toggle line unwrap step size will be 50% greater than tog
gle line pull step size (150% of pull rate); 1 is inactive
45 #define COARSE_SCALING_STEP_PERCENT 0.020 //range 0 to 1; servo pull percent change each update (10 times/sec) to achieve
target yaw rate when yaw rate deviation >= FINE_SCALING_CUTOFF_DEG_SEC
46 #define COARSE_SCALING_UNWIND_GAIN 1.5 //damping factor to increase rate of toggle line unwind; prevents underdamped over
shoot of yaw rate and desired heading at low deviation angles
47 //usage example: 1.5 means toggle line unwrap step size will be 50% greater than t
oggle line pull step size (150% of pull rate)
48
49 //Desired yaw rate coefficients; controls desired yaw rate for a given heading deviation -- get these from Excel plot of esired yaw rate vs. heading deviation
50 #define DESIREDYAW_COEFF_1 -0.00000000000937 //x^6 term
51 #define DESIREDYAW_COEFF_2 0.00000000662071 //x^5 term
52 #define DESIREDYAW_COEFF_3 -0.00000185521159 //x^4 term
53 #define DESIREDYAW_COEFF_4 0.00026074766721 //x^3 term
54 #define DESIREDYAW_COEFF_5 -0.01906582376881 //x^2 term
55 #define DESIREDYAW_COEFF_6 0.76467370416140 //x term
56 #define DESIREDYAW_COEFF_7 -0.41502129438777 //constant term
57 58
59 //Landing routine constants:
60 #define NUM_LOOPS_BEFORE_SCALING_TURN_LANDING 1 //number of loops to hold steering value before stepping to a new servo p
ull value; value of 1 updates every loop, 2 every 2 loops, etc.
61 #define FINE_SCALING_CUTOFF_DEG_SEC_LANDING 5 //if yaw rate deviation (+/-) is less than this value, steering gain steppi
ng uses fine increments
62 #define FINE_SCALING_STEP_PERCENT_LANDING 0.015 //range 0 to 1; servo pull percent change each update (10 times/sec) to a
chieve target yaw rate when yaw rate deviation < FINE_SCALING_CUTOFF_DEG_SEC
63 #define COARSE_SCALING_STEP_PERCENT_LANDING 0.035 //range 0 to 1; servo pull percent change each update (10 times/sec) to
achieve target yaw rate when yaw rate deviation >= FINE_SCALING_CUTOFF_DEG_SEC
64 #define LANDING_RADIUS_THRESHOLD 7.5 //when within this distance from target, landingRoutine is active, m
65 #define LANDING_RADIUS 6 //desired spiral radius from target, m; MUST BE LESS THAN LANDING_RADIUS_THRESHOLD!!!
66 //used to calculate required yaw rate to steer within LANDING_RADIUS_THRESHOLD
67 //be careful with size! smaller size requires a greater yaw rate!
68 69
70 //Flare routine constants:
71 #define FLARE_HEIGHT 4 //distance above ground to initiate flare maneuver, meters
72 #define FLARE_PRCNT 1.0 //range 0 to 1; percentage of maximum servo pull used in flare maneuver
73 74
75 //END Josh's code
76 //***********************************************************************************************************************
*******************************************************************************
77
78 //********************************************************************//
79 //***Debug Code Start -- comment ALL out for Monkey!***
80 //
81 //Used to feed fake data to control routine for compilation on
82 //Windows-based PC
83
84 #define DEGREES_TO_RAD 0.017453292519943295769236907684886
85 #define RAD_TO_DEG 57.295779513082320876798154814105
86 #define SERVO_RIGHT_WINCH_3 2
87 #define SERVO_LEFT_WINCH_4 3
88 #define SERVO_R_WINCH_MAX_TRAVEL 5.75
89 #define SERVO_L_WINCH_MAX_TRAVEL -5.75
90 #define SERVO_R_WINCH_SCALE_FACTOR 1.75
91 #define SERVO_L_WINCH_SCALE_FACTOR -1.75
92
93 float gGPSheading = 284;
94 float gGPSlatitude = 37.2862000;
95 float gGPSlongitude = -121.8517000;
96 float gGPSgndspeed = 15;
97 float gGPSaltitude = 82.1550;
98 double gGPSTOW = 579857000;
99
100 //***Debug Code End***
101 //********************************************************************//
102
