1
2 //********************************************************************//
3 //***Debug Code Start -- comment ALL out for Monkey!***
4 //This feeds fake data to the control routines for compilation, simulation,
5 //and debugging on a Windows PC
6
7 #include <math.h>
8 #include <stdio.h>
9 #include "simple_control.h"
10 #include "pwm.h"
11 #include <stdlib.h>
12 13
14 main() {
15 //Simulation Input Globals
16 int i;
17
18 for (i = 0; i<=20; i++){ 19
20 //printf("Start main\n"); 21
22 gGPSheading = ((int)(gGPSheading-5*i)+360)%360;
23 gGPSlatitude = gGPSlatitude+0.000010;
24 gGPSlongitude = gGPSlongitude-0.000000;
25 gGPSaltitude = gGPSaltitude + i;
26 gGPSTOW = gGPSTOW + 250;
27
28 simple_control();
29 }
30 return(0);
31 }
32 33
34 //GLOBALS
35 //***Debug Code End***
36 //____________________________________________________________________//
37
38
39
40
41 //_______________________________________________________________________________________________________________________
_____________________________________________________________________
42 //***********************************************************************************************************************
*********************************************************************
43 //Start Josh's control code -- current version 3/12/2012 -- last updated 4/22/2012 44
45
46
47
48 //********************************************************************//
49 //Function: simple_control
50 //
51 //Desc: Control loop for flight; determines heading and distance to target
52 //Receives:
53 //Returns:
54 //CONSTANTS: EARTHRAD, TARGETLAT, TARGETLONG, LANDING_RADIUS_THRESHOLD,
55 // DEGREES_TO_RAD, RAD_TO_DEG, TARGET_ALTITUDE, FLARE_HEIGHT
56 //Globals: headingAct[2]
57 //____________________________________________________________________//
58
59
60 void simple_control(void) { 61
62
63 //Local variables:
64 //(for arrays, 0 is current value and 1 is value from previous loop)
65 double latCurrent; //current device latitude, dec. degrees
66 double longCurrent; // current device longitude, dec. degrees
67 double altCurrent; // current device altitude AGL, meters
68 double dLat; //difference between target and current latitude
69 double dLong; //difference between target and current longitude
70 double latCurrentRad; //current pos. latitude in radians
71 double targetLatRad; //target pos. latitude in radians
72 double aDist; //value for Haversine distance calculation
73 double cDist; //value for Haversine distance calculation
74 double yHead; //variable for heading-to-target calculation
75 double xHead; //variable for heading-to-target calculation
76 float headingDes; //desired heading to target, calculated at each loop update, degrees
77 float distMag; //distance magnitude from current pos. to target, m
78 int headingDesRnd; //holds rounded desired heading to work with modulo operator
79 80
81 //DEBUG LINE
82 printf("\n______________________________________________________________\nStart control loop\n***************************
***********************************\n"); 83
84
85 //Writes current lat/long in degrees from Monkey
86 //latCurrent = gGPS.latitude;
87 //longCurrent = gGPS.longitude;
88 //altCurrent = gGPS.altitude;
89 //DEBUG LINE
90 latCurrent = gGPSlatitude;
91 longCurrent = gGPSlongitude;
92 altCurrent = gGPSaltitude;
93 94
95 //Calculates distMag using Haversine formula
96 dLat = (TARGETLAT-latCurrent)*DEGREES_TO_RAD;
97 dLong = (TARGETLONG-longCurrent)*DEGREES_TO_RAD;
98 latCurrentRad = latCurrent*DEGREES_TO_RAD; //current latitude in radians
99 targetLatRad = TARGETLAT*DEGREES_TO_RAD; //current longitude in radians
100
101 aDist = 102
103 cDist =
104 distMag
105 106
107 //Calculates headingDes using results from above
108 yHead = sin(dLong) * cos(targetLatRad);
109 xHead = cos(latCurrentRad)*sin(targetLatRad) -
110 sin(latCurrentRad)*cos(targetLatRad)*cos(dLong);
111 headingDes = atan2(yHead, xHead)*RAD_TO_DEG; //returns desired heading in degrees from -180 to 180
112 headingDes >= 0 ? (headingDesRnd = (int)(headingDes+0.5)) : (headingDesRnd = (int)(headingDes-0.5)); //rounds headingDes
sin(dLat/2) * sin(dLat/2) +
sin(dLong/2) * sin(dLong/2) * cos(latCurrentRad) * cos(targetLatRad); 2 * atan2(sqrt(aDist), sqrt(1-aDist));
= EARTHRAD * cDist;
for the modulo operator on next line
113 headingDes = (headingDesRnd+360) % 360; //uses headingDesRnd to return 0 <= int headingDes < 360 114
115
116 //DEBUG LINE
117 printf("\n Distance to target: %.3f\n Desired heading: %.3f\n Altitude: %.3f\n", distMag, headingDes, altCurrent);
118 119
120 //If within desired radius of target, start circling
121 if (altCurrent < (TARGET_ALTITUDE + FLARE_HEIGHT)){
122      landingFlare();
123 }
124


else if (distMag < LANDING_RADIUS_THRESHOLD) {
landingRoutine(headingDes);
//If not to target yet, keep flying to it
126
127 }
128
129
130
131
132
133 }
134
135
136 } //End control loop routine 137
138
139
140
141 //********************************************************************//
142 //Function: steerToTarget
143 //
144 //Desc: Steers device proportionally as-the-crow-flies to target coords.
145 //
146 //Receives: float headingDes
147 //
148 //Returns:
149 //
150 //CONSTANTS: HEADING_DEADBAND, DESIREDYAW_COEFF[1-7], NUM_LOOPS_BEFORE_SCALING_TURN,
151 // DESIRED_YAW_DEADBAND, SERVO_RIGHT_WINCH_3, SERVO_LEFT_WINCH_4,
152 // SERVO_R_WINCH_MAX_TRAVEL, SERVO_L_WINCH_MAX_TRAVEL, SERVO_R_WINCH_SCALE_FACTOR,
153 // SERVO_L_WINCH_SCALE_FACTOR, FINE_SCALING_UNWIND_GAIN, COARSE_SCALING_UNWIND_GAIN,
154 // FINE_SCALING_STEP_PERCENT, COARSE_SCALING_STEP_PERCENT, FINE_SCALING_CUTOFF_DEG_SEC
155 //
156 //Globals: headingAct[2], servoOutPrcnt[2], desiredYawRate[2], steeringActive,
157 // steeringScale, achievedYawRate[2], steerDelayCounter, turnedLeft[2], turnedRight[2],
158 // gGPS.heading, gGPS.latitude, gGPS.longitude, gGPS.TOW, headingTime[2]
159 //____________________________________________________________________//
160
161
162 void steerToTarget(float headingDes) { 163
164
165 //Local variables:
166 float headingDev; //deviation angle from actual heading to desired, degrees
167 int loopCtr; //loop counter for rewriting achievedYawRate array values to "older" positions in array
168
169 //DEBUG LINE - comment out below for debug
170 //Led_On(LED_RED); //Red LED blinks while control is in this routine (turned off at end of loop)
171
172 //Writes current GPS heading in degrees and GPS time-of-week in milliseconds from Monkey to arrays' 0 position
173 //headingAct[0] = gGPS.heading;
174 //headingTime[0] = gGPS.TOW; //NOTE: using TOW for headingTime will cause a momentary glitch when TOW reverts to 0 each w
else { steerToTarget(headingDes);
eek
175 //DEBUG LINE
176 headingAct[0] = gGPSheading;
177 headingTime[0] = gGPSTOW;
178
179 //DEBUG LINES BELOW
180 printf(" Actual heading: %.3f\n", headingAct[0]);
181 printf("\n\n***Start steer to target routine***\n");
182
183 //Determines the flight heading deviation from actual to the desired heading
184 //Positive clockwise, negative counterclockwise; range -180 to 180 degrees
185 if ((headingDes-headingAct[0] >= -180) && (headingDes-headingAct[0] <= 180)){
186 187 }
headingDev = headingDes-headingAct[0];

188
else if (headingDes-headingAct[0] < -180){ headingDev = 360 + (headingDes-headingAct[0]); }
else{
headingDev = (headingDes-headingAct[0]) - 360; }
189
190
191
192
193
194
195
196
197
198
199
//Determines heading angle (yaw) change from previous loop to current loop and achievedYawRate in deg/s
//Positive clockwise, negative counterclockwise
//Delivers rate achieved between -180 to 180 deg/s
if (headingAct[0] != headingAct[1]){//only updates achievedYawRate when a new GPS heading is available; approx. 4 times/s ec
200
201
202
203
204
205
206
207
208
209  }
210
211
212
213
if ((headingAct[0]-headingAct[1] >= -180) && (headingAct[0]-headingAct[1] <= 180)){
214
(headingDev),4)*DESIREDYAW_COEFF_3 +
pow(fabs(headingDev),3)*DESIREDYAW_COEFF_4 + pow(fabs(headingDev),2)*DESIREDYAW_COEFF_5 + (fabs(headingDev))*DESIREDY
achievedYawRate[0] = (headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000);
}
else if (headingAct[0]-headingAct[1] < -180){
achievedYawRate[0] = (360 + headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000); }
else{
achievedYawRate[0] = (headingAct[0]-headingAct[1] - 360)/((headingTime[0]-headingTime[1])/1000); }
//Returns desiredYawRate = fn(headingDev) as a rate (+/- deg/s), positive clockwise
if (headingDev<0) {
desiredYawRate[0]=(pow(fabs(headingDev),6)*DESIREDYAW_COEFF_1 + pow(fabs(headingDev),5)*DESIREDYAW_COEFF_2 + pow(fabs
AW_COEFF_6 + DESIREDYAW_COEFF_7)*(-1); 215 }
216 217
else {
desiredYawRate[0]=(pow(fabs(headingDev),6)*DESIREDYAW_COEFF_1 + pow(fabs(headingDev),5)*DESIREDYAW_COEFF_2 + pow(fabs
218
(headingDev),4)*DESIREDYAW_COEFF_3 +
pow(fabs(headingDev),3)*DESIREDYAW_COEFF_4 + pow(fabs(headingDev),2)*DESIREDYAW_COEFF_5 + (fabs(headingDev))*DESIREDY
AW_COEFF_6 + DESIREDYAW_COEFF_7); 219 }
220
221
222
223
224
225
226
//Writes steering scale value of previous loop to "old" array value
227
//Resets steeringScale to starting value when turn direction changes as a result of crossing over desired heading, or in heading deadband to prepare for next turn AFTER leaving deadband
if (!steeringActive || (turnedLeft[0] == 1 && headingDev > HEADING_DEADBAND) || (turnedRight[0] == 1 && headingDev < -HEA DING_DEADBAND)){
steeringScale[1] = steeringScale[0];
228
229
230  }
231
232 //Writes previous turn flag values to "old" array values
233 turnedLeft[1] = turnedLeft[0];
241 242
on't scale unless +/- yaw rate error is greater than deadband value AND we are not in the heading deadband
steeringScale[0] = 0; steeringScale[1] = 0;
234 turnedRight[1] = turnedRight[0]; 235
236 //Calculates new steeringScale value based on desired yaw rate and achieved delta yaw angle during previous control loop
237 //If yaw rate is within deadband, or enough loops haven't occurred before update, steeringScale[0] is unchanged
238 if (steerDelayCounter % NUM_LOOPS_BEFORE_SCALING_TURN == 0) { //If NUM_LOOPS_BEFORE_SCALING_TURN = 1, steering value is c
hanged every loop; if = 2, changed every 2 loops, and so on
239
240 if((fabs(achievedYawRate[0]-desiredYawRate[0]) > DESIRED_YAW_DEADBAND) && (fabs(headingDev) > HEADING_DEADBAND)){ //D


		//In the case of different signs for desired yaw rate vs. achieved yaw rate, increase servo pull:
244
if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUT OFF_DEG_SEC)){ //For yaw rate deviation less than cutoff threshold, use FINE scaling
245
246
247
248
steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT; }
249
250
251
252
253
steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT; }
254
if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0])){ //Increa se yaw rate, fine stepping
255 256 257
steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT;
258 259 260
steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT;
261
262
263
264
265
} }
266
if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0])){ //Increa se yaw rate, coarse stepping
267 268 269
steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT;
270 271 272
steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT;
else if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALI NG_CUTOFF_DEG_SEC)){ //For yaw rate deviation greater than cutoff threshold, use COARSE scaling
//In the case of desired and achieved yaw rate both positive or both negative:
else if(fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUTOFF_DEG_SEC){ //For yaw rate deviation less than cutoff threshold, use FINE scaling
}
else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0])){ //In
crease yaw rate, fine stepping
}
else{steeringScale[0] = steeringScale[1] - (FINE_SCALING_STEP_PERCENT * FINE_SCALING_UNWIND_GAIN); //Decrease
yaw rate, fine stepping
else if(fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALING_CUTOFF_DEG_SEC){ //For yaw rate deviation more than cutoff threshold, use COARSE scaling
273
274
275
276
277
278 }
279
280
281
282
283
284 }
285
286 if (steeringScale[0] < -1){
}
} }
}
else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0])){ //In
crease yaw rate, coarse stepping
}
else{steeringScale[0] = steeringScale[1] - (COARSE_SCALING_STEP_PERCENT * COARSE_SCALING_UNWIND_GAIN) ; //Dec
rease yaw rate, coarse stepping
//Keeps the system from driving servos past 100% pull
if (steeringScale[0] > 1){ steeringScale[0] = 1;
287
288  }
289
290
291 //servoOutPrcnt is the fraction of maximum servo pull for turning, 0 to 1
292 servoOutPrcnt[0]=steeringScale[0];
293 294
295 //DEBUG LINE
296 printf("\nThis loop's steering commands:\n steeringScale[0,1]: [%.6f, %6f]\n servoOutPrcnt[0,1]: [%.6f, %.6f]\n", steeri

gScale[0], steeringScale[1], servoOutPrcnt[0], servoOutPrcnt[1]);
305
306
307
308
309
310
311
312
313
314
if ((headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] < 0 )) //then turn left!
315
316
317
318
319
320
321
322
323
by
if (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0){//flag set for left turn, but ONLY for a turn not caused negative servo value
turnedLeft[0] = 1;
turnedRight[0] = 0;
324
325
326
327
328
329
330
331
332
333
else if ((headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0 ] < 0)) //then turn right!
334
335
336
337
338
339
340
341
342
343
344
345
346
347
348
349
350
351
352
353
354
355
if (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0){//flag set for left turn, but ONLY for a turn not caused by negative servo value
//Writes current pre-turn values to previous values for next loop
servoOutPrcnt[1] = servoOutPrcnt[0];
//Determines which direction to turn and commands the servos to steer, scaled by steeringScale //If steeringScale=1, full deflection is commanded (5.75 inch toggle line pull on parafoil)
{
servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe - (fabs(servoOutPrcnt[0])*SERVO_L_WINCH_MAX_TRAVEL*SERVO_L_WINCH_SCALE_FACTOR);
servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe; //For DEBUG, disable next line
//Servos_Update_All();
}
//DEBUG LINE BELOW
printf("\nSteering Left\n"); }
{
servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe + (fabs(servoOutPrcnt[0])*SERVO_R_WINCH_MAX_TRAVEL*SERVO_R_WINCH_SCALE_FACTOR);
else
servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe; //For DEBUG, disable next line
//Servos_Update_All();
turnedRight[0] = 1; turnedLeft[0] = 0; }
//DEBUG LINE BELOW
printf("\nSteering Right\n"); }
{ turnedLeft[0] = 0; turnedRight[0] = 0;
//DEBUG LINE BELOW
printf("\nServos Unchanged -- In Heading Deadband\n"); }
//Sets steeringActive flag true if steering occurred earlier this loop and //disables the reset of steering scale to 1 until straight flight resumes if (headingDev > -HEADING_DEADBAND && headingDev < HEADING_DEADBAND) {
steeringActive = 0;


356
} else{
357
358
359
360
361
362
363
364
365
366 }
367 else{
steeringActive = 1; }
//Counter for how many loops occur at minimum steer before steer output begins scaling //Intended to account for delay in heading change after initial steering input
if (steeringActive) { steerDelayCounter
+= 1; =0;
368
369  }
370
371
372 //DEBUG LINE
373 printf("\n\nValues for this loop:\n steerDelayCounter (value for next loop): %d\n headingDev from actual-->desired: %.3f\
steerDelayCounter
n achievedYawRate[0] (prev. loop to now): %.3f\n"
374 " desiredYawRate[0,1]: [%.3f, %.3f]\n steeringActive: %d\n",
375 steerDelayCounter, headingDev, achievedYawRate[0], desiredYawRate[0], desiredYawRate[1], steeringActive);
376 377
378 //Writes current values to previous values for next loop
379 desiredYawRate[1] = desiredYawRate[0];
380 headingAct[1] = headingAct[0];
381 headingTime[1] = headingTime[0];
382 achievedYawRate[1] = achievedYawRate[0];
383
384 //DEBUG LINE - comment out below for debug
385 //Led_Off(LED_RED); //Red LED blinks while control is in this routine (turned on at beginning of loop)
386
387
388  }
389
390
391
392
393
394 //********************************************************************//
395 //Function: landingRoutine
396 //
397 //Desc: Steers device proportionally in a circle around the target coordinate,
398 // with radius <= LANDING_RADIUS_THRESHOLD
399 //
400 //Receives: float headingDes
401 //
402 //Returns:
403 //
404 //CONSTANTS: LANDING_RADIUS_THRESHOLD, HEADING_DEADBAND,
405 // NUM_LOOPS_BEFORE_SCALING_TURN_LANDING, DESIRED_YAW_DEADBAND,
406 // LANDING_RADIUS, DEGREES_TO_RAD, SERVO_RIGHT_WINCH_3, SERVO_LEFT_WINCH_4,
407 // SERVO_R_WINCH_MAX_TRAVEL, SERVO_L_WINCH_MAX_TRAVEL, SERVO_R_WINCH_SCALE_FACTOR,
408 // SERVO_L_WINCH_SCALE_FACTOR, FINE_SCALING_UNWIND_GAIN, COARSE_SCALING_UNWIND_GAIN
409 //
410 //Globals: headingAct[2], servoOutPrcnt[2], desiredYawRate[2], steeringActive,
411 // steeringScale, achievedYawRate[2], steerDelayCounter, turnedLeft[2], turnedRight[2],
412 // gGPS.heading, gGPS.latitude, gGPS.longitude, gGPS.gndspeed, gGPS.TOW, headingTime[2]
413 //____________________________________________________________________//
414
415
416 void landingRoutine(float headingDes) { 417
418


419 //Local variables:
420 float headingDev; //deviation angle from actual heading to desired, degrees
421 int loopCtr; //loop counter for rewriting achievedYawRate array values to "older" positions in array
422
423 //DEBUG LINE - comment out below for debug
424 //Led_On(LED_RED); //Red LED is on steady in this routine
425 426
427 //Writes current GPS heading in degrees and GPS time-of-week in milliseconds from Monkey to arrays' 0 position
428 //headingAct[0] = gGPS.heading;
429 //headingTime[0] = gGPS.TOW; //NOTE: using TOW for headingTime will cause a momentary glitch when TOW reverts to 0 each w
eek
430 //DEBUG LINE
431 headingAct[0] = gGPSheading;
432 headingTime[0] = gGPSTOW;
433 434
435 //DEBUG LINES BELOW
436 printf(" Actual heading: %.3f\n", headingAct[0]);
437 printf("\n\n***Start landing routine***\n");
438
439 //Determines the flight heading deviation from actual to the desired heading assuming straight flight to determine requir
ed landing spiral direction
440 //Positive clockwise, negative counterclockwise; range -180 to 180 degrees
441 if ((headingDes-headingAct[0] >= -180) && (headingDes-headingAct[0] <= 180)){
442
443  }
444
445
446
447
448
449
450
451
headingDev = headingDes-headingAct[0];
else if (headingDes-headingAct[0] < -180){ headingDev = 360 + (headingDes-headingAct[0]); }
else{
headingDev = (headingDes-headingAct[0]) - 360; }
//Determines new desired heading tangent to a radius around target, and the direction of spiral depending on target being approached from right or left side
if (headingDev < 0){ //If true, we are approaching target to the right, and should spiral in a left/CCW pattern headingDes = headingDes + 90; //Causes parafoil to fly tangent to a CCW circle around target
452
453
454 }
455 else if (headingDev >= 0){ //If true, we are approaching target to the left, and should fly in a right/CW pattern
456 457 } 458
459 //Now re-determine headingDev for the new spiral-flight desired heading
460 //Determines the flight heading deviation from actual to the desired heading for landing spiral flight path
461 //Positive clockwise, negative counterclockwise; range -180 to 180 degrees
462 if ((headingDes-headingAct[0] >= -180) && (headingDes-headingAct[0] <= 180)){
463
464  }
465
466
467
468
469
470
471
472
473
474
475
476
477
478
headingDev = headingDes-headingAct[0];
headingDes = headingDes - 90;
else if (headingDes-headingAct[0] < -180){ headingDev = 360 + (headingDes-headingAct[0]); }
else{
headingDev = (headingDes-headingAct[0]) - 360; }
//DEBUG LINE
printf("\n Desired heading for landing spiral: %.3f\n", headingDes);
//Determines heading angle (yaw) change from previous loop to current loop and achievedYawRate in deg/s
//Positive clockwise, negative counterclockwise
//Delivers rate achieved between -180 to 180 deg/s
if (headingAct[0] != headingAct[1]){//only updates achievedYawRate when a new GPS heading is available; approx. 4 times/s ec


479
if ((headingAct[0]-headingAct[1] >= -180) && (headingAct[0]-headingAct[1] <= 180)){
505
506  }
507
508
509
510
511
512
513
514
//Resets steeringScale to starting value when turn direction changes as a result of crossing over desired heading, or in heading deadband to prepare for next turn AFTER leaving deadband
if (!steeringActive || (turnedLeft[0] == 1 && headingDev > HEADING_DEADBAND) || (turnedRight[0] == 1 && headingDev < -HEA DING_DEADBAND)){
515
516
517  }
518
519
520
521
522
523
524
525
526
527
steeringScale[0] = 0; steeringScale[1] = 0;
528 529
//Calculates new steeringScale value based on desired yaw rate and achieved delta yaw angle during previous control loop //If yaw rate is within deadband, or enough loops haven't occurred before update, steeringScale[0] is unchanged
if (steerDelayCounter % NUM_LOOPS_BEFORE_SCALING_TURN_LANDING == 0) { //If NUM_LOOPS_BEFORE_SCALING_TURN_LANDING = 1, ste ering value is changed every loop; if = 2, changed every 2 loops, and so on
530
531
532
533
if((fabs(achievedYawRate[0]-desiredYawRate[0]) > DESIRED_YAW_DEADBAND) && (fabs(headingDev) > HEADING_DEADBAND)){ //D on't scale unless +/- yaw rate error is greater than deadband value AND we are not in the heading deadband
534
535
536
537
steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT_LANDING; }
achievedYawRate[0] = (headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000);
480
481
482
483
484
485
486
487
488  }
489
490 //Returns desiredYawRate = fn(headingDev, LANDING_RADIUS, gGPS.gndspeed) as a rate (+/- deg/s), positive clockwise
491 //Calculates the yaw rate required to maintain a circular flight path of radius LANDING_RADIUS
}
else if (headingAct[0]-headingAct[1] < -180){
achievedYawRate[0] = (360 + headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000); }
else{
achievedYawRate[0] = (headingAct[0]-headingAct[1] - 360)/((headingTime[0]-headingTime[1])/1000); }
492
493 //desiredYawRate[0]= (headingDev/fabs(headingDev) * gGPS.gndspeed / LANDING_RADIUS) * RAD_TO_DEG;
494 //DEBUG LINE -- make sure to activate above line for Monkey!
495 desiredYawRate[0]= (headingDev/fabs(headingDev) * gGPSgndspeed / LANDING_RADIUS) * RAD_TO_DEG;
496 497
498 //Prevents desired yaw rate from exceeding 180 deg/s, plus some margin. Steering will fail if actual
499 //yaw rate exceeds 180 deg/s.
500 if (desiredYawRate[0] > 170){
501
502 }
503
504 else if (desiredYawRate[0] < -170){
desiredYawRate[0] = 170;
desiredYawRate[0] = -170;
//Writes steering scale value of previous loop to "old" array value
steeringScale[1] = steeringScale[0];
//Writes previous turn flag values to "old" array values
turnedLeft[1] = turnedLeft[0]; turnedRight[1] = turnedRight[0];
//In the case of different signs for desired yaw rate vs. achieved yaw rate, increase servo pull:
if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUT OFF_DEG_SEC_LANDING)){ //For yaw rate deviation less than cutoff threshold, use FINE scaling
else if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALI


														 G_CUTOFF_DEG_SEC_LANDING)){ //For yaw rate deviation greater than cutoff threshold, use coarse scaling steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT_LANDING;
}
543
if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0])){ //Increa se yaw rate, fine stepping
544 545 546
steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT_LANDING;
547 548 549
steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT_LANDING;
550
551
552
553
554
Decrease yaw rate, fine stepping
555
if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0])){ //Increa se yaw rate, coarse stepping
556 557 558
steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT_LANDING;
559 560 561
steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT_LANDING;
} }
//In the case of desired and achieved yaw rate both positive or both negative:
else if(fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUTOFF_DEG_SEC_LANDING){ //For yaw rate deviati on less than cutoff threshold, use FINE scaling
} }
}
else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0])){ //In
crease yaw rate, fine stepping
}
else{steeringScale[0] = steeringScale[1] - (FINE_SCALING_STEP_PERCENT_LANDING * FINE_SCALING_UNWIND_GAIN); //
else if(fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALING_CUTOFF_DEG_SEC_LANDING){ //For yaw rate deviat ion more than cutoff threshold, use COARSE scaling
}
else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0])){ //In
crease yaw rate, coarse stepping
}
else{steeringScale[0] = steeringScale[1] - (COARSE_SCALING_STEP_PERCENT_LANDING * COARSE_SCALING_UNWIND_GAIN)
; //Decrease yaw rate, coarse stepping }
562
563
564
565
566
567 }
568
569
570
571
572
573 }
574
575 if (steeringScale[0] < -1){
//Keeps the system from driving servos past 100% pull
if (steeringScale[0] > 1){ steeringScale[0] = 1;
576
577  }
578
579
580 //servoOutPrcnt is the fraction of maximum servo pull for turning, 0 to 1
581 servoOutPrcnt[0]=steeringScale[0];
582 583 584
585 //DEBUG LINE
586 printf("\nThis loop's steering commands:\n steeringScale[0,1]: [%.6f, %6f]\n servoOutPrcnt[0,1]: [%.6f, %.6f]\n", steeri
steeringScale[0] = -1;
ngScale[0], steeringScale[1], servoOutPrcnt[0], servoOutPrcnt[1]); 587
588 //Writes current pre-turn values to previous values for next loop
589 servoOutPrcnt[1] = servoOutPrcnt[0];
590 591


592
//Determines which direction to turn and commands the servos to steer, scaled by steeringScale //If steeringScale=1, full deflection is commanded (5.75 inch toggle line pull on parafoil)
593 594
if ((headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] < 0 )) //then turn left!
595
596
597
598
599
600
601
602
603
{
servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe - (fabs(servoOutPrcnt[0])*SERVO_L_WINCH_MAX_TRAVEL*SERVO_L_WINCH_SCALE_FACTOR);
604
605
606
607
608
609
610
611
turnedLeft[0] = 1; turnedRight[0] = 0; }
612
613
614
615
616
617
618
619
620
{
servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe + (fabs(servoOutPrcnt[0])*SERVO_R_WINCH_MAX_TRAVEL*SERVO_R_WINCH_SCALE_FACTOR);
621
622
623
624
625
626
627
628
629
630
631
632
633
634
635
636
637
638
639
640
641
642
643
644
645
646
647
648
649
650
651
turnedRight[0] = 1; turnedLeft[0] = 0; }
else
servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe; //For DEBUG, disable next line
//Servos_Update_All();
if (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0){//flag set for left turn, but ONLY for a turn not cause d by negative servo value
//DEBUG LINE BELOW
printf("\nSteering Left\n"); }
else if ((headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0 ] < 0)) //then turn right!
servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe; //For DEBUG, disable next line
//Servos_Update_All();
if (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0){//flag set for left turn, but ONLY for a turn not caused by negative servo value
//DEBUG LINE BELOW
printf("\nSteering Right\n"); }
{ turnedLeft[0] = 0; turnedRight[0] = 0;
//DEBUG LINE BELOW
printf("\nServos Unchanged -- In Heading Deadband\n"); }
//Sets steeringActive flag true if steering occurred earlier this loop and //disables the reset of steering scale to 1 until straight flight resumes if (headingDev > -HEADING_DEADBAND && headingDev < HEADING_DEADBAND) {
steeringActive = 0; }
else{
steeringActive = 1; }
//Counter for how many loops occur at minimum steer before steer output begins scaling //Intended to account for delay in heading change after initial steering input
if (steeringActive) {
steerDelayCounter += 1;

652 }
653 else{
steerDelayCounter = 0;
654
655  }
656
657
658 //DEBUG LINE
659 printf("\n\nValues for this loop:\n steerDelayCounter (value for next loop): %d\n headingDev from actual-->desired: %.3f\
n achievedYawRate[0] (prev. loop to now): %.3f\n"
660 " desiredYawRate[0,1]: [%.3f, %.3f]\n steeringActive: %d\n",
661 steerDelayCounter, headingDev, achievedYawRate[0], desiredYawRate[0], desiredYawRate[1], steeringActive);
662 663
664 //Writes current values to previous values for next loop
665 desiredYawRate[1] = desiredYawRate[0];
666 headingAct[1] = headingAct[0];
667 headingTime[1] = headingTime[0];
668 achievedYawRate[1] = achievedYawRate[0];
669
670
671  }
672
673
674
675
676 //********************************************************************//
677 //Function: landingFlare
678 //
679 //Desc: Performs a simple flare maneuver when parafoil system altitude
680 // is less than FLARE_HEIGHT above ground
681 //
682 //Receives:
683 //
684 //Returns:
685 //
686 //CONSTANTS: SERVO_RIGHT_WINCH_3, SERVO_LEFT_WINCH_4, SERVO_R_WINCH_MAX_TRAVEL,
687 // SERVO_L_WINCH_MAX_TRAVEL, SERVO_R_WINCH_SCALE_FACTOR,
688 // SERVO_L_WINCH_SCALE_FACTOR, FLARE_PRCNT
689 //
690 //Globals:
691 //____________________________________________________________________//
692
693
694 void landingFlare(void) { 695
696 //Sets both servos to a percentage of maximum pull to perform braking flare near landing
697 servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe +
698 (FLARE_PRCNT*SERVO_R_WINCH_MAX_TRAVEL*SERVO_R_WINCH_SCALE_FACTOR);
699
700 servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe -
701 (FLARE_PRCNT*SERVO_L_WINCH_MAX_TRAVEL*SERVO_L_WINCH_SCALE_FACTOR);
702
703 //For DEBUG, disable next line
704 //Servos_Update_All();
705
706 //DEBUG LINE
707 printf("\n\n***Initiate landing flare***\n Left Servo Value: %.3f\n Right Servo Value: %.3f\n", FLARE_PRCNT, FLARE_PR
CNT); 708
709
710 }
711
712
713 //End Josh's control code
