% d12 rocket engine data
rawd12 = [0.049 2.569
0.116 9.369
0.184 17.275
0.237 24.258
0.282 29.730
0.297 27.010
0.311 22.589
0.322 17.990
0.348 14.126
0.386 12.099
0.442 10.808
0.546 9.876
0.718 9.306
0.879 9.105
1.066 8.901
1.257 8.698
1.436 8.310
1.590 8.294
1.612 4.613
1.650 0.000];
Thrust_timed12 = (rawd12(:,1))';
ThrustNtd12 = (rawd12(:,2))';

Total_impulse_d12 = 17;
Propel_weight_d12 = 0.0221;
Engine_weight_d12 = 0.058;

%C6 rocket engine
thrustprofileC6 = [
0.031 0.946
0.092 4.826
0.139 9.936
0.192 14.090
0.209 11.446
0.231 7.381
0.248 6.151
0.292 5.489
0.370 4.921
0.475 4.448
0.671 4.258
0.702 4.542
0.723 4.164
0.850 4.448
1.063 4.353
1.211 4.353
1.242 4.069
1.303 4.258
1.468 4.353
1.656 4.448
1.821 4.448
1.834 2.933
1.847 1.325
1.860 0.000];
Thrust_timeC6 = (thrustprofileC6(:,1))';
ThrustNtC6 = (thrustprofileC6(:,2))';

%rocket properties
m = 276.72e-3;
Ixx = 2.156e-3;
com = 96e-3;
g = 9.81;
d= 0.081;
D = 0.077;
Ar = pi*D*D/4;
cp = 1e-3;
l = 0.32;

%aero properties
rho = 1.225; 
Cd = 0.23;