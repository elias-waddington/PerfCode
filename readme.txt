Light documentation if COVID-19 kills me.

What I'm currently working on is the following:

1. "Aircraft File" (A)
2. "Drag File" (D)
3. Performance blocks (P)


At the start of a sim, "A" is loaded to create (currently) 2 structures:
ac - aircraft file
mission - flight conditions

I suppose these should be broken out to two individual ones, neh?

"D" is a function that calls (ac,mission) to create CD, CL

"P" is a family of functions whose inputs are what you'd like to do, and output the fuel, time, distance it would take to do that. 

I think all of this together should give a pretty good picture of how things can work effectively.

As of 3/19, status is:
- "A" file is partially completed. Would like to make it compatible with a new weight-estimation code to run fresh, too
- "D" code updated from previous work
- "P" codes unfinished


Anytime you write a new mission you should start it with "clear mission" and then rewrite the mission parameters which are:
mission.M = Mach no.
mission.a = Speed of sound (use speedofsound.m)
mission.altitude = 15000 (in ft)
mission.rho = airdensity(mission.altitude)
mission.viscocity = airviscocity(mission.altitude)
mission.v_cruise = in ft/sec - this is to be the *true airspeed* of the aircraft. Calculable from M and a


importfig1.m -> use 'fig1.csv' to export AR, CLdes
CLdes = interpl(AR,CLdes,7.3) - 7.3 being the AR of the aircraft you want to find
CLdes is that big ugly thing of CLdes*sqrt(AR)*etc that you can then solve for CLdes and follow the excel files
This is true for all 'fig' .csv and .dat files for the delta method.

List of supporting functions:
airdensity.m
airpressure.m
airviscocity.m
Cdc_find.m
H2CT.m
importfig[1,3,4C,4d,10,13].m
interp3D_V003.m
RLS_find.m
Rwf_find.m
speedatmos.m
speedofsound.m
thrustBWB.m

