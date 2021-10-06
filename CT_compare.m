clc
clear
run('bwbaircraftfile.m')

mission.altitude = 35000; %ft
mission.M = 0.774;
mission.a = speedofsound(mission.altitude); %fps
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
mission.v_cruise = mission.M*mission.a; %fps
% R = 2935-climb_distance-cruise_alt/1000*3; %NM, rule of thumb for descent
R = 2600; %nm
state.W2 = 180000;
state.WF2 = 24000;

i = 2;
dt = 60;
Weight(1) = state.W2;
distance = 0;
while  distance < R*6076
    [CD,CL] = dragBWB(ac,mission,Weight(i-1));
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD;
    CT = H2CT(Tr,mission.v_cruise);
    CDgraph(i-1) = CD;
    CLgraph(i-1) = CL;
    CTgraph(i-1) = CT;
    Weight(i) = Weight(i-1)-CT*Tr*dt;
    
    distance = distance + dt*mission.v_cruise;
%     state.t3 = R/(mission.v_cruise*0.592484);
    i = i+1;
end
CT = H2CT(Tr,mission.v_cruise);
[CD,CL] = dragBWB(ac,mission,state.W2);
state.W3 = (sqrt(state.W2)- R*6076/(2/CT*sqrt(CL)/CD*sqrt(2/(mission.rho*ac.wing.S))))^2;
state.WF3 = state.WF2 - (state.W2-state.W3);

plot(CTgraph)
state.W3-Weight(end)