clc
clear
run('bwbaircraftfile.m')

%Step 0 - Init
state.W0 = 191876.81;
state.WF0 = 60000;
% CT = 0.376/3600;

% state.W0 = ac.Wo;
g = 32.2;

%Step 1 - Taxi & TO
state.W1 = state.W0*.998; %Raymer est. for taxi
state.WF1 = state.WF0-(state.W0-state.W1);

%Step 2 - Climb
Vclimb = 1.46667*360; %fps
mission.v_cruise = Vclimb;
dt = 30; %seconds
Fuelburn = 0;
h = 0;
time = 0;
mission.M = 0.5;
cruise_alt = 40000;
while h < cruise_alt %change mission.M to linearly scale from 0.4 to cruise as increase
    mission.altitude = h;
    mission.a = speedofsound(h);
    mission.viscocity = airviscocity(h);
%     mission.M = mission.v_cruise/mission.a;
    mission.v_cruise = mission.M*mission.a; %ft/sec
    P = airpressure(h);
    rho = airdensity(h);
    mission.rho = rho;
    [CD,CL] = dragBWB(ac,mission,state.W1-Fuelburn);
%     Vclimb = 1.46667*360+(761-528)/37000;
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD);
    Ta = thrust737(h)*1.3;
    Fuelburn = Fuelburn + H2burn(Ta,mission.v_cruise,dt); %in lbs
    hdot = mission.v_cruise*(Ta-D)/(state.W1-Fuelburn); %in m/s
    h = h + hdot*dt;
    time = time+dt;
    if h > 10000
        mission.M = 0.5;
    end
    if h > 20000
        mission.M = 0.6;
    end
    if h > 30000
        mission.M = 0.7;
    end
    if h > 35000
        mission.M = 0.774;
    end
end
state.WF2 = state.WF1-Fuelburn;
state.W2 = state.W1-Fuelburn;
state.t2 = time;
climb_distance = Vclimb*time/6076; %distance travelled in NM
% climb_distance = 0;

%Step 3 - Cruise
clear mission
mission.altitude = cruise_alt; %ft
mission.M = 0.774;
mission.a = speedofsound(mission.altitude); %fps
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
mission.v_cruise = mission.M*mission.a; %fps
R = 2935-climb_distance-mission.altitude/1000*3; %NM, rule of thumb for descent
% CT = CTest(mission);
% CT_cruise = .061/3
% % CT = 0.5/3600; %from https://booksite.elsevier.com/9780340741528/appendices/data-b/table-3/default.htm
[CD,CL] = dragBWB(ac,mission,state.W2);
CT = 5.14286E-05;
state.W3 = (sqrt(state.W2)- R*6076/(2/CT*sqrt(CL)/CD*sqrt(2/(mission.rho*ac.wing.S))))^2;
% CP = 5.14286E-05;
% state.W3 = state.W2/(exp(R/(CP/g*CL/CD)));
state.WF3 = state.WF2 - (state.W2-state.W3);
state.t3 = R/(mission.v_cruise*0.592484);

%stop here
[CD,CL] = dragBWB(ac,mission,state.W3);
Treq = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*(CD);
hdot =  mission.v_cruise*(Ta-Treq)/(state.W3); %in m/s