% This flight code is made to find the remaining fuel given a payload weight
clear all


clc

verbose = 0; %toggles mass data output
dave_numbers = 0; %toggles profile, CF drag terms
drag_breakout = 0; %toggles o, i, w drag terms (1 is on, 0 is off)

N_sweep = 30;
min_iterations = 8;
summarytable_out = array2table(zeros(1,14));
AR_count = 0;
S_count = 1;
input_count = 1;

% ! LOAD AIRCRAFT AND WEIGHTS FILES
load('V03_ac.mat')
load('V03_weight.mat')
CLto = 2.2;
    
%Fuel Cell Type
FCmap_drag = allloadin_V003('FCmapR1_LD_drag.dat','n');
FCmap_eff = allloadin_V003('FCmapR1_LD_eff.dat','n');
load_LD1_LowDrag; 
% FCmap_drag = allloadin_V003('FCmapR2_Drag_current.dat','n');
% FCmap_eff = allloadin_V003('FCmapR2_Eff_current.dat','n');
% load_R2_current_LD;
% FCmap_drag = allloadin_V003('FCmapR2_Drag_currentLTD.dat','n');
% FCmap_eff = allloadin_V003('FCmapR2_Eff_currentLTD.dat','n');
% load_R2_current_LD_LTD;

%Section being run
% AR_sweep = linspace(7,12,3);
% S_sweep = linspace(1600,3500,5);
% % input_sweep = linspace(2,2.4,3);
% input_sweep = 3;
AR_sweep = 1;
% S_sweep = linspace(0.5,0.8,31); %Mach sweep
S_sweep = 0.773;
input_sweep = linspace(0,50000,11); %Payload Sweep
% input_sweep = 35000;

%Initial Count
file_out = 0;
for outer_loop = 1:length(S_sweep) % ! UPDATE

% clear ac
% clear weight
% run('aircraftfile_V02.m')
ac.FCsize = MP_Wlb_mat(1,1); %maximum power output at SL, M = 0.25
% ac.FCsize_toc = 995;

%This section for setting up sweeps

file_out = file_out+1

% AR_count = AR_count + 1;
% if AR_count > length(AR_sweep)
%     AR_count = 1;
    S_count = S_count+1;
    if S_count > length(S_sweep)
        S_count = 1;
%         input_count = input_count + 1;
    end
% end

[REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,N_sweep); %CDgen_V2 is the old, inefficient for debugging
no_of_iterations = 0;
%fixed AR for debugging

has_looped = 0;
input_count = 0;

for inner_loop = 1:length(input_sweep)
    
    if has_looped == 1
        file_out = file_out + 1;
        no_of_iterations = 0;
%         clear weight
        deltaFuel = 100;
        input_count = input_count+1;
    else
        has_looped = 1;
%         clear weight
        deltaFuel = 100;
        input_count = input_count+1;
    end

    disp(file_out)
%     disp(AR_sweep(AR_count))
%     disp(S_sweep(S_count))
    disp(input_sweep(input_count))
%     ac.FCsize = input_sweep(input_count);
% press_ratio = input_sweep(input_count);
% CLto = input_sweep(input_count);

is_borked = 0;

%% Below here from performance code
%% Step 0 - Init

dt = 60; %seconds
g = 32.2;
deltaFuel = 1000;
mission_length = 10*3600/dt;

% ac.weight0 = 225000; %initial weight guess
weight.Wpay = input_sweep(input_count); %payload for this mission
state.WF0 = weight.fuelsys/0.65;

% Cruise parameters
% LRC_Mach = 0.773;
LRC_Mach = S_sweep(S_count);
LRC_range = 2935; %NM
max_TOFL = 8200; %ft

while abs(deltaFuel) > 25
    
hgraph = zeros(1,mission_length);
Tgraph = zeros(1,mission_length);
vgraph = zeros(1,mission_length);
tgraph = zeros(1,mission_length);
CDgraph = zeros(1,mission_length);
CLgraph = zeros(1,mission_length);
CDograph = zeros(1,mission_length);
CDigraph = zeros(1,mission_length);
CDwgraph = zeros(1,mission_length);
CDowinggraph = zeros(1,mission_length);
CDofusgraph = zeros(1,mission_length);
Wgraph = zeros(1,mission_length);
Pgraph = zeros(1,mission_length);
MPgraph = zeros(1,mission_length);
PPgraph = zeros(1,mission_length);
Mgraph = zeros(1,mission_length);
hdotgraph = zeros(1,mission_length);
Regraph = zeros(1,mission_length);
Cfgraph = zeros(1,mission_length);
CDofusegraph = zeros(1,mission_length);
Pagraph = zeros(1,mission_length);
Tagraph = zeros(1,mission_length);
Trgraph = zeros(1,mission_length);
Prgraph = zeros(1,mission_length);
HDgraph = zeros(1,mission_length);
tic
ac.WF = state.WF0;

fprintf('WF = %3.1d \n',ac.WF)

% [ac,weight] = weight_estimate(ac,weight);

fprintf('FC = %3.1d \n',weight.fuelcell)
fprintf('Batt = %3.1d \n \n',weight.battery)

state.W0 = weight.WE+ac.WF+weight.Wpay;


% size_power_cbrk = 0;

%% Step 1 - Taxi & TO
[Taxi_Power,Taxi_Thrust] = taxi_estimator(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);
Taxi_Power = Taxi_Power + HotelLoads(0.05,1,1);
FCeff = interp3D_V003(FCmap_eff,1,Taxi_Power/weight.fuelcell,0.26,1,'n');
FB_taxi = H2CT(Taxi_Thrust,36.66,0,weight,FCeff)*60*Taxi_Thrust*35;

state.W1 = state.W0*.998; %Raymer est. for TTO. NEEDS ADDRESSING - actual method breaks + is way less than we're estimating
state.WF1 = state.WF0-(state.W0-state.W1);
counter = 0;

[MTOW_LFL] = landinglength_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);

% CLto = 2;
press_ratio = 1;
static_scaling = 0.775; 
cruise_scaling = 0.45; %see spreadsheet, this assumed a FanEff of 0.752 at cruise at max power

TOFL = takeoff_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);

%% Step 2 - Climb

Fuelburn = 0;
h = 0;
time = 0;
mission.M = 0.25;
% cruise_alt = 36000;
cruise_alt = 37000;
counter = counter+1;
climb_distance = 0;

mission.a = speedofsound(h);
mission.v_cruise = mission.a*mission.M;
% mission.M = mission.v_cruise/mission.a;
energy_generated_so_far = 0;
energy_needed_so_far = 0;

while h < cruise_alt %change mission.M to linearly scale from 0.4 to cruise as increase
    if h > -1 && h < 10000
        v20k = (-speedofsound(0)*0.25+speedofsound(10000)*0.35)/10000;
        mission.v_cruise = speedofsound(0)*0.25 + v20k*(h);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 4000 - 500*((h)/10000);
%         hdot = hdot/60;
    end
    if h >= 10000 && h < 20000
        v20k = (speedofsound(20000)*0.45-speedofsound(10000)*0.35)/10000;
        mission.v_cruise = speedofsound(10000)*0.35 + v20k*(h-10000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 3500 - 1000*((h-10000)/10000);
%         hdot = hdot/60;
    end
    if h >= 20000 && h < 30000
        v20k = (speedofsound(30000)*0.55-speedofsound(20000)*0.45)/10000;
        mission.v_cruise = speedofsound(20000)*0.45 + v20k*(h-20000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 2500 - 1500*((h-20000)/10000);
%         hdot = hdot/60;
    end
    if h >= 30000
        v20k = (speedofsound(cruise_alt)*LRC_Mach-speedofsound(30000)*0.55)/(cruise_alt-30000);
        mission.v_cruise = speedofsound(30000)*0.55 + v20k*(h-30000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 1000 - ((h-30000)/10000)*1000; %set 30k + 10k > cruise_alt
%         hdot = hdot/60;
    end
    mission.altitude = h;
    mission.viscocity = airviscocity(h);
%     mission.M = mission.v_cruise/mission.a;
%     mission.v_cruise = mission.M*mission.a; %ft/sec
    P = airpressure(h);
    mission.rho = airdensity(h);
    rho = airdensity(h);
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W1-Fuelburn);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    HotelP = HotelLoads(mission.M,h,1);
    CL = (state.W1-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    Mgraph(counter) = mission.M;
%     CDprofilegraph = [CDprofilegraph,CDprofilegraph];
%     Vclimb = 1.46667*360+(761-528)/37000;
%     HexDrag = interp3D_V003(FCmap_drag,1,weight.fuelcell,mission.M,h,'n')*ac.fcweight;
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag; 
    Ta = (FCmaxpower(weight,h,mission.M)-HotelP)/1.3558/mission.v_cruise*FC2fan(h); %watts to lbs-force available
    hdot = mission.v_cruise*(Ta-D)/(state.W1-Fuelburn); %in m/s
    if hdot < 100/60
        disp('Cannot Climb Enough! HDOT < 100!')
    end
    hdot_cbrk = 0;
    if h > -1 && h < 10000
        hdot_compare = (4000 - 500*((h)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
    if h >= 10000 && h < 20000
        hdot_compare = (3500 - 1000*((h-10000)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
    if h >= 20000 
        hdot_compare = (2500 - 100*((h-20000)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
%     if h >= 30000
%         hdot_compare = (1000 - ((h-30000)/10000)*1000)/60; %set 30k + 10k > cruise_alt
%         if hdot > hdot_compare
%             hdot = hdot_compare;
%             Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
%             hdot_cbrk = 1;
%         end
%         hdot = hdot/60;
%     end
    if hdot_cbrk == 0
        Tr = Ta;
    end
    
    FCeff = interp2(MP_h_rng,MP_M_rng,MP_Eff_mat,h,mission.M);
    Fuelburn = Fuelburn + H2CT(Tr,mission.v_cruise,h,weight,FCeff)*dt*Tr; %in lbs
    
    hgraph(counter) = h;
   
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    climb_distance = climb_distance + mission.v_cruise*dt;
    time = time+dt;
    tgraph(counter) = time;
    Wgraph(counter) = state.W1 - Fuelburn;
    PPgraph(counter) = Tr*mission.v_cruise*1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h)+HotelP;
    hdotgraph(counter) = hdot*dt;
    Regraph(counter) = Re;
    Pagraph(counter) = FCmaxpower(weight,h,mission.M);
    Tagraph(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = D;
    Prgraph(counter) = D/FC2fan(h)*1.3558*mission.v_cruise;
    HDgraph(counter) = HexDrag;
    energy_generated_so_far = energy_generated_so_far + FCmaxpower(weight,h,mission.M)*dt;
    energy_needed_so_far = energy_needed_so_far + Pgraph(counter)*dt;
     h = h + hdot*dt;

    counter = counter + 1;
end
state.WF2 = state.WF1-Fuelburn;
state.W2 = state.W1-Fuelburn;
state.t2 = time;

dataout.time_to_climb = state.t2/dt;
dataout.climb_dist = climb_distance/6076;

if climb_distance/6076 > 600
    is_borked = 1;
    disp('BORK due to climb distance')
    break
end

TA_climb = [Tgraph(1)*1.01,Tgraph(1:counter-1)];
TA_climb_alt = [hgraph(1:counter-1),37000];
dataout.TR_V35 = max(Tgraph);
% Vclimb = 1.46667*360; %fps
% climb_distance = Vclimb*time/6076; %distance travelled in NM
% clear Vclimb
% climb_distance = 0;

% outputtable = table(tgraph',Tgraph',vgraph',Mgraph',hgraph',CDgraph',CLgraph',CDograph',CDigraph',CDwgraph', CDowinggraph', Wgraph');
% outputtable.Properties.VariableNames = {'Time, min.' 'Thrust, lbf' 'VTAS, fps' 'Mach No.' 'Altitude, ft' 'Total Aircraft CD' 'Total Aircraft CL' 'Profile Drag' 'Induced Drag' 'Wave Drag' 'Wing Alone Profile Drag' 'Weight, Lbs'};
% writetable(outputtable,'data_table.csv');

%% Step 3 - Cruise
clear mission
mission.altitude = cruise_alt; %ft
mission.M = LRC_Mach;
mission.a = speedofsound(mission.altitude); %fps
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
mission.v_cruise = mission.M*mission.a; %fps
cruise_spd = mission.v_cruise;
qbar = 0.5*mission.v_cruise^2*mission.rho;
R = LRC_range-climb_distance/6076-cruise_alt/1000*3; %NM, rule of thumb for descent
Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
Target_Distance = R*6076; % range in feet
Fuelburn = 0;
Distance_Flown = 0;
size_power_cbrk = false;
% climb_distance/6076

while Distance_Flown < Target_Distance
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W2-Fuelburn);


    CL = (state.W2-Fuelburn)/qbar/ac.wing.S;
    
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    if HexDrag > 100000
%         HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
          HexDrag = HDgraph(counter-1)*0.25;
    end
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD+HexDrag;
%     CT = H2CT(Tr,mission.v_cruise,h);
%     FCeff = interp2(MP_h_rng,MP_M_rng,MP_Eff_mat,,mission.altitude);
    FCeff = interp3D_V003(FCmap_eff,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n');
    Fuelburn = Fuelburn + H2CT(Tr,mission.v_cruise,h,weight,FCeff)*dt*Tr;
    HotelP = HotelLoads(mission.M,mission.altitude,2);
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    hgraph(counter) = mission.altitude;
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    hdotgraph(counter) = 0;
    time = time + dt;
    tgraph(counter) = time;
    Distance_Flown = Distance_Flown + mission.v_cruise * dt ;
    Wgraph(counter) = state.W2-Fuelburn;
    PPgraph(counter) = Tr*mission.v_cruise* 1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h) + HotelP;
    HDgraph(counter) = HexDrag;
    Mgraph(counter) = mission.M;
    Regraph(counter) = Re;
    
    Pagraph(counter) = FCmaxpower(weight,mission.altitude,mission.M);
    Tagraph(counter) = FCmaxpower(weight,mission.altitude,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = Tr;
    Prgraph(counter) = Tr/FC2fan(h)*1.3558*mission.v_cruise;
    
    if ~size_power_cbrk
        check_power = calcPower(Tr,mission.v_cruise,h);
        if check_power > ac.size.sizing_power
            ac.size.sizing_power = check_power;
        end
        energy_generated_so_far = dt*counter*ac.size.sizing_power;
%         ac.size.battery_energy = energy_needed_so_far-energy_generated_so_far;
%         baten = ac.size.battery_energy
        baten = 0; % set to 0 for assumption, should calculate, but should always have enough power from FC no matter what
        size_power_cbrk = true;
%         ac.size.max_power = Tgraph(1)*vgraph(1)*1.3558/(0.7*0.95*0.995);
        dataout.TR_cruise = Tr;
    end
    counter = counter + 1;


end

%     state.W3_old = (sqrt(state.W2)- R*6076/(2/CT*sqrt(CL)/CD*sqrt(2/(mission.rho*ac.wing.S))))^2;
%     state.WF3_old = state.WF2 - (state.W2-state.W3_old);
%     state.t3_old = R/(mission.v_cruise*0.592484);
    state.WF3 = state.WF2-Fuelburn;
    state.W3 = state.W2-Fuelburn;
	state.t3 = time;

%% Step 4 - Descent
Fuelburn = 0;
h = cruise_alt;
mission.M = LRC_Mach;
hdot = 1000/60; %fps
while h > 1
    if h >= 0 && h < 10000
        mission.v_cruise = 421.95 ;% - (10000-h)/10000*(185.66);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         v20k = (-speedofsound(0)*0.25+speedofsound(10000)*0.35)/10000;
%         mission.v_cruise = speedofsound(0)*0.25 + v20k*(h);
        idle_thrust_percent = 0.1-h/10000*0.1;
    end
    if h > 10000 && h < 25000
%         hdot = 1500/60; %fps
        mission.v_cruise = cruise_spd - (25000-h)/15000*(327.46);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
        idle_thrust_percent = 0.1;
    end
    if h >= 25000 
%         hdot = 1000/60; %fps
        mission.v_cruise = cruise_spd;
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
        idle_thrust_percent = 0.1;
    end
    mission.altitude = h;
%     mission.a = speedofsound(h);
    mission.viscocity = airviscocity(h);    
%     mission.M = mission.v_cruise/mission.a;
    HotelP = HotelLoads(mission.M,h,3);
    P = airpressure(h);
    rho = airdensity(h);
    mission.rho = rho;
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W3-Fuelburn);
    
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W3-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
%     HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    HexDrag = 100; %lbs, assuming this now because this will have to be fixed with HOTEL LOADS
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag;
    Tr = idle_thrust_percent*interp1(TA_climb_alt,TA_climb,h);

%     Tr = D - hdot*(state.W3-Fuelburn)/mission.v_cruise;
    hdot = -mission.v_cruise*(Tr-D)/(state.W3-Fuelburn);
    FCeff = interp3D_V003(FCmap_eff,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n');
    Fuelburn = Fuelburn + H2CT(Tr,mission.v_cruise,h,weight,FCeff)*dt*Tr; %in lbs
    h = h - hdot*dt;
    if h < 0
        h = 0;
    end

    time = time+dt;
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    hgraph(counter) = mission.altitude;
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    tgraph(counter) = time;
    Wgraph(counter) = state.W3-Fuelburn;
    Mgraph(counter) = mission.M;
    PPgraph(counter) = Tr*mission.v_cruise* 1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h) + HotelP;
    HDgraph(counter) = HexDrag;
    hdotgraph(counter) = -hdot*dt;
    Regraph(counter) = Re;
    
    Pagraph(counter) = FCmaxpower(weight,h,mission.M);
    Tagraph(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = D;
    Prgraph(counter) = D/FC2fan(h)*1.3558*mission.v_cruise;
    
    if counter == 398
        thing = 1;
    end
    
    counter = counter + 1;
end

state.W4 = state.W3-Fuelburn;
state.WF4 = state.WF3 - Fuelburn;
state.t4 = time; %descent_distance/(mission.v_cruise*0.592484);

% outputtable = table(tgraph',Tgraph',vgraph',Mgraph',hgraph',CDgraph',CLgraph',CDograph',CDigraph',CDwgraph', CDowinggraph', Wgraph');
% outputtable.Properties.VariableNames = {'Time, min.' 'Thrust, lbf' 'VTAS, fps' 'Mach No.' 'Altitude, ft' 'Total Aircraft CD' 'Total Aircraft CL' 'Profile Drag' 'Induced Drag' 'Wave Drag' 'Wing Alone Profile Drag' 'Weight, Lbs'};
% writetable(outputtable,'data_table.csv');

%% Step 5 - Abort Climb Out
clear mission
Fuelburn = 0;
h = 0;
% time = 0;
alt_altitude = 15000;
mission.M = 0.25; %climb mach
climb_distance = 0;
while h < alt_altitude
    if h > -1 && h < 10000
        v20k = (-speedofsound(0)*0.25+speedofsound(10000)*0.35)/10000;
        mission.v_cruise = speedofsound(0)*0.25 + v20k*(h);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 4000 - 500*((h)/10000);
%         hdot = hdot/60;
    end
    if h >= 10000 && h < 20000
        v20k = (speedofsound(20000)*0.45-speedofsound(10000)*0.35)/10000;
        mission.v_cruise = speedofsound(10000)*0.35 + v20k*(h-10000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 3500 - 1000*((h-10000)/10000);
%         hdot = hdot/60;
    end
    if h >= 20000 && h < 30000
        v20k = (speedofsound(30000)*0.55-speedofsound(20000)*0.45)/10000;
        mission.v_cruise = speedofsound(20000)*0.45 + v20k*(h-20000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 2500 - 1500*((h-20000)/10000);
%         hdot = hdot/60;
    end
    if h >= 30000
        v20k = (speedofsound(cruise_alt)*LRC_Mach-speedofsound(30000)*0.55)/(cruise_alt-30000);
        mission.v_cruise = speedofsound(30000)*0.55 + v20k*(h-30000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         hdot = 1000 - ((h-30000)/10000); %set 30k + 10k > cruise_alt
%         hdot = hdot/60;
    end
    mission.altitude = h;
    mission.a = speedofsound(h);
    mission.viscocity = airviscocity(h);
    mission.v_cruise = mission.M*mission.a; %ft/sec
    HotelP = HotelLoads(mission.M,h,1);
    P = airpressure(h);
    rho = airdensity(h);
    mission.rho = rho;
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W4-Fuelburn);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W4-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag;
%     Ta = thrustBWB(h);
%     Ta = D + hdot*(state.W4-Fuelburn)/mission.v_cruise;
    Ta = (FCmaxpower(weight,h,mission.M)-HotelP)/1.3558/mission.v_cruise*FC2fan(h);
    hdot = mission.v_cruise*(Ta-D)/(state.W4-Fuelburn); %in m/s
    hdot_cbrk = 0;
    if h > -1 && h < 10000
        hdot_compare = (4000 - 500*((h)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
    if h >= 10000 && h < 20000
        hdot_compare = (3500 - 1000*((h-10000)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
    if hdot_cbrk == 0
        Tr = Ta;
    end
    
    FCeff = interp2(MP_h_rng,MP_M_rng,MP_Eff_mat,h,mission.M);
    Fuelburn = Fuelburn + H2CT(Tr,mission.v_cruise,h,weight,FCeff)*dt*Tr; %in lbs
    
    
    hgraph(counter) = h;
    h = h + hdot*dt;
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    climb_distance = climb_distance + mission.v_cruise*dt;
    time = time+dt;
    tgraph(counter) = time;
    Wgraph(counter) = state.W4 - Fuelburn;
    Mgraph(counter) = mission.M;
    PPgraph(counter) = Tr*mission.v_cruise* 1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h)+HotelP;
    HDgraph(counter) = HexDrag;
    hdotgraph(counter) = hdot*dt;
    Regraph(counter) = Re;
    
    Pagraph(counter) = FCmaxpower(weight,h,mission.M);
    Tagraph(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = D;
    Prgraph(counter) = D/FC2fan(h)*1.3558*mission.v_cruise;
    
    counter = counter + 1;
end
% Vclimb = 1.46667*360; %fps
% climb_distance = Vclimb*time/6076; %distance travelled in NM
state.WF5 = state.WF4-Fuelburn;
state.W5 = state.W4-Fuelburn;
state.t5 = time;

%% Step 6 - Cruise to Alternate
clear mission
mission.M = 0.569;
mission.altitude = 15000;
descent_distance = mission.altitude/1000*3;
mission.a = speedofsound(mission.altitude);
R_alt = 200-climb_distance/6076-descent_distance; %NM
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
mission.v_cruise = mission.M*mission.a; %fps
Target_Distance = R_alt*6076;
Distance_Flown = 0;
Fuelburn = 0;

while Distance_Flown < Target_Distance
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W5-Fuelburn);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W5-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    if HexDrag > 100000
        HexDrag = HDgraph(counter-1)*0.25;
    end
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD + HexDrag;
%     CT = H2CT(Tr,mission.v_cruise,h);
    FCeff = interp3D_V003(FCmap_eff,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n');
    Fuelburn = Fuelburn + H2CT(Ta,mission.v_cruise,h,weight,FCeff)*dt*Tr;
    HotelP = HotelLoads(mission.M,mission.altitude,2);
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    hgraph(counter) = mission.altitude;
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    time = time + dt;
    tgraph(counter) = time;
    Distance_Flown = Distance_Flown + mission.v_cruise * dt ;
    Wgraph(counter) = state.W5 - Fuelburn;
    PPgraph(counter) = Tr*mission.v_cruise* 1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h)+HotelP;
    HDgraph(counter) = HexDrag;
    Mgraph(counter) = mission.M;
    hdotgraph(counter) = 0;
    Regraph(counter) = Re;
    
    Pagraph(counter) = FCmaxpower(weight,h,mission.M);
    Tagraph(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = Tr;
    Prgraph(counter) = Tr/FC2fan(h)*1.3558*mission.v_cruise;
    
    counter = counter + 1;

end

%     state.W6_old = (sqrt(state.W5)- R*6076/(2/CT*sqrt(CL)/CD*sqrt(2/(mission.rho*ac.wing.S))))^2;
%     state.WF6_old = state.WF5 - (state.W5-state.W6_old);
%     state.t6_old = R/(mission.v_cruise*0.592484);
    state.WF6 = state.WF5-Fuelburn;
    state.W6 = state.W5-Fuelburn;
	state.t6 = time;

%% Step 7 - 30 minute loiter
E_alt = 0.5; %hrs
% state.W7_old = state.W6/(exp(E_alt*3600*CT/(CL/CD)));
% state.WF7_old = state.WF6 - (state.W6-state.W7_old);
% state.t7_old = E_alt;
Fuelburn = 0;
loiter_time = 0;

while loiter_time < E_alt*60*60+1
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W6-Fuelburn);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W6-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    if HexDrag > 90000
        HexDrag = HDgraph(counter-1)*0.25;
    end
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD + HexDrag;
%     CT = H2CT(Tr,mission.v_cruise,h);
    FCeff = interp3D_V003(FCmap_eff,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n');
    Fuelburn = Fuelburn + H2CT(Ta,mission.v_cruise,h,weight,FCeff)*dt*Tr;
    HotelP = HotelLoads(mission.M,mission.altitude,4);
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    hgraph(counter) = mission.altitude;
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    time = time + dt;
    loiter_time = loiter_time + dt;
    tgraph(counter) = time;
    Wgraph(counter) = state.W6 - Fuelburn;
    Mgraph(counter) = mission.M;
    PPgraph(counter) = Tr*mission.v_cruise* 1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h)+HotelP;
    HDgraph(counter) = HexDrag;
    hdotgraph(counter) = 0;
%     Distance_Flown = Distance_Flown + mission.v_cruise * dt ;
    Regraph(counter) = Re;
    
    Pagraph(counter) = FCmaxpower(weight,h,mission.M);
    Tagraph(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = Tr;
    Prgraph(counter) = Tr/FC2fan(h)*1.3558*mission.v_cruise;
    
    counter = counter + 1;

end

state.W7 = state.W6 - Fuelburn;
state.WF7 = state.WF6 - Fuelburn;
satate.t7 = time;

%% Step 8 - Descent and Landing
Fuelburn = 0;
h = alt_altitude;
% time = 0;
mission.M = 0.5;
descent_angle = 3; %deg
descent_angle = deg2rad(descent_angle);
% hdot = 1750/60; %ft/sec
while h > 0
    if h > -1 && h < 10000
%         hdot = 1500/60 - 750/60*(10000-h)/10000; %fps
        mission.v_cruise = 421.95 ; %- (10000-h)/10000*(185.66);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
        idle_thrust_percent = 0.1-h/10000*0.1;
    end
    if h > 10000 && h < 25000
%         hdot = 1500/60; %fps
        mission.v_cruise = cruise_spd - (25000-h)/15000*(327.46);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
        idle_thrust_percent = 0.1;
    end
    if h >= 25000 
%         hdot = 1000/60; %fps
        mission.v_cruise = cruise_spd;
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
        idle_thrust_percent = 0.1;
    end
    mission.altitude = h;
%     mission.a = speedofsound(h);
    mission.viscocity = airviscocity(h);
%     mission.M = mission.v_cruise/mission.a; %ft/sec
    P = airpressure(h);
    rho = airdensity(h);
    mission.rho = rho;
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W7-Fuelburn);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W7-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    HotelP = HotelLoads(mission.M,h,3);
%     HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    HexDrag = 100; %lbs, assuming this now because this will have to be fixed with HOTEL LOADS
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag;
%     Tr = D - hdot*(state.W7-Fuelburn)/mission.v_cruise;
    Tr = idle_thrust_percent*interp1(TA_climb_alt,TA_climb,h);
%     Tr = D+(state.W3-Fuelburn)*sin(descent_angle);
%     mission.v_cruise = hdot/((Tr-D)/(state.W7-Fuelburn)); %in ft/sec
    FCeff = interp3D_V003(FCmap_eff,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n');
    Fuelburn = Fuelburn + H2CT(Ta,mission.v_cruise,h,weight,FCeff)*dt*Tr; %in lbs
%     hdot = mission.v_cruise*(Tr-D)/(state.W7-Fuelburn); %in m/s
    hdot = -mission.v_cruise*(Tr-D)/(state.W7-Fuelburn);
    h = h - hdot*dt;
    if h < 0
        h = 0;
    end
    time = time+dt;
    CDgraph(counter) = CD;
    CLgraph(counter) = CL;
    if drag_breakout == 1
    CDograph(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    CDigraph(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    CDwgraph(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if dave_numbers == 1
    CDowinggraph(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    CDofusegraph(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    Cfgraph(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    hgraph(counter) = mission.altitude;
    Tgraph(counter) = Tr;
    vgraph(counter) = mission.v_cruise;
    tgraph(counter) = time;
    Wgraph(counter) = state.W7 - Fuelburn;
    Mgraph(counter) = mission.M;
    PPgraph(counter) = Tr*mission.v_cruise* 1.3558;
    MPgraph(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    Pgraph(counter) = calcPower(Tr,mission.v_cruise,h) + HotelP;
    HDgraph(counter) = HexDrag;
    hdotgraph(counter) = -hdot*dt;
    Regraph(counter) = Re;
    
    Pagraph(counter) = FCmaxpower(weight,h,mission.M);
    Tagraph(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    Trgraph(counter) = D;
    Prgraph(counter) = D/FC2fan(h)*1.3558*mission.v_cruise;

    counter = counter + 1;
end

state.W8 = state.W7-Fuelburn;
state.WF8 = state.WF7 - Fuelburn;
descent_distance = alt_altitude/1000*3;
state.t8 = time; %descent_distance/(mission.v_cruise*0.592484);


%% Step 9 - Taxi to Gate (final calculations)
state.WFgoal = state.WF0*0.05;
% state.WFerror =state.WF8- state.WFgoal;
state.WFused = state.WF0-state.WF8;
% state.WFerrorpercent = state.WFerror/state.WF0*100


% if CBRK_sizing == 0

deltaFuel = (state.WF0*.95-state.WFused);
% state.WF0 = state.WF0-deltaFuel;
LRC_range = LRC_range + deltaFuel*.15;
% length(CDgraph)

if no_of_iterations < min_iterations
    no_of_iterations = no_of_iterations + 1;
    deltaFuel = 100;
else
    no_of_iterations = no_of_iterations + 1;
end
fprintf('DeltaFuel = %3.1d \n',deltaFuel)

% no_of_iterations = no_of_iterations + 1;
if no_of_iterations > 25
    is_borked = 1;
    break
end
[FAR_LFL] = landinglength_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);
toc

end
tgraph = [1:1:length(vgraph)]*dt/60;
% readout = ' \n %5.4f \n %5.4f \n %5.4f \n '%5.4f \n'
% fprintf(readout,state.WOE0,state.WF0,state.W0)
counter = counter-1;

if verbose == 1

CDgraph = CDgraph(1:counter);
CLgraph = CLgraph(1:counter);
CDograph = CDograph(1:counter);
CDigraph = CDigraph(1:counter);
CDwgraph = CDwgraph(1:counter);
CDowinggraph = CDowinggraph(1:counter);
CDofusegraph = CDofusegraph(1:counter);
hgraph = hgraph(1:counter);
Tgraph = Tgraph(1:counter);
vgraph = vgraph(1:counter);
tgraph = tgraph(1:counter);
Wgraph = Wgraph(1:counter);
Mgraph = Mgraph(1:counter);
Pgraph = Pgraph(1:counter);
MPgraph = MPgraph(1:counter);
PPgraph = PPgraph(1:counter);
hdotgraph = hdotgraph(1:counter);
Regraph = Regraph(1:counter);
Cfgraph = Cfgraph(1:counter);
Pagraph = Pagraph(1:counter);
Tagraph = Tagraph(1:counter);
Trgraph = Trgraph(1:counter);
Prgraph = Prgraph(1:counter);
HDgraph = HDgraph(1:counter);



outputtable = table(tgraph',vgraph',Mgraph',hgraph',hdotgraph',CDgraph',CLgraph',CDograph',CDigraph',CDwgraph',CDowinggraph',CDofusegraph', Cfgraph',Regraph', Wgraph', HDgraph', Pagraph',Prgraph',Tagraph',Trgraph', Tgraph',PPgraph', MPgraph', Pgraph');
outputtable.Properties.VariableNames = {'Time, min.' 'VTAS, fps' 'Mach No.' 'Altitude, ft' 'Climb Rate, Feet per Minute' 'Total Aircraft CD' 'Total Aircraft CL' 'Profile Drag' 'Induced Drag' 'Wave Drag' 'Wing Profile Drag' 'Fuselage Profile Drag' 'Fuselage Skin Friction' 'Reynolds Number' 'Weight, Lbs' 'HEx Drag, lbs' 'FC Power Available, Watts' 'FC Power Required, Watts' 'Thrust Available, Lb' 'Thrust Required, Lb' 'Thrust Commanded, Lb' 'Thrust Power Commanded, Watts' 'Motor Power Commanded, Watts' 'Fuel Cell Power Commanded, Watts'};

outputfilename = ['data_table',int2str(file_out),'.csv'];
weightout.WE(file_out) = weight.WE;
weightout.WF(file_out) = state.WF0;
weightout.wingloading(file_out) = state.W0/ac.wing.S;

writetable(outputtable,outputfilename);

end




n = file_out;
dataout.n = file_out;
% if CBRK_sizing == 0

summarytable = table(input_sweep(input_count),dataout.n,weight.Wpay,ac.WF,state.W0,LRC_range,max(Tgraph),TOFL,MTOW_LFL,climb_distance/6076,state.t2,max(CLgraph),LRC_Mach,is_borked);
summarytable_out = [summarytable_out;summarytable];
end

is_borked = 0;

% summarytable = table(weightout.WE',weightout.WF',weightout.wingloading');
% summarytable.Properties.VariableNames = {'Empty Weight','Fuel Weight','MTOW Wing Loading'};
% writetable(summarytable,'data_table_summary.csv');
% %% Plot Some stuff (optional)
% plot(hgraph)
% xlabel('Flight Time, Minutes')
% ylabel('Altitude, ft.')
% title('Full Flight Cycle of CHEETA BWB')
end
% end
summarytable_out.Properties.VariableNames = {'Input','Ref. File','Payload Weight','Fuel Weight','MTOW','Cruise Range','Max Thrust','TOFL','Maximum Landing Field Length','Climb Distance','Time to Climb','Maximum CL during Flight','Cruise Mach','Errored Out'};
outputfilename2 = ['output_comparison3.xlsx'];
writetable(summarytable_out,outputfilename2);


%Audio to tell me it's done.
[y,Fs] = audioread('finished.wav');
sound(y,Fs)

format short g
c = clock