% Drag table generation
clear all


clc

verbose = 1; %toggles mass data output
dave_numbers = 0; %toggles profile, CF drag terms
drag_breakout = 0; %toggles o, i, w drag terms (1 is on, 0 is off)

N_sweep = 30;
min_iterations = 8;
summarytable_out = array2table(zeros(1,37));
AR_count = 0;
S_count = 1;
input_count = 1;

takeoff_type = 2; %power = 1, thrust = 0, hybrid = 2, 3 = time step
if takeoff_type == 0
    [TakeoffParam,RunwayLength] = TO_params;
elseif takeoff_type == 1
    [TakeoffParam,RunwayLength] = TO_params_power;
elseif takeoff_type == 2
    [TakeoffParam,RunwayLength] = TO_params;
end
    
% %Fuel Cell Type
% FCmap_drag = allloadin_V003('FCmapR1_LD_drag.dat','n');
% FCmap_eff = allloadin_V003('FCmapR1_LD_eff.dat','n');
% load_LD1_LowDrag; 
% FCmap_drag = allloadin_V003('FCmapR2_Drag_current.dat','n');
% FCmap_eff = allloadin_V003('FCmapR2_Eff_current.dat','n');
% load_R2_current_LD;
% FCmap_drag = allloadin_V003('FCmapR2_Drag_currentLTD.dat','n');
% FCmap_eff = allloadin_V003('FCmapR2_Eff_currentLTD.dat','n');
% load_R2_current_LD_LTD;
FCmap_drag = allloadin_V003('FCmapR2_Drag_SOFC.dat','n');
FCmap_eff = allloadin_V003('FCmapR2_Eff_SOFC.dat','n');
load_R2_SOFC;
% FCmap_drag = allloadin_V003('FCmapR2_Drag_SOFCx2.dat','n');
% FCmap_eff = allloadin_V003('FCmapR2_Eff_SOFCx2.dat','n');
% load_R2_SOFCx2;

% %Section being run
% % 
% AR_sweep = linspace(7,12,3);
% S_sweep = linspace(4000,6000,3);
% % S_sweep = 3000;
% input_sweep = linspace(2,2.6,3);
% input_sweep = 3;
% 
AR_sweep = linspace(7,12,11);
S_sweep = linspace(1600,2500,10);
input_sweep = linspace(1.5,2.6,12);



% AR_sweep = 9;
% S_sweep = 2000;
% input_sweep = 2.2;

%Initial Count
file_out = 0;
% for outer_loop = 1:length(AR_sweep)*length(S_sweep) %*length(input_sweep)
% for file_out = 1:length(AR_sweep)
% for file_out = 1:1
% no_of_iterations = 0;
clear ac
clear weight
run('aircraftfile_V02.m')
ac.FCsize = MP_Wlb_mat(1,1); %maximum power output at SL, M = 0.25
% ac.FCsize_toc = 995;

%these are the parameters that must update before aero analysis wwith the current setup
ac.wing.Swetw = ac.wing.S*1.5735;
ac.wing.croot = 2*ac.wing.S/ac.wing.bref*(1+ac.wing.taper_ratio);
ac.wing.ctip = ac.wing.croot*ac.wing.taper_ratio;
ac.wing.MAC = (ac.wing.croot+ac.wing.ctip)/2; 
ac.wing.tr = ac.wing.croot*.154; %15.4 % based off of http://airfoiltools.com/airfoil/details?airfoil=b737a-il

[REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,N_sweep); %CDgen_V2 is the old, inefficient for debugging
no_of_iterations = 0;
%fixed AR for debugging

has_looped = 0;
input_count = 0;



is_borked = 0;

%% Below here from performance code
%% Step 0 - Init

% INITIAL GUESSES
% wingSref = [3000,3100,3200,3300,3400];
ac.size.battery_energy = 0;
ac.size.sizing_power = 50000*ac.FCsize;
ac.size.max_power = 3e7;
% weight.fuelcell = ac.size.sizing_power/ac.FCsize;
% weight.fuelcell = 20000;

% WEIGHT SIZING SETUP

% ac.FCsize = 890;

ac.WF = 50000; %initial fuel guess

dt = 60; %seconds
g = 32.2;
deltaFuel = 1000;
mission_length = 20*3600/dt;

ac.weight0 = 210000; %initial weight guess
weight.Wpay = 35000; %payload for this mission
state.WF0 = ac.WF;

% Cruise parameters
% LRC_Mach = 0.65;
LRC_Mach = 0.773;
LRC_range = 2935; %NM
max_TOFL = 8200; %ft

% while abs(deltaFuel) > 25
    
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

[ac,weight] = weight_estimate(ac,weight);

fprintf('FC = %3.1d \n',weight.fuelcell)
fprintf('Batt = %3.1d \n \n',weight.battery)

state.W0 = 210000;


% size_power_cbrk = 0;

%% Step 1 - Taxi & TO
CLto = 2.2;
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
%         disp('Cannot Climb Enough! HDOT < 100!')
        hdot = 100/60;
        Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
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

% if climb_distance/6076 > 600
%     is_borked = 1;
%     disp('BORK due to climb distance')
%     break
% end

TA_climb = [Tgraph(1)*1.01,Tgraph(1:counter-1)];
TA_climb_alt = [hgraph(1:counter-1),37000];
dataout.TR_V35 = max(Tgraph);
% Vclimb = 1.46667*360; %fps
% climb_distance = Vclimb*time/6076; %distance travelled in NM
% clear Vclimb
% climb_distance = 0;

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
%         dataout.TR_cruise = Tr;
%         if dataout.Tcruise_test < dataout.TR_cruise
%             disp('Error - Is Sized By Cruise')
%             dataout.sizedby = 37;
%         end
    end
    counter = counter + 1;


end

%     state.W3_old = (sqrt(state.W2)- R*6076/(2/CT*sqrt(CL)/CD*sqrt(2/(mission.rho*ac.wing.S))))^2;
%     state.WF3_old = state.WF2 - (state.W2-state.W3_old);
%     state.t3_old = R/(mission.v_cruise*0.592484);
    state.WF3 = state.WF2-Fuelburn;
    state.W3 = state.W2-Fuelburn;
	state.t3 = time;

% outputtable = table(tgraph',Tgraph',vgraph',Mgraph',hgraph',CDgraph',CLgraph',CDograph',CDigraph',CDwgraph', CDowinggraph', Wgraph');
% outputtable.Properties.VariableNames = {'Time, min.' 'Thrust, lbf' 'VTAS, fps' 'Mach No.' 'Altitude, ft' 'Total Aircraft CD' 'Total Aircraft CL' 'Profile Drag' 'Induced Drag' 'Wave Drag' 'Wing Alone Profile Drag' 'Weight, Lbs'};
% writetable(outputtable,'data_table.csv');



% end
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
weightout.WE = weight.WE;
weightout.WF = state.WF0;
weightout.wingloading = state.W0/ac.wing.S;

writetable(outputtable,outputfilename);

end




n = file_out;
% if CBRK_sizing == 0

% summarytable = table(ac.wing.AR,ac.wing.S,input_sweep(input_count),ac.wing.c1_4_sweep,ac.wing.taper_ratio,ac.wing.bref,file_out,weight.wing,weight.emp.VT+weight.emp.HT,weight.fuse,weight.lg.main+weight.lg.nose,weight.propulsors,weight.motors,weight.fuelcell,weight.battery,weight.treverse,weight.system,weight.fuelsys,weight.ops,weight.Wpay,ac.WF,state.W0,LRC_range,max(Tgraph),TOFL,MTOW_LFL,climb_distance/6076,state.t2,dataout.sizedby,max(CLgraph),dataout.TA_TO,dataout.TR_TO,dataout.TA_V35,dataout.TR_V35,dataout.TA_cruise,dataout.TR_cruise,is_borked);
% summarytable_out = [summarytable_out;summarytable];
% end

is_borked = 0;

% summarytable = table(weightout.WE',weightout.WF',weightout.wingloading');
% summarytable.Properties.VariableNames = {'Empty Weight','Fuel Weight','MTOW Wing Loading'};
% writetable(summarytable,'data_table_summary.csv');
% %% Plot Some stuff (optional)
% plot(hgraph)
% xlabel('Flight Time, Minutes')
% ylabel('Altitude, ft.')
% title('Full Flight Cycle of CHEETA BWB')
% end
% end

%Audio to tell me it's done.
% [y,Fs] = audioread('finished.wav');
% sound(y,Fs)

format short g
c = clock