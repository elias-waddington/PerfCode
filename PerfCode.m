% clear all
clc

run_parallel = 0;

set.verbose = 0; %toggles mass data output
set.dave_numbers = 0; %toggles profile, CF drag terms
set.drag_breakout = 0; %toggles o, i, w drag terms (1 is on, 0 is off)

%Meta Info
min_iterations = 8;

%Init
N_sweep = 30;
summarytable_out = array2table(zeros(1,37));
AR_count = 0;
S_count = 1;
input_count = 1;

% Select take-off type
takeoff_type = 2; %power = 1, thrust = 0, hybrid = 2, 3 = time step, 4 = PEM max thrust FC
if takeoff_type == 0
    [TakeoffParam,RunwayLength] = TO_params;
elseif takeoff_type == 1
    [TakeoffParam,RunwayLength] = TO_params_power;
elseif takeoff_type == 2
    [TakeoffParam,RunwayLength] = TO_params;
elseif takeoff_type == 4
    [TakeoffParam,RunwayLength] = TO_params;
end
    
% %Fuel Cell Type
FCmap_drag = allloadin_V003('FCmapR2_1214_drag.dat','n');
FCmap_eff = allloadin_V003('FCmapR2_1214_eff.dat','n');
load_R2_1214_cor; 

% Battery Usage
set.battery_use = 1;

%Section being run
AR_sweep = 10;
S_sweep = 1840;
% input_sweep = linspace(1,2,21);
input_sweep = 2.2;

AR_sweep = linspace(7,12,3);
S_sweep = linspace(1600,2500,5);

%Initial Count
file_out = 0;


S_ct = 1;
AR_ct = 1;
input_ct = 1;

length_of_run = length(S_sweep)*length(AR_sweep)*length(input_sweep);
RefMat = zeros(length_of_run,3);
for i = 1:length_of_run
    RefMat(i,1) = input_sweep(input_ct);
    RefMat(i,2) = S_sweep(S_ct);
    RefMat(i,3) = AR_sweep(AR_ct);

    input_ct = input_ct+1;
    if input_ct > length(input_sweep)
        input_ct = 1; 
        S_ct = S_ct+1;
        if S_ct > length(S_sweep)
            S_ct = 1;
            AR_ct = AR_ct+1;
            if AR_ct > length(AR_sweep)
                AR_ct = 1;
            end
        end
    end
end

parfor i = 1:length_of_run
    file_out = i;
    fprintf('%d \\ %d \n',i, length_of_run)
    ac = struct ;
    weight = struct ;
    mission = struct ;
    dataout = struct ;
    state = struct ;
    weightout = struct ;
    is_borked = 0;
    ac = aircraftfile_V04_func;
    ac.FCsize = MP_Wlb_mat(1,1); %maximum power output at SL, M = 0.25

  %from inputs
    CLto = RefMat(i,1);
    ac.wing.AR = RefMat(i,3);
    ac.wing.S = RefMat(i,2);
    
    Oversize = 1;
%   CLto = input_sweep(input_count);


    %these are the parameters that must update before aero analysis wwith the current setup
    ac.wing.Swetw = ac.wing.S*1.5735;
    ac.wing.croot = 2*ac.wing.S/ac.wing.bref/(1+ac.wing.taper_ratio);
    ac.wing.ctip = ac.wing.croot*ac.wing.taper_ratio;
    ac.wing.MAC = (ac.wing.croot+ac.wing.ctip)/2; 
    ac.wing.tr = ac.wing.croot*.154; %15.4 % based off of http://airfoiltools.com/airfoil/details?airfoil=b737a-il

    [REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,N_sweep); %CDgen_V2 is the old, inefficient for debugging
    no_of_iterations = 0;
    %fixed AR for debugging

    is_borked = 0;

%% Below here from performance code
%% Step 0 - Init

% INITIAL GUESSES
% wingSref = [3000,3100,3200,3300,3400];
ac.size.battery_energy = 0;
ac.size.sizing_power = 3e7;
ac.size.max_power = 3e7;
weight.fuelcell = ac.size.sizing_power/ac.FCsize;

% WEIGHT SIZING SETUP
ac.WF = 25000; %initial fuel guess

dt = 60; %seconds
g = 32.2;
deltaFuel = 1000;
mission_length = 20*3600/dt;

ac.WF = 10000;

ac.weight0 = 200000; %initial weight guess
weight.Wpay = 35000; %payload for this mission
state.WF0 = ac.WF;

% Cruise parameters
LRC_Mach = 0.773;
LRC_range = 2935; %NM
max_TOFL = 8200; %ft
cruise_alt = 37000;
alt_altitude = 15000;
alt_Mach = 0.569;

[Tcr,Pcr] = power2thrust_v01(LRC_Mach,ac,cruise_alt);
[Talt,Palt] = power2thrust_v01(alt_Mach,ac,alt_altitude);

while abs(deltaFuel) > 25
    
dat.h = zeros(1,mission_length);
dat.T = zeros(1,mission_length);
dat.v = zeros(1,mission_length);
dat.t = zeros(1,mission_length);
dat.CD = zeros(1,mission_length);
dat.CL = zeros(1,mission_length);
dat.CDo = zeros(1,mission_length);
dat.CDi = zeros(1,mission_length);
dat.CDw = zeros(1,mission_length);
dat.CDowing = zeros(1,mission_length);
dat.CDofus = zeros(1,mission_length);
dat.W = zeros(1,mission_length);
dat.P = zeros(1,mission_length);
dat.MP = zeros(1,mission_length);
dat.PP = zeros(1,mission_length);
dat.M = zeros(1,mission_length);
dat.hdot = zeros(1,mission_length);
dat.Re = zeros(1,mission_length);
dat.Cf = zeros(1,mission_length);
dat.CDofuse = zeros(1,mission_length);
dat.Pa = zeros(1,mission_length);
dat.Ta = zeros(1,mission_length);
dat.Tr = zeros(1,mission_length);
dat.Pr = zeros(1,mission_length);
dat.HD = zeros(1,mission_length);
dat.TSFC = zeros(1,mission_length);
tic
ac.WF = state.WF0;

fprintf('WF = %3.1d \n',ac.WF)

ac.size.sizing_power = ac.size.sizing_power*Oversize; %This is where Oversize goes
[ac,weight] = weight_estimate_hybGD_v02(ac,weight);

fprintf('FC = %3.1d \n',weight.fuelcell)
fprintf('Batt = %3.1d \n \n',weight.battery)

state.W0 = weight.WE+ac.WF+weight.Wpay;
%% Step 1 - Taxi & TO
[Taxi_Power,Taxi_Thrust] = taxi_estimator(weight,Msweep,REsweep,CLsweep,CDout,FCmap_drag,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);
Taxi_Power = Taxi_Power + HotelLoads(0.05,1,1);
FCin_taxi = Taxi_Power/weight.fuelcell;
if FCin_taxi < 260
    FCin_taxi = 259;
end
FCeff = interp3D_V003(FCmap_eff,1,FCin_taxi,0.25,1,'n');
FB_taxi = H2CT(Taxi_Thrust,Taxi_Power,FCeff)*Taxi_Thrust*60*35;

state.W1 = state.W0-FB_taxi; %
state.WF1 = state.WF0-(FB_taxi);
counter = 0;

[MTOW_LFL] = landinglength_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);

% CLto = 2;
press_ratio = 1;
static_scaling = 0.775; 
cruise_scaling = 0.45; %see spreadsheet, this assumed a FanEff of 0.752 at cruise at max power

% Takeoff_Parameter = (state.W0/ac.wing.S)/(press_ratio*CLto*dat.Ta(1)/static_scaling/state.W0);
% TOFL = interp1(TakeoffParam,RunwayLength,Takeoff_Parameter);

if takeoff_type == 2
    TOFL_error = 1;
    no_of_iterations_to = 1;
    
    if no_of_iterations_to > 25
        is_borked = 1;
        break
    end
    no_of_iterations_to = no_of_iterations_to+1;

    dataout.HotelP = HotelLoads(0.25,0,1);
    h = 37000;
    mission.altitude = h;
    mission.a = speedofsound(h);
    mission.viscocity = airviscocity(h);
    mission.M = LRC_Mach;
    mission.v_cruise = mission.M*mission.a; %ft/sec
    P = airpressure(h);
    mission.rho = airdensity(h);
    rho = airdensity(h);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W0)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    % Hex Drag calculate here
%     HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,0,0.25)*weight.fuelcell;
    HexDragCruise = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell; %also assumes maximum power point. needs to update to 3d interp to choose correct point
    if HexDragCruise < 0
        HexDragCruise = 0;
    end
    dataout.Tcruise_test = (100/60)*state.W0/mission.v_cruise + CD*qbar*ac.wing.S + HexDragCruise;
    dataout.TR_cruise = dataout.Tcruise_test;
    dataout.TA_cruise = dataout.Tcruise_test;
    Pshaft = interp1(Tcr,Pcr, dataout.TR_cruise);
    dataout.P_C = Pshaft/FC2shaft(mission.altitude) + HotelLoads(mission.M,h,1);
    FC_C_size =  dataout.P_C/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,h,LRC_Mach); %assumes max power point, need to update here to change edge case
%     [TOFL,dataout.TA_TO,dataout.TR_TO] = takeoff_solver(weight,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Wlb_mat,state,ac,CLto);
%     dataout.TA_TO = dataout.TR_TO;
    ac.size.sizing_power = dataout.P_C*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,37000,.773); %Adjust to sea level power output
    ac.size.max_power = ac.size.sizing_power*FC2shaft(0); %adjust sizing power to motor output
    dataout.sizedby = 36;
    [dataout.TA_TO,FPR] = thrust2powerv02(0,FC_C_size*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)*FC2shaft,ac);
    Takeoff_Parameter = (state.W0/ac.wing.S)/(press_ratio*CLto*dataout.TA_TO/state.W0);
    TOFL = interp1(TakeoffParam,RunwayLength,Takeoff_Parameter);
    if isnan(TOFL) == 1;
        disp('TOFL error')
    end
%     datzxc raout.TA_TO = ac.size.sizing_power
    dataout.TR_TO = 0;

    if TOFL > max_TOFL
        dataout.sizedby = 0;
        TOFL = 8200;
        ToP_required = interp1(RunwayLength,TakeoffParam,max_TOFL);
        dataout.TA_TO = (state.W0/ac.wing.S)/(1*CLto*ToP_required)*state.W0;
        t_error = 1;
        P_shaft_guess = ac.size.sizing_power;
        while t_error > 0.005
            [Fnet,dataout.TR_TO] = thrust2powerv02(0,P_shaft_guess,ac);
            t_error = (-Fnet+dataout.TA_TO)/dataout.TA_TO;
            P_shaft_guess = P_shaft_guess + P_shaft_guess*t_error*.6;
        end
        FC_TO_size = P_shaft_guess*(FC2shaft(0))/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25);
        ac.size.sizing_power = FC_TO_size*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25);
        ac.size.max_power = ac.size.sizing_power/FC2shaft(0); %adjust sizing power to motor output
       
        if FC_TO_size < FC_C_size
            ac.size.sizing_power = FC_C_size*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25);
            ac.size.max_power = ac.size.sizing_power*FC2shaft(0);
            dataout.sizedby = 36;
            disp('WTF Actually sized by cruise')
        end
        
    end
   dataout.TA_V35 = state.W0/ac.wing.S/(1*CLto*dataout.TA_TO/state.W0);
   dataout.TR_V35 = 0;
%    ac.size.sizing_power = ac.size.sizing_power*1.2; %20% too big
   
end

weight.final.ac = state.W1;
weight.final.fuel = state.WF1;
counter = 1;

%% Step 2 - Climb
[state,dat,counter,weight] = Climb_v01(ac,state,weight,0.25,LRC_Mach,0,cruise_alt,counter,set,dat,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Eff_mat,dt);

%% Step 3 - Cruise
[state,dat,counter,weight] = Cruise_v01(ac,state,weight,LRC_Mach,cruise_alt,counter,set,dat,FCmap_drag,FCmap_eff,Msweep,REsweep,CLsweep,CDout,dt,LRC_range);

%% Step 4 - Descent
[state,dat,counter,weight]  = Descent_v01(ac,state,weight,LRC_Mach,cruise_alt,0,counter,set,dat,FCmap_eff,Msweep,REsweep,CLsweep,CDout,dt,MP_h_rng,MP_M_rng,LP_Drag_mat,LP_Eff_mat);

%% Step 5 - Abort Climb Out
[state,dat,counter,weight] = Climb_v01(ac,state,weight,0.25,alt_Mach,0,alt_altitude,counter,set,dat,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Eff_mat,dt);

%% Step 6 - Cruise to Alternate
[state,dat,counter,weight] = Cruise_v01(ac,state,weight,alt_Mach,alt_altitude,counter,set,dat,FCmap_drag,FCmap_eff,Msweep,REsweep,CLsweep,CDout,dt,200);

%% Step 7 - 30 minute loiter
[state,dat,counter,weight]  = Loiter_v01(ac,state,weight,alt_Mach,alt_altitude,counter,set,dat,Msweep,REsweep,CLsweep,CDout,FCmap_drag,FCmap_eff, 0.5,dt);

%% Step 8 - Descent and Landing
[state,dat,counter,weight]  = Descent_v01(ac,state,weight,alt_Mach,alt_altitude,0,counter,set,dat,FCmap_eff,Msweep,REsweep,CLsweep,CDout,dt,MP_h_rng,MP_M_rng,LP_Drag_mat,LP_Eff_mat);

%% Step 9 - Taxi to Gate (final calculations)

state.WFused = state.WF0-weight.final.fuel;

deltaFuel = (state.WF0*.95-state.WFused);
state.WF0 = state.WF0-deltaFuel;

if isnan(deltaFuel) == 1
    is_borked = 1;
    break
end
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
dat.t = [1:1:length(dat.v)]*dt/60;
% readout = ' \n %5.4f \n %5.4f \n %5.4f \n '%5.4f \n'
% fprintf(readout,state.WOE0,state.WF0,state.W0)
counter = counter-1;

if set.verbose == 1
    dat.CD= dat.CD(1:counter);
    dat.CL = dat.CL(1:counter);
    dat.CDo = dat.CDo(1:counter);
    dat.CDi = dat.CDi(1:counter);
    dat.CDw = dat.CDw(1:counter);
    dat.CDowing = dat.CDowing(1:counter);
    dat.CDofuse = dat.CDofuse(1:counter);
    dat.h = dat.h(1:counter);
    dat.T = dat.T(1:counter);
    dat.v = dat.v(1:counter);
    dat.t = dat.t(1:counter);
    dat.W = dat.W(1:counter);
    dat.M = dat.M(1:counter);
    dat.P = dat.P(1:counter);
    dat.MP = dat.MP(1:counter);
    dat.PP = dat.PP(1:counter);
    dat.hdot = dat.hdot(1:counter);
    dat.Re = dat.Re(1:counter);
    dat.Cf = dat.Cf(1:counter);
    dat.Pa = dat.Pa(1:counter);
    dat.Ta = dat.Ta(1:counter);
    dat.Tr = dat.Tr(1:counter);
    dat.Pr = dat.Pr(1:counter);
    dat.HD = dat.HD(1:counter);
    dat.TSFC = dat.TSFC(1:counter);


    outputtable = table(dat.t',dat.v',dat.M',dat.h',dat.hdot',dat.CD',dat.CL',dat.CDo',dat.CDi',dat.CDw',dat.CDowing',dat.CDofuse', dat.Cf',dat.Re', dat.W', dat.HD', dat.Pa',dat.Pr',dat.Ta',dat.Tr', dat.T',dat.PP', dat.MP', dat.P',dat.TSFC');
    outputtable.Properties.VariableNames = {'Time, min.' 'VTAS, fps' 'Mach No.' 'Altitude, ft' 'Climb Rate, Feet per Minute' 'Total Aircraft CD' 'Total Aircraft CL' 'Profile Drag' 'Induced Drag' 'Wave Drag' 'Wing Profile Drag' 'Fuselage Profile Drag' 'Fuselage Skin Friction' 'Reynolds Number' 'Weight, Lbs' 'HEx Drag, lbs' 'FC Power Available, Watts' 'FC Power Required, Watts' 'Thrust Available, Lb' 'Thrust Required, Lb' 'Thrust Commanded, Lb' 'Thrust Power Commanded, Watts' 'Motor Power Commanded, Watts' 'Fuel Cell Power Commanded, Watts' 'TSFC lb/lbf-hr'};

    outputfilename = ['data_table',int2str(file_out),'.csv'];
    weightout.WE(file_out) = weight.WE;
    weightout.WF(file_out) = state.WF0;
    weightout.wingloading(file_out) = state.W0/ac.wing.S;

    writetable(outputtable,outputfilename);
    
    
end

n = file_out;
% if CBRK_sizing == 0

summarytable = table(ac.wing.AR,ac.wing.S,input_sweep(input_count),ac.wing.c1_4_sweep,ac.wing.taper_ratio,ac.wing.bref,file_out,weight.wing,weight.VT + weight.HT,weight.fuse,weight.lg,weight.propulsors,weight.motors,weight.fuelcell,weight.battery,weight.treverse,weight.system,weight.fuelsys,weight.ops,weight.Wpay,ac.WF,state.W0,LRC_range,max(dat.T),TOFL,MTOW_LFL,dat.climb_dist/6076,state.t2,dataout.sizedby,max(dat.CL),dataout.TA_TO,dataout.TR_TO,dataout.TA_V35,dataout.TR_V35,min(dat.TSFC),max(dat.TSFC),is_borked);
summarytable_out = [summarytable_out;summarytable];
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
end

summarytable_out.Properties.VariableNames = {'AR','Sref','Input','1/4 Chord Sweep','Taper Ratio','Wing Span','Ref. File','Wing weight','Empennage Weight','Fuselage weight','Landing Gear','Propulsors','Motors','Fuel Cells','Battery','Thrust Reversers','System Weight','Fuel Tanks','Operational Items','Payload Weight','Fuel Weight','MTOW','Cruise Range','Max Thrust','TOFL','Maximum Landing Field Length','Climb Distance','Time to Climb','Sized By','Maximum CL during Flight','TAvailTO','FPR','ToP','T Target V35','Min TSFC','Max TSFC','Errored Out'};
outputfilename2 = ['output_comparison2.xlsx'];
writetable(summarytable_out,outputfilename2);


%Audio to tell me it's done.
[y,Fs] = audioread('finished.wav');
sound(y,Fs)

format short g
c = clock