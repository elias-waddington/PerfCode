clear all


% clc

verbose = 1; %toggles mass data output
dave_numbers = 0; %toggles profile, CF drag terms
drag_breakout = 0; %toggles o, i, w drag terms (1 is on, 0 is off)

N_sweep = 30;
min_iterations = 8;
summarytable_out = array2table(zeros(1,36));
AR_count = 0;
S_count = 1;
input_count = 1;

takeoff_type = 3; %power = 1, thrust = 0, hybrid = 2
if takeoff_type == 0
    [TakeoffParam,RunwayLength] = TO_params;
elseif takeoff_type == 1
    [TakeoffParam,RunwayLength] = TO_params_power;
elseif takeoff_type == 2
    [TakeoffParam,RunwayLength] = TO_params;
end
    
%Fuel Cell Type
FCmap_drag = allloadin_V003('FCmapR1_LD_drag.dat','n');
FCmap_eff = allloadin_V003('FCmapR1_LD_eff.dat','n');
load_LD1_LowDrag; 

%Section being run
% 
% AR_sweep = linspace(7,12,6);
% S_sweep = linspace(1500,2500,6);
% input_sweep = linspace(1.6,2.6,6);


AR_sweep = 8;
S_sweep = 2300;
input_sweep = 2.3;

%Initial Count
file_out = 0;
for outer_loop = 1:length(AR_sweep)*length(S_sweep) %*length(input_sweep)
% for file_out = 1:length(AR_sweep)
% for file_out = 1:1
no_of_iterations = 0;
clear ac
clear weight
run('aircraftfile_V02.m')
ac.FCsize = MP_Wlb_mat(1,1); %maximum power output at SL, M = 0.25
% ac.FCsize_toc = 995;

%This section for setting up sweeps

file_out = file_out+1

AR_count = AR_count + 1;
if AR_count > length(AR_sweep)
    AR_count = 1;
    S_count = S_count+1;
    if S_count > length(S_sweep)
        S_count = 1;
%         input_count = input_count + 1;
    end
end
    

%     for sweepin' AR
    ac.wing.AR = AR_sweep(AR_count);
%     ac.wing.AR = AR_sweep(file_out);
    ac.wing.A = ac.wing.AR;
    fprintf('AR = %3.1d \n',ac.wing.AR)
    ac.wing.S = S_sweep(S_count);
    ac.wing.bref = sqrt(ac.wing.AR*ac.wing.S);
%     ac.wing.S = 1/ac.wing.AR*ac.wing.bref^2;

[REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout, CDowing, CDofuse, Cffout] = CDgen(ac,N_sweep); %CDgen_V2 is the old, inefficient for debugging
no_of_iterations = 0;
%fixed AR for debugging

has_looped = 0;
input_count = 0;

for inner_loop = 1:length(input_sweep)
    
    if has_looped == 1
        file_out = file_out + 1;
        no_of_iterations = 0;
        clear weight
        deltaFuel = 100;
        input_count = input_count+1;
    else
        has_looped = 1;
        clear weight
        deltaFuel = 100;
        input_count = input_count+1;
    end

    disp(file_out)
    disp(AR_sweep(AR_count))
    disp(S_sweep(S_count))
    disp(input_sweep(input_count))
%     ac.FCsize = input_sweep(input_count);
% press_ratio = input_sweep(input_count);
CLto = input_sweep(input_count);

is_borked = 0;

%% Below here from performance code
%% Step 0 - Init

% INITIAL GUESSES
% wingSref = [3000,3100,3200,3300,3400];
ac.size.battery_energy = 0;
ac.size.sizing_power = 4.5e7;
ac.size.max_power = 4.5e7;
weight.fuelcell = ac.size.sizing_power/ac.FCsize;

% WEIGHT SIZING SETUP

% ac.FCsize = 890;

ac.WF = 15000; %initial fuel guess

dt = 60; %seconds
g = 32.2;
deltaFuel = 1000;
mission_length = 10*3600/dt;

ac.weight0 = 195000; %initial weight guess
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
tic
ac.WF = state.WF0;

fprintf('WF = %3.1d \n',ac.WF)

[ac,weight] = weight_estimate(ac,weight);

fprintf('FC = %3.1d \n',weight.fuelcell)
fprintf('Batt = %3.1d \n \n',weight.battery)

state.W0 = weight.WE+ac.WF+weight.Wpay;


% size_power_cbrk = 0;

%% Step 1 - Taxi & TO
state.W1 = state.W0*.998; %Raymer est. for TTO. NEEDS ADDRESSING
state.WF1 = state.WF0-(state.W0-state.W1);
counter = 0;

% CLto = 2;
press_ratio = 1;
static_scaling = 0.775; 
cruise_scaling = 0.45; %see spreadsheet, this assumed a FanEff of 0.752 at cruise at max power

% Takeoff_Parameter = (state.W0/ac.wing.S)/(press_ratio*CLto*Tagraph(1)/static_scaling/state.W0);
% TOFL = interp1(TakeoffParam,RunwayLength,Takeoff_Parameter);

if takeoff_type == 0
    ToP_required = interp1(RunwayLength,TakeoffParam,max_TOFL);
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,0,0.25)*weight.fuelcell;
    dataout.TR_TO = (state.W0/ac.wing.S)/(1*CLto*ToP_required)*state.W0;
    

    % First, test cruise's effect on TO and V35 thrust sizes
    dataout.Tcruise_test = dataout.TR_TO*cruise_scaling*static_scaling;
    TOFL = max_TOFL;
    %check for TOC scaling
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
    
    HexDragCruise = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    %checking thrust required at top of climb
    dataout.toc_test = (100/60)*state.W0/mission.v_cruise + CD*qbar*ac.wing.S + HexDragCruise;
    dataout.toc_to = dataout.toc_test/static_scaling/cruise_scaling;


    dataout.TA_TO = dataout.TR_TO;
    % dataout.TR_TO =
    dataout.TA_V35 = dataout.TR_TO * static_scaling;
    % TR now requires a hex drag term; calculate Dhex and incorporate it
    % HexDrag = interp3D_V003(FCmap_drag,1,ac.size.sizing_power/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    % directly to TR_V35
    dataout.TR_V35 = dataout.TR_TO * static_scaling;
    dataout.TA_cruise = dataout.Tcruise_test;
    dataout.HotelP = HotelLoads(mission.M,h,1);
    %Believe the line below lacks accounting for HEX drag. But is that already accounted for in the
    %power?
    ac.size.sizing_power = ((dataout.TR_TO * static_scaling)) * speedofsound(0)*0.25 /FC2fan(0)*1.3558 + dataout.HotelP ; %penalize cataout.TR_TO with + HexDrag
    ac.size.max_power = calcMotorPower(dataout.TR_V35,speedofsound(0)*0.25,0,weight);

    dataout.sizedby = 0;

    if dataout.toc_to > dataout.TR_TO
        dataout.sizedby = 36;
        disp('SIZED BY TOC')
        dataout.TA_TO = dataout.toc_to;
        dataout.TA_V35 = dataout.TA_TO * static_scaling;
        dataout.TR_V35 = dataout.TA_TO * static_scaling;
        dataout.TA_cruise = dataout.toc_test;
        dataout.Tcruise_test = dataout.TA_cruise;
        ac.size.sizing_power = ((dataout.toc_to * static_scaling))* speedofsound(0)*0.25/FC2fan(0) *1.3558 + dataout.HotelP; %must be in power required at take-off amounts
        ac.size.max_power = calcMotorPower(dataout.TR_V35,speedofsound(0)*0.25,0,weight);
        Takeoff_Parameter = (state.W0/ac.wing.S)/(press_ratio*CLto*dataout.toc_to/static_scaling/state.W0);
        TOFL = interp1(TakeoffParam,RunwayLength,Takeoff_Parameter);
    end

elseif takeoff_type == 1
%       debug issues:
%       sizing power of thrust = 2.537e+07, 3.9188e+07
%       sizing power of power = 2.2226e+07, 3.0317e+07
    
    ToP_required = interp1(RunwayLength,TakeoffParam,max_TOFL);
    dataout.TOpow = state.W0*745.7*(state.W0/ac.wing.S)/(1*CLto*ToP_required)/(FC2fan(0)/FanEff(0)) + HotelLoads(0.25,0,1); %BHP to Watts
    
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
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,0,0.25)*weight.fuelcell;
    HexDragCruise = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    dataout.Tcruise_test = (100/60)*state.W0/mission.v_cruise + CD*qbar*ac.wing.S + HexDragCruise;
    dataout.Cpow = dataout.Tcruise_test*mission.v_cruise*1.3558/FC2fan(h)+ HotelLoads(mission.M,h,1);
    
    if dataout.TOpow > dataout.Cpow
        ac.size.sizing_power = dataout.TOpow;
        ac.size.max_power = dataout.TOpow*FC2fan(0)/(FanEff(0) * 0.95);
        TOFL = max_TOFL;
        dataout.sizedby = 0;
    elseif dataout.TOpow < dataout.Cpow
%         ac.size.sizing_power = dataout.Cpow; %power at altitude,
%         recalculate for sea level
%         ac.size.max_power = dataout.TOpow/(FanEff(0) * 0.95); %motor power
        ac.size.sizing_power = dataout.Cpow*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,37000,.773); %Adjust to sea level power output
        ac.size.max_power = ac.size.sizing_power*FC2fan(0)/(FanEff(0) * 0.95); %adjust sizing power to motor output
        
        Takeoff_Parameter = (state.W0/ac.wing.S)/(press_ratio*CLto*(ac.size.sizing_power/745.7)/state.W0);
        TOFL = interp1(TakeoffParam,RunwayLength,Takeoff_Parameter);
        dataout.sizedby = 36;
    end
    dataout.TA_TO = 0;
    dataout.TR_TO = 0;
    dataout.TA_V35 = 0;
    dataout.TR_V35 = 0;
    dataout.TA_cruise = 0;
    dataout.TR_cruise = 0;
elseif takeoff_type == 2
    ToP_required = interp1(RunwayLength,TakeoffParam,max_TOFL);
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,0,0.25)*weight.fuelcell;
    dataout.TR_TO = (state.W0/ac.wing.S)/(1*CLto*ToP_required)*state.W0; 
    dataout.TR_V35 = dataout.TR_TO*static_scaling + HexDrag;
    dataout.P_V35 = dataout.TR_V35*0.25*speedofsound(0)*1.3558/FC2fan(0)+ HotelLoads(0.25,0,1);
    
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
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,0,0.25)*weight.fuelcell;
    HexDragCruise = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    dataout.Tcruise_test = (100/60)*state.W0/mission.v_cruise + CD*qbar*ac.wing.S + HexDragCruise;
    dataout.P_C = dataout.Tcruise_test*mission.v_cruise*1.3558/FC2fan(h)+ HotelLoads(mission.M,h,1)
    
    if dataout.P_V35 > dataout.P_C
        ac.size.sizing_power = dataout.P_V35;
        ac.size.max_power = dataout.TOpow*FC2fan(0)/(FanEff(0) * 0.95);
        TOFL = max_TOFL;
        dataout.sizedby = 0;
    elseif dataout.P_V35 < dataout.P_C
%         ac.size.sizing_power = dataout.Cpow; %power at altitude,
%         recalculate for sea level
%         ac.size.max_power = dataout.TOpow/(FanEff(0) * 0.95); %motor power
        ac.size.sizing_power = dataout.P_C*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,37000,.773); %Adjust to sea level power output
        ac.size.max_power = ac.size.sizing_power*FC2fan(0)/(FanEff(0) * 0.95); %adjust sizing power to motor output
        
        Takeoff_Parameter = (state.W0/ac.wing.S)/(press_ratio*CLto*(ac.size.sizing_power/745.7)/state.W0);
        TOFL = interp1(TakeoffParam,RunwayLength,Takeoff_Parameter);
        dataout.sizedby = 36;
    end
    dataout.TA_TO = 0;
%     dataout.TR_TO = 0;
    dataout.TA_V35 = 0;
%     dataout.TR_V35 = 0;
    dataout.TA_cruise = 0;
    dataout.TR_cruise = 0;
elseif takeoff_type == 3
    TOFL_error = 1;
    T = 40000; % Initial Guess
    while abs(TOFL_error) > 0.001
        h = 0;
        mission.altitude = h;
        HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,0.25)*weight.fuelcell; %assume constant drag at full power, 0.25, SL
        mission.a = speedofsound(h);
        mission.viscocity = airviscocity(h);
        P = airpressure(h);
        mission.rho = airdensity(h);
        rho = airdensity(h);
        location = zeros(100,1);
        V = zeros(100,1);
        ii = 1;
        aoa = 5; %deg
        mu = 0.05;
        Vstall = sqrt(state.W0*2/(rho*ac.wing.S*CLto));
        Vrotate = Vstall*1.08;
        P_shaft = weight.fuelcell*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)*FC2fan(0)/FanEff(0);
        %acceleration phase. DT = 1 second
        while V(ii) < Vrotate
            qbar = 0.5*V(ii)^2*mission.rho;
            CL = aoa*2*pi^2/180;
            L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
            Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
            if Re < 6e6
                Re = 6e6;
            end
            M = V(ii)/speedofsound(h);
            [T,FPR(ii)] = thrust2powerv02(M,P_shaft);
            if M < 0.11
                M = 0.11;
            end
            CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
            D = CD*qbar*ac.wing.S+HexDrag;
    %         T = 40000; %lbs, fix this later
            Vdot = (T-D-mu*(state.W0-L))/(state.W0/32.2);
            ii = ii +1;
            V(ii) = V(ii-1)+Vdot;
            location(ii) = location(ii-1)+V(ii);
        end
    rotate_begin = ii;
    %Normal Take-off

    gamma = asind((T-D-HexDrag)/state.W0);
    R = Vrotate^2/(0.1*32.2);
    htr = R*(1-cosd(gamma));
    if htr < 35
        Str = R*sind(gamma);
        Sc = (35-htr)/tand(gamma);
    elseif htr > 35
        Str = sqrt(R^2-(R-35)^2);
        Sc = 0;
    end

    S_AETO = Sc + Str + location(rotate_begin);

    %OEI TO case
    gamma = asind((T*2/3-D-HexDrag)/state.W0);
    R = Vrotate^2/(0.1*32.2);
    htr = R*(1-cosd(gamma));
    if htr < 35
        Str = R*sind(gamma);
        Sc = (35-htr)/tand(gamma);
    elseif htr > 35
        Str = sqrt(R^2-(R-35)^2);
        Sc = 0;
    end

    S_OEITO = Sc + Str + location(rotate_begin);

    %OEI Stop Case
    mu = 0.5;
        while V(ii) > 0
            qbar = 0.5*V(ii)^2*mission.rho;
            CL = aoa*2*pi^2/180;
            L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
            Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
            if Re < 6e6
                Re = 6e6;
            end
            M = V(ii)/speedofsound(h);
            if M < 0.11
                M = 0.11;
            end
            CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
            D = CD*qbar*ac.wing.S+HexDrag;
    %         T = 40000*0.95; %lbs, fix this later
            Vdot = (T*2/3*0-D-mu*(state.W0-L))/(state.W0/32.2);
            ii = ii +1;
            V(ii) = V(ii-1)+Vdot;
            location(ii) = location(ii-1)+V(ii);
        end
    ii = rotate_begin;
    S_OEIstop = max(location);

    %All Engine Stop case
    mu = 0.5;
        while V(ii) > 0
            qbar = 0.5*V(ii)^2*mission.rho;
            CL = aoa*2*pi^2/180;
            L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
            Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
            if Re < 6e6
                Re = 6e6;
            end
            M = V(ii)/speedofsound(h);
            if M < 0.11
                M = 0.11;
            end
            CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
            D = CD*qbar*ac.wing.S+HexDrag;
    %         T = 40000*0.95; %lbs, fix this later
            Vdot = (T*.95*0-D-mu*(state.W0-L))/(state.W0/32.2);
            ii = ii +1;
            V(ii) = V(ii-1)+Vdot;
            location(ii) = location(ii-1)+V(ii);
        end
    % rotate_begin = ii;
    S_AEstop = max(location);
    S_vector = [S_AETO, S_OEITO, S_OEIstop, S_AEstop];
    TOFL = max(S_vector)

    TOFL_error = ((TOFL-8200)/8200);
    T = T + T*TOFL_error*0.75;
    end
    dataout.TR_TO = T;
    dataout.HotelP = HotelLoads(0.25,0,1);
    dataout.P_to = ((dataout.TR_TO)) * speedofsound(0)*0.25 /FC2fan(0)*1.3558 + dataout.HotelP ;
    FC_TO_size = dataout.P_to/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25);
    
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
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,0,0.25)*weight.fuelcell;
    HexDragCruise = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    dataout.Tcruise_test = (100/60)*state.W0/mission.v_cruise + CD*qbar*ac.wing.S + HexDragCruise;
    dataout.TR_cruise = dataout.Tcruise_test;
    dataout.P_C = dataout.Tcruise_test*mission.v_cruise*1.3558/FC2fan(h)+ HotelLoads(mission.M,h,1);
    FC_C_size =  dataout.P_C/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,h,LRC_Mach);
% ac.size.sizing_power = dataout.P_C*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,37000,.773); %Adjust to sea level power output
% ac.size.max_power = ac.size.sizing_power*FC2fan(0)/(FanEff(0) * 0.95); %adjust sizing power to motor output
    
    if FC_TO_size > FC_C_size
        ac.size.sizing_power = dataout.P_to;
        ac.size.max_power = ac.size.sizing_power*FC2fan(0)/(FanEff(0) * 0.95);
        dataout.sizedby = 0;
    elseif FC_C_size > FC_TO_size
        ac.size.sizing_power = FC_C_size*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25);
        ac.size.max_power = ac.size.sizing_power*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,0,.25)/interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,37000,.773)*FC2fan(0)/(FanEff(0) * 0.95);
        dataout.sizedby = 36;
        T = (ac.size.sizing_power-dataout.HotelP)/(speedofsound(0)*0.25*1.3558/FC2fan(0));
        dataout.TA = T;
        
        
        h = 0;
        mission.altitude = h;
        HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,0.25)*weight.fuelcell; %assume constant drag at full power, 0.25, SL
        mission.a = speedofsound(h);
        mission.viscocity = airviscocity(h);
        P = airpressure(h);
        mission.rho = airdensity(h);
        rho = airdensity(h);
        location = zeros(100,1);
        V = zeros(100,1);
        ii = 1;
        aoa = 5; %deg
        mu = 0.05;
        Vstall = sqrt(state.W0*2/(rho*ac.wing.S*CLto));
        Vrotate = Vstall*1.08;
        %acceleration phase. DT = 1 second
        while V(ii) < Vrotate
            qbar = 0.5*V(ii)^2*mission.rho;
            CL = aoa*2*pi^2/180;
            L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
            Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
            if Re < 6e6
                Re = 6e6;
            end
            M = V(ii)/speedofsound(h);
            if M < 0.11
                M = 0.11;
            end
            CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
            D = CD*qbar*ac.wing.S+HexDrag;
    %         T = 40000; %lbs, fix this later
            Vdot = (T-D-mu*(state.W0-L))/(state.W0/32.2);
            ii = ii +1;
            V(ii) = V(ii-1)+Vdot;
            location(ii) = location(ii-1)+V(ii);
        end
        rotate_begin = ii;
        %Normal Take-off

        gamma = asind((T-D-HexDrag)/state.W0);
        R = Vrotate^2/(0.1*32.2);
        htr = R*(1-cosd(gamma));
        if htr < 35
            Str = R*sind(gamma);
            Sc = (35-htr)/tand(gamma);
        elseif htr > 35
            Str = sqrt(R^2-(R-35)^2);
            Sc = 0;
        end

        S_AETO = Sc + Str + location(rotate_begin);

        %OEI TO case
        gamma = asind((T*2/3-D-HexDrag)/state.W0);
        R = Vrotate^2/(0.1*32.2);
        htr = R*(1-cosd(gamma));
        if htr < 35
            Str = R*sind(gamma);
            Sc = (35-htr)/tand(gamma);
        elseif htr > 35
            Str = sqrt(R^2-(R-35)^2);
            Sc = 0;
        end

        S_OEITO = Sc + Str + location(rotate_begin);

        %OEI Stop Case
        mu = 0.5;
            while V(ii) > 0
                qbar = 0.5*V(ii)^2*mission.rho;
                CL = aoa*2*pi^2/180;
                L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
                Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
                if Re < 6e6
                    Re = 6e6;
                end
                M = V(ii)/speedofsound(h);
                if M < 0.11
                    M = 0.11;
                end
                CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
                D = CD*qbar*ac.wing.S+HexDrag;
        %         T = 40000*0.95; %lbs, fix this later
                Vdot = (T*2/3*0-D-mu*(state.W0-L))/(state.W0/32.2);
                ii = ii +1;
                V(ii) = V(ii-1)+Vdot;
                location(ii) = location(ii-1)+V(ii);
            end
        ii = rotate_begin;
        S_OEIstop = max(location);

        %All Engine Stop case
        mu = 0.5;
            while V(ii) > 0
                qbar = 0.5*V(ii)^2*mission.rho;
                CL = aoa*2*pi^2/180;
                L = 1/2*rho*V(ii)^2*ac.wing.S*CL;
                Re = mission.rho*V(ii)*ac.wing.MAC/mission.viscocity;
                if Re < 6e6
                    Re = 6e6;
                end
                M = V(ii)/speedofsound(h);
                if M < 0.11
                    M = 0.11;
                end
                CD = interp3(Msweep,REsweep,CLsweep,CDout,M,Re,CL,'spline');
                D = CD*qbar*ac.wing.S+HexDrag;
        %         T = 40000*0.95; %lbs, fix this later
                Vdot = (T*0-D-mu*(state.W0-L))/(state.W0/32.2);
                ii = ii +1;
                V(ii) = V(ii-1)+Vdot;
                location(ii) = location(ii-1)+V(ii);
            end
        % rotate_begin = ii;
        S_AEstop = max(location);
        S_vector = [S_AETO, S_OEITO, S_OEIstop, S_AEstop];
        TOFL = max(S_vector)

        end

    dataout.TA_TO = 0;
    dataout.TA_V35 = 0;
    dataout.TR_V35 = 0;
    dataout.TA_cruise = 0;
    
end
end
end
% end