function [ac] = aircraftfile_V04_func

% % %% This is what the structures inputs will need to compute drag
% 
ac.VD = 340; %keas, for weight estimation only - formarely 293, should bee 496
ac.WP = 35000; %lbs, payload

%Configuration High-level
ac.wingloc = 2; %0 for low, 1 for mid, 2 for high

% Wing values, inputs
ac.wing.bref = 135.6466; %ft
ac.wing.S = 1840; %wing ref area, ft^2
% ac.wing.Swetw = 3398.75; %wing wetted area, ft^2, updates in weight estimate
ac.wing.tc = 0.0907968; %mac thickness
ac.wing.tcavg = ac.wing.tc*1.1; %mac thickness
ac.wing.c1_4_sweep = 20.1798; %deg
ac.wing.ct_cmax_sweep = 18.6174; %update with variable
ac.wing.c1_2_sweep = 16.8849; %update
ac.wing.incidence_root = 1; %deg  
ac.wing.incidence_tip = 0; %deg 
ac.wing.A = ac.wing.bref^2/ac.wing.S;
ac.wing.AR = ac.wing.A;
ac.wing.taper_ratio = 0.25749;
% % % 
ac.wing.hc = 1.65; %per cent, camber *

%next 4 are intitial estiamtes updated in weight estimate function
ac.wing.Swetw = ac.wing.S*1.5735;
ac.wing.croot = 2*ac.wing.S/ac.wing.bref*(1+ac.wing.taper_ratio);
ac.wing.ctip = ac.wing.croot*ac.wing.taper_ratio;
ac.wing.MAC = (ac.wing.croot+ac.wing.ctip)/2; %technically this is c avg
% % % CDow values
% % 
ac.wing.L1 = 1.2; %airfoil thickness location parameter as t/c max > 0.3 *
% % % 
% % % CDLw values
ac.wing.Cla = 2*pi; %needs update *
ac.wing.R = 0.927;  % *
ac.wing.v = -0.0002; % *
ac.wing.wB = .0016; % *
% % % 
% % % Torenbeek Wing weight estimation parameters
ac.wing.tr = ac.wing.croot*.154; %15.4 % based off of http://airfoiltools.com/airfoil/details?airfoil=b737a-il
ac.wing.weight.has_spoilers = 1.02; % 2% incresae for spoilers
ac.wing.weight.has_engines = 0.95; % 5% decrease for engines mounted on wing
ac.wing.weight.has_lndgear = 0.95; % 10% decrease for no landing gear on wing
ac.wing.weight.is_braced = 1; %no weight change as wings do not have bracing

%%% GD wing weight estiamtion
ac.MH = 0.641;
ac.nult = 3.75;
% % 
% % % 
% % % 
% % % Fuselage values, inputs
% ac.fuse.Swet = 8157.942102; %ft^2 % *
ac.fuse.Swet = 6321;
ac.fuse.height = 18.5; %ft
ac.fuse.width = 12.3333; %ft %Max width is 19.26
ac.fuse.d = 14.9974;
ac.fuse.lh = 65.583; %ft
ac.fuse.length = 123;
ac.fuse.Sfus = 246; %ft^2, front planform area
ac.fuse.Sbfus = 14.1; % ft^2. not too sure about this - Part 6, Page 45 (77) - base area, so around the APU?
ac.fuse.Splffus = 2183.3; %ft^2 fuse planform area 
ac.fuse.lpax = 89.2; %ft
ac.fuse.qbard = 767.798;
ac.fuse.lf = ac.fuse.length;
ac.fuse.hf = ac.fuse.height;
% % % 
% % % CDofus values
ac.fuse.Rwf = 1; %Part 6, Page 44 "Use 1 for just fuselage"
ac.fuse.Cffus = 0.002;
ac.fuse.CLo = .2; %what is this?
ac.fuse.cdc = 1.2; %where do I get this and why?
% % % 
% % % Torenbeek Fuse Weight Estimation Parameters
ac.fuse.weight.has_press_fuse = 1 + 0.08*0.48; %1.08 if fuse is pressurized, 1 if not NEEDS UPDATE
ac.fuse.weight.has_main_gear = 1.07; %1.07 if main gear attached, 1 if not
ac.fuse.Kf = ac.fuse.weight.has_press_fuse*ac.fuse.weight.has_main_gear; %pressurized fuselage
ac.fuse.fgs = ac.fuse.Swet;
ac.fuse.nult = 3.75; %g *
ac.fuse.vpax = 15704; %ft^3, volume of passenger compartment * assumiong same as 737
ac.fuse.Kinl = 1;

% % 
% % % Empennage values, inputs
% % % Htail values
ac.HT.tc = 0.11; %*
ac.HT.S = 544.2; %ft^2 *
ac.HT.Swet = 807.25; %ft^2 *
ac.HT.b = 47.08; %ft^2 *
ac.HT.A = ac.HT.b^2/ac.HT.S^2; %*
ac.HT.ct_cmax_sweep = 34.36709559; %deg *
ac.HT.cmac = 8.68; %ft *
% HTail 0 lift Drag
ac.HT.L1 = 1.2; %*
% HTail Drag due to Lift
ac.HT.ih = 0; %variable incidence on tail, what should this be? *
ac.HT.Cla = 2*pi; %update with actual airfoil *
ac.HT.alpha0Lh = 0; %guess *
ac.HT.eh = 0.5; %Roskam Pt 6 Page 69/101 *
ac.HT.epsilont_h = 0; %unknown, assume no twist on tail *
ac.HT.TR = 8.712; % root thickness
% % 
% % Torenbeek weight constants
ac.HT.kh = 1.1; %variable incidence stabilizer
ac.HT.c1_2_sweep = 34.36709559; %deg, update with variable later
% 
% % Vertical tail values
ac.VT.Swet = 505.18; %*
ac.VT.tc = 0.06; %*
% ac.VT.S = 252.6; %*
ac.VT.S = 352.6;
ac.VT.b = 22.7; %*
ac.VT.A = ac.VT.b^2/ac.VT.S; %*
ac.VT.ct_cmax_sweep = 45.01085735; %deg %*
ac.VT.cmac = 8.68; %ft %*
% Vtail 0 lift
ac.VT.Rwf = 1; %*
ac.VT.RLS = 1.05; %*
ac.VT.Cfw = 0.0022; %*
ac.VT.L1 = 1.2; %*
% % Torenbeek weight constants
ac.VT.kh = 1; %fuse mounteded h-stabilizer %*
% ac.VT.c1_2_sweep = 45.01085735; %deg, update with variable later %*
ac.VT.c1_2_sweep = 14.4;
ac.VT.zh = 0;
ac.fuse.lv = 60; %guess
ac.VT.Sr = 56.8458; %rudder area
ac.VT.taper = .203; %from 737
ac.VT.c1_4sweep = 30; % from 737
% % 
% % 
% % Nacelle inputs 
ac.n.n= 2; % 
ac.n.length = 15.19; %ft %*
ac.n.Swet = 94.5; %ft^2 %*
ac.n.b = 6.75853; %ft %*
ac.n.c = 13.91; %ft %*
ac.n.S = 60; %ft^2, WAG %*
ac.n.Sb = 0.5; %ft^2, WAG %*
ac.n.CDLv = 0; %Assumed to be 0 since we're assuming no sideslip angle %*
% 
% Roskam Part VI Step 1: 4.5.2
ac.n.in = deg2rad(1); %nacelle angle of incidence %*
ac.n.delcl1 = -0.3; % below the wind %*
ac.n.delcl2 = -0.056*ac.n.in; %*
ac.n.epsilon = 0; %*

ac.n.Rwf = 1.02;
ac.n.Cffus = 0.0025; %update for new Mach no. %*
ac.n.db = 0.25; %tapers to a point? %*
ac.n.nu = 0.55; %body fineness, lookup from table, should be constant %*
ac.n.TTO = 48400; %lbf %*
% 
% roskam part V torenbeek weight estimation
ac.n.nt = 3;  %*
ac.n.ne = 3; %*
ac.kfsp = 0.5925; %lb/gal %
ac.dwfdt = 5.199; %lbs/sec %*

ac.prop.num = 9;
ac.prop.a2 = 1.124; %m^2 -_-
ac.prop.a8 = 0.754; %m^2 -_-
% 
% mission stuff for weight estimation - Same crew compliment as 737
ac.npax = 178;
ac.cargo_volume = 1592.37; %ft^3  %*

ac.klav = .6;
ac.kbuf = (1.02+5.68)/2;
ac.pc = 8.35; %Psi at cruise

ac.crew = 2; %*
ac.steward = 4; %*
if exist('ac.WF') == 1
    ac.fuelvolume = ac.WF/0.5925; %gal
else
    ac.fuelvolume = 20000/0.5925; %gal, estimate
end

