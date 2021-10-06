% Mass Buildup for BWB

%Instruction:
% Run ('aircraftfile.m') and then 

function [ac,weight] = weight_estimate(ac,weight)

%Diagnostics & Debug
% clear
% run('ESDU_bwbaircraftfile.m')

%here we update the aircraft values that are derived and not driven
ac.wing.Swetw = ac.wing.S*1.5735;
ac.wing.croot = 2*ac.wing.S/ac.wing.bref*(1+ac.wing.taper_ratio);
ac.wing.ctip = ac.wing.croot*ac.wing.taper_ratio;
ac.wing.MAC = (ac.wing.croot+ac.wing.ctip)/2;
ac.wing.tr = ac.wing.croot*.154; %15.4 % based off of http://airfoiltools.com/airfoil/details?airfoil=b737a-il



WTO = ac.weight0; %this will have to be a guess i suppose
ac.fuelvolume = ac.WF/0.5925; %need to recalculate fuel volume
WF = ac.WF;
WP = ac.WP; %lbs, payload
% WLH2 = ac.WLH2; %additional LH2 weight

WMZF = WTO-ac.WF; %lbs
WE = WTO-WF-WP; %initial guess

WE_final = WF;
delta = 100000;

while delta > 50 %while diff is > 50 lbs
%things to define in the aircraft file
WE = WE_final;
WTO = WE_final + WF + WP;
WMZF = WTO - WF;
WE_final = 0;
% weight.wing = 0.0017*ac.WMZF*(wing.b/cosd(wing.sweep_at_1_2))^.75*(1+sqrt(6.3*cosd(wing.sweep_at_1_2)/wing.b))*ac.nult^0.55*(wing.b*wing.Sref/(wing.tr*ac.WMZF*cosd(wing.sweep_at_1_2)))^0.3;

% weight.wing = 0.0017*WMZF*(ac.wing.bref/cosd(ac.wing.c1_2_sweep))^0.75*(1+(6.3*cosd(ac.wing.c1_2_sweep)/ac.wing.bref)^0.5)*ac.fuse.nult^0.55*(ac.wing.bref*ac.wing.S/(ac.wing.tr*WMZF*cosd(ac.wing.c1_2_sweep)))^0.3*ac.wing.weight.has_spoilers*ac.wing.weight.has_engines*ac.wing.weight.has_lndgear*ac.wing.weight.is_braced;
weight.wing = (0.00428*ac.wing.S^.48*ac.wing.AR*ac.MH^0.43*(WTO*ac.nult)^.84*ac.wing.taper_ratio^.14)/((100*ac.wing.tcavg)^.76*cosd(ac.wing.c1_2_sweep)^1.54);
WE_final = WE_final + weight.wing;

% weight.emp.HT = ac.HT.kh*ac.HT.S*(3.81*(ac.HT.S^0.2*ac.VD)/(1000*sqrt(cosd(ac.HT.c1_2_sweep)))-0.287);
weight.HT = 0.0034*((WTO*ac.nult)^0.813*ac.HT.S^.584*(ac.HT.b/ac.HT.TR)^.033*(ac.HT.cmac/ac.fuse.lh)^.28)^.915;
WE_final = WE_final+weight.HT;
% weight.VT = ac.VT.kh*ac.VT.S*(3.81*(ac.VT.S^0.2*ac.VD)/(1000*sqrt(cosd(ac.VT.c1_2_sweep)))-0.287);
weight.VT = .19*((1+ac.VT.zh/ac.VT.b)^.5*(WTO*ac.nult)^.363*ac.VT.S^1.089*ac.MH^.601*ac.fuse.lv^-.726*(1+ac.VT.Sr/ac.VT.S)^.217*ac.VT.A^.336*(1+ac.VT.taper)^.363*cosd(ac.VT.c1_4sweep)^.484)^1.014;
WE_final = WE_final+weight.VT;

% weight.fuse = 0.021*ac.fuse.Kf*(ac.VD*ac.fuse.lh/(ac.fuse.width+ac.fuse.height))^0.5*ac.fuse.Swet^1.2;
weight.fuse = 2*10.43*ac.fuse.Kinl^1.42*(ac.fuse.qbard/100)^.283*(WTO/1000)^.95*(ac.fuse.lf/ac.fuse.hf)^.71;
WE_final = WE_final+weight.fuse;

% weight.n = 7.435*ac.n.n*ac.n.
% weight.n = 2000;
% WE_final = WE_final+weight.n;

%% Landing Gear
%Assumes 737-esque gear placement
% A = 20;
% B = .1;
% C = 0;
% D = 0.000002;
% if ac.wingloc == 0
%     kgr = 1;
% elseif ac.wingloc == 2
%     kgr = 1.08;
% end
% weight.lg.nose = kgr*(A+B*(WTO)^0.75+C*WTO+D*WTO^(1.5));
% WE_final = WE_final+weight.lg.nose;
% 
% A = 40;
% B = .16;
% C = .019;
% D = .000015;
% weight.lg.main = kgr*(A+B*(WTO)^0.75+C*WTO+D*WTO^(1.5));
% WE_final = WE_final+weight.lg.main;
weight.lg = 62.61*(WTO/1000)^.84;
WE_final = WE_final + weight.lg;


%% Engines and Powerplant
weight.propulsors = 14380; %lbs. Fixed per Dave Hall estimate
WE_final = WE_final+weight.propulsors;

weight.motors = ac.size.max_power/20000*2.2;
WE_final = WE_final + weight.motors;


weight.fuelcell = ac.size.sizing_power/ac.FCsize;%calculate fuel cell size = to top of climb cruise power

WE_final = WE_final+weight.fuelcell;

% if exist('ac.size.battery_energy','var') == 0
%     ac.size.battery_energy = 102054453*60; %assumed watt-seconds (jouiles)
% end
% ac.size.battery_energy
%battery assumptions
battery_energy_density = 600*3600; %WH/kg to W-s/kg
battery_safetyfactor = 1.;
weight.battery = ac.size.battery_energy/battery_energy_density*2.2*battery_safetyfactor;
% battery = weight.battery
if weight.battery < 0
    weight.battery = 0; % Change for Min. Battery Mass
end

WE_final = WE_final+weight.battery;

weight.treverse = weight.propulsors*0.18; %lbs
WE_final = WE_final+weight.treverse;

weight.system = 0;
weight.fuelsys = ac.WF*0.65; %fuel tank weight, 65% of system mass if fuel tank
WE_final = WE_final+weight.fuelsys;

weight.propsys = 36*ac.n.n*5; %should this still be here? probably not, these systems don't exist
WE_final = WE_final+weight.propsys;
weight.system = weight.system+weight.propsys;


%% Subsystems
% weight.sys.fcs = 0.64*WTO^(2/3)*1.2*1.15; %1.2 due to LE devices, 1.15 due to lift dumpers
weight.sys.fcs = 56.01*((WTO*ac.fuse.qbard)/100000)^.576;
WE_final = WE_final+weight.sys.fcs;
weight.system = weight.system+weight.sys.fcs;

weight.sys.hydraulic = 0.0075*WTO; %between .006-.012
WE_final = WE_final+weight.sys.hydraulic;
weight.system = weight.system+weight.sys.hydraulic;

% weight.sys.avionics = 120+20*ac.n.n+0.006*WTO;
% weight.sys.avionics = 0.575*WE^0.556*2935^0.25; %hard coded to range of 2,935 NM
weight.sys.avionics = ac.crew*(15+.032*WTO/1000)+ac.n.ne*(5+.006*WTO/1000)+.015*WTO/1000+.012*WTO;
WE_final = WE_final+weight.sys.avionics;
weight.system = weight.system+weight.sys.avionics;

% weight.sys.elec = 10.8*ac.fuse.vpax^0.7*(1-0.018*ac.fuse.vpax^0.35);
weight.sys.elec = 1163*((weight.fuelsys+weight.sys.avionics)/1000)^.506;
WE_final = WE_final+weight.sys.elec;
weight.system = weight.system+weight.sys.elec;

% weight.sys.api = 6.75*ac.fuse.lpax^1.28;
weight.sys.api = 469*(ac.fuse.vpax*(ac.crew+ac.steward+ac.npax)/10000)^.419;
WE_final = WE_final+weight.sys.api;
weight.system = weight.system+weight.sys.api;

% weight.sys.o2 = 30+1.2*ac.npax;
% weight.sys.o2 = 40+2.4*ac.npax; %basically we're assuming ETOPS?
weight.sys.o2 = 7*(ac.crew+ac.steward+ac.npax)^.702;
WE_final = WE_final+weight.sys.o2;
weight.system = weight.system+weight.sys.o2;

% weight.sys.apu = WTO*0.0073;
% WE_final = WE_final+weight.sys.apu;
% weight.system = weight.system+weight.sys.apu;

% weight.sys.furnishings = .211*WMZF^0.91; 
%NOW INCLUDES PROVISIONS ( GD )
weight.sys.furnishings = 55*(ac.npax/2)+32*ac.npax + 15*(ac.crew+ac.steward) + ac.klav*ac.npax^1.33 + ac.kbuf*ac.npax^1.12 + 109*(ac.npax*(1+ac.pc)/100)^.505 + .771*WTO/1000;
WE_final = WE_final+weight.sys.furnishings;
weight.system = weight.system+weight.sys.furnishings;

weight.sys.cargo = 0.08*ac.cargo_volume;
WE_final = WE_final+weight.sys.cargo;
weight.system = weight.system+weight.sys.cargo;

% weight.sys.auxgear = 0.01*WE;
% WE_final = WE_final+weight.sys.auxgear;
% weight.system = weight.system+weight.sys.auxgear;

weight.sys.paint = 0.004*WTO;
WE_final = WE_final+weight.sys.paint;
weight.system = weight.system+weight.sys.paint;

weight.ops = 0;
%Provision Weight
% weight.provision = 205*ac.crew + 150*ac.steward + ac.npax*14 + ac.npax*3 + ac.npax*3;
% WE_final = WE_final+weight.provision;
% weight.ops = weight.ops + weight.provision;

% weight.trappedfuel = 0.81*ac.fuelvolume^(2/3);
% WE_final = WE_final+weight.trappedfuel;

weight.crew = ac.crew*200 + ac.steward*200;
WE_final = WE_final+weight.crew;
weight.ops = weight.ops + weight.crew;


% WE_final-WE
% WE_final

delta = abs(WE_final-WE);
WE = WE_final;
WTO = WF+WP+WE;
end
weight.WE = WE;

% summarytable = table(weight.wing,weight.emp.VT+weight.emp.HT,weight.fuse,weight.lg.main+weight.lg.nose,weight.propulsors,weight.motors,weight.fuelcell,weight.battery,weight.treverse,weight.system,weight.fuelsys,weight.ops,weight.Wpay,ac.WF);
% summarytable.Properties.VariableNames = {'Wing weight','Empennage Weight','Fuselage weight','Landing Gear','Propulsors','Motors','Fuel Cells','Battery','Thrust Reversers','System Weight','Fuel Tanks','Operational Items','Payload Weight','Fuel Weight'};
% writetable(summarytable,'weight_table.xlsx');
% 
% a = struct2table(weight);
% b = struct2table(weight.emp);
% c = struct2table(weight.lg);
% d = struct2table(weight.sys);
% writetable([a,b,c,d],'weight_breakouts.xlsx');

end