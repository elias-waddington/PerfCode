% HEXdrag_core test

clear 
FCmap_drag = allloadin_V003('FCmapR2_1214_drag.dat','n');
FCmap_eff = allloadin_V003('FCmapR2_1214_eff.dat','n');
load_R2_1214_cor; 

M = .773;
h = 37000;

load_R2_1214_cor; 
P_needed = 16e6; %watts
weight.fuelcell = 20000; %lbs


FCeff = interp3D_V003(FCmap_eff,1,P_needed/weight.fuelcell,M,h,'n');
HexDrag = interp3D_V003(FCmap_drag,1,P_needed/weight.fuelcell,M,h,'n')*weight.fuelcell

N_stacks = weight.fuelcell/(365.3*2.2);

Ar = .986*N_stacks;
Hdot = P_needed*(1-FCeff)/FCeff;

[Dcore] = HEXdrag_core(M,h,Ar,Hdot)