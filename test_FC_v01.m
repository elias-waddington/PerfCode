%Fuel Cell Efficiency and Test Script
clc

fcweight = 10000; %fc weight
preq = 1e8; %power required
h = 0.01; %altitude
M = 0.2501; %mach no.

X = allloadin_V003('FCmapR1_LD_drag.dat','y');
X
Z = interp3D_V003(X,1,1098,M,h,'y') %*fcweight