load CDwout.mat
REsweep = linspace(1.5e7,3.5e7,N_sweep);
Msweep = linspace(0.24,0.85,N_sweep);
CLsweep = linspace(0.01,1.1,N_sweep);

Re = 2.12e7;
M = 0.773;
CL = 0.48;

CDw = interp3(REsweep,Msweep,CLsweep,CDwout,Re,M,CL)
%Should give ~8.5 counts (8.5e-4)