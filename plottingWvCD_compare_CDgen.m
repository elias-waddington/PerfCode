%iterative M vs CD plot
%constant weight, variable M, CD
clc
clear all
close all
run('aircraftfile_V02.m')
% run('b737aircraftfile.m')
mission.altitude = 37000; %ft
mission.M = 0.78;
mission.a = speedofsound(mission.altitude);
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
% W = 173804.4*.9; %lbs
% W = linspace(1,350000);
W = linspace(20000,350000);

% W = 190000;
mission.v_cruise = mission.M*mission.a; %in fps
mission.qbar = 0.5*mission.v_cruise^2*mission.rho;
CL_sweep = W./(mission.qbar*ac.wing.S);
N_sweep = 30;
[REsweep,Msweep,CLsweep, CDout, CDoout, CDiout, CDwout] = CDgen(ac,N_sweep);
Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
for i = 1:length(W)

    
%     [ac,CD,CL,CDo,CDi,CDw,ProfileDrag] = dragBWB(ac,mission,W(i));
    CLgraph(i) = CL_sweep(i);
    CDgraph(i) = interp3(REsweep,Msweep,CLsweep,CDout,Re,mission.M,CL_sweep(i));
%     CLgraph = CL_sweep;
    CDograph(i) = interp3(REsweep,Msweep,CLsweep,CDoout,Re,mission.M,CL_sweep(i));;
    CDigraph(i) = interp3(REsweep,Msweep,CLsweep,CDiout,Re,mission.M,CL_sweep(i));;
    CDwgraph(i) = interp3(REsweep,Msweep,CLsweep,CDwout,Re,mission.M,CL_sweep(i));;
%     PDgraph(i) = ProfileDrag;
    LDgraph(i) = CLgraph(i)/CDgraph(i);
end

run('aircraftfile_V02.m')
i = 1;
% W = linspace(0,350000);
for i = 1:length(W)

    
    [ac,CD,CL,CDo,CDi,CDw,ProfileDrag] = dragBWB(ac,mission,W(i));
    CDgraph2(i) = CD;
    CLgraph2(i) = CL;
    CDograph2(i) = CDo;
    CDigraph2(i) = CDi;
    CDwgraph2(i) = CDw;
    PDgraph2(i) = ProfileDrag;
    LDgraph2(i) = CL/CD;
end

figure(1)
plot(W,CDgraph)
title('W vs D at Constant Mach')
xlabel('Weight')
ylabel('CD')

figure(2)
plot(CDgraph,CLgraph,CDgraph2,CLgraph2)
title('Drag Polar')
xlabel('C_D')
ylabel('C_L')
legend('PreGen','Insitu','location','SouthEast')

figure(3)
plot(CLgraph,LDgraph,CLgraph2,LDgraph2)
title('L/D Graph')
xlabel('C_L')
ylabel('L/D')
legend('PreGen','Insitu','location','SouthEast')

figure(4)
plot(CLgraph,CDigraph,CLgraph2,CDigraph2)
title('Induced Drag Comparison')
xlabel('C_L')
ylabel('Induced Drag')
legend('PreGen','Insitu','location','SouthEast')

figure(5)
plot(CLgraph,CDograph,CLgraph2,CDograph2)
title('PArasite Drag Comparison')
xlabel('C_L')
ylabel('Parasite Drag')
legend('PreGen','Insitu','location','SouthEast')

figure(6)
plot(CLgraph,CDwgraph,CLgraph2,CDwgraph2)
title('Wave Drag Comparison')
xlabel('C_L')
ylabel('Wave Drag')
legend('PreGen','Insitu','location','SouthEast')

outputtable = table(CDgraph',CLgraph',CDgraph2',CLgraph2')
outputtable.Properties.VariableNames = {'737CD','737CL','cCD','cCL'}
outputfilename = ['comparison.csv'];
writetable(outputtable,outputfilename);

outputtable = table(CLgraph',LDgraph',CLgraph2',LDgraph2')
outputtable.Properties.VariableNames = {'737CD','737CL','cCD','cCL'}
outputfilename = ['comparison2.csv'];
writetable(outputtable,outputfilename);