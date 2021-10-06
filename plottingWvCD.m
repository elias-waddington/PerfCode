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
W = linspace(1,240000);
% W = 190000;
mission.v_cruise = mission.M*mission.a; %in fps
mission.qbar = 0.5*mission.v_cruise^2*mission.rho;

for i = 1:length(W)

    
    [ac,CD,CL,CDo,CDi,CDw,ProfileDrag] = dragBWB(ac,mission,W(i));
    CDgraph(i) = CD;
    CLgraph(i) = CL;
    CDograph(i) = CDo;
    CDigraph(i) = CDi;
    CDwgraph(i) = CDw;
    PDgraph(i) = ProfileDrag;
    LDgraph(i) = CL/CD;
end

run('aircraftfile_V02.m')
i = 1;
W = linspace(0,350000);
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
legend('B737-800','CHEETA','location','SouthEast')

figure(3)
plot(CLgraph,LDgraph,CLgraph2,LDgraph2)
title('L/D Graph')
xlabel('C_L')
ylabel('L/D')
legend('B737-800','CHEETA','location','SouthEast')

figure(4)
plot(CLgraph,CDwgraph)
title('Wave Drag vs CL')
xlabel('CL')
ylabel('Wave Drag')

% plot(CDgraph,CLgraph)



% plot(CDgraph,CLgraph)

outputtable = table(CDgraph',CLgraph',CDgraph2',CLgraph2')
outputtable.Properties.VariableNames = {'737CD','737CL','cCD','cCL'}
outputfilename = ['comparison.csv'];
writetable(outputtable,outputfilename);

outputtable = table(CLgraph',LDgraph',CLgraph2',LDgraph2')
outputtable.Properties.VariableNames = {'737CD','737CL','cCD','cCL'}
outputfilename = ['comparison2.csv'];
writetable(outputtable,outputfilename);