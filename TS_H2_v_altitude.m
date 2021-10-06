% Trade Study for fuel efficiency at altitudes
close all
clear all
% mission.M = 0.774; % cruise speed

h = linspace(15000,37000,20);
M = linspace(0.3,0.85,20);
% M = 0.773;

run('aircraftfile_V02.m')
W = 187500; %lbs, constant weight for analysis
% weight.fuelcell = 17308.16;
weight.fuelcell = 3.0115e+04;

for j = 1:length(M)
    mission.M = M(j);
for i = 1:length(h)
    mission.a = speedofsound(h(i));
%     a(i) = mission.a;
    mission.altitude = h(i);
    mission.viscocity = airviscocity(h(i));
%     viscocity(i)= mission.viscocity;
%     mission.M = mission.v_cruise/mission.a;
    mission.v_cruise = mission.M*mission.a; %ft/sec
    P = airpressure(h(i));
    mission.rho = airdensity(h(i));
    rho(i) = airdensity(h(i));
    
    [ac,CD(i),CL(i),CDo(i),CDi(i),CDw(i),~,~,~] = dragBWB(ac,mission,W);
    D = 1/2*rho(i)*mission.v_cruise^2*ac.wing.S*(CD(i));
    Dgraph(i) = D;
    FF(i,j) = H2CT(D,mission.v_cruise,h(i),weight)*D;
    FpF(i,j) =  H2CT(D,mission.v_cruise,h(i),weight)*D/mission.v_cruise;
end
    
end


if length(M) == 1
        plot(h,FF)
    xlabel('Altitude, feet')
% ylabel('Mach No')
ylabel('Lbs of fuel used per second')
else
figure(1)
surf(M,h,FF)
ylabel('Altitude, feet')
xlabel('Mach No')
zlabel('Lbs of fuel used per second')

figure(2)
surf(M,h,FpF)
ylabel('Altitude, feet')
xlabel('Mach No')
zlabel('Lbs of fuel used per foot')
end
%Notes:
% Looking at altitudes from 10k to 37k feet, there are 3 distinct sections.
% The first two sections are the linear interpolation of the fuel cell
% efficiency erroring out due to the system requiring too much power. In
% short, the key metric here is aerodynamic efficiency reducing the net
% force required on the system. Would be great to have larger bounds of the
% LH2 system.

% Overall, it looks like we're ikn the right ballpark.
