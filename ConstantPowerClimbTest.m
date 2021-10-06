% Max Power from sea level to cruise calculations and corresponding thrust
% requirements/hdot
clear all
run('aircraftfile_V02.m')
close all

% h = linspace(0,37000,371);
weight.fuelcell = 23466.40905;

% hgraph = 0;
% dt = 60;
% i = 1;
% Fuelburn = 0;
% h = 1;
% h_old(1) = 0;
% counter = 1;
fuelcell_weights = [30e6/1328.26164,40e6/1328.26164,50e6/1328.26164];
FClabel = [30 ,40,50];
FClabel = num2str(FClabel);
W = 200000;


% for j = 1:3
%     weight.fuelcell = fuelcell_weights(j);
weight.fuelcell = 4e7/1328;
    hgraph = 0;
    dt = 60;
    i = 1;
    Fuelburn = 0;
    h = 1;
    h_old(1) = 0;
    W = 200524-23466.4+weight.fuelcell;
while hgraph(i) < 37000
    h = hgraph(i);
    MaxPowerfromFC(i) = FCmaxpower(weight,h);
    MaxThrustPower(i) = MaxPowerfromFC(i) * FanEff(h) * 0.92*0.95*0.995*0.97*0.99*0.99*0.95;

    if h > -1 && h < 10000
        v20k = (-speedofsound(0)*0.2+speedofsound(10000)*0.35)/10000;
        mission.v_cruise = speedofsound(0)*0.2 + v20k*(h);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;

        hdot_old(i) = 4000 - 500*((h)/10000);
        
%         hdot = hdot/60;
    end
    if h >= 10000 && h < 20000
        v20k = (speedofsound(20000)*0.45-speedofsound(10000)*0.35)/10000;
        mission.v_cruise = speedofsound(10000)*0.35 + v20k*(h-10000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         Tavail(i) = MaxThrustPower(i)/mission.v_cruise;
        hdot_old(i) = 3500 - 1000*((h-10000)/10000);
        
%         hdot = hdot/60;
    end
    if h >= 20000 && h < 30000
        v20k = (speedofsound(30000)*0.55-speedofsound(20000)*0.45)/10000;
        mission.v_cruise = speedofsound(20000)*0.45 + v20k*(h-20000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         Tavail(i) = MaxThrustPower(i)/mission.v_cruise;
        hdot_old(i) = 2500 - 1500*((h-20000)/10000);
%         hdot = hdot/60;
    end
    if h >= 30000
        v20k = (speedofsound(37000)*0.775-speedofsound(30000)*0.55)/(37000-30000);
        mission.v_cruise = speedofsound(30000)*0.55 + v20k*(h-30000);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         Tavail(i) = MaxThrustPower(i)/mission.v_cruise;
        hdot_old(i) = 1000 - ((h-30000)/10000)*1000; %set 30k + 10k > cruise_alt
%         hdot = hdot/60;
    end
    h_old(i+1) = h_old(i) + hdot_old(i)/60*dt;
    Tavail(i) = MaxThrustPower(i)/mission.v_cruise/1.3558;
    mission.rho = airdensity(h);
    mission.altitude = h;
    mission.viscocity = airviscocity(h);
    [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,W-Fuelburn);
    Fuelburn = Fuelburn + H2CT(Tavail(i),mission.v_cruise,h,weight)*dt*Tavail(i);
    rho = mission.rho;
    vgraph(i) = mission.v_cruise;
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD);
    Dgraph(i) = D;
%     Ta = MaxThrustPower(i)/mission.v_cruise;
    hdot = (Tavail(i)-D)*mission.v_cruise/(W-Fuelburn);
    hdotgraph(i) = hdot*dt;
    
    hgraph(i+1) = h + hdot*dt;
    i = i+1;
end
hgraph = hgraph(1:i-1);
h_old = h_old(i:i-1);
% time2climb(j) = length(hgraph);
j = 1;
figure(1)
subplot(3,1,1)
plot(MaxPowerfromFC,hgraph)
ylabel('Altitude')
xlabel('Max Power from Fuel Cells, Watts')
title(sprintf('Fuel Cell Power Lapse for %i MW System',FClabel(j)))


figure(2)
subplot(3,1,2)
plot(hgraph)
xlabel('Time, Minutes')
ylabel('Feet per Minute Climb Rate')
title('Climb rate at various speeds')

figure(3)
subplot(3,1,3)
plot(hdotgraph,hgraph)
ylabel('Altitude, Feet, MSL')
xlabel('Feet per Minute Climb Rate')
title('Climb rate at altitudes')

clear MaxPowerfromFC
clear MaxThrustPower
clear hdotgraph

Tavail(1)

% end

figure(5)
plot(Dgraph,hgraph)

