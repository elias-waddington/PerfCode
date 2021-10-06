clear all

weight.fuelcell = 10000;

FCmap_drag = allloadin_V003('FCmapR2_1214_drag.dat','n');
FCmap_eff = allloadin_V003('FCmapR2_1214_eff.dat','n');
load_R2_1214_cor; 

h_count = linspace(0,37000,3701);
cruise_alt = 37000;
LRC_Mach = 0.773;

delta = zeros(1,length(h_count));
pout = zeros(1,length(h_count));
tout = zeros(1,length(h_count));
tdout = zeros(1,length(h_count));

[Ti, ai, Pi, rhoi] = atmosisa(0);
[Th, ah, PH, rhoh] = atmosisa(37000/3.3);

for i = 1:length(h_count)
    h = h_count(i);
if h > -1 && h < 10000
    v20k = (-speedofsound(0)*0.25+speedofsound(10000)*0.35)/10000;
    mission.v_cruise = speedofsound(0)*0.25 + v20k*(h);
    mission.a = speedofsound(h);
    mission.M = mission.v_cruise/mission.a;
%         hdot = 4000 - 500*((h)/10000);
%         hdot = hdot/60;
end
if h >= 10000 && h < 20000
    v20k = (speedofsound(20000)*0.45-speedofsound(10000)*0.35)/10000;
    mission.v_cruise = speedofsound(10000)*0.35 + v20k*(h-10000);
    mission.a = speedofsound(h);
    mission.M = mission.v_cruise/mission.a;
%         hdot = 3500 - 1000*((h-10000)/10000);
%         hdot = hdot/60;
end
if h >= 20000 && h < 30000
    v20k = (speedofsound(30000)*0.55-speedofsound(20000)*0.45)/10000;
    mission.v_cruise = speedofsound(20000)*0.45 + v20k*(h-20000);
    mission.a = speedofsound(h);
    mission.M = mission.v_cruise/mission.a;
%         hdot = 2500 - 1500*((h-20000)/10000);
%         hdot = hdot/60;
end
if h >= 30000
    v20k = (speedofsound(cruise_alt)*LRC_Mach-speedofsound(30000)*0.55)/(cruise_alt-30000);
    mission.v_cruise = speedofsound(30000)*0.55 + v20k*(h-30000);
    mission.a = speedofsound(h);
    mission.M = mission.v_cruise/mission.a;
%         hdot = 1000 - ((h-30000)/10000)*1000; %set 30k + 10k > cruise_alt
%         hdot = hdot/60;
end
[T, A, P, RHO] = atmosisa(h/3.3);
% delta(i) = P/Pi;

HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
% delta(i) = P/Pi;
delta(i) = RHO/rhoi;
pout(i) = (FCmaxpower(weight,h,mission.M)-HexDrag*mission.v_cruise* 1.3558)/FCmaxpower(weight,0,0.25); %*FanEff(h);
tout(i) = FanEff(h)*0.95*0.995*0.97*0.99*0.95.*pout(i)/1.3558/mission.v_cruise;
tdout(i) = FanEff(h)*0.95*0.995*0.97*0.99*0.95.*delta(i)/1.3558/mission.v_cruise;
end

pout = pout + (1-max(pout));

figure(1)
plot(tout./tout(1)-tout(length(tout))/tout(1)+tdout(length(tdout))/tdout(1),h_count,tdout./tdout(1),h_count)
xlabel('Thrust Available, Non-dimensionalised')
ylabel('Altitude, Ft')
title('Thrust Lapse Comparison, Sized by Top of Climb')
legend('Fuel Cell','Jet Engine','Location','northeast')

figure(2)
plot(pout-pout(length(pout))/pout(1)+delta(length(delta))/delta(1),h_count,delta,h_count)
xlabel('Power Available, Non-dimensionalised')
ylabel('Altitude, feet')
title('Power Lapse Comparison, Sized by Top of Climb')
legend('Fuel Cell','Jet Engine','Location','northeast')

% figure(3)
% plot(