function [state,dat,counter,weight] = Climb_v01(ac,state,weight,Min,Mfin,Altin,Altfin,counter,set,dat,Msweep,REsweep,CLsweep,CDout,MP_h_rng,MP_M_rng,MP_Drag_mat,MP_Eff_mat,dt)

Fuelburn = 0;
h = Altin;
time = 0;
mission.M = Min;
cruise_alt = Altfin;
LRC_Mach = Mfin;

counter = counter+1;
cint = counter;
climb_distance = 0;

mission.a = speedofsound(h);
mission.v_cruise = mission.a*mission.M;
% mission.M = mission.v_cruise/mission.a;
energy_generated_so_far = 0;
energy_needed_so_far = 0;

state.W1 = weight.final.ac;
state.WF1 = weight.final.fuel

while h < Altfin %change mission.M to linearly scale from 0.4 to cruise as increase
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
    mission.altitude = h;
    mission.viscocity = airviscocity(h);
%     mission.M = mission.v_cruise/mission.a;
%     mission.v_cruise = mission.M*mission.a; %ft/sec
%     P = airpressure(h);
    mission.rho = airdensity(h);
    rho = airdensity(h);
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W1-Fuelburn);
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    HotelP = HotelLoads(mission.M,h,1);
    CL = (state.W1-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    dat.CD(counter) = CD;
    dat.CL(counter) = CL;
    if set.drag_breakout == 1
    dat.CDo(counter) = interp3(Msweep,REsweep,CLsweep,CDoout,mission.M,Re,CL,'spline');
    dat.CDi(counter) = interp3(Msweep,REsweep,CLsweep,CDiout,mission.M,Re,CL,'spline');
    dat.CDw(counter) = interp3(Msweep,REsweep,CLsweep,CDwout,mission.M,Re,CL,'spline');
    end
    if set.dave_numbers == 1
    dat.CDowing(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    dat.CDofuse(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    dat.Cf(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    dat.M(counter) = mission.M;
%     CDprofilegraph = [CDprofilegraph,CDprofilegraph];
%     Vclimb = 1.46667*360+(761-528)/37000;
%     HexDrag = interp3D_V003(FCmap_drag,1,weight.fuelcell,mission.M,h,'n')*ac.fcweight;
    HexDrag = interp2(MP_h_rng,MP_M_rng,MP_Drag_mat,h,mission.M)*weight.fuelcell;
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag; 
    %     Ta = thrust2powerv03(mission.M,(FCmaxpower(weight,h,mission.M)-HotelP)*FC2shaft,ac,h);
    [Tout,Pout] = power2thrust_v01(mission.M,ac,h);
    Pshaft = (FCmaxpower(weight,h,mission.M)-HotelP)*FC2shaft;
    Ta = interp1(Pout,Tout,Pshaft);
    hdot = mission.v_cruise*(Ta-D)/(state.W1-Fuelburn); %in m/s
    if hdot < 100/60
        disp('Cannot Climb Enough! HDOT < 100!')
    end
    hdot_cbrk = 0;
    if h > -1 && h < 10000
        hdot_compare = (4000 - 500*((h)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
    if h >= 10000 && h < 20000
        hdot_compare = (3500 - 1000*((h-10000)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
    if h >= 20000 
        hdot_compare = (2500 - 100*((h-20000)/10000))/60;
        if hdot > hdot_compare
            hdot = hdot_compare;
            Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
            hdot_cbrk = 1;
        end
%         hdot = hdot/60;
    end
%     if h >= 30000
%         hdot_compare = (1000 - ((h-30000)/10000)*1000)/60; %set 30k + 10k > cruise_alt
%         if hdot > hdot_compare
%             hdot = hdot_compare;
%             Tr = D + hdot*(state.W1-Fuelburn)/mission.v_cruise;
%             hdot_cbrk = 1;
%         end
%         hdot = hdot/60;
%     end
    if hdot_cbrk == 0
        Tr = Ta;
    end
    
    FCeff = interp2(MP_h_rng,MP_M_rng,MP_Eff_mat,h,mission.M);
    Fuelburn = Fuelburn + H2CT(Tr,FCmaxpower(weight,h,mission.M),FCeff)*dt*Tr; %in lbs
    
    dat.h(counter) = h;
   
    dat.T(counter) = Tr;
    dat.v(counter) = mission.v_cruise;
    climb_distance = climb_distance + mission.v_cruise*dt;
    time = time+dt;
    dat.t(counter) = time;
    dat.W(counter) = state.W1 - Fuelburn;
    dat.PP(counter) = Tr*mission.v_cruise*1.3558;
    dat.MP(counter) = Pshaft;
    dat.P(counter) = FCmaxpower(weight,h,mission.M);
    dat.hdot(counter) = hdot*dt;
    dat.Re(counter) = Re;
    dat.Pa(counter) = FCmaxpower(weight,h,mission.M);
    dat.Ta(counter) = Tr;
    dat.Tr(counter) = D;
    dat.Pr(counter) = interp1(Tout,Pout,D)/FC2shaft + HotelP;
    dat.HD(counter) = HexDrag;
    dat.TSFC(counter) = H2CT(Tr,FCmaxpower(weight,h,mission.M),FCeff);
    if set.battery_use == 1
        energy_generated_so_far = energy_generated_so_far + FCmaxpower(weight,h,mission.M)*dt;
        energy_needed_so_far = energy_needed_so_far + dat.P(counter)*dt;
    end
     h = h + hdot*dt;

    counter = counter + 1;
end
state.WF2 = state.WF1-Fuelburn;
state.W2 = state.W1-Fuelburn;
state.t2 = time;

weight.final.fuel = state.WF2;
weight.final.ac = state.W2;
state.time = state.t2;

dat.time_to_climb = state.t2/dt;
dat.climb_dist = climb_distance/6076;

if climb_distance/6076 > 600
    dat.is_borked = 1;
    disp('BORK due to climb distance')
end

dat.TA_climb = [dat.T(1)*1.01,dat.T(cint:counter-1)];
dat.TA_climb_alt = [dat.h(cint:counter-1),37000];
dat.TR_V35 = max(dat.T);


end

