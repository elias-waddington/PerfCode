function [state,dat,counter,weight]  = Descent_v01(ac,state,weight,Min,Altin,Altfin,counter,set,dat,FCmap_eff,Msweep,REsweep,CLsweep,CDout,dt,MP_h_rng,MP_M_rng,LP_Drag_mat,LP_Eff_mat)
Fuelburn = 0;
h = Altin;
mission.M = Min;
cruise_spd = Min*speedofsound(h);
% hdot = 1000/60; %fps
% TA_climb_alt = dat.h(1:50);
% TA_climb = dat.Ta(1:50);

state.WF3 = weight.final.fuel;
state.W3 = weight.final.ac;
% state.time = state.t3;

while h > 0
    if h >= Altfin && h < 10000
        mission.v_cruise = 421.95 ;% - (10000-h)/10000*(185.66);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         v20k = (-speedofsound(0)*0.25+speedofsound(10000)*0.35)/10000;
%         mission.v_cruise = speedofsound(0)*0.25 + v20k*(h);
%         idle_thrust_percent = 0.1-h/10000*0.1;
    end
    if h > 10000 && h < 25000
%         hdot = 1500/60; %fps
        mission.v_cruise = cruise_spd - (25000-h)/15000*(cruise_spd-421.95);
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         idle_thrust_percent = 0.1;
    end
    if h >= 25000 
%         hdot = 1000/60; %fps
        mission.v_cruise = cruise_spd;
        mission.a = speedofsound(h);
        mission.M = mission.v_cruise/mission.a;
%         idle_thrust_percent = 0.1;
    end
    mission.altitude = h;
%     mission.a = speedofsound(h);
    mission.viscocity = airviscocity(h);    
%     mission.M = mission.v_cruise/mission.a;
    HotelP = HotelLoads(mission.M,h,3);
%     P = airpressure(h);
    rho = airdensity(h);
    mission.rho = rho;
    
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    CL = (state.W3-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
%     HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
% %     HexDrag = 100; %lbs, assuming this now because this will have to be fixed with HOTEL LOADS
% %     D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag;
% %     Tr = idle_thrust_percent*interp1(dat.TA_climb_alt,dat.TA_climb,h);
    HexDrag = interp2(MP_h_rng,MP_M_rng,LP_Drag_mat,h,mission.M)*weight.fuelcell;
    Pshaft = (FCminpower(weight,h,mission.M)-HotelP)*FC2shaft;
    D = 1/2*rho*mission.v_cruise^2*ac.wing.S*(CD) + HexDrag;
    [Tout,Pout] = power2thrust_v01(mission.M,ac,h);
    Tr = interp1(Pout,Tout,Pshaft);

%     Tr = D - hdot*(state.W3-Fuelburn)/mission.v_cruise;
    hdot = -mission.v_cruise*(Tr-D)/(state.W3-Fuelburn);
    FCeff = interp2(MP_h_rng,MP_M_rng,LP_Eff_mat,h,mission.M);
    Fuelburn = Fuelburn + H2CT(Tr,FCminpower(weight,h,mission.M),FCeff)*dt*Tr;
    h = h - hdot*dt;
    if h < 0
        h = 0;
    end

    time = dat.t(counter-1) + dt;
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
    [Tout,Pout] = power2thrust_v01(mission.M,ac,h);
%     CDowinggraph(counter) = CDowing;
    dat.h(counter) = mission.altitude;
    dat.T(counter) = Tr;
    dat.v(counter) = mission.v_cruise;
    dat.t(counter) = time;
    dat.W(counter) = state.W3-Fuelburn;
    dat.M(counter) = mission.M;
    dat.PP(counter) = Tr*mission.v_cruise* 1.3558;
    dat.MP(counter) = interp1(Tout,Pout,Tr);
    dat.P(counter) = interp1(Tout,Pout,Tr)/FC2shaft + HotelP;
    dat.HD(counter) = HexDrag;
    dat.hdot(counter) = -hdot*dt;
    dat.Re(counter) = Re;
    dat.Pa(counter) = FCmaxpower(weight,h,mission.M);
    dat.Ta(counter) = interp1(Pout,Tout,(FCmaxpower(weight,h,mission.M)-HotelP)*FC2shaft);
    dat.Tr(counter) = D;
    dat.Pr(counter) = interp1(Tout,Pout,D)/FC2shaft(mission.altitude);
    dat.TSFC(counter) = H2CT(Tr,dat.P(counter-1),FCeff);

    counter = counter + 1;
end

state.W4 = state.W3-Fuelburn;
state.WF4 = state.WF3 - Fuelburn;
state.t4 = time; %descent_distance/(mission.v_cruise*0.592484);

weight.final.fuel = state.WF4;
weight.final.ac = state.W4;
state.time = state.t4;

end

