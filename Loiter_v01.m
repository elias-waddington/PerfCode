function [state,dat,counter,weight]  = Loiter_v01(ac,state,weight,M,cruise_alt,counter,set,dat,Msweep,REsweep,CLsweep,CDout,FCmap_drag,FCmap_eff, E_alt,dt)
% E_alt = 0.5; %hrs
mission.M = M;
% state.W7_old = state.W6/(exp(E_alt*3600*CT/(CL/CD)));
% state.WF7_old = state.WF6 - (state.W6-state.W7_old);
% state.t7_old = E_alt;
Fuelburn = 0;
loiter_time = 0;
mission.v_cruise = mission.M*speedofsound(cruise_alt);
[Talt,Palt] = power2thrust_v01(mission.M,ac,cruise_alt);
mission.viscocity = airviscocity(cruise_alt);
h = cruise_alt;
rho = airdensity(h);
mission.rho = rho;
 qbar = 0.5*mission.v_cruise^2*rho;
 mission.altitude = h;
 
 state.WF6 = weight.final.fuel;
state.W6 = weight.final.ac;

while loiter_time < E_alt*60*60+1
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W6-Fuelburn);
   
    CL = (state.W6-Fuelburn)/qbar/ac.wing.S;
    Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    HexDrag = interp3D_V003(FCmap_drag,1,dat.P(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    if HexDrag > 90000
        HexDrag = dat.HD(counter-1)*0.25;
    end
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD + HexDrag;
    Pshaft = interp1(Talt,Palt,Tr);
    P_FC = Pshaft/FC2shaft(mission.altitude);
    HotelP = HotelLoads(mission.M,mission.altitude,4);
    P_needed = HotelP + P_FC;
%     CT = H2CT(Tr,mission.v_cruise,h);
    FCeff = interp3D_V003(FCmap_eff,1,P_needed/weight.fuelcell,mission.M,mission.altitude,'n');
    Fuelburn = Fuelburn + H2CT(Tr,P_needed,FCeff)*dt*Tr;
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
    dat.h(counter) = mission.altitude;
    dat.T(counter) = Tr;
    dat.v(counter) = mission.v_cruise;
%     time = time + dt;
    loiter_time = loiter_time + dt;
    dat.t(counter) = dat.t(counter-1) + dt;
    dat.W(counter) = state.W6 - Fuelburn;
    dat.M(counter) = mission.M;
    dat.PP(counter) = Tr*mission.v_cruise* 1.3558;
    dat.MP(counter) = calcMotorPower(Tr,mission.v_cruise,h);
    dat.P(counter) = calcPower(Tr,mission.v_cruise,h)+HotelP;
    dat.HD(counter) = HexDrag;
    dat.hdot(counter) = 0;
%     Distance_Flown = Distance_Flown + mission.v_cruise * dt ;
    dat.Re(counter) = Re;
    dat.TSFC(counter) = H2CT(Tr,P_needed,FCeff);
    dat.Pa(counter) = FCmaxpower(weight,h,mission.M);
    dat.Ta(counter) = FCmaxpower(weight,h,mission.M)*FC2fan(h)/1.3558/mission.v_cruise;
    dat.T(counter) = Tr;
    dat.Pr(counter) = P_needed; %Tr/FC2fan(h)*1.3558*mission.v_cruise;
    
    counter = counter + 1;

end

state.W7 = state.W6 - Fuelburn;
state.WF7 = state.WF6 - Fuelburn;
% state.t7 = time;

weight.final.fuel = state.WF7;
weight.final.ac = state.W7;
% state.time = state.t7;
end

