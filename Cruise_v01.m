function [state,dat,counter,weight] = Cruise_v01(ac,state,weight,LRC_Mach,cruise_alt,counter,set,dat,FCmap_drag,FCmap_eff,Msweep,REsweep,CLsweep,CDout,dt,LRC_range)
mission.altitude = cruise_alt; %ft
mission.M = LRC_Mach;
mission.a = speedofsound(mission.altitude); %fps
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
mission.v_cruise = mission.M*mission.a; %fps
% cruise_spd = mission.v_cruise;
h = cruise_alt;
qbar = 0.5*mission.v_cruise^2*mission.rho;
R = LRC_range-dat.climb_dist-cruise_alt/1000*3; %NM, rule of thumb for descent
Re = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
Target_Distance = R*6076; % range in feet
Fuelburn = 0;
Distance_Flown = 0;
size_power_cbrk = false;
% climb_distance/6076
[Tcr,Pcr] = power2thrust_v01(mission.M,ac,cruise_alt);

state.W2 =  weight.final.ac;
state.WF2 =  weight.final.fuel;


while Distance_Flown < Target_Distance
%     [ac,CD,CL,CDo,CDi,CDw,~,~,CDowing] = dragBWB(ac,mission,state.W2-Fuelburn);


    CL = (state.W2-Fuelburn)/qbar/ac.wing.S;
    
    CD = interp3(Msweep,REsweep,CLsweep,CDout,mission.M,Re,CL,'spline');
    HexDrag = interp3D_V003(FCmap_drag,1,dat.P(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
    if HexDrag > 100000
%         HexDrag = interp3D_V003(FCmap_drag,1,Pgraph(counter-1)/weight.fuelcell,mission.M,mission.altitude,'n')*weight.fuelcell;
          HexDrag = dat.HD(counter-1)*0.25;
    end
    Tr = 1/2*mission.rho*mission.v_cruise^2*ac.wing.S*CD+HexDrag;
%     CT = H2CT(Tr,mission.v_cruise,h);
%     FCeff = interp2(MP_h_rng,MP_M_rng,MP_Eff_mat,,mission.altitude);
    Pshaft = interp1(Tcr,Pcr,Tr);
    P_FC = Pshaft/FC2shaft(mission.altitude);
    HotelP = HotelLoads(mission.M,mission.altitude,2);
    P_needed = P_FC+HotelP;
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
    dat.CDo(counter) = interp3(Msweep,REsweep,CLsweep,CDowing,mission.M,Re,CL,'spline');
    dat.CDo(counter) = interp3(Msweep,REsweep,CLsweep,CDofuse,mission.M,Re,CL,'spline');
    dat.Cf(counter) = interp3(Msweep,REsweep,CLsweep,Cffout,mission.M,Re,CL,'spline');
    end
%     CDowinggraph(counter) = CDowing;
    dat.h(counter) = mission.altitude;
    dat.T(counter) = Tr;
    dat.v(counter) = mission.v_cruise;
    dat.hdot(counter) = 0;
    time = dat.t(counter-1) + dt;
    dat.t(counter) = time;
    Distance_Flown = Distance_Flown + mission.v_cruise * dt ;
    dat.W(counter) = state.W2-Fuelburn;
    dat.PP(counter) = Tr*mission.v_cruise* 1.3558;
    dat.MP(counter) = Pshaft;
    dat.P(counter) = P_needed; %calcPower(Tr,mission.v_cruise,h) + HotelP;
    dat.HD(counter) = HexDrag;
    dat.M(counter) = mission.M;
    dat.Re(counter) = Re;
    dat.TSFC(counter) = H2CT(Tr,P_needed,FCeff);
    dat.Pa(counter) = FCmaxpower(weight,mission.altitude,mission.M);
    dat.Ta(counter) = interp1(Pcr,Tcr,(FCmaxpower(weight,h,mission.M)-HotelP)*FC2shaft);
    dat.Tr(counter) = Tr;
    dat.Pr(counter) = P_needed; %Tr/FC2fan(h)*1.3558*mission.v_cruise;
    
    if ~size_power_cbrk
        check_power = P_needed;
        if check_power > ac.size.sizing_power
            ac.size.sizing_power = check_power;
        end
        dat.energy_generated_so_far = dt*counter*ac.size.sizing_power;
%         ac.size.battery_energy = energy_needed_so_far-energy_generated_so_far;
%         baten = ac.size.battery_energy
        dat.baten = 0; % set to 0 for assumption, should calculate, but should always have enough power from FC no matter what
        size_power_cbrk = true;
%         ac.size.max_power = Tgraph(1)*vgraph(1)*1.3558/(0.7*0.95*0.995);
        dataout.TR_cruise = Tr;
%         if dataout.Tcruise_test < dataout.TR_cruise
%             disp('Error - Is Sized By Cruise')
%             dataout.sizedby = 37;
%         end
    end
    counter = counter + 1;


end

%     state.W3_old = (sqrt(state.W2)- R*6076/(2/CT*sqrt(CL)/CD*sqrt(2/(mission.rho*ac.wing.S))))^2;
%     state.WF3_old = state.WF2 - (state.W2-state.W3_old);
%     state.t3_old = R/(mission.v_cruise*0.592484);


    state.WF3 = state.WF2-Fuelburn;
    state.W3 = state.W2-Fuelburn;
	state.t3 = time;
    
    weight.final.fuel = state.WF3;
    weight.final.ac = state.W3;
    state.time = state.t3;

end

