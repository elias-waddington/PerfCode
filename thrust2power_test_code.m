% Thrust to Power Calculator
clear all
clc

R_JpkgK = 287.058;
gamma = 1.4;
cp_JpkgK = gamma * R_JpkgK / (gamma - 1);
eta_fan = 0.9;
pi_inl = 0.99;

A2_m2 = 16 * 0.669;
A8_m2 = 16 * 0.4558;
M0_space = linspace(0, 0.3, 21);
T0_K = 288.15;
p0_Pa = 101325;
% M0 = 0.1; %this varies for velocity
M2 = 0.65;

for i = 1:length(M0_space)
    M0 = M0_space(i)
%     # Free stream conditions
    Tt0_K = T0_K * (1 + (gamma - 1) / 2 * M0^2)
    pt0_Pa = p0_Pa * (Tt0_K / T0_K)^(gamma / (gamma-1))
    V0_mps = M0 * (gamma * R_JpkgK * T0_K)^0.5
    
%         # Fan face conditions
%     pi_inl = 0.99
    Tt2_K = Tt0_K
    pt2_Pa = pt0_Pa * pi_inl
%     mdot_kgps = D(M2) * A2_m2 * pt2_Pa * (gamma / R_JpkgK / Tt2_K)^0.5
    mdot_kgps = thrust2power_D(M2) * A2_m2 * pt2_Pa * (gamma / R_JpkgK / Tt2_K)^0.5
    
%         # Nozzle exit
%     guess = 1.1;
    verror = 1;
    FPR = 1.1;
    while abs(verror) > 0.001
        
        [mdot_kgps,mdot8] = thrust2power_test(FPR, A8_m2, Tt2_K, pt2_Pa, p0_Pa, mdot_kgps, eta_fan);
%     x0 = 1;
%     z = fzero(fun,x0)
%         verror = delta;
    verror = (mdot_kgps-mdot8)/mdot_kgps
    FPR = FPR + verror
    end
    
    pt8_Pa = pt2_Pa * FPR;
    M9 = (2 / (gamma - 1) * ((pt8_Pa / p0_Pa)^((gamma - 1) / gamma) - 1))^0.5;
    M8 = min(M9, 1);
    Tt8_K = Tt2_K * FPR ^ ((gamma - 1) / gamma / eta_fan);
    T8_K = Tt8_K / (1 + (gamma - 1) / 2 * M8^2);
    p8_Pa = pt8_Pa * (T8_K / Tt8_K) ^ (gamma / (gamma-1));
    V8_mps = M8 * (gamma * R_JpkgK * T8_K)^0.5;
    rho8 = p8_Pa / R_JpkgK / T8_K;
%         # Thrust
    Fnet_N(i) = mdot_kgps * (V8_mps - V0_mps) + A8_m2 * (p8_Pa - p0_Pa);

%     # Power
    Pshaft_W(i) = mdot_kgps * cp_JpkgK * (Tt8_K - Tt2_K);
end

yyaxis left
plot(M0_space,Fnet_N.*0.224809)
title('Recreation in Matlab')
xlabel('Mach')
ylabel('Thrust, lbf')

yyaxis right
plot(M0_space,Pshaft_W)
ylabel('Power, W')