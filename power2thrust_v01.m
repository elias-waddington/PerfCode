 function [Tout,Pout] = power2thrust_v01(M0,ac,h)
% Thrust to Power Calculator, no BLI
% M0 = 0.1;
R_JpkgK = 287.058;
gamma = 1.4;
cp_JpkgK = gamma * R_JpkgK / (gamma - 1);
% cp_JpkgK = gamma * R_JpkgK / (gamma - 1);
eta_fan = 0.9;
pi_inl = 0.99;

A2_m2 = ac.prop.num  * ac.prop.a2 ;
A8_m2 = ac.prop.num  * ac.prop.a8 ;
[T0_K, a, p0_Pa, rho] = atmosisa(h/3.3);

%     # Free stream conditions
Tt0_K = T0_K * (1 + (gamma - 1) / 2 * M0^2);
pt0_Pa = p0_Pa * (Tt0_K / T0_K)^(gamma / (gamma-1));
V0_mps = M0 * (gamma * R_JpkgK * T0_K)^0.5;

%     # Fan face conditions
Tt2_K = Tt0_K;
pt2_Pa = pt0_Pa * pi_inl;

FPR_range = linspace(1.01,2,11);
i = 1;
Tout = zeros(1,length(FPR_range));
Pout = zeros(1,length(FPR_range));
for i = 1:length(FPR_range)
    FPR = FPR_range(i);
    % Maybe start iteration here
%     pt8 = pt2_Pa * FPR;
%     Tt8 = Tt2_K * FPR ^ ((gamma - 1) / gamma / eta_fan);
%     M9 = (2 / (gamma - 1) * ((pt8 / p0_Pa)^((gamma - 1) / gamma) - 1))^0.5;
%     M8 = min(M9, 1);

    pt8_Pa = pt2_Pa * FPR;
    M9 = (2 / (gamma - 1) * ((pt8_Pa / p0_Pa)^((gamma - 1) / gamma) - 1))^0.5;
    M8 = min(M9, 1);

    Tt8_K = Tt2_K * FPR ^ ((gamma - 1) / gamma / eta_fan);
    T8_K = Tt8_K / (1 + (gamma - 1) / 2 * M8^2);
    p8_Pa = pt8_Pa * (T8_K / Tt8_K) ^ (gamma / (gamma-1));
    V8_mps = M8 * (gamma * R_JpkgK * T8_K)^0.5;
    % rho8 = p8_Pa / R_JpkgK / T8_K;
    %         # Thrust
    mdot_kgps = thrust2power_D(M8) * A8_m2 * pt8_Pa * (gamma / R_JpkgK / Tt8_K)^0.5;
    Fnet_N = mdot_kgps * (V8_mps - V0_mps) + A8_m2 * (p8_Pa - p0_Pa);
    Fnet_lbf = Fnet_N*0.224809;
    Pshaft = mdot_kgps*cp_JpkgK*(Tt8_K-Tt2_K);
    Tout(i) = Fnet_lbf;
    if isnan(Tout(i))
        check = 1;
    end
    Pout(i) = Pshaft;
    
    % double check M2.
%     dblchk = thrust2power_D(.8) - mdot_kgps*sqrt(R_JpkgK*Tt2_K)/(A2_m2*pt2_Pa*sqrt(gamma))
    
end

% [save all together] -> create lookup table of F -> Pshaft for given M, h, ac.

end