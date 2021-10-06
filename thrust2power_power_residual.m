function [Power_Residual] = thrust2power_power_residual(FPR, A8_m2, Tt2_K, pt2_Pa, p0_Pa, eta_fan)

% Return residual of nozzle mass flow as a function of FPR.
gamma = 1.4;
R_JpkgK = 287.058;
cp_JpkgK = gamma * R_JpkgK / (gamma - 1);
pt8 = pt2_Pa * FPR;
Tt8 = Tt2_K * FPR ^ ((gamma - 1) / gamma / eta_fan);
M9 = (2 / (gamma - 1) * ((pt8 / p0_Pa)^((gamma - 1) / gamma) - 1))^0.5;
M8 = min(M9, 1);
%     mdot8 = D(M8) * A8_m2 * pt8 * (gamma / R_JpkgK / Tt8)^0.5;
mdot8 = thrust2power_D(M8) * A8_m2 * pt8 * (gamma / R_JpkgK / Tt8)^0.5;
Power_Residual = mdot8 * cp_JpkgK * (Tt8 - Tt2_K);
if ~isreal(Power_Residual)
        disp('PR imag')
    end
%     delta = ;
end

