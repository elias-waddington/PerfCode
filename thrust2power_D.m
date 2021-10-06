function [output] = thrust2power_D(M)
%  Return corrected flow per unit area for given Mach number, M.
gamma = 1.4;
output = M / (1 + (gamma - 1) / 2 * M^2)^((gamma + 1) / 2 / (gamma - 1));
end

