clear all
clc

M = linspace(0,0.4,21);
P_shaft = 2.5e6 * 16;


for i = 1:length(M)
    M0 = M(i)
    [Fnet_lbf,FPR] = thrust2powerv02(M0,P_shaft)
    F(i) = Fnet_lbf;
    FPRg(i) = FPR;
end


yyaxis left
plot(M,F)
title('Recreation in Matlab')
xlabel('Mach')
ylabel('Thrust, lbf')

yyaxis right
plot(M,FPRg)
ylabel('FPR')