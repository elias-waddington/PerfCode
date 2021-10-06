function [MotorPower] = calcMotorPower(Tr,velocity,h,weight)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

ThrustPower = Tr*velocity*1.35581795; % in watts
prop_eff = FanEff(h) * 0.95;
% thrust_to_powerplant_eff = prop_eff*0.95*0.995*0.97*0.99*0.95; %from ansell
MotorPower = ThrustPower/prop_eff; %in watts


end

