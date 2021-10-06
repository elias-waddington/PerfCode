function [CT] = H2CT(Tr,FCpower,FCeff)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% FCeff = FCefficiency(h,PowerplantPower,weight);

H2PowerDemand = FCpower/FCeff/(1-0.0025); %1%/day loss, so 1/4% loss in flight: from Wolfgang for LH2 losses

H2Edensity = 119930040; %watt-seconds/kg < really joules/kg
H2masspersecond = H2PowerDemand/H2Edensity; %to kg/sec
H2masspersecond = H2masspersecond*2.2; %to lbs/sec

CT = H2masspersecond/Tr; %lbm/lbf-sec;


end

