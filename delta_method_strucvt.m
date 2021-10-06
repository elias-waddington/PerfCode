clear all
% close all
%double check with metric to see if it makes a difference. nope!

% A-4 Skyhawk
% AR = 2.91;
% tc = 0.0748;
% c1_4_sweep = 33.21;
% hc = 0.773;
% run('b737aircraftfile')

% % 737
AR = 9.45;
c1_4_sweep = 25.02; %(25.5*(1-0.29)+30*0.29);
hc = 0.015;
tc = 0.1203;


% % L-1011 SC
% AR = 6.95;
% c1_4_sweep = 35;
% hc = 1.6;
% tc = 0.1050;


Mach = linspace(0.7,0.85);
% Mach = 0.8;
CL = 0.4;
CDwave = zeros(1,100);

    [iCLAR,iCLdest] = importfig1('fig1.csv');
    CLdes = interp1(iCLAR,iCLdest,AR)*cos(deg2rad(c1_4_sweep))*(1+hc/10)/sqrt(AR);

    [iCLdesin, itc23, iM2D2D1out] = importfig3('fig3.csv');
    M2D2D1 = interp2(iCLdesin,itc23,iM2D2D1out,CLdes,tc^(2/3));
%     M2D2D1 = -.399;
    MDD = sqrt(1+M2D2D1);


    [iinvAR, idMAR] = importfig4dMAR('fig4deltaMAR.csv');
    deltaMAR = interp1(iinvAR,idMAR,1/AR);

    [C4deg, dMDC4] = importfig4C4('fig4deltaMDc4.csv');
    deltaMDC4 = interp1(C4deg,dMDC4,c1_4_sweep);

for i = 1:length(Mach)
    mission.M = Mach(i);
    
%%%Delta Method for Wing Components

    deltaCL = CL-CLdes;



    Mdes = MDD+deltaMAR+deltaMDC4;
%     Mdes = MDD;
%         Mdes = 0.75;
%     Mcruise = mission.M;
    Mcruise = mission.M;
    deltaM = -Mdes+Mcruise;

    [itc23, imdes, idcdc] = importfig10('fig10.csv');
    dcdc_raw = interp2(itc23,imdes,idcdc,tc^(2/3),deltaM);
    delta_cdc_wing = dcdc_raw*tc^(5/3)*(1+hc/10);

    [outFN, outWF, out03] = allloadin_V003('f16t20v2.dat','n');

    dCdpnstuff = interp3D_V003(outFN,1,deltaM,deltaCL,AR*tc^(1/3),'n');

    delta_CDP_wing = dCdpnstuff*tc^(1/3)*(1+hc/10);

    if isnan(delta_cdc_wing)
        delta_cdc_wing = 0;
%         sprintf('Compressibility Drag Error, Assumed 0')
    end

    if dCdpnstuff == 9999999
        delta_CDP_wing = 0;
%         sprintf('Pressure Drag Error, Assumed 0')
    end
    
    if Mcruise < Mdes

    delta_CDP_wing = delta_CDP_wing*-((Mdes-0.2)-Mcruise)/0.2;
    delta_cdc_wing = delta_cdc_wing*-((Mdes-0.2)-Mcruise)/0.2;

% This is a hack to get the deltaCDPwing that is vaguely okay and doesn't
% spike.
    end
    if mission.M > 0.81;
        [iSbSpi, iMach, iCdpi] = importfig13('fig13.csv');
        SbSpi = 1+3/127.5711;
        Cdpi1 = interp2(iSbSpi, iMach, iCdpi, SbSpi, mission.M);
        Cdpi = Cdpi1/(127/12)^2*(12)^2*pi/1285;
    else
        Cdpi = 0;
    end
    
    CDwave(i) = delta_CDP_wing+delta_cdc_wing+Cdpi;
end


exp = readtable('737 experimental at cl04.csv');
exp = table2array(exp);
Mexp = exp(:,1);
CDexp = exp(:,2);

vos = readtable('737 vos delta at cl04.csv');
vos = table2array(vos);
Mvos = vos(:,1);
CDvos = vos(:,2);

figure(1)
plot(Mach,CDwave*10000,Mexp,CDexp,Mvos,CDvos)
xlabel('Mach Number')
ylabel('C_D, Counts')
title('737 Wave Drag Coefficient CL 0.4')
legend({'Calculated Wave Drag','Experimental Wave Drag','Vos Wave Drag'},'Location','northwest')

% M = [Mach;CDwave*10000]
% writematrix(M,'output1.csv')
% M = [Mexp,CDexp]
% writematrix(M,'output2.csv')
% M = [Mvos,CDvos]
% writematrix(M,'output3.csv')


% exp = readtable('737 experimental at cl02.csv');
% exp = table2array(exp);
% Mexp = exp(:,1);
% CDexp = exp(:,2);
% 
% vos = readtable('737 vos delta at cl02.csv');
% vos = table2array(vos);
% Mvos = vos(:,1);
% CDvos = vos(:,2);
% 
% figure(2)
% plot(Mach,CDwave*10000,Mexp,CDexp,Mvos,CDvos)
% xlabel('Mach Number')
% ylabel('C_D, Counts')
% title('737 Wave Drag Coefficient CL 0.2')
% legend({'Calculated Wave Drag','Experimental Wave Drag','Vos Wave Drag'},'Location','northwest')