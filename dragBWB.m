function [ac,CD,CL,CDo,CDi,CDw,CDC,CDP,CDowing] = dragBWB(ac,mission,W)
    
    qbar = 0.5*mission.v_cruise^2*mission.rho;
    
    B = sqrt(1-mission.M^2);
    
    
    %%%Wing Drag
    %Wing Zero-lift Drag
    
    RNfus = mission.rho*mission.v_cruise*ac.fuse.length/mission.viscocity;
    
    Rwf = Rwf_find(mission.M,RNfus);
%     Rwf = 1.08;
    RLS = RLS_find(mission.M,ac.wing.ct_cmax_sweep);
    
    
    %this CFW uses a RN based off CMAC, not RNfus
    RN = mission.rho*mission.v_cruise*ac.wing.MAC/mission.viscocity;
    Cfw = .455/(log10(RN))^2.58/(1+.144*mission.M^2)^.65;
    
%     CDow_old = ac.wing.Rwf*ac.wing.RLS*ac.wing.Cfw*(1+ac.wing.L1*ac.wing.tc+100*ac.wing.tc^4)*ac.wing.Swetw/ac.wing.S;
    CDow = Rwf*RLS*Cfw*(1+ac.wing.L1*ac.wing.tc+100*ac.wing.tc^4)*ac.wing.Swetw/ac.wing.S;
    CDowing = CDow;
    %Wing lift drag
    nu = ac.wing.Cla/(2*pi/B);
    
    w = ac.wing.wB/B;
    
    CL = W/qbar/ac.wing.S;
    CLw = CL*1.; %need update
    F = 1.07*(1+ac.fuse.d/ac.wing.bref)^2;
    CLaw = 2*pi*ac.wing.A/(2+sqrt(4+ac.wing.A^2*B^2/nu^2*(1+tand(ac.wing.ct_cmax_sweep)^2/B^2)))*ac.wing.Swetw/ac.wing.S*F;
    R = 0.927; %%need to update with lookup? - Fig 4.7
%     e = 1.1*CLaw/ac.wing.A/(R*(CLaw/ac.wing.A)+(1-R)*pi);
%     e = 4.61*(1-0.045*ac.wing.AR^0.68)*cosd(ac.wing.c1_4_sweep)^0.15-3.1
%     Raymer method. Pretty crappy.
%     e = 1.78*(1-0.045*ac.wing.AR^.68)-0.64
    e = 0.95; %fixed, should be okay
    epsilon_t = ac.wing.incidence_root - ac.wing.incidence_tip;
    epsilon_t = deg2rad(epsilon_t);

    CDLw = CLw^2/(pi*ac.wing.A*e)+2*pi*CLw*epsilon_t*ac.wing.v+R*pi^2*epsilon_t^2*w;
    CDwing = CDow + CDLw;
    
%     ProfileDrag = R*pi^2*epsilon_t^2*w+CDow;
    
    %%%Delta Method for Wing Components
    if exist('ac.CLdes','var') == 0
        [iCLAR,iCLdest] = importfig1('fig1.csv');
        ac.CLdes = interp1(iCLAR,iCLdest,ac.wing.AR)*cos(deg2rad(ac.wing.c1_4_sweep))*(1+ac.wing.hc/10)/sqrt(ac.wing.AR);
    end
    deltaCL = CL-ac.CLdes;
    
    if exist('ac.Mdes','var') == 0
        [iCLdesin, itc23, iM2D2D1out] = importfig3('fig3.csv');
        M2D2D1 = interp2(iCLdesin,itc23,iM2D2D1out,ac.CLdes,ac.wing.tcavg^(2/3));
        ac.MDD = sqrt(1+M2D2D1);

        [iinvAR, idMAR] = importfig4dMAR('fig4deltaMAR.csv');
        ac.deltaMAR = interp1(iinvAR,idMAR,1/ac.wing.AR);

        [C4deg, dMDC4] = importfig4C4('fig4deltaMDc4.csv');
        ac.deltaMDC4 = interp1(C4deg,dMDC4,ac.wing.c1_4_sweep);

        ac.Mdes = ac.MDD+ac.deltaMAR+ac.deltaMDC4;
    end
%     Mdes = MDD;

%     Mcruise = mission.M;

% if ac.Mdes-0.25 < mission.M;

    deltaM = -ac.Mdes+mission.M;

    [itc23, imdes, idcdc] = importfig10('fig10.csv');
    dcdc_raw = interp2(itc23,imdes,idcdc,ac.wing.tcavg^(2/3),deltaM);
    delta_cdc_wing = dcdc_raw*ac.wing.tcavg^(5/3)*(1+ac.wing.hc/10);

    [outFN, outWF, out03] = allloadin_V003('f16t20v2.dat','n');

    dCdpnstuff = interp3D_V003(outFN,1,deltaM,deltaCL,ac.wing.AR*ac.wing.tcavg^(1/3),'n');

    delta_CDP_wing = dCdpnstuff*ac.wing.tcavg^(1/3)*(1+ac.wing.hc/10);

    if isnan(delta_cdc_wing)
        delta_cdc_wing = 0;
%         sprintf('Compressibility Drag Error, Assumed 0')
    end

    if dCdpnstuff == 9999999
        delta_CDP_wing = 0;
%         sprintf('Pressure Drag Error, Assumed 0')
    end
    
    if mission.M < ac.Mdes
      offset = 0.2;
    delta_CDP_wing = delta_CDP_wing*(-((ac.Mdes-offset)-mission.M)/offset)^2;
    delta_cdc_wing = delta_cdc_wing*(-((ac.Mdes-offset)-mission.M)/offset)^2;
   end
% This is a hack to get the deltaCDPwing that is vaguely okay and doesn't
% spike.
 
% else
%     delta_cdc_wing = 0;
%     delta_CDP_wing = 0;
% end
    
    %%%Fuselage Drag
    %CDofus values
    df = (ac.fuse.width+ac.fuse.height)/2;
    lf = ac.fuse.length; %should be same since tail is short in X
%     Rwf = 1; %Part 6, Page 44 "Use 1 for just fuselage"
    Cffus = .455/(log10(RNfus))^2.58/(1+.144*mission.M^2)^.65;
    Rwf = Rwf_find(mission.M,RNfus);
%     Rwf = 1.08;
    db = sqrt(4/pi*ac.fuse.Sbfus);
    df = sqrt(4/pi*ac.fuse.Sfus);
    CDofusbase = Rwf*Cffus*(1+60/(lf/df)^3+.0025*(lf/df))*ac.fuse.Swet/ac.wing.S;
    CDbfus = (0.029*(db/df)^3/(CDofusbase*(ac.wing.S/ac.fuse.Sfus))^0.5)*ac.fuse.Sfus/ac.wing.S;
    CDofus = Rwf*Cffus*(1+60/(lf/df)^3+.0025*(lf/df))*ac.fuse.Swet/ac.wing.S+CDbfus;

    %CDLfus values
    CLalpha =  2*pi*(ac.wing.A/(2+sqrt(4+ac.wing.A^2)));
    alpha = ((W/qbar/ac.wing.S)-ac.fuse.CLo)/CLalpha; %in radians
    [Min,CDCf] = Cdc_find('fig420.csv');
    cdcf = interp1(Min,CDCf,abs(mission.M*sin(alpha)));
    CDLfus = 2*alpha^2*ac.fuse.Sbfus/ac.wing.S+nu*cdcf*alpha^3*ac.fuse.Splffus/ac.wing.S;
    CDfus = CDofus + CDLfus;
    
    %%%Delta Method for Fuselage Compressibility Drag
    if mission.M > 0.81;
        [iSbSpi, iMach, iCdpi] = importfig13('fig13.csv');
        SbSpi = 1+ac.fuse.Sbfus/ac.fuse.Sfus;
        Cdpi = interp2(iSbSpi, iMach, iCdpi, SbSpi, mission.M);
        Cdpi = Cdpi/(ac.fuse.length/(ac.fuse.width+ac.fuse.height)*2)^2*((ac.fuse.height+ac.fuse.width)/4)^2*pi/ac.wing.S;
    else
        Cdpi = 0;
    end
    
    
    %%% Empennage, Horizontal Tail
    %HTail 0 Lift Drag
%     RNHT = mission.rho*mission.v_cruise*ac.HT.cmac/mission.viscocity;
%     Rwf = Rwf_find(mission.M,RNHT);
    Rwf = 1;
    RLS = RLS_find(mission.M,ac.HT.ct_cmax_sweep);
%     Cfw = .455/(log10(RNHT))^2.58/(1+.144*mission.M^2)^.65; %save as
%     maing wing, above
    CDoh = Rwf*RLS*Cfw*(1+ac.HT.L1*ac.HT.tc+100*ac.HT.tc^4)*ac.HT.Swet/ac.wing.S;
    
    %HTail Drag due to Lift
    depdalpha = 2*CLaw/(pi*ac.wing.A);
    ac.HT.nu = ac.HT.Cla/(2*pi/B);
    CLah = 2*pi*ac.HT.A/(2+sqrt(4+ac.HT.A^2*B^2/ac.HT.nu^2*(1+tand(ac.HT.ct_cmax_sweep)^2/B^2)))*ac.HT.Swet/ac.wing.S*F; %should F be here?
    alpha_h = alpha*(1-depdalpha)+ac.HT.ih;
    CLh = CLah*(alpha_h-ac.HT.alpha0Lh);
    % CDLh = CLh^2/(pi*HT_A*eh)+2*pi*CLh*epsilont_h*v_h+R*pi^2*episolont_h^2*w_h^2;
    CDLh = (CLh^2/(pi*ac.HT.A*ac.HT.eh)*ac.HT.S/ac.wing.S);

    %Vtail 0 lift
%     RNVT = mission.rho*mission.v_cruise*ac.VT.cmac/mission.viscocity;
%     Rwf = Rwf_find(mission.M,RNVT);
    RLS = RLS_find(mission.M,ac.VT.ct_cmax_sweep);
%     Cfw = .455/(log10(RNVT))^2.58/(1+.144*mission.M^2)^.65;
    
    CDov = Rwf*RLS*Cfw*(1+ac.VT.L1*ac.VT.tc+100*ac.VT.tc^4)*ac.VT.Swet/ac.wing.S;
    CDLv = 0; %Assumed to be 0 since we're assuming no sideslip angle

    CDoemp = CDov+CDoh;
    CDemp = CDLh+CDLv+CDov+CDoh;
    
    
    %%% Pylons

%     %Roskam Part VI Step 1: 4.5.2
    CDnint = 0.036*(ac.n.c*ac.n.b/ac.wing.S)*(ac.n.delcl1+ac.n.delcl2)^2;

    %Roskam Part VI Step 2: 4.5.1

    %nacelle up wash angle - How do we calculate this? "Ch10"
    n.alpha = alpha+ac.n.in+ac.n.epsilon;

    n.RN = mission.rho*mission.v_cruise*ac.n.length/mission.viscocity;
    
    n.df = ac.n.b;
    n.lf = ac.n.length;
    
    RNn = mission.rho*mission.v_cruise*n.lf/mission.viscocity;
    Rwfn = Rwf_find(mission.M,RNn);
%     Rwfn = 1.08;
%     RLS = RLS_find(mission.M,ac.VT.ct_cmax_sweep);
    Cfn = .455/(log10(RNn))^2.58/(1+.144*mission.M^2)^.65;
    
    CDonbase = Rwfn*Cfn*(1+60/(n.lf/n.df)^3+.0025*(n.lf/n.df))*ac.n.Swet/ac.wing.S;
    CDbn = (0.029*(ac.n.db/n.df)^3/(CDonbase*(ac.wing.S/ac.n.S))^0.5)*ac.n.S/ac.wing.S;
    CDOn = Rwfn*Cfn*(1+60/(n.lf/n.df)^3+0.0025*(n.lf/n.df))*ac.n.Swet/ac.wing.S+CDbn;

    
    n.Splfn = ac.n.S; %WAG
    % cdc = 1.2; %uses same CDC as above, no need to redefine (should it be
    % reused?
    CDLn = 2*n.alpha^2*ac.n.Sb/ac.wing.S+ac.n.nu*ac.fuse.cdc*n.alpha^3*n.Splfn/ac.wing.S;
    % 
    % 
    CDn = (CDLn + CDOn)*ac.n.n;
    CDp = CDnint*ac.n.n;
    CDnp = CDn + CDp;
    
    %Dave to account for
%     CDnp = 0;
%     CDn = 0;
%     CDp = 0;
%     CDnint = 0;
%     CDOn = 0;
%     CDLn = 0;
    
    CDo = CDow+CDofus+CDoemp+CDnint+CDOn*ac.n.n;
    CDi = CDLw+CDLfus+CDLh+CDLv+CDLn*ac.n.n;
    CDw = delta_CDP_wing + delta_cdc_wing + Cdpi;
    CDC = delta_cdc_wing;
    CDP = delta_CDP_wing;
    CD = CDo + CDi + CDw;
%     M123 = mission.M;
%     CD_readout = ' \n Mach No.: %5.4f \n CD emp: %5.4f \n CD fus: %5.4f \n CD wing %5.4f \n CD nacelle: %5.4f \n CD pressure: %5.4f \n CD comp: %5.4f \n CD fuselage comp: %5.4f \n';
%     fprintf(CD_readout,M123,CDemp , CDfus , CDwing , CDnp,delta_CDP_wing,delta_cdc_wing,Cdpi);
%     CL/CD
%     10,  11, 12, 13
    
end