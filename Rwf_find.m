function [Rwf] = Rwf_find(M,RNfus)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    i = 1;

    if M <= 0.4
        %Mach 0.25

            Re = [2939188.4,3535859.4,4319679.3,5277254.4,6597782.5,7998527.3,9622271.2,12030053.5,14472218.7,16752524.7,19542040.1,21599721.9,23874067.7,25785243.5,27423765.7,29166407.8,31259590,32990936.9,34818176.3,36464722.2,37605454,39383809.8,40929848.2,43196793,46296892.3,47378933.9,49619476,51965973,57881770.4,65471684,72924969.2,84415346,97716196.7,114868425.7,141417017,178170632,215997219.5,257852435.3,317447740.9,393838098.3,470154722.4,548441657.6,690979054.6];
            Rw = [1.0608,1.065,1.0687,1.0699,1.073,1.0742,1.0754,1.0754,1.0736,1.0699,1.0656,1.0632,1.0595,1.0553,1.051,1.0443,1.0394,1.0327,1.0223,1.0125,1.004,0.9924,0.9808,0.9704,0.9618,0.9563,0.9515,0.9484,0.9417,0.9368,0.9337,0.9313,0.9295,0.9276,0.9252,0.924,0.924,0.9234,0.9234,0.9234,0.9234,0.9252,0.9258];
        Rwf = interp1(Re,Rw,RNfus);
                    Mini(i) = 0.25;
        i = i+1;
    end

    if M > 0.25 && M <= 0.6
        %Mach 0.4

            Re = [2973947.7,3388441.6,3920426.7,4713389.5,5453391.4,6358187.5,7470219,8912509.4,10471285.5,11930720.4,13284136.6,15135612.5,16218101,18337213.2,20892961.3,23622906.6,25703957.8,27331701.4,29062524.3,30199517.2,31866424.2,33368242.5,35754716,37728250.5,40738027.8,44326829.2,47133895.4,51286138.4,55377498.4,62613310.5,67608297.5,80044838.6,95499258.6,113937494.9,135935639.1,164689786.5,204173794.5,239883291.9,281838293.1,318664242.2,386070543.2,467735141.3,588843655.4,655641849.4,702532899];
            Rw = [1.02,1.02,1.0206,1.0223,1.024,1.0268,1.0296,1.033,1.0375,1.0415,1.0449,1.0505,1.0533,1.0567,1.0584,1.0573,1.0556,1.0522,1.0482,1.0454,1.0398,1.0341,1.0262,1.02,1.0138,1.0087,1.0059,0.9986,0.9946,0.9901,0.9884,0.9834,0.9805,0.9794,0.9777,0.9772,0.9755,0.9749,0.9749,0.9749,0.9743,0.9743,0.9743,0.9743,0.9749];
            Mini(i) = 0.4;
        Rwf(i) = interp1(Re,Rw,RNfus);
        i = i+1;
    end
        %Mach 0.6
    if M > 0.4 && M <= 0.7

            Re = [3046988.3,3498948.3,3836913.6,4406043.3,5177577.7,6084214.2,6933207.5,8084906.9,9948901.6,10661259.3,11963661.8,13844201.5,15535439,16905615.4,17977417.9,20173578.1,21784788.4,23524681.8,25403536,28072152.1,30314200.6,34279839.6,37303212.5,40593237.4,45203521.2,50725680.9,56054363,61942817.9,67925908.2,75640437.4,82311683.7,95984766.1,110222194.5,138797067.9,174779917.4,211795906.3,266703552,325680587.4,394655268.2,478237840.1,549174899.3,625806948.2];
            Rw = [0.9797,0.9803,0.9815,0.9839,0.9857,0.9876,0.99,0.9924,0.9961,0.9985,1.0021,1.0052,1.0094,1.0124,1.0143,1.0197,1.0246,1.0288,1.0325,1.0367,1.0373,1.0355,1.0337,1.0307,1.027,1.0234,1.0209,1.0185,1.0167,1.0149,1.0149,1.0149,1.0155,1.0149,1.0149,1.0149,1.0149,1.0155,1.0155,1.0149,1.0161,1.0161];
            Mini(i) = 0.6;

        Rwf(i) = interp1(Re,Rw,RNfus);
        i = i+1;
    end
        %Mach 0.7
    if M > 0.6 && M <= 0.8
            Re = [3004194.6,3530888.2,3842662.9,4413325.1,5227121.3,6003385.1,6842094.6,7561690.9,8618105.2,9822107,11367868.7,13156895.7,15582964.8,17354843,19180085.2,21525928.3,23973557.7,26699497.5,29507535,35218458.6,39831114.5,45395764,54181714.5,60808473.4,69303787.8,79595882.1,92122345.2,110800866.4,139561577.6,173103986.5,221417171.1,294319685.9,373579281.2,463365805.7,592690832.3];
            Rw = [0.9546,0.9558,0.9569,0.9575,0.9593,0.9617,0.9646,0.9652,0.9676,0.9705,0.9746,0.9799,0.9853,0.9882,0.9923,0.9988,1.0059,1.0094,1.0118,1.0136,1.0136,1.0136,1.0136,1.0136,1.0136,1.0136,1.0136,1.0147,1.0142,1.0142,1.0142,1.0142,1.0142,1.0142,1.0147];	
            Mini(i) = 0.7;
        Rwf(i) = interp1(Re,Rw,RNfus);
        i = i+1;
    end
        %Mach 0.8
    if M > 0.7 && M <= 0.85
            Re = [3007882.5,3618498.8,4124626.4,4923882.6,6015395.9,7937167.6,9771619.7,11665131.3,14251026.7,17544748.9,21106426.8,25587435.2,29391884,33245979.3,37316968.4,44206379.7,51171730.1,60618989.9,70712847.8,88407334,106354495.7,126963512.8,155108447.7,193921257.2,236909207.4,291664077.6,361849876,428654140.8,503896262.5,578817703.8,644710210.7];
            Rw = [0.924,0.9252,0.9271,0.9289,0.9313,0.9362,0.941,0.9453,0.952,0.9593,0.969,0.9787,0.9872,0.9933,1,1.0055,1.0103,1.0146,1.0146,1.0134,1.0134,1.0134,1.014,1.014,1.0146,1.0146,1.0146,1.0146,1.0146,1.0146,1.0146];
            Mini(i) = 0.8;
        Rwf(i) = interp1(Re,Rw,RNfus);
        i = i+1;
    end
        %Mach 0.85
    if M > 0.8 && M <= 0.9
            Re = [2984807.9,3535859.4,4156512.6,4886109.7,5612597.3,6547168.4,7815856.7,8772912.8,9923286.2,11486842.6,13296759.7,15630754.2,17680381.8,20783838.2,24244620.2,27423765.7,31019785.9,34022997.7,38189133,41565125.6,46654798.8,51965973,58329235.9,68041819.7,72924969.2,86388284.4,100773068.2,118461873.1,138187330.6,162443501.1,195420400.8,235091787.5,287206311.8,356319397.1,418864523.5,496194760.3,556954090.6,615598650.1];
            Rw = [0.9021,0.9027,0.9051,0.9069,0.9093,0.911,0.9134,0.9164,0.92,0.9218,0.9266,0.9325,0.9379,0.9451,0.9534,0.963,0.9731,0.9809,0.9887,0.994,1.0006,1.0048,1.0096,1.0131,1.0137,1.0137,1.0143,1.0143,1.0143,1.0143,1.0143,1.0143,1.0143,1.0143,1.0149,1.0155,1.0155,1.0155];
            Mini(i) = 0.85;
        Rwf(i) = interp1(Re,Rw,RNfus);
        i = i+1;
    end
        %Mach 0.9
    if M >= 0.85
            Re = [2958329.5,3530888.2,4118121,4953102.1,5432082.6,6533479.5,7980024.4,9524487.9,11108534.7,12467177.2,13778374.2,15703297.1,18456389.5,20554993.6,22542726,24913587.8,26905672.1,30664558.5,33889609.1,36882036,40138692,43016000.7,45746311.9,49025598.2,52945672.9,57620735.9,63680819.5,68772722.1,83355669.3,98726058.7,127255565.3,157840331.4,201893442,240968140.4,298882732.8,376464075.3,504280657.1,574731738.8,640082245.4];
            Rw = [0.8656,0.8681,0.87,0.8707,0.8726,0.8757,0.8789,0.8826,0.8858,0.889,0.8921,0.8972,0.9054,0.911,0.9167,0.9224,0.9312,0.9426,0.9539,0.9647,0.9748,0.9823,0.9874,0.9937,0.9994,1.0032,1.0082,1.012,1.0132,1.0145,1.0139,1.0139,1.0139,1.0145,1.0145,1.0145,1.0145,1.0145,1.0145];
            Rwf(i) = interp1(Re,Rw,RNfus);
            Mini(i) = 0.9;
        i = i+1;
    end
if length(Mini) >= 2;
    Rwf = interp1(Mini,Rwf,M);
%     Rwf = Mini(1)
end


end
