function [RLS] = RLS_find(M,tcmaxsweep)
i = 1;
xcos = abs(cosd(tcmaxsweep));
    if M <= 0.6
        Sweep = [0.50,0.523913033,0.550931666,0.571428559,0.596583838,0.618944085,0.648757749,0.683229797,0.70093166,0.75031054,0.780124204,0.798757743,0.81832296,0.833229792,0.847204946,0.862111778,0.877950287,0.896583826,0.905900596,0.923602459,0.938509291,0.949689414,0.961801215,0.97577637,0.988819848,1];
        Rls = [0.810020059,0.826052121,0.846893803,0.861322659,0.878957928,0.89498999,0.92064129,0.944689384,0.957515034,0.994388778,1.015230459,1.021643284,1.032865728,1.039278553,1.044088172,1.050500997,1.05370741,1.056913822,1.058517028,1.060120235,1.061723441,1.061723441,1.063326647,1.063326647,1.063326647,1.064929853];
        RLS(i) = interp1(Sweep,Rls,xcos);
        Mini(i) = 0.25;
        i = i+1;
        
        
    end

    if M <= .8 && M > 0.25
        Sweep = [0.50,0.521118002,0.538819865,0.548136635,0.567701851,0.581677006,0.594720484,0.607763962,0.621739116,0.637577625,0.648757749,0.667391289,0.681366443,0.699068306,0.718633523,0.731677,0.747515509,0.760558987,0.775465819,0.79037265,0.801552774,0.819254637,0.836024823,0.848136623,0.865838486,0.882608672,0.900310534,0.918012397,0.933850906,0.951552768,0.969254631,0.984161463,0.990683202,1];
        Rls = [0.879518084,0.893975914,0.910040169,0.918072297,0.930923702,0.945381531,0.955020085,0.964658638,0.977510042,0.987148596,0.998393574,1.012851404,1.024096383,1.038554213,1.054618469,1.064257022,1.075502001,1.083534128,1.091566256,1.102811235,1.106024086,1.115662639,1.123694767,1.126907618,1.131726895,1.138152597,1.139759022,1.142971874,1.142971874,1.144578299,1.146184725,1.14779115,1.14779115,1.146184725];
        RLS(i) = interp1(Sweep,Rls,xcos);
        Mini(i) = 0.6;
        i = i+1;
    
    
    
    end

    if M <= .9 && M > .6
        Sweep = [0.50,0.517956666,0.535603726,0.548606822,0.565325089,0.582972149,0.59783283,0.613622305,0.631269364,0.649845216,0.665634691,0.681424165,0.696284847,0.710216736,0.726935003,0.743653269,0.756656366,0.778947388,0.7900929,0.802167204,0.817027885,0.830030982,0.843962871,0.851393212,0.869040271,0.886687331,0.901548012,0.922910242,0.937770924,0.949845228,0.966563495,0.982352969,0.995356065,1];
        Rls = [1,1.012851404,1.027309234,1.035341362,1.049799192,1.062650596,1.073895575,1.083534128,1.099598384,1.110843363,1.125301193,1.136546171,1.146184725,1.155823278,1.168674682,1.17670681,1.184738938,1.197590342,1.207228895,1.212048172,1.218473874,1.226506002,1.229718853,1.23453813,1.239357406,1.245783108,1.245783108,1.24899596,1.253815236,1.253815236,1.255421662,1.257028087,1.257028087,1.257028087];
         RLS(i) = interp1(Sweep,Rls,xcos);
        Mini(i) = 0.8;
        i = i+1;
    
    
    
    end

    if M > .8
        Sweep = [0.50,0.516459617,0.527639741,0.539751542,0.550931666,0.562111789,0.576086944,0.587267068,0.597515515,0.6142857,0.629192532,0.640372656,0.656211165,0.668322966,0.679503089,0.695341598,0.703726691,0.722360231,0.735403708,0.748447186,0.767080726,0.780124204,0.794099358,0.802484451,0.815527929,0.831366438,0.846273269,0.861180101,0.878881964,0.892857118,0.911490658,0.930124198,0.948757737,0.965527923,0.983229786,0.999068295,1];
        Rls = [1.095999991,1.10559999,1.113599989,1.123199988,1.131199987,1.137599987,1.148799985,1.156799985,1.163199984,1.175999983,1.185599982,1.195199981,1.20639998,1.214399979,1.222399978,1.236799977,1.238399977,1.251199975,1.259199975,1.270399974,1.279999973,1.286399972,1.295999971,1.30239997,1.30719997,1.318399969,1.323199968,1.329599968,1.335999967,1.339199967,1.347199966,1.350399966,1.353599965,1.353599965,1.355199965,1.355199965,1.355199965];
        RLS(i) = interp1(Sweep,Rls,xcos);
        Mini(i) = 0.9;
        i = i+1;
    
    
    end
    if length(Mini) >= 2;
        RLS = interp1(Mini,RLS,M);
    end
end

