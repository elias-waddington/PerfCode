Light Readme.

Core code is 'PerfCode.m'.

That core sets up the aircraft files from 'aircraftfile_v04_func.m', which is a geomtry file. It then estimates the weight using the weight_estimate_hybGD.m weight script, using Roskam (GD & Torenbeek methods) for weight estimation. It also runs 'CDgen.m' which estimates aerodynamic performance into tables.

It then runs through steps of the performance in this script. At the end, it will export, "outputcomparison2.xlsx' and potentially *.csvs for each run, if verbose mode is turned on.

Line 78 has the for loop to change to parfor to do paralleization (and vice versa)
