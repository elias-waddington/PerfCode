Light Readme.

Core code is 'performance_alt_climb_v09_c010.m'. This will be purged down to be something similar in an update.

That core sets up the aircraft files from 'aircraftfile_v04_func.m', which is a geomtry file. It then estimates the weight using the weight_estimate_hybGD.m weight script, using Roskam (GD & Torenbeek methods) for weight estimation. It also runs 'CDgen.m' which estimates aerodynamic performance into tables.

It then runs through steps of the performance in this script. At the end, it will export, "outputcomparison2.xlsx' and potentially *.csvs for each run, if verbose mode is turned on.
