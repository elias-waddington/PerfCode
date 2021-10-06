function [Eta2_rng, Kh_rng] = importKh(filename, dataLines)
%IMPORTFILE Import data from a text file
%  [ETA2_RNG, KH_RNG] = IMPORTFILE(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the data as column
%  vectors.
%
%  [ETA2_RNG, KH_RNG] = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  [Eta2_rng, Kh_rng] = importfile("D:\Box Sync\CHEETA UIUC Only\Notes\EGW\Actual Work\010\Kh_lookup.csv", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 05-Oct-2021 11:01:53

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Eta2_rng", "Kh_rng"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(filename, opts);

%% Convert to output type
Eta2_rng = tbl.Eta2_rng;
Kh_rng = tbl.Kh_rng;
end