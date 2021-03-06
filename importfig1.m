function [AR, CLdes] = importfig1(filename, dataLines)
%IMPORTFILE Import data from a text file
%  [AR, CLDES] = IMPORTFILE(FILENAME) reads data from text file FILENAME
%  for the default selection.  Returns the data as column vectors.
%
%  [AR, CLDES] = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  [AR, CLdes] = importfile("C:\Users\Elias.DESKTOP-57LHO0I\Box Sync\CHEETA UIUC Only\Notes\EGW\Actual Work\fig1.csv", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 27-Mar-2020 12:39:38

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
opts.VariableNames = ["AR", "CLdes"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(filename, opts);

%% Convert to output type
AR = tbl.AR;
CLdes = tbl.CLdes;
end