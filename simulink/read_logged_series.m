function [t, values] = read_logged_series(ts, width)
%READ_LOGGED_SERIES Time and values (N x width) of a To Workspace log (Timeseries).
%   Depending on the signal shape, Simulink stores N x width or width x 1 x N.
t = ts.Time;
values = squeeze(ts.Data);
if size(values, 1) == width && size(values, 2) ~= width
    values = values';
end
end
