function build_docking_model()
%BUILD_DOCKING_MODEL Builds docking_model.slx in this folder, from scratch.
%   Loop: State -> Sampler (Ts) -> Sensor (+ Noise) -> Controller (pilot) -> Dynamics -> State.
%   The .slx is GENERATED: edit this script or the .m functions, not the model by hand.
mdl = 'docking_model';
folder = fileparts(mfilename('fullpath'));
P = docking_params();
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
file = fullfile(folder, [mdl '.slx']);
if isfile(file)
    delete(file);                            % generated file: always rebuilt
end
new_system(mdl);
set_param(mdl, 'StopTime', num2str(P.t_max), 'Solver', 'ode45', 'MaxStep', '0.1', 'RelTol', '1e-6');

add_block('simulink/Continuous/Integrator', [mdl '/State'], 'Position', [420 95 460 135], ...
    'InitialCondition', 'getfield(docking_params(), ''x0'')');
matlab_block(mdl, 'Dynamics', [260 90 360 140], ["function dx = f(x, u)", "dx = docking_dynamics(x, u);"], ...
    {'[13 1]', '[6 1]'}, '[13 1]');
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Sampler'], 'Position', [520 185 560 215], ...
    'SampleTime', 'getfield(docking_params(), ''Ts'')');
add_block('simulink/Sources/Random Number', [mdl '/Noise'], 'Position', [520 240 560 270], ...
    'Mean', 'zeros(6, 1)', 'Variance', 'ones(6, 1)', 'Seed', 'getfield(docking_params(), ''noise_seed'')', ...
    'SampleTime', 'getfield(docking_params(), ''Ts'')');
add_block('simulink/Sources/Constant', [mdl '/Sensor_mode'], 'Position', [520 300 560 330], ...
    'Value', 'getfield(docking_params(), ''sensor'')', 'SampleTime', 'getfield(docking_params(), ''Ts'')');
matlab_block(mdl, 'Sensor', [380 180 480 230], ["function pose = f(x, noise, mode)", ...
    "pose = docking_sensor(x, noise, mode);"], {'[13 1]', '[6 1]', '1'}, '[12 1]');
matlab_block(mdl, 'Controller', [220 180 320 220], ["function u = f(pose)", "u = docking_controller(pose);"], ...
    {'[12 1]'}, '[6 1]');
matlab_block(mdl, 'Contact', [520 20 620 60], ["function stop = f(x)", ...
    "P = docking_params();", "stop = x(3) <= P.z_contact;"], {'[13 1]'}, '1', 'boolean');
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop'], 'Position', [680 25 710 55]);
log_block(mdl, 'Log_state', 'state', [520 370 580 400]);
log_block(mdl, 'Log_pose', 'measured_pose', [380 370 440 400]);
log_block(mdl, 'Log_command', 'command', [140 270 200 300]);
log_block(mdl, 'Log_sampled_state', 'sampled_state', [640 370 700 400]);

connect(mdl, 'Dynamics/1', 'State/1');
connect(mdl, 'State/1', 'Dynamics/1');
connect(mdl, 'State/1', 'Sampler/1');
connect(mdl, 'State/1', 'Contact/1');
connect(mdl, 'State/1', 'Log_state/1');
connect(mdl, 'Contact/1', 'Stop/1');
connect(mdl, 'Sampler/1', 'Sensor/1');
connect(mdl, 'Sampler/1', 'Log_sampled_state/1');
connect(mdl, 'Noise/1', 'Sensor/2');
connect(mdl, 'Sensor_mode/1', 'Sensor/3');
connect(mdl, 'Sensor/1', 'Controller/1');
connect(mdl, 'Sensor/1', 'Log_pose/1');
connect(mdl, 'Controller/1', 'Dynamics/2');
connect(mdl, 'Controller/1', 'Log_command/1');
save_system(mdl, file);
close_system(mdl, 0);
end

function matlab_block(mdl, name, position, lines, input_sizes, output_size, output_type)
% Explicit sizes: Simulink's size inference fails in the State <-> Dynamics loop.
path = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', position);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = char(strjoin(lines, newline));
for i = 1:numel(input_sizes)
    chart.Inputs(i).Props.Array.Size = input_sizes{i};
    chart.Inputs(i).DataType = 'double';
end
chart.Outputs(1).Props.Array.Size = output_size;
chart.Outputs(1).DataType = 'double';
if nargin > 6
    chart.Outputs(1).DataType = output_type;
end
end

function log_block(mdl, name, variable, position)
add_block('simulink/Sinks/To Workspace', [mdl '/' name], 'Position', position, ...
    'VariableName', variable, 'SaveFormat', 'Timeseries');
end

function connect(mdl, source, destination)
add_line(mdl, source, destination, 'autorouting', 'smart');
end
