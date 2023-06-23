% parpool open
% Load the model and set parameters
mdl = 'sl_aerialmanipulator';
load_system(mdl)
% Build the Rapid Accelerator target
rtp = Simulink.BlockDiagram.buildRapidAcceleratorTarget(mdl);
% Run parallel simulations
parfor i=1:4
   simOut{i} = sim(mdl,'SimulationMode', 'rapid',...
               'RapidAcceleratorUpToDateCheck', 'off',...
               'SaveTime', 'on',...
               'StopTime', num2str(10*i));
end
