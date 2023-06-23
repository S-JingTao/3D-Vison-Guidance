%% read data
t = system_state.time;
StateData = system_state.signals.values;
PlotStateData = [StateData(:,1:6) StateData(:,13:16)];
[m, n] = size(StateData);
EffectorData = reshape(effector_pos.signals.values, [3 m])';
MarkerPosData = marker_pos.signals.values;
%% draw quadrator
poserror = EffectorData - repmat(CenterBias', m, 1);
for i=1:m
    normposerror(i) = abs(norm(poserror(i,:)) - TargetHeight);
end
% plot
figure(1)
set(gcf, 'Position', [500 141 450 468]);
subdata = [zeros(m,1) [1:m]' zeros(m,1) PlotStateData(:,4:end)];
for i = 1:100:m
    UAM_plot(aerialmanipulator, subdata(i,:), 1.5, 2.8, 1);
    pause(0.1);
    hold off
end
% draw end effector traj
hold on
plot3(EffectorData(:,1), EffectorData(:,2), EffectorData(:,3), 'r', ...
    'LineWidth', 1.5);
% draw markers
plot3(Marker(1,:), Marker(2,:), Marker(3,:), '.k', 'MarkerSize', 10);

%%
% UAM_plot(aerialmanipulator, PlotStateData(1,:), 1.5, 2.5, 1);
% UAM_plot(aerialmanipulator, PlotStateData(90,:), 1.5, 2.5, 1);
% UAM_plot(aerialmanipulator, PlotStateData(floor(m/2),:), 1.5, 2.5, 1);
% UAM_plot(aerialmanipulator, PlotStateData(end,:), 1.5, 2.5, 1);



