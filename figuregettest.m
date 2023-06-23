%% read data
t = system_state.time;
StateData = system_state.signals.values;
PlotStateData = [StateData(:,1:6) StateData(:,13:16)];
[m, n] = size(StateData);
EffectorData = reshape(effector_pos.signals.values, [3 m])';
MarkerPosData = marker_pos.signals.values;
%% draw quadrator
figure(1)
set(gcf, 'Position', [500 141 368 368]);
% for i = 1:15:m
%     UAM_plot(aerialmanipulator, PlotStateData(i,:), 1.5, 2.1, 1);
%     pause(0.01);
%     hold off
% end
UAM_plot(aerialmanipulator, PlotStateData(1,:), 1.5, 2.5, 1);
% UAM_plot(aerialmanipulator, PlotStateData(90,:), 1.5, 2.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(floor(m/2),:), 1.5, 2.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(end,:), 1.5, 2.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(1,:), 1.5, 2.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(500,:), 1.5, 2.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(floor(m/2),:), 1.5, 2.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(end,:), 1.5, 2.5, 1);
%% draw end effector traj
hold on
plot3(EffectorData(:,1), EffectorData(:,2), EffectorData(:,3), 'r', ...
    'LineWidth', 1.5);
% draw markers
plot3(Marker(1,:), Marker(2,:), Marker(3,:), '.k', 'MarkerSize', 10);
% text
text(StateData(1,1)+0.3, StateData(1,2)+0.3, StateData(1,3)+0.05, ...
    '$t=0 sec$', 'FontSize', 12,'Interpreter','latex')
text(StateData(500,1)+0.3, StateData(500,2)+0.3, StateData(500,3)+0.05, ...
    '$t=4.5 sec$', 'FontSize', 12,'Interpreter','latex')
text(StateData(floor(m/2),1)+0.3, StateData(floor(m/2),2)+0.3, StateData(floor(m/2),3)+0.05, ...
    '$t=15 sec$', 'FontSize', 12,'Interpreter','latex')
text(StateData(end,1)+0.3, StateData(end,2)+0.3, StateData(end,3)+0.05, ...
    '$t=30 sec$', 'FontSize', 12,'Interpreter','latex')
text(Marker(1,1)+0.3, Marker(2,1)-0.2, Marker(3,1), ...
    '\itMarkers', 'FontSize', 8)
% label
xlabel(['$x$' ' (m)'],'Interpreter','latex')
ylabel(['$y$' ' (m)'],'Interpreter','latex')
zlabel(['$z$' ' (m)'],'Interpreter','latex')
%title('3D position')
%title('ÈýÎ¬¿Õ¼ä¹ì¼£Í¼')
set(gca, 'ZLim', [0 2.8])
grid on
%% draw camera marker pos 
point1 = []; point2 = []; point3 = []; point4 = [];
for i=1:m
    data = MarkerPosData(:,:,i);
    point1 = [point1; data(:,1)'];
    point2 = [point2; data(:,2)'];
    point3 = [point3; data(:,3)'];
    point4 = [point4; data(:,4)'];
end
%
figure(2)
set(gcf, 'Position', [450 500 368 252])
line1 = plot(point1(:,1), point1(:,2), 'b');
hold on
line2 = plot(point2(:,1), point2(:,2), 'c');
line3 = plot(point3(:,1), point3(:,2), 'g');
line4 = plot(point4(:,1), point4(:,2), 'm');
%
plot([point1(1,1) point2(1,1) point3(1,1) point4(1,1) point1(1,1)], ...
    [point1(1,2) point2(1,2) point3(1,2) point4(1,2) point1(1,2)], 'o--k', 'LineWidth', 1.5);
plot([point1(floor(m/2),1) point2(floor(m/2),1) point3(floor(m/2),1) point4(floor(m/2),1) point1(floor(m/2),1)], ...
    [point1(floor(m/2),2) point2(floor(m/2),2) point3(floor(m/2),2) point4(floor(m/2),2) point1(floor(m/2),2)], 'd--b', 'LineWidth', 1.5);
plot([point1(end,1) point2(end,1) point3(end,1) point4(end,1) point1(end,1)], ...
    [point1(end,2) point2(end,2) point3(end,2) point4(end,2) point1(end,2)], 'x--r', 'LineWidth', 1.5);
axis equal
set(gca, 'XLim', [0 1280], 'YLim', [0 800]);
% label
xlabel('pixels')
ylabel('pixels')
 %title('Trajectory in the image plan of the markers')
 %title('Í¼ÏñÆ½Ãæ¹ì¼£Í¼')
% legend
leg = legend([line1 line2 line3 line4], 'point1', 'point2', 'point3', 'point4');
set(leg, 'EdgeColor', [1 1 1])
% text
text(point3(2,1)-50, point3(2,2)+60, '   Initial state', 'FontSize', 8)
grid on
%% draw quadrotor and robot arm velocity command
% read data
dqvalue = dq.signals.values;
qvalue = [StateData(:,7:12) StateData(:,17:20)];
% plot
figure(7)
set(gcf, 'Position', [450 500 368 368])
subplot(3,1,1)
plot(t, dqvalue(:,1), 'k-', 'LineWidth', 1)
hold on
plot(t, qvalue(:,1), 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m/s)')
grid on
 h=legend('$\dot x^d$', '$\dot x$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
 title('desired value and measured value')
% plot
subplot(3,1,2)
plot(t, dqvalue(:,2), 'k-', 'LineWidth', 1)
hold on
plot(t, qvalue(:,2), 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m/s)')
grid on
 h=legend('$\dot y^d$', '$\dot y$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
 title('desired value and measured value')
% plot
subplot(3,1,3)
plot(t, dqvalue(:,3), 'k-', 'LineWidth', 1)
hold on
plot(t, qvalue(:,3), 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m/s)')
grid on
 h=legend('$\dot z^d$', '$\dot z$'); set(h,'Interpreter','latex')
set(h, 'EdgeColor', [1 1 1])
title('desired value and measured value')
% % plot
figure(10)
set(gcf, 'Position', [450 500 500 180])
plot(t, dqvalue(:,7), 'k-', 'LineWidth', 1)
 hold on
 plot(t, qvalue(:,7), 'r--', 'LineWidth', 1)
 xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 ylabel('(rad/s)')
 h=legend('$\dot \theta_1^d$', '$\dot \theta_1$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
 % plot
 figure(11)
 set(gcf, 'Position', [450 500 500 180])
 plot(t, dqvalue(:,8), 'k-', 'LineWidth', 1)
 hold on
 plot(t, qvalue(:,8), 'r--', 'LineWidth', 1)
 xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 ylabel('(rad/s)')
 h=legend('$\dot \theta_2^d$', '$\dot \theta_2$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
% % plot
 figure(12)
 set(gcf, 'Position', [450 500 500 180])
 plot(t, dqvalue(:,9), 'k-', 'LineWidth', 1)
 hold on
 plot(t, qvalue(:,9), 'r--', 'LineWidth', 1)
 xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 ylabel('(rad/s)')
 h=legend('$\dot \theta_3^d$', '$\dot \theta_3$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
% % plot
 figure(13)
 set(gcf, 'Position', [450 500 500 180])
 plot(t, dqvalue(:,10), 'k-', 'LineWidth', 1)
 hold on
 plot(t, qvalue(:,10), 'r--', 'LineWidth', 1)
 xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 ylabel('(rad/s)')
 h=legend('$\dot \theta_4^d$', '$\dot \theta_4$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])

