%% read data%针对改进后的结果图
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
%title('三维空间轨迹图')
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
 %title('图像平面轨迹图')
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
pavalue=[StateData(:,1:6)];
%% PLOT POSITION AND ATTITUDE
%position 位置
figure(3)
set(gcf, 'Position', [450 500 500 180])
plot(t, pavalue(:,1), 'r-', 'LineWidth', 1)
hold on
plot(t, pavalue(:,2), 'b-', 'LineWidth', 1)
hold on 
plot(t, pavalue(:,3), 'k-', 'LineWidth', 1)
ylim=get(gca,'Ylim');
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m)')
grid on
 h=legend('$ x$', '$ y$','$ z$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
 %attitude 姿态
 figure(4)
 set(gcf, 'Position', [450 500 500 180])
plot(t, pavalue(:,4), 'r-', 'LineWidth', 1)
hold on
plot(t, pavalue(:,5), 'b-', 'LineWidth', 1)
hold on 
plot(t, pavalue(:,6), 'k-', 'LineWidth', 1)
ylim=get(gca,'Ylim');
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(\circ)')
grid on
 h=legend('$\phi$', '$\theta$', '$\psi$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
 
% plot x方向速度
figure(5)
 set(gcf, 'Position', [450 500 500 180])
plot(t, dqvalue(:,1), 'r--', 'LineWidth', 1)
hold on
plot(t, qvalue(:,1), 'k-', 'LineWidth', 1)
ylim=get(gca,'Ylim');
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot x$' '(m/s)'],'Interpreter','latex')
%xlabel(['$t$' ' (sec)'],'Interpreter','latex')
%ylabel('(m/s)')
%ylabel(['\dot x' '(m.s^-1)'],'Interpreter','latex')
grid on
% h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
% set(h, 'EdgeColor', [1 1 1])
% title('desired value and measured value')

% plot y方向速度
figure(6)
 set(gcf, 'Position', [450 500 500 180])
plot(t, dqvalue(:,2), 'r--', 'LineWidth', 1)
hold on
plot(t, qvalue(:,2), 'k-', 'LineWidth', 1)
ylim=get(gca,'Ylim');
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot y$' '(m/s)'],'Interpreter','latex')
%xlabel(['$t$' ' (sec)'],'Interpreter','latex')
%ylabel('(m/s)')
grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
 %set(h, 'EdgeColor', [1 1 1])
 %title('desired value and measured value')
 
% plot z方向速度
figure(7)
 set(gcf, 'Position', [450 500 500 180])
plot(t, dqvalue(:,3), 'r--', 'LineWidth', 1)
hold on
plot(t, qvalue(:,3), 'k-', 'LineWidth', 1)
ylim=get(gca,'Ylim');
%xlabel(['$t$' ' (sec)'],'Interpreter','latex')
%ylabel('(m/s)')
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot z$' '(m/s)'],'Interpreter','latex')
grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
%set(h, 'EdgeColor', [1 1 1])
%title('desired value and measured value')

% plot 偏航角速度
figure(8)
 set(gcf, 'Position', [450 500 500 180])
plot(t, dqvalue(:,6), 'r--', 'LineWidth', 1)
hold on
plot(t, qvalue(:,6), 'k-', 'LineWidth', 1)
ylim=get(gca,'Ylim');
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \psi$' '($^\circ$/s)'],'Interpreter','latex')
%xlabel(['$t$' ' (sec)'],'Interpreter','latex')
%ylabel('(\circ/s)')
grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
%set(h, 'EdgeColor', [1 1 1])
%title('desired value and measured value')

% % plot关节1角速度
figure(10)
set(gcf, 'Position', [450 500 500 180])
plot(t, dqvalue(:,7), 'r-', 'LineWidth', 2)
 hold on
 plot(t, qvalue(:,7), 'k--', 'LineWidth', 1)
 ylim=get(gca,'Ylim');
 xlabel(['$t$' ' (s)'],'Interpreter','latex')
 ylabel(['$\dot \eta_1$' '($^\circ$/s)'],'Interpreter','latex')
 %xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 %ylabel('(\circ/s)')
 grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
 %set(h, 'EdgeColor', [1 1 1])
 
 % plot关节2角速度
 figure(11)
 set(gcf, 'Position', [450 500 500 180])
 plot(t, dqvalue(:,8), 'r-', 'LineWidth', 2)
 hold on
 plot(t, qvalue(:,8), 'k--', 'LineWidth', 1)
 ylim=get(gca,'Ylim');
 xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \eta_2$' '($^\circ$/s)'],'Interpreter','latex')
 %xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 %ylabel('(\circ/s)')
 grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
 %set(h, 'EdgeColor', [1 1 1])
 
% % plot关节3角速度
 figure(12)
 set(gcf, 'Position', [450 500 500 180])
 plot(t, dqvalue(:,9), 'r-', 'LineWidth', 2)
 hold on
 plot(t, qvalue(:,9), 'k--', 'LineWidth', 1)
 ylim=get(gca,'Ylim');
 xlabel(['$t$' ' (s)'],'Interpreter','latex')
 ylabel(['$\dot \eta_3$' '($^\circ$/s)'],'Interpreter','latex')
 %xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 %ylabel('(\circ/s)')
 grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
 %set(h, 'EdgeColor', [1 1 1])
 
% % plot关节4角速度
 figure(13)
 set(gcf, 'Position', [450 500 500 180])
 plot(t, dqvalue(:,10), 'r-', 'LineWidth', 2)
 hold on
 plot(t, qvalue(:,10), 'k--', 'LineWidth', 1)
 ylim=get(gca,'Ylim');
 xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \eta_4$' '($^\circ$/s)'],'Interpreter','latex')
 %xlabel(['$t$' ' (sec)'],'Interpreter','latex')
 %ylabel('(\circ/s)')
 grid on
 %h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
 %set(h, 'EdgeColor', [1 1 1])

