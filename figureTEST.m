%% 读仿真数据
t = system_state.time;
StateData = system_state.signals.values;
[n, m] = size(StateData);
PlotStateData=reshape(StateData, [10 m])';
EffectorData = reshape(effector_pos.signals.values, [3 m])';
MarkerPosData = marker_pos.signals.values;
%% draw quadrator
figure(3)
set(gcf, 'Position', [500 141 368 368]);
% for i = 1:15:m
%     UAM_plot(aerialmanipulator, PlotStateData(i,:), 1.5, 2.1, 1);
%     pause(0.01);
%     hold off
% end
UAM_plot(aerialmanipulator, PlotStateData(1,:), 1.5, 3.5, 1);
%UAM_plot(aerialmanipulator, PlotStateData(90,:), 1.5, 3.5, 1);
%UAM_plot(aerialmanipulator, PlotStateData(floor(m/2),:), 1.5, 3.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(end,:), 1.5, 3.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(100,:), 1.5, 3.5, 1);
UAM_plot(aerialmanipulator, PlotStateData(150,:), 1.5, 3.5, 1);
%UAM_plot(aerialmanipulator, PlotStateData(floor(m/2),:), 1.5, 3.5, 1);
%UAM_plot(aerialmanipulator, PlotStateData(end,:), 1.5, 3.5, 1);
%% draw end effector traj
hold on
plot3(EffectorData(:,1), EffectorData(:,2), EffectorData(:,3), 'r', ...
    'LineWidth', 1.5);
% draw markers
plot3(Marker(1,:), Marker(2,:), Marker(3,:), '.k', 'MarkerSize', 10);
% text
text(PlotStateData(1,1)+0.3, PlotStateData(1,2)+0.3, PlotStateData(1,3)+0.05, ...
    '$t=0 s$', 'FontSize', 12,'Interpreter','latex')
text(PlotStateData(100,1)+0.3, PlotStateData(100,2)+0.3, PlotStateData(100,3)+0.05, ...
    '$t=6 s$', 'FontSize', 12,'Interpreter','latex')
text(PlotStateData(150,1)+0.3, PlotStateData(150,2)+0.3, PlotStateData(150,3)+0.05, ...
    '$t=10 s$', 'FontSize', 12,'Interpreter','latex')
text(PlotStateData(end,1)+0.3, PlotStateData(end,2)+0.3, PlotStateData(end,3)+0.05, ...
    '$t=15 s$', 'FontSize', 12,'Interpreter','latex')
text(Marker(1,1)+0.3, Marker(2,1)-0.2, Marker(3,1), ...
    '\ittarget object', 'FontSize', 8)
% label
xlabel(['$x$' ' (m)'],'Interpreter','latex')
ylabel(['$y$' ' (m)'],'Interpreter','latex')
zlabel(['$z$' ' (m)'],'Interpreter','latex')
%title('3D position')
set(gca, 'ZLim', [0 3.5])
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
figure(4)
set(gcf, 'Position', [550 400 800 400])%set(gcf, 'Position', [450 500 368 252])
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
set(gca, 'XLim', [0 12000], 'YLim', [-1000 3000]);
% label
xlabel('U （pixel）')
ylabel('V （pixel）')
 %title('Trajectory in the image plan of the markers')
% legend
leg = legend([line1 line2 line3 line4], 'point.1', 'point.2', 'point.3', 'point.4');
set(leg, 'EdgeColor', [1 1 1])
% text
text(point3(2,1)-500, point3(2,2)-300, '   initial target', 'FontSize', 8)
grid on
%% draw camera velocities
%获取数据
Camera_V = reshape(camera_v.signals.values, [6 m])';
figure(5)
set(gcf, 'Position', [450 500 500 180]) 
plot(t, Camera_V(:,1), 'k-', 'LineWidth', 1)
hold on
plot(t, Camera_V(:,2), 'r-', 'LineWidth', 1)
hold on
plot(t, Camera_V(:,3), 'b-', 'LineWidth', 1)
hold on
plot(t, Camera_V(:,4), 'g-', 'LineWidth', 1)
hold on
plot(t, Camera_V(:,5), 'c-', 'LineWidth', 1)
hold on
plot(t, Camera_V(:,6), 'y-', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(velocities)')
 h=legend('$\ v_x$', '$\ v_y$','$\ v_z$','$\ \omega_x$','$\ \omega_y$','$\ \omega_z$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
title('Camera Cartesian velocities')
grid on
%% the desired value and the measured value(dx,dy,dz,th1,th2,th3,th4)
quad_measured=quad_measured_state.signals.values;
quad_desired=quad_desired_state1.signals.values;
Desired_yaw=desired_yaw.signals.values;
figure(7)
set(gcf, 'Position', [450 500 500 180])
plot(t,quad_desired(:,1), 'r--', 'LineWidth', 1)
hold on
plot(t,quad_measured(:,7), 'k-', 'LineWidth', 1)
%ylim([-1,2])
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot x$' '(m/s)'],'Interpreter','latex')
grid on
% h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
% set(h, 'EdgeColor', [1 1 1])
%title('the desired and the measured quad velocities')
figure(8)
set(gcf, 'Position', [450 500 500 180])
plot(t,quad_desired(:,2), 'r--', 'LineWidth', 1)
hold on
plot(t,quad_measured(:,8), 'k-', 'LineWidth', 1)
%ylim([-1,2])
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot y$' '(m/s)'],'Interpreter','latex')
grid on
% h=legend('期望值 ', '实际值 '); set(h,'Interpreter','latex')
% set(h, 'EdgeColor', [1 1 1])
%title('the desired and the measured quad dy')
figure(9)
set(gcf, 'Position', [450 500 500 180])
plot(t,quad_desired(:,3), 'r--', 'LineWidth', 1)
hold on
plot(t,quad_measured(:,9), 'k-', 'LineWidth', 1)
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot z$' '(m/s)'],'Interpreter','latex')
grid on
% h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
% set(h, 'EdgeColor', [1 1 1])
%title('the desired and the measured quad dz')
figure(10)
set(gcf, 'Position', [450 500 500 180])
plot(t,10*Desired_yaw(:,1), 'r--', 'LineWidth', 1)
hold on
plot(t,10*quad_measured(:,10), 'k-', 'LineWidth', 1)
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \psi$' '($^\circ$/s)'],'Interpreter','latex')
grid on
% h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
% set(h, 'EdgeColor', [1 1 1])
% robot
Robot_dV = reshape(robot_desired_v.signals.values, [4 m])';
Robot_mV=reshape(robot_measured_v.signals.values, [4 m])';
figure(11)
set(gcf, 'Position', [450 500 500 180])
plot(t,Robot_dV(:,1), 'r--', 'LineWidth', 1)
hold on
plot(t,Robot_mV(:,1), 'k-', 'LineWidth', 1)
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \eta_1$' '($^\circ$/s)'],'Interpreter','latex')
grid on
%h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
%set(h, 'EdgeColor', [1 1 1])
%title('the desired and the measured robot velocities')
figure(12)
set(gcf, 'Position', [450 500 500 180])
plot(t,Robot_dV(:,2)/10, 'r--', 'LineWidth', 1)
hold on
plot(t,Robot_mV(:,2)/10, 'k-', 'LineWidth', 1)
%ylim([-100,100])
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \eta_2$' '($^\circ$/s)'],'Interpreter','latex')
grid on
%h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
%set(h, 'EdgeColor', [1 1 1])
figure(13)
set(gcf, 'Position', [450 500 500 180])
plot(t,Robot_dV(:,3)/10, 'r--', 'LineWidth', 1)
hold on
plot(t,Robot_mV(:,3)/10, 'k-', 'LineWidth', 1)
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \eta_3$' '($^\circ$/s)'],'Interpreter','latex')
grid on
%h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
%set(h, 'EdgeColor', [1 1 1])
figure(14)
set(gcf, 'Position', [450 500 500 180])
plot(t,Robot_dV(:,4), 'r--', 'LineWidth', 1)
hold on
plot(t,Robot_mV(:,4), 'k-', 'LineWidth', 1)
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel(['$\dot \eta_4$' '($^\circ$/s)'],'Interpreter','latex')
grid on
%h=legend('期望值', '实际值'); set(h,'Interpreter','latex')
%set(h, 'EdgeColor', [1 1 1])
figure(15)
set(gcf, 'Position', [180 200 200 72])
plot(t(1:24),Robot_dV(1:24,2)/10, 'r--', 'LineWidth', 1)
hold on
plot(t(1:24),Robot_mV(1:24,2)/10, 'k-', 'LineWidth', 1)
figure(16)
set(gcf, 'Position', [180 200 200 72])
plot(t(1:19),Robot_dV(1:19,3)/10, 'r--', 'LineWidth', 1)
hold on
plot(t(1:19),Robot_mV(1:19,3)/10, 'k-', 'LineWidth', 1)

