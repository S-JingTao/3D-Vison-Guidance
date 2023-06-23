%% 读仿真数据
t = system_state.time;
StateData = system_state.signals.values;
[n, m] = size(StateData);
PlotStateData=reshape(StateData, [10 m])';
%%
StateDataP1 = system_stateP1.signals.values;
StateDataP2 = system_stateP2.signals.values;
StateDataP3 = system_stateP3.signals.values;
%StateDataP4 = system_stateP4.signals.values;
%StateDataP5 = system_stateP5.signals.values;
%StateDataP6 = system_stateP6.signals.values;
[n1, m1] = size(StateDataP1);
[n2, m2] = size(StateDataP2);
[n3, m3] = size(StateDataP3);
%[n4, m4] = size(StateDataP4);
%[n5, m5] = size(StateDataP5);
%[n6, m6] = size(StateDataP6);
EffectorDataP1 = reshape(effector_posP1.signals.values, [3 m1])';
EffectorDataP2 = reshape(effector_posP2.signals.values, [3 m2])';
EffectorDataP3 = reshape(effector_posP3.signals.values, [3 m3])';
%EffectorDataP4 = reshape(effector_posP4.signals.values, [3 m4])';
%EffectorDataP5 = reshape(effector_posP5.signals.values, [3 m5])';
%EffectorDataP6 = reshape(effector_posP6.signals.values, [3 m6])';
MarkerPosDataP1 = marker_posP1.signals.values;
MarkerPosDataP2 = marker_posP2.signals.values;
MarkerPosDataP3 = marker_posP3.signals.values;
%MarkerPosDataP4 = marker_posP4.signals.values;
%MarkerPosDataP5 = marker_posP5.signals.values;
%MarkerPosDataP6 = marker_posP6.signals.values;
%% draw quadrator
figure(4)
set(gcf, 'Position', [500 141 368 368]);
UAM_plot(aerialmanipulator, PlotStateData(end,:), 1.5, 3.5, 1);
%% draw end effector traj
hold on
plot3(EffectorDataP1(:,1), EffectorDataP1(:,2), EffectorDataP1(:,3), 'r', ...
    'LineWidth', 1.5);
hold on 
plot3(EffectorDataP2(:,1), EffectorDataP2(:,2), EffectorDataP2(:,3), 'b', ...
    'LineWidth', 1.5);
hold on 
plot3(EffectorDataP3(:,1), EffectorDataP3(:,2), EffectorDataP3(:,3), 'k', ...
    'LineWidth', 1.5);
% draw markers
plot3(Marker(1,:), Marker(2,:), Marker(3,:), '.k', 'MarkerSize', 10);
text(EffectorDataP1(1,1)-0.6, EffectorDataP1(1,2)-0.6, EffectorDataP1(1,3)-0.1, ...
    '$P_1$ ($\eta_1$)', 'FontSize', 12,'Interpreter','latex')
text(EffectorDataP2(1,1)-0.6, EffectorDataP2(1,2)-0.6, EffectorDataP2(1,3)-0.1, ...
    '$P_2$ ($\eta_2$)', 'FontSize', 12,'Interpreter','latex')
text(EffectorDataP3(1,1)-0.6, EffectorDataP3(1,2)-0.6, EffectorDataP3(1,3)-0.1, ...
    '$P_3$ ($\eta_3$)', 'FontSize', 12,'Interpreter','latex')
% label
xlabel(['$x$' ' (m)'],'Interpreter','latex')
ylabel(['$y$' ' (m)'],'Interpreter','latex')
zlabel(['$z$' ' (m)'],'Interpreter','latex')
%title('3D position')
set(gca, 'ZLim', [0 5.5])
set(gca, 'XLim', [-1.5 1.7])
set(gca, 'YLim', [-1.7 1.5])
grid on
%% draw camera marker pos
%P1
P1_point1 = []; P1_point2 = []; P1_point3 = []; P1_point4 = [];
for i=1:m1
    dataP1 = MarkerPosDataP1(:,:,i);
    P1_point1 = [P1_point1; dataP1(:,1)'];
    P1_point2 = [P1_point2; dataP1(:,2)'];
    P1_point3 = [P1_point3; dataP1(:,3)'];
    P1_point4 = [P1_point4; dataP1(:,4)'];
end
figure(5)
set(gcf, 'Position', [550 400 800 400])
P1_line1 = plot(P1_point1(:,1), P1_point1(:,2), 'b');
hold on
P1_line2 = plot(P1_point2(:,1), P1_point2(:,2), 'c');
P1_line3 = plot(P1_point3(:,1), P1_point3(:,2), 'g');
P1_line4 = plot(P1_point4(:,1), P1_point4(:,2), 'm');
%
plot([P1_point1(1,1) P1_point2(1,1) P1_point3(1,1) P1_point4(1,1) P1_point1(1,1)], ...
    [P1_point1(1,2) P1_point2(1,2) P1_point3(1,2) P1_point4(1,2) P1_point1(1,2)], 'o--k', 'LineWidth', 1.5);
plot([P1_point1(floor(m/2),1) P1_point2(floor(m/2),1) P1_point3(floor(m/2),1) P1_point4(floor(m/2),1) P1_point1(floor(m/2),1)], ...
    [P1_point1(floor(m/2),2) P1_point2(floor(m/2),2) P1_point3(floor(m/2),2) P1_point4(floor(m/2),2) P1_point1(floor(m/2),2)], 'd--b', 'LineWidth', 1.5);
plot([P1_point1(end,1) P1_point2(end,1) P1_point3(end,1) P1_point4(end,1) P1_point1(end,1)], ...
    [P1_point1(end,2) P1_point2(end,2) P1_point3(end,2) P1_point4(end,2) P1_point1(end,2)], 'x--r', 'LineWidth', 1.5);
axis equal
set(gca, 'XLim', [0 12000], 'YLim', [-1000 3000]);
% label
xlabel('U （像素）','FontSize',12)
ylabel('V （像素）','FontSize',12)
 %title('Trajectory in the image plan of the markers P1')
% legend
leg = legend([P1_line1 P1_line2 P1_line3 P1_line4], '目标点1', '目标点2', '目标点3', '目标点4');
set(leg, 'EdgeColor', [1 1 1])
% text
text(P1_point3(2,1)-500, P1_point3(2,2)-300, '   初始图像', 'FontSize', 8)
grid on
%P2
P2_point1 = []; P2_point2 = []; P2_point3 = []; P2_point4 = [];
for i=1:m2
    dataP2 = MarkerPosDataP2(:,:,i);
    P2_point1 = [P2_point1; dataP2(:,1)'];
    P2_point2 = [P2_point2; dataP2(:,2)'];
    P2_point3 = [P2_point3; dataP2(:,3)'];
    P2_point4 = [P2_point4; dataP2(:,4)'];
end
figure(6)
set(gcf, 'Position', [550 400 800 400])
P2_line1 = plot(P2_point1(:,1), P2_point1(:,2), 'b');
hold on
P2_line2 = plot(P2_point2(:,1), P2_point2(:,2), 'c');
P2_line3 = plot(P2_point3(:,1), P2_point3(:,2), 'g');
P2_line4 = plot(P2_point4(:,1), P2_point4(:,2), 'm');
%
plot([P2_point1(1,1) P2_point2(1,1) P2_point3(1,1) P2_point4(1,1) P2_point1(1,1)], ...
    [P2_point1(1,2) P2_point2(1,2) P2_point3(1,2) P2_point4(1,2) P2_point1(1,2)], 'o--k', 'LineWidth', 1.5);
plot([P2_point1(floor(m/2),1) P2_point2(floor(m/2),1) P2_point3(floor(m/2),1) P2_point4(floor(m/2),1) P2_point1(floor(m/2),1)], ...
    [P2_point1(floor(m/2),2) P2_point2(floor(m/2),2) P2_point3(floor(m/2),2) P2_point4(floor(m/2),2) P2_point1(floor(m/2),2)], 'd--b', 'LineWidth', 1.5);
plot([P2_point1(end,1) P2_point2(end,1) P2_point3(end,1) P2_point4(end,1) P2_point1(end,1)], ...
    [P2_point1(end,2) P2_point2(end,2) P2_point3(end,2) P2_point4(end,2) P2_point1(end,2)], 'x--r', 'LineWidth', 1.5);
axis equal
set(gca, 'XLim', [0 12000], 'YLim', [-1000 3000]);
% label
xlabel('U （像素）','FontSize',12)
ylabel('V （像素）','FontSize',12)
 %title('Trajectory in the image plan of the markers P2')
% legend
leg = legend([P2_line1 P2_line2 P2_line3 P2_line4], '目标点1', '目标点2', '目标点3', '目标点4');
set(leg, 'EdgeColor', [1 1 1])
% text
text(P2_point3(2,1)-500, P2_point3(2,2)-300, '   初始图像', 'FontSize', 8)
grid on
%P3
P3_point1 = []; P3_point2 = []; P3_point3 = []; P3_point4 = [];
for i=1:m3
    dataP3 = MarkerPosDataP3(:,:,i);
    P3_point1 = [P3_point1; dataP3(:,1)'];
    P3_point2 = [P3_point2; dataP3(:,2)'];
    P3_point3 = [P3_point3; dataP3(:,3)'];
    P3_point4 = [P3_point4; dataP3(:,4)'];
end
figure(7)
set(gcf, 'Position', [550 400 800 400])
P3_line1 = plot(P3_point1(:,1), P3_point1(:,2), 'b');
hold on
P3_line2 = plot(P3_point2(:,1), P3_point2(:,2), 'c');
P3_line3 = plot(P3_point3(:,1), P3_point3(:,2), 'g');
P3_line4 = plot(P3_point4(:,1), P3_point4(:,2), 'm');
%
plot([P3_point1(1,1) P3_point2(1,1) P3_point3(1,1) P3_point4(1,1) P3_point1(1,1)], ...
    [P3_point1(1,2) P3_point2(1,2) P3_point3(1,2) P3_point4(1,2) P3_point1(1,2)], 'o--k', 'LineWidth', 1.5);
plot([P3_point1(floor(m/2),1) P3_point2(floor(m/2),1) P3_point3(floor(m/2),1) P3_point4(floor(m/2),1) P3_point1(floor(m/2),1)], ...
    [P3_point1(floor(m/2),2) P3_point2(floor(m/2),2) P3_point3(floor(m/2),2) P3_point4(floor(m/2),2) P3_point1(floor(m/2),2)], 'd--b', 'LineWidth', 1.5);
plot([P3_point1(end,1) P3_point2(end,1) P3_point3(end,1) P3_point4(end,1) P3_point1(end,1)], ...
    [P3_point1(end,2) P3_point2(end,2) P3_point3(end,2) P3_point4(end,2) P3_point1(end,2)], 'x--r', 'LineWidth', 1.5);
axis equal
set(gca, 'XLim', [0 12000], 'YLim', [-1000 3000]);
% label
xlabel('U （像素）','FontSize',12)
ylabel('V （像素）','FontSize',12)
 %title('Trajectory in the image plan of the markers P3')
% legend
leg = legend([P3_line1 P3_line2 P3_line3 P3_line4], '目标点1', '目标点2', '目标点3', '目标点4');
set(leg, 'EdgeColor', [1 1 1])
% text
text(P3_point3(2,1)-500, P3_point3(2,2)-300, '   初始图像', 'FontSize', 8)
grid on
