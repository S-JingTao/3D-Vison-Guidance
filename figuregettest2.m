%% read data
t = system_state.time;
StateData = system_state.signals.values;
PlotStateData = [StateData(:,1:6) StateData(:,13:16)];
[m, n] = size(StateData);
EffectorData = reshape(effector_pos.signals.values, [3 m])';%B = reshape(A,m,n) ����m*n��B���� Ԫ���Ǵ�A��һ��һ��ȡ��,����B���ŷŵ�. 
MarkerPosData = marker_pos.signals.values;
%% draw main task position error (end effector position)
poserror = EffectorData - repmat(CenterBias', m, 1);%����һ�������������������ظ�ʱʹ�ã��书������A�����ݶѵ��ڣ�MxN���ľ���B�У�B����Ĵ�С��MxN��A��������ݾ��������A��һ��3x4x5�ľ�����B = repmat(A,2,3)�����ľ�����6x12x5
for i=1:m
    normposerror(i) = abs(norm(poserror(i,:)) - TargetHeight);
end
% plot
figure(1)
set(gcf, 'Position', [450 500 500 180])
%set(gcf, 'Position', [450 500 270 150])
plot(t, normposerror, 'k-', 'LineWidth', 1)
hold on 
fill([2,8,8,2],[0,0,3,3],'r','FaceAlpha',0.2','edgealpha',0)
grid on
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel('(m)')
set(gca, 'YLim', [0 2.1])
 %title('norm of the end effector position error (main task)')
%% draw sub-task one error (camera direction)
% read data
d_s1error = d_s1.signals.values;
for i=1:m
    norm_ds1(i) = norm(d_s1error(i,:));
end
% plot
figure(2)
set(gcf, 'Position', [450 500 500 180])
%set(gcf, 'Position', [450 500 270 150])
plot(t, norm_ds1, 'k', 'LineWidth', 1)
grid on
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel('(m)')
set(gca, 'YLim', [0 1.1])
 %title('norm of camera direction error (subtask one)')
% h=legend('$x$', '$y$', '$z$'); set(h,'Interpreter','latex')
%% draw sub-task two error (gravity control)
% read data
d_s2error = d_s2.signals.values;
% plot
figure(3)
set(gcf, 'Position', [450 500 500 180])
%set(gcf, 'Position', [450 500 270 150])
plot(t, d_s2error(:,1), 'k-', 'LineWidth', 1)
grid on
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel('(m)')
set(gca, 'YLim', [0.012 0.031])
%title('norm of the CG error (subtask two)')
%% draw sub-task three error (joint limit avoidance)
% read data
d_s3error = d_s3.signals.values;
for i=1:m
    norm_ds3(i) = norm(d_s3error(i,2:3));
end
% plot
figure(4)
set(gcf, 'Position', [450 500 500 180])
%set(gcf, 'Position', [450 500 270 150])
plot(t, norm_ds3, 'k-', 'LineWidth', 1)
grid on
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel('(rad)')
set(gca, 'YLim', [0.4 1.2])
 %title('norm of the joint error (subtask three)')


