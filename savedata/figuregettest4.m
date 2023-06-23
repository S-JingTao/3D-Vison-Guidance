%% load data
load('t.mat');
load('nonalpha_StateData.mat'); nonalpha_StateData = system_state.signals.values;
load('nonalpha_EffectorData.mat'); nonalpha_EffectorData = effector_pos.signals.values;
load('nonalpha_poserror'); nonalpha_poserror = poserror;
load('nonalpha_d_s1error'); nonalpha_d_s1error = d_s1error;
load('nonalpha_d_s2error'); nonalpha_d_s2error = d_s2error;
load('nonalpha_d_s3error'); nonalpha_d_s3error = d_s3error;
load('alpha_StateData.mat'); alpha_StateData = system_state.signals.values;
load('alpha_EffectorData.mat'); alpha_EffectorData = effector_pos.signals.values;
load('alpha_poserror'); alpha_poserror = poserror;
load('alpha_d_s1error'); alpha_d_s1error = d_s1error;
load('alpha_d_s2error'); alpha_d_s2error = d_s2error;
load('alpha_d_s3error'); alpha_d_s3error = d_s3error;
% norm
m = length(t);
TargetHeight = 0.3;
for i=1:m
    norm_nonalpha_poserror(i) = abs(norm(nonalpha_poserror(i,:)) - TargetHeight);
    norm_alpha_poserror(i) = abs(norm(alpha_poserror(i,:)) - TargetHeight);
end
for i=1:m
    norm_nonalpha_d_s1error(i) = norm(nonalpha_d_s1error(i,:));
    norm_alpha_d_s1error(i) = norm(alpha_d_s1error(i,:));
end
for i=1:m
    norm_nonalpha_d_s3error(i) = norm(nonalpha_d_s3error(i,:));
    norm_alpha_d_s3error(i) = norm(alpha_d_s3error(i,:));
end
%%
% main task comparison
figure(1)
set(gcf, 'Position', [450 500 270 150])
plot(t, norm_nonalpha_poserror, 'k-', 'LineWidth', 1)
hold on
plot(t, norm_alpha_poserror, 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m)')
h=legend('without $\lambda$', 'with $\lambda$'); set(h,'Interpreter','latex')
set(h, 'EdgeColor', [1 1 1])
% title('norm of the end effector position error comparison')
% subtask one comparison
figure(2)
set(gcf, 'Position', [450 500 270 150])
plot(t, norm_nonalpha_d_s1error, 'k-', 'LineWidth', 1)
hold on
plot(t, norm_alpha_d_s1error, 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m)')
h=legend('without $\lambda$', 'with $\lambda$'); set(h,'Interpreter','latex')
set(h, 'EdgeColor', [1 1 1])
% title('norm of camera direction error comparison')
% subtask two comparison
figure(3)
set(gcf, 'Position', [450 500 270 150])
plot(t, nonalpha_d_s2error, 'k-', 'LineWidth', 1)
hold on
plot(t, alpha_d_s2error, 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(m)')
h=legend('without $\lambda$', 'with $\lambda$'); set(h,'Interpreter','latex')
set(h, 'EdgeColor', [1 1 1])
% title('norm of the CG error comparison')
% subtask three comparison
figure(4)
set(gcf, 'Position', [450 500 270 150])
plot(t, norm_nonalpha_d_s3error, 'k-', 'LineWidth', 1)
hold on
plot(t, norm_alpha_d_s3error, 'r--', 'LineWidth', 1)
xlabel(['$t$' ' (sec)'],'Interpreter','latex')
ylabel('(rad)')
h=legend('without $\lambda$', 'with $\lambda$'); set(h,'Interpreter','latex')
set(h, 'EdgeColor', [1 1 1])
% title('norm of the joint error comparison')


