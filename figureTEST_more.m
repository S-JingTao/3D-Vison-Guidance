%% 获取数据
t = system_state.time;
StateData = system_state.signals.values;
[n, m] = size(StateData);
PlotStateData=reshape(StateData, [10 m])';
EffectorData = reshape(effector_pos.signals.values, [3 m])';
MarkerPosData = marker_pos.signals.values;
%% translation motion of UAM
figure(3)
set(gcf, 'Position', [450 500 500 180])
plot(t,PlotStateData(:,1), 'r-', 'LineWidth', 1)
hold on
plot(t,PlotStateData(:,2), 'b-', 'LineWidth', 1)
hold on
plot(t,PlotStateData(:,3), 'k-', 'LineWidth', 1)
hold on
ylim=get(gca,'Ylim');
plot([0.5 0.5],ylim,'k--','LineWidth', 1)
text(0.55,-1,'t=0.5','FontSize',10)
plot([3 3],ylim,'k--','LineWidth', 1)
text(3.1,-1,'t=3.0','FontSize',10)
plot([6 6],ylim,'k--','LineWidth', 1)
text(6.1,-1,'t=6.0','FontSize',10)
%title('translation motion')
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel([ '(m)'],'Interpreter','latex')
grid on
 h=legend('$ x $', '$y $', '$z $'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])
%% euler angles of UAM
figure(4)
set(gcf, 'Position', [450 500 500 180])
plot(t,100*PlotStateData(:,4), 'r-', 'LineWidth', 1)
hold on
plot(t,100*PlotStateData(:,5), 'b-', 'LineWidth', 1)
hold on
plot(t,100*PlotStateData(:,6), 'k-', 'LineWidth', 1)
hold on
ylim=get(gca,'Ylim');
plot([0.46 0.46],ylim,'k--','LineWidth', 1)
plot([3 3],ylim,'k--','LineWidth', 1)
text(0.5,-17,'t=0.5','FontSize',10)
text(3.1,-17,'t=3.0','FontSize',10)
%title('Euler angles')
xlabel(['$t$' ' (s)'],'Interpreter','latex')
ylabel([ '($^\circ$)'],'Interpreter','latex')
grid on
 h=legend('$\phi$', '$\theta$', '$\psi$'); set(h,'Interpreter','latex')
 set(h, 'EdgeColor', [1 1 1])