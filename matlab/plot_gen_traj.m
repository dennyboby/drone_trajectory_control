function plot_gen_traj(qd_lift, t0, tf, title)
    syms t real positive;
    figure
    subplot(1,3,1);
    dqd_lift = jacobian(qd_lift,t);
    ddqd_lift = jacobian(dqd_lift, t);
    fplot(qd_lift, [t0, tf],'LineWidth',2);
    xlabel('Time \textit{(in seconds)}','Interpreter','latex','fontsize',10);
    ylabel('Positions \textit{(in m)}', 'Interpreter','latex','fontsize',10);
    subplot(1,3,2);
    fplot(dqd_lift, [t0, tf],'LineWidth',2);
    xlabel('Time \textit{(in seconds)}','Interpreter','latex','fontsize',10);
    ylabel('Velocities \textit{(in m/s)}', 'Interpreter','latex','fontsize',10);
    subplot(1,3,3);
    fplot(ddqd_lift, [t0, tf],'LineWidth',2);
    xlabel('Time \textit{(in seconds)}','Interpreter','latex','fontsize',10);
    ylabel('Accelerations \textit{(in m/s2)}', 'Interpreter','latex','fontsize',10);
    string = sprintf('%s (Blue-X, Red-Y, Orange-Z)', title);
    sgtitle(string,'Interpreter','latex','fontsize',10);
end