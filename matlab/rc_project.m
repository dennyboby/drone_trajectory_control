close all; clear all; clc;
syms t real positive;
% % 1.4 Problem Statement
% % Part 1 Generate
% t0 = 0; t_lift = 5; t_translate = 15;
% p0 = [0,0,0]; p1 = [0,0,1]; v = [0,0,0]; a = [0,0,0];
% p2 = [1,0,1]; p3 = [1,1,1]; p4 = [0,1,1]; p5 = [0,0,1];
% % Lift to p1=[0,0,1]
% qd_movep1 = gen_traj(t0, t_lift, p0, v, a, p1, v, a);
% dqd_movep1 = jacobian(qd_movep1,t);
% ddqd_movep1 = jacobian(dqd_movep1, t);
% plot_gen_traj(qd_movep1, t0, t_lift, 'Trajectory from p0 to p1')
% % Move to p2=[1,0,1]
% qd_movep2 = gen_traj(t_lift, t_lift + t_translate, p1, v, a, p2, v, a);
% dqd_movep2 = jacobian(qd_movep2,t);
% ddqd_movep2 = jacobian(dqd_movep2, t);
% plot_gen_traj(qd_movep2, t_lift, t_lift + t_translate, 'Trajectory from p1 to p2')
% % Move to p3=[1,1,1]
% qd_movep3 = gen_traj(t_lift+t_translate, t_lift+(2*t_translate), p2, v, a, p3, v, a);
% dqd_movep3 = jacobian(qd_movep3,t);
% ddqd_movep3 = jacobian(dqd_movep3, t);
% plot_gen_traj(qd_movep3, t_lift+t_translate, t_lift+(2*t_translate), 'Trajectory from p2 to p3')
% % Move to p4=[0,1,1]
% qd_movep4 = gen_traj(t_lift+(2*t_translate), t_lift+(3*t_translate), p3, v, a, p4, v, a);
% dqd_movep4 = jacobian(qd_movep4,t);
% ddqd_movep4 = jacobian(dqd_movep4, t);
% plot_gen_traj(qd_movep4, t_lift+(2*t_translate), t_lift+(3*t_translate), 'Trajectory from p3 to p4')
% % Move to p5=[0,0,1]
% qd_movep5 = gen_traj(t_lift+(3*t_translate), t_lift+(4*t_translate), p4, v, a, p5, v, a);
% dqd_movep5 = jacobian(qd_movep5,t);
% ddqd_movep5 = jacobian(dqd_movep5, t);
% plot_gen_traj(qd_movep5, t_lift+(3*t_translate), t_lift+(4*t_translate), 'Trajectory from p4 to p5')

% Part 2 Sliding mode controller
syms x y z phi theta psi 'real';
syms u1 u2 u3 u4 'real';
m = 0.027; l = 0.046; [Ix, Iy] = deal(16.571710*10^-6); Iz = 29.261652*10^-6;
Ip = 12.65625*10^-8; kF = 1.28192*10^-8; kM = 5.964552*10^-3;
w_max = 2618; w_min = 0;
q = [x y z phi theta psi]';
u = [u1 u2 u3 u4]';
allocation_matrix = [];

