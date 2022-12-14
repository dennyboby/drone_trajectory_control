function qd = gen_traj(t0, tf, pos_i, vel_i, acc_i, pos_f, vel_f, acc_f)
    syms t real positive;
    x_coeff = quinticpoly(t0, tf, pos_i(1), vel_i(1), acc_i(1), pos_f(1), vel_f(1), acc_f(1));
    y_coeff = quinticpoly(t0, tf, pos_i(2), vel_i(2), acc_i(2), pos_f(2), vel_f(2), acc_f(2));
    z_coeff = quinticpoly(t0, tf, pos_i(3), vel_i(3), acc_i(3), pos_f(3), vel_f(3), acc_f(3));
    qd = [x_coeff(1) + x_coeff(2)*t + x_coeff(3)*t^2 + x_coeff(4)*t^3 + x_coeff(5)*t^4 + x_coeff(6)*t^5;
          y_coeff(1) + y_coeff(2)*t + y_coeff(3)*t^2 + y_coeff(4)*t^3 + y_coeff(5)*t^4 + y_coeff(6)*t^5;
          z_coeff(1) + z_coeff(2)*t + z_coeff(3)*t^2 + z_coeff(4)*t^3 + z_coeff(5)*t^4 + z_coeff(6)*t^5];
end