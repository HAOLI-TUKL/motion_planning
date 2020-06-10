clear all;
clc;
syms p_x0 p_y0 p_z0;
syms p_xf p_yf p_zf;
syms v_x0 v_y0 v_z0;
syms v_xf v_yf v_zf;
syms a1 a2 a3 b1 b2 b3;
syms T J;

a1 = (-12) / (T^3) *(p_xf - v_x0 * T - p_x0) + 6 / (T^2) * (v_xf - v_x0) ;
a2 = (-12) / (T^3) *(p_yf - v_y0 * T - p_y0) + 6 / (T^2) * (v_yf - v_y0) ;
a3 = (-12) / (T^3) *(p_zf - v_z0 * T - p_z0) + 6 / (T^2) * (v_zf - v_z0) ;

b1 = 6 / (T^2) *(p_xf - v_x0 * T - p_x0) - 2 / T * (v_xf - v_x0) ;
b2 = 6 / (T^2) *(p_yf - v_y0 * T - p_y0) - 2 / T * (v_yf - v_y0) ;
b3 = 6 / (T^2) *(p_zf - v_z0 * T - p_z0) - 2 / T * (v_zf - v_z0) ;

J = T + (1/3 * a1^2 * T^3 + a1 * b1 * T^2 + b1^2 * T) + (1/3 * a2^2 * T^3 + a2 * b2 * T^2 + b2^2 * T) + (1/3 * a3^2 * T^3 + a3 * b3 * T^2 + b3^2 * T);

J_dot = diff(J,T);
pretty(J_dot)