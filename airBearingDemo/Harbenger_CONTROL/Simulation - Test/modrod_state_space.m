function [A,B,C,D] = modrod_state_space(states_e, inputs_e, J_e)
syms s1 s2 s3 p q r l m n jxx jxy jxz jyy jyz jzz real 
% quaternion = [q0; qx; qy; qz];
sigma_e = quat2MRP(states_e(1:4));
omega_e = states_e(5:end);
sigma = [s1; s2; s3];
omega = [p;q;r];
J = [jxx,jxy,jxz;jxy,jyy,jyz;jxz,jyz,jzz];
sigma_dot = 1/4*((1-norm(sigma)^2)*eye(3) + 2*cross_mat(sigma) + 2*(sigma*sigma'))*omega;
omega_dot = J^-1*(-cross(omega,J*omega) + [l;m;n]);
symbolic_states = [sigma; omega];
symbolic_states_dot = [sigma_dot; omega_dot];
sympolic_inputs = [l; m; n];
A = jacobian(symbolic_states_dot, symbolic_states);
B = jacobian(symbolic_states_dot, sympolic_inputs);
A = subs(A, [symbolic_states',reshape(J,1,[])], [sigma_e', omega_e', reshape(J_e,1,[])]);
B = subs(B, [symbolic_states',reshape(J,1,[])], [sigma_e', omega_e', reshape(J_e,1,[])]);
A = subs(A, sympolic_inputs, inputs_e);
B = subs(B, sympolic_inputs, inputs_e);
A = double(A);
B = double(B);
C = [eye(3),zeros(3)];
D = [zeros(3)];
end
