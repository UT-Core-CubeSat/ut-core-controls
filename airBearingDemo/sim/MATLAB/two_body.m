function g = two_body(mu_E,r_E,R)
% this function calculates the translational acceleration of a satellite
% relative to a ECI frame. 

% Currently the function uses newtons two body problem, but should be
% expanded to account of earth's oblateness.

% INPUTS
% R    - satellite position vector [x;y;z]
% mu_E - earth's gravitational parameter

% OUTPUTS
% g - vector acceleration due to gravity [x;y;z]
%--------------------------------------------------------------------------
% unpack position vector
x = R(1);
y = R(2);
z = R(3);

r = norm(R);

% model newtons 2 body problem
a_g = -mu_E .* R ./ r^3;

% model J2 perturbation (earth's oblateness)
J2 = 1.08262668355e-3; % earth's J2 value

a_J2 = -(3/2)*J2*(mu_E/r^2)*(r_E/r)^2 * ...
       [(1-5*(z/r)^2)*(x/r);
        (1-5*(z/r)^2)*(y/r);
        (3-5*(z/r)^2)*(z/r)];

% total acceleration
g = a_g + a_J2;
end
