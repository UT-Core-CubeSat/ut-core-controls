function p = invquatmultiply(q, r)
%QUATMULTIPLY Multiply two quaternions.

p =  [q(1), -q(2),  -q(3),  -q(4);
     q(2), q(1), q(4),  -q(3);
     q(3), -q(4),  q(1), q(2);
     q(4), q(3),  -q(2), q(1)]^-1 * r;
end