function p = quatmultiply(q, r)
%QUATMULTIPLY Multiply two quaternions.
%   P = QUATMULTIPLY(Q, R) multiplies quaternions Q and R, returning their
%   product P.

if size(q, 2) ~= 4 || size(r, 2) ~= 4
  error('Expecting quaternions as rows.');
elseif size(q, 1) ~= size(r, 1)
  error('Number of quaternions don''t match.');
end

 p  =     [q(1), -q(2), -q(3), -q(4);
          q(2), q(1), q(4), -q(3);
          q(3),-q(4), q(1),  q(2);
          q(4), q(3),-q(2), q(1)] * r';
end