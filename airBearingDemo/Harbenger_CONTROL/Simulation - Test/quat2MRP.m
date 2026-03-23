function mrp = quat2MRP(q)
    % Convert quaternion to Modified Rodrigues Parameters (MRP) directly
    % Input: q - Quaternion in the form [q0, q1, q2, q3] (scalar-first convention)
    % Output: mrp - Modified Rodrigues Parameters
  q=q(:);
  p=q'*q;
  if(p<0.99999 || p>1.00001),
    disp('Warning: quat2mrp: quaternion is not of unit norm');
    disp(1-p);
  end
  q=q./sqrt(p); % normalize

  mrp=q(2:4)./(1 + q(1));

end