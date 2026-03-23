function mrp = quat2MRP(q)
    % Convert quaternion to Modified Rodrigues Parameters (MRP) directly
    % Input: q - Quaternion in the form [q0, q1, q2, q3] (scalar-first convention)
    % Output: mrp - Modified Rodrigues Parameters

%     % Extract quaternion components
%     q0 = q(1);
%     q1 = q(2);
%     q2 = q(3);
%     q3 = q(4);
% 
%     % Compute the denominator for MRP (1 + q0)
%     denom = 1 + q0;
% 
%     % Check for the singularity at q0 = -1 (180-degree rotation)
%     if abs(denom) < 1e-8
%         error('Quaternion represents a 180-degree rotation, which is singular for MRP.');
%     end
% 
%     % Calculate MRP components
%     mrp = [q1; q2; q3] / denom;
% 
%     if norm(mrp) > 1
%         mrp = -[q1; q2; q3] / (1 - q0);
%     end
  q=q(:);
  p=q'*q;
  if(p<0.99999 || p>1.00001),
    disp('Warning: quat2mrp: quaternion is not of unit norm');
    disp(1-p);
  end
  q=q./sqrt(p); % normalize

  mrp=q(2:4)./(1 + q(1));

end