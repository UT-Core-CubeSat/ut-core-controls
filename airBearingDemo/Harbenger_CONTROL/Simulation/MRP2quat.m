function q = MRP2quat(m)

  m=m(:);
  magsq=m'*m;
  q=[(1 - magsq); 2*m(1:3)]./(1 + magsq);
   
return