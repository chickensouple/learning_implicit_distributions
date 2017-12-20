
function t = Transform_rotateY(t, a)
  
  ca = cos(a);
  sa = sin(a);
    
  for i =1:3
      tx = t(i,1);
      tz = t(i,3);
      t(i,1) = ca*tx - sa*tz;
      t(i,3) = sa*tx + ca*tz;
  end