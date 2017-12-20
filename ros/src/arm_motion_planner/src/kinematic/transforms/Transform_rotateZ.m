  
  
  function t = Transform_rotateZ(t, a)
  
  ca = cos(a);
  sa = sin(a);
    
  for i =1:3
      tx = t(i,1);
      ty = t(i,2);
      t(i,1) = ca*tx + sa*ty;
      t(i,2) = -sa*tx + ca*ty;
  end
  
  