
function t = Transform_rotateX(t, a)
  ca = cos(a);
  sa = sin(a);
  
  for i =1:3
      ty = t(i,2);
      tz = t(i,3);
      t(i,2) = ca*ty + sa*tz;
      t(i,3) = -sa*ty + ca*tz; 
  end

  