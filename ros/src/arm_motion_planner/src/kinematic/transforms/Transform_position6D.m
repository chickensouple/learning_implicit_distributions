function p = Transform_position6D(t1)

  p(1) = t1(1,4);
  p(2) = t1(2,4);
  p(3) = t1(3,4);
  p(4) = atan2(t1(3,2), t1(3,3));
  p(5) = -asin(t1(3,1));
  p(6) = atan2(t1(2,1), t1(1,1));
  
