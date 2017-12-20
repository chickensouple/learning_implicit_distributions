function t = Transform_translateX(t, x)
  t(1,4) = t(1,4) + t(1,1)*x;
  t(2,4) = t(2,4) + t(2,1)*x;
  t(3,4) = t(3,4) + t(3,1)*x;
  