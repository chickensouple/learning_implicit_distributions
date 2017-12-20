
function t = Transform_translateZ(t, z)

  t(1,4) = t(1,4) + t(1,3)*z;
  t(2,4) = t(2,4) + t(2,3)*z;
  t(3,4) = t(3,4) + t(3,3)*z;
