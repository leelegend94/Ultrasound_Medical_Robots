  t2 = fabs(n1);
  t3 = fabs(n2);
  t4 = (n1/fabs(n1));
  t5 = (n2/fabs(n2));
  t6 = cp1*n2;
  t7 = cp2*n1;
  t8 = cp3*n1;
  t9 = cp3*n2;
  t10 = t2*t2;
  t11 = t3*t3;
  t12 = -t7;
  t13 = -t8;
  t14 = -t9;
  t15 = cp1+t13;
  t16 = cp2+t14;
  t19 = t6+t12;
  t20 = t10+t11+1.0;
  t17 = t15*t15;
  t18 = t16*t16;
  t21 = t19*t19;
  t22 = 1.0/t20;
  t23 = t22*t22;
  t24 = t17*t22;
  t25 = t18*t22;
  t26 = t21*t22;
  t27 = t24+t25+t26;
  t28 = sqrt(t27);
  t29 = 1.0/t28;
  t30 = -t28;
  t31 = r+t30;
  A0[0][0] = t31*t31;
  A0[0][1] = t29*t31*(cp3*t15*t22*2.0+cp2*t19*t22*2.0+t2*t4*t17*t23*2.0+t2*t4*t18*t23*2.0+t2*t4*t21*t23*2.0);
  A0[0][2] = t29*t31*(cp3*t16*t22*2.0-cp1*t19*t22*2.0+t3*t5*t17*t23*2.0+t3*t5*t18*t23*2.0+t3*t5*t21*t23*2.0);
  A0[0][3] = r*2.0-t28*2.0;
