%% cgpopsIPOPTSolutionHD21.m
% initnuminterval = 10, initcolpts = 4
% Phase = 0, currnuminterval = 10, currcolpts = 40
% LavrentievConstraintFlagG = 0
% MeshIterG = 1
iterIPOPTHD = 0;

NLPobjIPOPTHD = 0.0000000000000000000000000e+00;
runTimeIPOPTHD = 3.5347000000000003416822381e-02;


nx(1) = 3;
nu(1) = 2;
nq(1) = 0;
Nt(1) = 40;
ns = 3;
systemHD.phase(1).x(1).point(1) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(2) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(3) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(4) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(5) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(6) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(7) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(8) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(9) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(10) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(11) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(12) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(13) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(14) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(15) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(16) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(17) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(18) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(19) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(20) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(21) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(22) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(23) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(24) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(25) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(26) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(27) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(28) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(29) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(30) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(31) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(32) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(33) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(34) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(35) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(36) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(37) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(38) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(39) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(40) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x(1).point(41) = 9.0000000000000000000000000e+00;
systemHD.phase(1).x(2).point(1) = 4.9999999999999988897769754e-01;
systemHD.phase(1).x(2).point(2) = 5.1061702691195753889275011e-01;
systemHD.phase(1).x(2).point(3) = 5.2952665677796328935755810e-01;
systemHD.phase(1).x(2).point(4) = 5.4557060202436502560630061e-01;
systemHD.phase(1).x(2).point(5) = 5.4999999999999993338661852e-01;
systemHD.phase(1).x(2).point(6) = 5.6061702691195758330167109e-01;
systemHD.phase(1).x(2).point(7) = 5.7952665677796333376647908e-01;
systemHD.phase(1).x(2).point(8) = 5.9557060202436451490370928e-01;
systemHD.phase(1).x(2).point(9) = 5.9999999999999997779553951e-01;
systemHD.phase(1).x(2).point(10) = 6.1061702691195762771059208e-01;
systemHD.phase(1).x(2).point(11) = 6.2952665677796337817540007e-01;
systemHD.phase(1).x(2).point(12) = 6.4557060202436455931263026e-01;
systemHD.phase(1).x(2).point(13) = 6.5000000000000002220446049e-01;
systemHD.phase(1).x(2).point(14) = 6.6061702691195767211951306e-01;
systemHD.phase(1).x(2).point(15) = 6.7952665677796342258432105e-01;
systemHD.phase(1).x(2).point(16) = 6.9557060202436460372155125e-01;
systemHD.phase(1).x(2).point(17) = 7.0000000000000006661338148e-01;
systemHD.phase(1).x(2).point(18) = 7.1061702691195771652843405e-01;
systemHD.phase(1).x(2).point(19) = 7.2952665677796346699324204e-01;
systemHD.phase(1).x(2).point(20) = 7.4557060202436464813047223e-01;
systemHD.phase(1).x(2).point(21) = 7.5000000000000011102230246e-01;
systemHD.phase(1).x(2).point(22) = 7.6061702691195776093735503e-01;
systemHD.phase(1).x(2).point(23) = 7.7952665677796351140216302e-01;
systemHD.phase(1).x(2).point(24) = 7.9557060202436469253939322e-01;
systemHD.phase(1).x(2).point(25) = 8.0000000000000015543122345e-01;
systemHD.phase(1).x(2).point(26) = 8.1061702691195780534627602e-01;
systemHD.phase(1).x(2).point(27) = 8.2952665677796355581108401e-01;
systemHD.phase(1).x(2).point(28) = 8.4557060202436473694831420e-01;
systemHD.phase(1).x(2).point(29) = 8.5000000000000019984014443e-01;
systemHD.phase(1).x(2).point(30) = 8.6061702691195784975519700e-01;
systemHD.phase(1).x(2).point(31) = 8.7952665677796360022000499e-01;
systemHD.phase(1).x(2).point(32) = 8.9557060202436478135723519e-01;
systemHD.phase(1).x(2).point(33) = 8.9999999999999968913755310e-01;
systemHD.phase(1).x(2).point(34) = 9.1061702691195733905260568e-01;
systemHD.phase(1).x(2).point(35) = 9.2952665677796308951741366e-01;
systemHD.phase(1).x(2).point(36) = 9.4557060202436482576615617e-01;
systemHD.phase(1).x(2).point(37) = 9.4999999999999973354647409e-01;
systemHD.phase(1).x(2).point(38) = 9.6061702691195738346152666e-01;
systemHD.phase(1).x(2).point(39) = 9.7952665677796313392633465e-01;
systemHD.phase(1).x(2).point(40) = 9.9557060202436487017507716e-01;
systemHD.phase(1).x(2).point(41) = 9.5000000000000000000000000e+00;
systemHD.phase(1).x(3).point(1) = -nan;
systemHD.phase(1).x(3).point(2) = -nan;
systemHD.phase(1).x(3).point(3) = -nan;
systemHD.phase(1).x(3).point(4) = -nan;
systemHD.phase(1).x(3).point(5) = -nan;
systemHD.phase(1).x(3).point(6) = -nan;
systemHD.phase(1).x(3).point(7) = -nan;
systemHD.phase(1).x(3).point(8) = -nan;
systemHD.phase(1).x(3).point(9) = -nan;
systemHD.phase(1).x(3).point(10) = -nan;
systemHD.phase(1).x(3).point(11) = -nan;
systemHD.phase(1).x(3).point(12) = -nan;
systemHD.phase(1).x(3).point(13) = -nan;
systemHD.phase(1).x(3).point(14) = -nan;
systemHD.phase(1).x(3).point(15) = -nan;
systemHD.phase(1).x(3).point(16) = -nan;
systemHD.phase(1).x(3).point(17) = -nan;
systemHD.phase(1).x(3).point(18) = -nan;
systemHD.phase(1).x(3).point(19) = -nan;
systemHD.phase(1).x(3).point(20) = -nan;
systemHD.phase(1).x(3).point(21) = -nan;
systemHD.phase(1).x(3).point(22) = -nan;
systemHD.phase(1).x(3).point(23) = -nan;
systemHD.phase(1).x(3).point(24) = -nan;
systemHD.phase(1).x(3).point(25) = -nan;
systemHD.phase(1).x(3).point(26) = -nan;
systemHD.phase(1).x(3).point(27) = -nan;
systemHD.phase(1).x(3).point(28) = -nan;
systemHD.phase(1).x(3).point(29) = -nan;
systemHD.phase(1).x(3).point(30) = -nan;
systemHD.phase(1).x(3).point(31) = -nan;
systemHD.phase(1).x(3).point(32) = -nan;
systemHD.phase(1).x(3).point(33) = -nan;
systemHD.phase(1).x(3).point(34) = -nan;
systemHD.phase(1).x(3).point(35) = -nan;
systemHD.phase(1).x(3).point(36) = -nan;
systemHD.phase(1).x(3).point(37) = -nan;
systemHD.phase(1).x(3).point(38) = -nan;
systemHD.phase(1).x(3).point(39) = -nan;
systemHD.phase(1).x(3).point(40) = -nan;
systemHD.phase(1).x(3).point(41) = -nan;
systemHD.phase(1).lam(1).point(1) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(2) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(3) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(4) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(5) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(6) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(7) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(8) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(9) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(10) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(11) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(12) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(13) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(14) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(15) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(16) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(17) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(18) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(19) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(20) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(21) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(22) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(23) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(24) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(25) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(26) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(27) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(28) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(29) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(30) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(31) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(32) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(33) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(34) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(35) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(36) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(37) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(38) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(39) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(40) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(1).point(41) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(1) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(2) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(3) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(4) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(5) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(6) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(7) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(8) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(9) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(10) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(11) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(12) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(13) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(14) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(15) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(16) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(17) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(18) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(19) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(20) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(21) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(22) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(23) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(24) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(25) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(26) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(27) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(28) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(29) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(30) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(31) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(32) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(33) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(34) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(35) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(36) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(37) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(38) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(39) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(40) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(2).point(41) = -0.0000000000000000000000000e+00;
systemHD.phase(1).lam(3).point(1) = nan;
systemHD.phase(1).lam(3).point(2) = nan;
systemHD.phase(1).lam(3).point(3) = nan;
systemHD.phase(1).lam(3).point(4) = nan;
systemHD.phase(1).lam(3).point(5) = nan;
systemHD.phase(1).lam(3).point(6) = nan;
systemHD.phase(1).lam(3).point(7) = nan;
systemHD.phase(1).lam(3).point(8) = nan;
systemHD.phase(1).lam(3).point(9) = nan;
systemHD.phase(1).lam(3).point(10) = nan;
systemHD.phase(1).lam(3).point(11) = nan;
systemHD.phase(1).lam(3).point(12) = nan;
systemHD.phase(1).lam(3).point(13) = nan;
systemHD.phase(1).lam(3).point(14) = nan;
systemHD.phase(1).lam(3).point(15) = nan;
systemHD.phase(1).lam(3).point(16) = nan;
systemHD.phase(1).lam(3).point(17) = nan;
systemHD.phase(1).lam(3).point(18) = nan;
systemHD.phase(1).lam(3).point(19) = nan;
systemHD.phase(1).lam(3).point(20) = nan;
systemHD.phase(1).lam(3).point(21) = nan;
systemHD.phase(1).lam(3).point(22) = nan;
systemHD.phase(1).lam(3).point(23) = nan;
systemHD.phase(1).lam(3).point(24) = nan;
systemHD.phase(1).lam(3).point(25) = nan;
systemHD.phase(1).lam(3).point(26) = nan;
systemHD.phase(1).lam(3).point(27) = nan;
systemHD.phase(1).lam(3).point(28) = nan;
systemHD.phase(1).lam(3).point(29) = nan;
systemHD.phase(1).lam(3).point(30) = nan;
systemHD.phase(1).lam(3).point(31) = nan;
systemHD.phase(1).lam(3).point(32) = nan;
systemHD.phase(1).lam(3).point(33) = nan;
systemHD.phase(1).lam(3).point(34) = nan;
systemHD.phase(1).lam(3).point(35) = nan;
systemHD.phase(1).lam(3).point(36) = nan;
systemHD.phase(1).lam(3).point(37) = nan;
systemHD.phase(1).lam(3).point(38) = nan;
systemHD.phase(1).lam(3).point(39) = nan;
systemHD.phase(1).lam(3).point(40) = nan;
systemHD.phase(1).lam(3).point(41) = nan;
systemHD.phase(1).u(1).point(1) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(2) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(3) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(4) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(5) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(6) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(7) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(8) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(9) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(10) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(11) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(12) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(13) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(14) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(15) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(16) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(17) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(18) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(19) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(20) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(21) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(22) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(23) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(24) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(25) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(26) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(27) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(28) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(29) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(30) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(31) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(32) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(33) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(34) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(35) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(36) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(37) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(38) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(39) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(40) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(1).point(41) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(1) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(2) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(3) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(4) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(5) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(6) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(7) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(8) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(9) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(10) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(11) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(12) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(13) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(14) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(15) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(16) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(17) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(18) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(19) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(20) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(21) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(22) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(23) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(24) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(25) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(26) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(27) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(28) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(29) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(30) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(31) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(32) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(33) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(34) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(35) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(36) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(37) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(38) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(39) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(40) = 0.0000000000000000000000000e+00;
systemHD.phase(1).u(2).point(41) = 0.0000000000000000000000000e+00;
systemHD.phase(1).t.point(1) = 0.0000000000000000000000000e+00;
systemHD.phase(1).t.point(2) = 4.2468107647830599660210282e-01;
systemHD.phase(1).t.point(3) = 1.1810662711185297979454845e+00;
systemHD.phase(1).t.point(4) = 1.8228240809745912542894075e+00;
systemHD.phase(1).t.point(5) = 2.0000000000000000000000000e+00;
systemHD.phase(1).t.point(6) = 2.4246810764783059966021028e+00;
systemHD.phase(1).t.point(7) = 3.1810662711185315743023239e+00;
systemHD.phase(1).t.point(8) = 3.8228240809745912542894075e+00;
systemHD.phase(1).t.point(9) = 4.0000000000000000000000000e+00;
systemHD.phase(1).t.point(10) = 4.4246810764783059966021028e+00;
systemHD.phase(1).t.point(11) = 5.1810662711185306861239042e+00;
systemHD.phase(1).t.point(12) = 5.8228240809745921424678272e+00;
systemHD.phase(1).t.point(13) = 6.0000000000000008881784197e+00;
systemHD.phase(1).t.point(14) = 6.4246810764783068847805225e+00;
systemHD.phase(1).t.point(15) = 7.1810662711185315743023239e+00;
systemHD.phase(1).t.point(16) = 7.8228240809745930306462469e+00;
systemHD.phase(1).t.point(17) = 8.0000000000000000000000000e+00;
systemHD.phase(1).t.point(18) = 8.4246810764783059966021028e+00;
systemHD.phase(1).t.point(19) = 9.1810662711185315743023239e+00;
systemHD.phase(1).t.point(20) = 9.8228240809745930306462469e+00;
systemHD.phase(1).t.point(21) = 1.0000000000000000000000000e+01;
systemHD.phase(1).t.point(22) = 1.0424681076478305996602103e+01;
systemHD.phase(1).t.point(23) = 1.1181066271118531574302324e+01;
systemHD.phase(1).t.point(24) = 1.1822824080974591254289408e+01;
systemHD.phase(1).t.point(25) = 1.2000000000000000000000000e+01;
systemHD.phase(1).t.point(26) = 1.2424681076478305996602103e+01;
systemHD.phase(1).t.point(27) = 1.3181066271118531574302324e+01;
systemHD.phase(1).t.point(28) = 1.3822824080974591254289408e+01;
systemHD.phase(1).t.point(29) = 1.4000000000000000000000000e+01;
systemHD.phase(1).t.point(30) = 1.4424681076478304220245263e+01;
systemHD.phase(1).t.point(31) = 1.5181066271118531574302324e+01;
systemHD.phase(1).t.point(32) = 1.5822824080974591254289408e+01;
systemHD.phase(1).t.point(33) = 1.5999999999999998223643161e+01;
systemHD.phase(1).t.point(34) = 1.6424681076478304220245263e+01;
systemHD.phase(1).t.point(35) = 1.7181066271118531574302324e+01;
systemHD.phase(1).t.point(36) = 1.7822824080974591254289408e+01;
systemHD.phase(1).t.point(37) = 1.8000000000000000000000000e+01;
systemHD.phase(1).t.point(38) = 1.8424681076478304220245263e+01;
systemHD.phase(1).t.point(39) = 1.9181066271118531574302324e+01;
systemHD.phase(1).t.point(40) = 1.9822824080974591254289408e+01;
systemHD.phase(1).t.point(41) = 2.0000000000000000000000000e+01;
systemHD.phase(1).tau.point(1) = -1.0000000000000000000000000e+00;
systemHD.phase(1).tau.point(2) = -9.5753189235216940033978972e-01;
systemHD.phase(1).tau.point(3) = -8.8189337288814695359207008e-01;
systemHD.phase(1).tau.point(4) = -8.1771759190254078575321728e-01;
systemHD.phase(1).tau.point(5) = -8.0000000000000004440892099e-01;
systemHD.phase(1).tau.point(6) = -7.5753189235216944474871070e-01;
systemHD.phase(1).tau.point(7) = -6.8189337288814688697868860e-01;
systemHD.phase(1).tau.point(8) = -6.1771759190254083016213826e-01;
systemHD.phase(1).tau.point(9) = -5.9999999999999997779553951e-01;
systemHD.phase(1).tau.point(10) = -5.5753189235216937813532923e-01;
systemHD.phase(1).tau.point(11) = -4.8189337288814693138760958e-01;
systemHD.phase(1).tau.point(12) = -4.1771759190254076354875679e-01;
systemHD.phase(1).tau.point(13) = -3.9999999999999991118215803e-01;
systemHD.phase(1).tau.point(14) = -3.5753189235216931152194775e-01;
systemHD.phase(1).tau.point(15) = -2.8189337288814686477422811e-01;
systemHD.phase(1).tau.point(16) = -2.1771759190254069693537531e-01;
systemHD.phase(1).tau.point(17) = -1.9999999999999995559107901e-01;
systemHD.phase(1).tau.point(18) = -1.5753189235216935593086873e-01;
systemHD.phase(1).tau.point(19) = -8.1893372888146909183149091e-02;
systemHD.phase(1).tau.point(20) = -1.7717591902540741344296293e-02;
systemHD.phase(1).tau.point(21) = 0.0000000000000000000000000e+00;
systemHD.phase(1).tau.point(22) = 4.2468107647830599660210282e-02;
systemHD.phase(1).tau.point(23) = 1.1810662711185315743023239e-01;
systemHD.phase(1).tau.point(24) = 1.8228240809745921424678272e-01;
systemHD.phase(1).tau.point(25) = 1.9999999999999995559107901e-01;
systemHD.phase(1).tau.point(26) = 2.4246810764783055525128930e-01;
systemHD.phase(1).tau.point(27) = 3.1810662711185311302131140e-01;
systemHD.phase(1).tau.point(28) = 3.8228240809745916983786174e-01;
systemHD.phase(1).tau.point(29) = 3.9999999999999991118215803e-01;
systemHD.phase(1).tau.point(30) = 4.4246810764783051084236831e-01;
systemHD.phase(1).tau.point(31) = 5.1810662711185306861239042e-01;
systemHD.phase(1).tau.point(32) = 5.8228240809745912542894075e-01;
systemHD.phase(1).tau.point(33) = 5.9999999999999986677323704e-01;
systemHD.phase(1).tau.point(34) = 6.4246810764783046643344733e-01;
systemHD.phase(1).tau.point(35) = 7.1810662711185302420346943e-01;
systemHD.phase(1).tau.point(36) = 7.8228240809745908102001977e-01;
systemHD.phase(1).tau.point(37) = 7.9999999999999982236431606e-01;
systemHD.phase(1).tau.point(38) = 8.4246810764783042202452634e-01;
systemHD.phase(1).tau.point(39) = 9.1810662711185297979454845e-01;
systemHD.phase(1).tau.point(40) = 9.8228240809745903661109878e-01;
systemHD.phase(1).tau.point(41) = 1.0000000000000000000000000e+00;
systemHD.phase(1).x0(1) = 9.9999999999999977795539507e-01;
systemHD.phase(1).x0(2) = 4.9999999999999988897769754e-01;
systemHD.phase(1).x0(3) = -nan;
systemHD.phase(1).xf(1) = 9.0000000000000000000000000e+00;
systemHD.phase(1).xf(2) = 9.5000000000000000000000000e+00;
systemHD.phase(1).xf(3) = -nan;
systemHD.phase(1).t0 = 0.0000000000000000000000000e+00;
systemHD.phase(1).tf = 2.0000000000000000000000000e+01;
systemHD.s(1) = -nan;
systemHD.s(2) = -nan;
systemHD.s(3) = -nan;