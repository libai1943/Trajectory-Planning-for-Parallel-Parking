################################# Scenario Geometric Size
param SL;
param CL;
param SW;
param PPP{ii in {1..2}, jj in {1..4}, m in {1..2}};

################################# Settings about direct transcription collocation method and STD strategy
param NE;
param bp;

var tf>=1;
var hi=tf/NE;
set I :={1..NE};
set L1 :={1..bp};
set L2 :={bp+1..NE};
set L3 :={bp+1..NE-1};
set I1:={1..NE-1};
set J :={1..3};
set K :={0..3};
param tauj{j in K};
param dljtauk{j in K,k in J};
param omega{j in J};


################################# Starting position
param x0;
param y0;
param theta0;
param v0;
param vtf == 0;

################################# Bounded constraints
param amax;
param vmax;
param dcurmax;
param phymax;

################################# Vehicle Geometric Size
param n;
param l;
param m;
param b;

################################# State/Control Profiles
var xij{i in I,j in K};
var yij{i in I,j in K};
var thetaij{i in I,j in K};
var vij{i in I,j in K};
var aij{i in I,j in K};
var phyij{i in I,j in K};
var wij{i in I,j in K};

var AXij{i in I,j in K};
var BXij{i in I,j in K};
var CXij{i in I,j in K};
var DXij{i in I,j in K};

var AYij{i in I,j in K};
var BYij{i in I,j in K};
var CYij{i in I,j in K};
var DYij{i in I,j in K};

################################# Optimization Objective
minimize f:
tf;

################################# ODEs
s.t. DIFF_dxdt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*xij[i,j])-hi*vij[i,k]*cos(thetaij[i,k])=0;

s.t. DIFF_dydt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*yij[i,j])-hi*vij[i,k]*sin(thetaij[i,k])=0;

s.t. DIFF_dvdt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*vij[i,j])-hi*aij[i,k]=0;

s.t. DIFF_dthetadt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*thetaij[i,j])-hi*(tan(phyij[i,k]))*vij[i,k]/l=0;

s.t. DIFF_dphydt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*phyij[i,j])-hi*wij[i,k]=0;


s.t. EQ_diffx {i in I1}:
xij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*xij[i,j]);

s.t. EQ_diffy {i in I1}:
yij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*yij[i,j]);

s.t. EQ_diffv {i in I1}:
vij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*vij[i,j]);

s.t. EQ_difftheta {i in I1}:
thetaij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*thetaij[i,j]);

s.t. EQ_diffphy {i in I1}:
phyij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*phyij[i,j]);


################################# Two-point bounds
s.t. EQ_starting_x :
xij[1,0] = x0;
s.t. EQ_starting_y :
yij[1,0] = y0;
s.t. EQ_starting_v :
vij[1,0] = v0;
s.t. EQ_starting_theta :
thetaij[1,0] = theta0;

s.t. EQ_end_v :
vij[NE,3] = vtf;

################################# STD strategy based terminal condition
s.t. EQ_AYtf02 :
AYij[NE,3] <= 0;

s.t. EQ_BYtf02 :
BYij[NE,3] <= 0;

s.t. EQ_CYtf02 :
CYij[NE,3] <= 0;

s.t. EQ_DYtf02 :
DYij[NE,3] <= 0;


s.t. COMPARISON_A_101 {i in L2,j in K}:
AXij[i,j] <= SL;

s.t. COMPARISON_B_101 {i in L2,j in K}:
BXij[i,j] <= SL;

s.t. COMPARISON_C_101 {i in L2,j in K}:
CXij[i,j] <= SL;

s.t. COMPARISON_D_101 {i in L2,j in K}:
DXij[i,j] <= SL;


s.t. COMPARISON_A_102 {i in L2,j in K}:
AXij[i,j] >= 0;

s.t. COMPARISON_B_102 {i in L2,j in K}:
BXij[i,j] >= 0;

s.t. COMPARISON_C_102 {i in L2,j in K}:
CXij[i,j] >= 0;

s.t. COMPARISON_D_102 {i in L2,j in K}:
DXij[i,j] >= 0;

################################# Bounds on profiles

s.t. Bonds_v {i in I,j in K}:
vij[i,j]^2 <= (vmax)^2;

s.t. Bonds_w {i in I,j in K}:
(wij[i,j])^2 <= (dcurmax)^2;

s.t. Bonds_a {i in I,j in K}:
(aij[i,j])^2 <= (amax)^2;

s.t. Bonds_phy {i in I,j in K}:
phyij[i,j]^2 <= (phymax)^2;


################################# ABCD definitions
s.t. RELATIONSHIP_AX {i in I,j in K}:
AXij[i,j] = xij[i,j] + (l + n) * cos(thetaij[i,j]) - b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_BX {i in I,j in K}:
BXij[i,j] = xij[i,j] + (l + n) * cos(thetaij[i,j]) + b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_CX {i in I,j in K}:
CXij[i,j] = xij[i,j] - m * cos(thetaij[i,j]) + b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_DX {i in I,j in K}:
DXij[i,j] = xij[i,j] - m * cos(thetaij[i,j]) - b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_AY {i in I,j in K}:
AYij[i,j] = yij[i,j] + (l + n) * sin(thetaij[i,j]) + b * cos(thetaij[i,j]);

s.t. RELATIONSHIP_BY {i in I,j in K}:
BYij[i,j] = yij[i,j] + (l + n) * sin(thetaij[i,j]) - b * cos(thetaij[i,j]);

s.t. RELATIONSHIP_CY {i in I,j in K}:
CYij[i,j] = yij[i,j] - m * sin(thetaij[i,j]) - b * cos(thetaij[i,j]);

s.t. RELATIONSHIP_DY {i in I,j in K}:
DYij[i,j] = yij[i,j] - m * sin(thetaij[i,j]) + b * cos(thetaij[i,j]);


################################# Collision Avoidance Restrictions

s.t. eq_PPoutsideABCD {ii in {1..2}, jj in {1..4}, i in L1, j in K}:
abs((AXij[i,j] - PPP[ii,jj,1])*(BYij[i,j] - PPP[ii,jj,2]) - (AYij[i,j] - PPP[ii,jj,2])*(BXij[i,j] - PPP[ii,jj,1])) * 0.5 + abs((BXij[i,j] - PPP[ii,jj,1])*(CYij[i,j] - PPP[ii,jj,2]) - (BYij[i,j] - PPP[ii,jj,2])*(CXij[i,j] - PPP[ii,jj,1])) * 0.5 + abs((CXij[i,j] - PPP[ii,jj,1])*(DYij[i,j] - PPP[ii,jj,2]) - (CYij[i,j] - PPP[ii,jj,2])*(DXij[i,j] - PPP[ii,jj,1])) * 0.5 + abs((DXij[i,j] - PPP[ii,jj,1])*(AYij[i,j] - PPP[ii,jj,2]) - (DYij[i,j] - PPP[ii,jj,2])*(AXij[i,j] - PPP[ii,jj,1])) * 0.5 >= (l+n+m)*2*b + 0.01;


s.t. eq_AoutsidePRECTANGLE {ii in {1..2}, i in L1, j in K}:
abs((PPP[ii,1,1] - AXij[i,j])*( PPP[ii,2,2] - AYij[i,j]) - (PPP[ii,1,2] - AYij[i,j])*(PPP[ii,2,1] - AXij[i,j])) * 0.5 + abs((PPP[ii,2,1] - AXij[i,j])*(PPP[ii,3,2] - AYij[i,j]) - (PPP[ii,2,2] - AYij[i,j])*( PPP[ii,3,1] - AXij[i,j])) * 0.5 + abs((PPP[ii,3,1] - AXij[i,j])*( PPP[ii,4,2] - AYij[i,j]) - (PPP[ii,3,2] - AYij[i,j])*( PPP[ii,4,1] - AXij[i,j])) * 0.5 + abs((PPP[ii,4,1] - AXij[i,j])*( PPP[ii,1,2] - AYij[i,j]) - (PPP[ii,4,2] - AYij[i,j])*( PPP[ii,1,1] - AXij[i,j])) * 0.5 >= sqrt((PPP[ii,1,1] - PPP[ii,2,1])^2 + (PPP[ii,1,2] - PPP[ii,2,2])^2) * sqrt((PPP[ii,1,1] - PPP[ii,4,1])^2 + (PPP[ii,1,2] - PPP[ii,4,2])^2) + 0.01;


s.t. eq_BoutsidePRECTANGLE {ii in {1..2}, i in L1, j in K}:
abs((PPP[ii,1,1] - BXij[i,j])*( PPP[ii,2,2] - BYij[i,j]) - (PPP[ii,1,2] - BYij[i,j])*(PPP[ii,2,1] - BXij[i,j])) * 0.5 + abs((PPP[ii,2,1] - BXij[i,j])*(PPP[ii,3,2] - BYij[i,j]) - (PPP[ii,2,2] - BYij[i,j])*( PPP[ii,3,1] - BXij[i,j])) * 0.5 + abs((PPP[ii,3,1] - BXij[i,j])*( PPP[ii,4,2] - BYij[i,j]) - (PPP[ii,3,2] - BYij[i,j])*( PPP[ii,4,1] - BXij[i,j])) * 0.5 + abs((PPP[ii,4,1] - BXij[i,j])*( PPP[ii,1,2] - BYij[i,j]) - (PPP[ii,4,2] - BYij[i,j])*( PPP[ii,1,1] - BXij[i,j])) * 0.5 >= sqrt((PPP[ii,1,1] - PPP[ii,2,1])^2 + (PPP[ii,1,2] - PPP[ii,2,2])^2) * sqrt((PPP[ii,1,1] - PPP[ii,4,1])^2 + (PPP[ii,1,2] - PPP[ii,4,2])^2) + 0.01;

s.t. eq_CoutsidePRECTANGLE {ii in {1..2}, i in L1, j in K}:
abs((PPP[ii,1,1] - CXij[i,j])*( PPP[ii,2,2] - CYij[i,j]) - (PPP[ii,1,2] - CYij[i,j])*(PPP[ii,2,1] - CXij[i,j])) * 0.5 + abs((PPP[ii,2,1] - CXij[i,j])*(PPP[ii,3,2] - CYij[i,j]) - (PPP[ii,2,2] - CYij[i,j])*( PPP[ii,3,1] - CXij[i,j])) * 0.5 + abs((PPP[ii,3,1] - CXij[i,j])*( PPP[ii,4,2] - CYij[i,j]) - (PPP[ii,3,2] - CYij[i,j])*( PPP[ii,4,1] - CXij[i,j])) * 0.5 + abs((PPP[ii,4,1] - CXij[i,j])*( PPP[ii,1,2] - CYij[i,j]) - (PPP[ii,4,2] - CYij[i,j])*( PPP[ii,1,1] - CXij[i,j])) * 0.5 >= sqrt((PPP[ii,1,1] - PPP[ii,2,1])^2 + (PPP[ii,1,2] - PPP[ii,2,2])^2) * sqrt((PPP[ii,1,1] - PPP[ii,4,1])^2 + (PPP[ii,1,2] - PPP[ii,4,2])^2) + 0.01;


s.t. eq_DoutsidePRECTANGLE {ii in {1..2}, i in L1, j in K}:
abs((PPP[ii,1,1] - DXij[i,j])*( PPP[ii,2,2] - DYij[i,j]) - (PPP[ii,1,2] - DYij[i,j])*(PPP[ii,2,1] - DXij[i,j])) * 0.5 + abs((PPP[ii,2,1] - DXij[i,j])*(PPP[ii,3,2] - DYij[i,j]) - (PPP[ii,2,2] - DYij[i,j])*( PPP[ii,3,1] - DXij[i,j])) * 0.5 + abs((PPP[ii,3,1] - DXij[i,j])*( PPP[ii,4,2] - DYij[i,j]) - (PPP[ii,3,2] - DYij[i,j])*( PPP[ii,4,1] - DXij[i,j])) * 0.5 + abs((PPP[ii,4,1] - DXij[i,j])*( PPP[ii,1,2] - DYij[i,j]) - (PPP[ii,4,2] - DYij[i,j])*( PPP[ii,1,1] - DXij[i,j])) * 0.5 >= sqrt((PPP[ii,1,1] - PPP[ii,2,1])^2 + (PPP[ii,1,2] - PPP[ii,2,2])^2) * sqrt((PPP[ii,1,1] - PPP[ii,4,1])^2 + (PPP[ii,1,2] - PPP[ii,4,2])^2) + 0.01;


s.t. COMPARISON_A_22 {i in I,j in K}:
AYij[i,j] <= CL;

s.t. COMPARISON_B_22 {i in I,j in K}:
BYij[i,j] <= CL;

s.t. COMPARISON_C_22 {i in I,j in K}:
CYij[i,j] <= CL;

s.t. COMPARISON_D_22 {i in I,j in K}:
DYij[i,j] <= CL;


s.t. COMPARISON_A_33 {i in I,j in K}:
AYij[i,j] >= -1*SW;

s.t. COMPARISON_B_33 {i in I,j in K}:
BYij[i,j] >= -1*SW;

s.t. COMPARISON_C_33 {i in I,j in K}:
CYij[i,j] >= -1*SW;

s.t. COMPARISON_D_33 {i in I,j in K}:
DYij[i,j] >= -1*SW;



data;
param bp:= include bp; 
param SL:= include SL;  
param SW:= include SW;
param CL:= include CL;
param NE:= include NE;
 
param x0:= include x0;  
param y0:= include y0;  
param theta0:= include theta0;  
param v0:= include v0;  

param amax:= include amax;  
param vmax:= include vmax;  
param dcurmax:= include dcurmax;  
param phymax:= include phymax;  

param n:= include n;  
param l:= include l;  
param m:= include m;  
param b:= include b;  

param: dljtauk :=
0  1  -4.13939
0  2   1.73939
0  3  -3
1  1    3.22474
1  2   -3.56784
1  3   5.53197
2  1   1.16784
2  2    0.775255
2  3  -7.53197
3  1  -0.253197
3  2   1.0532
3  3    5;

param: tauj :=
	0		0
	1		0.1550510257216822
	2		0.6449489742783178
	3		1.0;

param: omega:=
	1		 3.76403062700467e-1 
	2		 5.12485826188421e-1
	3		 1.11111111111111e-1;
	
param PPP:= include PPP;