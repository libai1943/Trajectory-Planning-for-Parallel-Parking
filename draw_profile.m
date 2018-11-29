function draw_profile

% ==============================================================================
% MATLAB Source Codes for "Spatio-temporal decomposition: a knowledge-based
% initialization strategy for parallel parking motion optimization". 

% ==============================================================================
%   Copyright (C) 2016 Bai Li
% ==============================================================================
% Draw the optimized control/state profiles that are related to parking motion planning.
%
% ==============================================================================

global NE
load NENE.txt

load SL.txt
load SW.txt
load CL.txt

load ttff.txt
tf = ttff(1,1);

NE = NENE;

figure (99)
set(0,'DefaultLineLineWidth',2.5);

load v.txt
v = var_filer(v);
subplot(2,3,1)
box on
plot_lag_cur(v,NE,tf,0,1)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('v(t) / (m/s)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on


load p.txt
phy = var_filer(p);
clear p
subplot(2,3,2)
box on
plot_lag_cur(phy.*180./pi,NE,tf,0,2)
axis tight
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('\phi(t) / deg','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
grid on


load t.txt
theta = var_filer(t);
subplot(2,3,3)
box on
clear t
plot_lag_cur(theta.*180./pi,NE,tf,0,3)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('\theta(t) / deg','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on



load a.txt
a = var_filer(a);
subplot(2,3,4)
box on
set(0,'DefaultLineLineWidth',2.5);
plot_lag_cur(a,NE,tf,1,4)
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('a(t) / (m/s^2)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
axis tight
grid on



set(0,'DefaultLineLineWidth',2.5);
load o.txt
o = var_filer(o);
subplot(2,3,5)
box on
plot_lag_cur(o,NE,tf,1,5)
axis tight
xlabel('Time / sec','Fontsize',20,'FontWeight','bold')
ylabel('\omega(t) / (rad/s)','Fontsize',20,'FontWeight','bold')
set(gca,'FontSize',12,'FontWeight','bold')
grid on