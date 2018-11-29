function draw_figure

% ==============================================================================
% MATLAB Source Codes for "Spatio-temporal decomposition: a knowledge-based
% initialization strategy for parallel parking motion optimization". 

% ==============================================================================
%   Copyright (C) 2016 Bai Li
% ==============================================================================
% Draw the parking motion planning results in a static (rather than dynamic) figure.
% When an optimization process is naturally terminated, the user can use
% this function to see the result details. Note that all the updated txt
% files in the dictionary should NOT be deleted manually.
%
% ==============================================================================

figure (100)
global colorpool
colorpool = [34,177,76]./255;

load ttff.txt
load NENE.txt

index0 = 1;
    
    num = NENE(index0)*3;
    temp = NENE(index0)*4;
    
    NE = NENE(index0);
    tt = linspace(0,ttff(index0),(num+1));
    ttt = linspace(0,ttff(index0),(num));
    
    load x.txt
    x = x(((temp*(index0-1))+1):((temp*(index0-1)) + temp));
    tempp = x(1,1);
    x = reshape(x,4,NE);
    x = x';
    x = x(1:NE,2:4);
    x = reshape(x',1,num);
    x = [tempp,x];
    
    load y.txt
    y = y(((temp*(index0-1))+1):((temp*(index0-1)) + temp));
    tempp = y(1,1);
    y = reshape(y,4,NE);
    y = y';
    y = y(1:NE,2:4);
    y = reshape(y',1,num);
    y = [tempp,y];

    load t.txt
    t = t(((temp*(index0-1))+1):((temp*(index0-1)) + temp));
    tempp = t(1,1);
    t = reshape(t,4,NE);
    t = t';
    t = t(1:NE,2:4);
    t = reshape(t',1,num);
    t = [tempp,t];
    
    drawfigure(x,y,t,index0);

    box on

    axis equal
    
    xlabel('X axis','Fontsize',16,'FontWeight','bold')
    ylabel('Y axis','Fontsize',16,'FontWeight','bold')
    set(gca,'FontSize',12,'FontWeight','bold')

    load SW.txt
    load SL.txt
    load CL.txt
    
    x00 = min(x) - 5;
    x01 = max(x) + 5;
    y00 = -SW - 1;
    y01 = CL + 1;    
    axis([x00,x01,y00,y01])
    

set(0,'DefaultLineLineWidth',2);
fplot(@(x) ( - step(x) + step(x - SL) ) .* SW , [x00 x01], pi*3.1e-16)
fplot(@(x) (sin(0*x) + CL) , [x00 x01], pi*1e-17)
axis([x00,x01,y00,y01])
grid on