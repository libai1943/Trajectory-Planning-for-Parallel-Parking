function drawfigure(x,y,t,sn)
global colorpool
hold on

load m
load n
load l
load b
car_n = n;
car_l = l;
car_m = m;
car_b = b;

num = length(x);

for ii = 1:num
    hold on
    xx = x(1,ii);
    yy = y(1,ii);
    aa = t(1,ii).*180./pi;
    
    set(0,'DefaultLineLineWidth',1);
    D = [xx - (car_m)*cosd(aa) - car_b*sind(aa), yy - (car_m)*sind(aa) + car_b*cosd(aa)];   
    C = [xx - (car_m)*cosd(aa) + car_b*sind(aa), yy - (car_m)*sind(aa) - car_b*cosd(aa)];
    A = [xx + (car_n + car_l) * cosd(aa) - car_b*sind(aa), yy + (car_n + car_l) * sind(aa) + car_b*cosd(aa)];
    B = [xx + (car_n + car_l) * cosd(aa) + car_b*sind(aa), yy + (car_n + car_l) * sind(aa) - car_b*cosd(aa)];
    PP = [A(1),B(1),C(1),D(1),A(1);A(2),B(2),C(2),D(2),A(2)];
    if (ii == 1)
        set(0,'DefaultLineLineWidth',3);
        plot(PP(1,:),PP(2,:),'Color',colorpool(sn,:));
        set(0,'DefaultLineLineWidth',1);
    else
        plot(PP(1,:),PP(2,:),'Color',colorpool(sn,:));
    end
    h2 = get(gca, 'children');
    

    
end
set(0,'DefaultLineLineWidth',3);
plot(PP(1,:),PP(2,:),'Color',colorpool(sn,:));
set(0,'DefaultLineLineWidth',2);


    xx_online = x(1,1:ii);
    yy_online = y(1,1:ii);
    plot(xx_online,yy_online,'k')