function final = var_filer3(p)
global NE

p = reshape(p,NE,4);
p1 = p(1,1);
p = p(:,2:4);
p2 = reshape(p,1,3*NE);
final = [p1,p2];