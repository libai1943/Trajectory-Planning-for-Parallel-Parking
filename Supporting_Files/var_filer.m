function final = var_filer(p)
global NE

num = length(p);
K = round(num/NE);
final = (reshape(p,K,NE))';