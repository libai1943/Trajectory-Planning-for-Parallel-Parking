function Init_Generation(aaaaa)

load NENE.txt
NE = NENE;
num = NE*3;

format long

load x.txt
load y.txt
x = reshape(x,4,NE);
y = reshape(y,4,NE);
load p.txt
p = reshape(p,4,NE);
load v.txt
v = reshape(v,4,NE);

load a.txt
a = reshape(a,4,NE);
load o.txt
o = reshape(o,4,NE);
load t.txt
the = reshape(t,4,NE);

load AX.txt
AX = reshape(AX,4,NE);
load AY.txt
AY = reshape(AY,4,NE);

load BX.txt
BX = reshape(BX,4,NE);
load BY.txt
BY = reshape(BY,4,NE);

load CX.txt
CX = reshape(CX,4,NE);
load CY.txt
CY = reshape(CY,4,NE);

load DX.txt
DX = reshape(DX,4,NE);
load DY.txt
DY = reshape(DY,4,NE);

load ttff.txt
tf1 = ttff;




% winival(x,y,p,v,tf,the,a,o,AX,AY,BX,BY,CX,CY,DX,DY,,0);
%function winival(x,y,p,v,tf1,the, a, o, AX,AY,BX,BY,CX,CY,DX,DY, aaaaa, )
[K,NE] = size(x); % 4h 30l

format long
matx = zeros(NE*K,4);
matx(:,1) = 1;
for ii = 1:NE
    matx(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matx(((ii-1)*K + jj),3) = jj-1;
        matx(((ii-1)*K + jj),4) = x(jj,ii);
    end
end


mato = zeros(NE*K,4);
mato(:,1) = 1;
for ii = 1:NE
    mato(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        mato(((ii-1)*K + jj),3) = jj-1;
        mato(((ii-1)*K + jj),4) = o(jj,ii);
    end
end






matAX = zeros(NE*K,4);
matAX(:,1) = 1;
for ii = 1:NE
    matAX(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matAX(((ii-1)*K + jj),3) = jj-1;
        matAX(((ii-1)*K + jj),4) = AX(jj,ii);
    end
end
matBX = zeros(NE*K,4);
matBX(:,1) = 1;
for ii = 1:NE
    matBX(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matBX(((ii-1)*K + jj),3) = jj-1;
        matBX(((ii-1)*K + jj),4) = BX(jj,ii);
    end
end
matCX = zeros(NE*K,4);
matCX(:,1) = 1;
for ii = 1:NE
    matCX(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matCX(((ii-1)*K + jj),3) = jj-1;
        matCX(((ii-1)*K + jj),4) = CX(jj,ii);
    end
end
matDX = zeros(NE*K,4);
matDX(:,1) = 1;
for ii = 1:NE
    matDX(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matDX(((ii-1)*K + jj),3) = jj-1;
        matDX(((ii-1)*K + jj),4) = DX(jj,ii);
    end
end





matDY = zeros(NE*K,4);
matDY(:,1) = 1;
for ii = 1:NE
    matDY(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matDY(((ii-1)*K + jj),3) = jj-1;
        matDY(((ii-1)*K + jj),4) = DY(jj,ii);
    end
end
matCY = zeros(NE*K,4);
matCY(:,1) = 1;
for ii = 1:NE
    matCY(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matCY(((ii-1)*K + jj),3) = jj-1;
        matCY(((ii-1)*K + jj),4) = CY(jj,ii);
    end
end
matBY = zeros(NE*K,4);
matBY(:,1) = 1;
for ii = 1:NE
    matBY(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matBY(((ii-1)*K + jj),3) = jj-1;
        matBY(((ii-1)*K + jj),4) = BY(jj,ii);
    end
end
matAY = zeros(NE*K,4);
matAY(:,1) = 1;
for ii = 1:NE
    matAY(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matAY(((ii-1)*K + jj),3) = jj-1;
        matAY(((ii-1)*K + jj),4) = AY(jj,ii);
    end
end




maty = zeros(NE*K,4);
maty(:,1) = 1;
for ii = 1:NE
    maty(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        maty(((ii-1)*K + jj),3) = jj-1;
        maty(((ii-1)*K + jj),4) = y(jj,ii);
    end
end




matp = zeros(NE*K,4);
matp(:,1) = 1;
for ii = 1:NE
    matp(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matp(((ii-1)*K + jj),3) = jj-1;
        matp(((ii-1)*K + jj),4) = p(jj,ii);
    end
end







matv = zeros(NE*K,4);
matv(:,1) = 1;
for ii = 1:NE
    matv(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matv(((ii-1)*K + jj),3) = jj-1;
        matv(((ii-1)*K + jj),4) = v(jj,ii);
    end
end



matthe = zeros(NE*K,4);
matthe(:,1) = 1;
for ii = 1:NE
    matthe(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        matthe(((ii-1)*K + jj),3) = jj-1;
        matthe(((ii-1)*K + jj),4) = the(jj,ii);
    end
end



mata = zeros(NE*K,4);
mata(:,1) = 1;
for ii = 1:NE
    mata(((ii-1)*K + 1):((ii-1)*K + K),2) = ii;
    for jj = 1:K
        mata(((ii-1)*K + jj),3) = jj-1;
        mata(((ii-1)*K + jj),4) = a(jj,ii);
    end
end


fid = fopen(aaaaa, 'w');


for ii = 1:(NE*K)
    fprintf(fid,'let aij[%g,%g]:= %4.5f; \r\n',  0+mata(ii,2),mata(ii,3),mata(ii,4));
end


for ii = 1:(NE*K)
    fprintf(fid,'let phyij[%g,%g]:= %4.5f; \r\n',  0 + matp(ii,2),matp(ii,3),matp(ii,4));
end


for ii = 1:(NE*K)
    fprintf(fid,'let vij[%g,%g]:= %4.5f; \r\n',  0+matv(ii,2),matv(ii,3),matv(ii,4));
end


for ii = 1:(NE*K)
    fprintf(fid,'let xij[%g,%g]:= %4.5f; \r\n',  0 + matx(ii,2),matx(ii,3),matx(ii,4));
end

for ii = 1:(NE*K)
    fprintf(fid,'let yij[%g,%g]:= %4.5f; \r\n',  0+maty(ii,2),maty(ii,3),maty(ii,4));
end


for ii = 1:(NE*K)
    fprintf(fid,'let thetaij[%g,%g]:= %4.5f; \r\n',  0+matthe(ii,2),matthe(ii,3),matthe(ii,4));
end


for ii = 1:(NE*K)
    fprintf(fid,'let wij[%g,%g]:= %4.5f; \r\n',  0+mato(ii,2),mato(ii,3),mato(ii,4));
end


for ii = 1:(NE*K)
    fprintf(fid,'let AXij[%g,%g]:= %4.5f; \r\n',  0+matAX(ii,2),matAX(ii,3),matAX(ii,4));
end
for ii = 1:(NE*K)
    fprintf(fid,'let BXij[%g,%g]:= %4.5f; \r\n',  0+matBX(ii,2),matBX(ii,3),matBX(ii,4));
end
for ii = 1:(NE*K)
    fprintf(fid,'let CXij[%g,%g]:= %4.5f; \r\n',  0+matCX(ii,2),matCX(ii,3),matCX(ii,4));
end
for ii = 1:(NE*K)
    fprintf(fid,'let DXij[%g,%g]:= %4.5f; \r\n',  0+matDX(ii,2),matDX(ii,3),matDX(ii,4));
end




for ii = 1:(NE*K)
    fprintf(fid,'let AYij[%g,%g]:= %4.5f; \r\n',  0+matAY(ii,2),matAY(ii,3),matAY(ii,4));
end
for ii = 1:(NE*K)
    fprintf(fid,'let BYij[%g,%g]:= %4.5f; \r\n',  0+matBY(ii,2),matBY(ii,3),matBY(ii,4));
end
for ii = 1:(NE*K)
    fprintf(fid,'let CYij[%g,%g]:= %4.5f; \r\n',  0+matCY(ii,2),matCY(ii,3),matCY(ii,4));
end
for ii = 1:(NE*K)
    fprintf(fid,'let DYij[%g,%g]:= %4.5f; \r\n',  0+matDY(ii,2),matDY(ii,3),matDY(ii,4));
end

fprintf(fid,'let tf:= %4.5f; \r\n', tf1);
fclose(fid);