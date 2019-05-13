function V=wind_time(x,y,z,t) %codegen
global U
% u = 0;
% v = 0;
% w = 0;

% x,y and z are the locations 
% t is the real time
% Cap U is a structured array with 100 cells (corresponding to 100 snap shots) 
% and in each cell with U, V and W for the whole mesh points

a=100;     % velocity data are 12.1 seconds apart

% priocidity in time -- after 1210 seconds the data are repeated

lim=floor(t/(100*a));

if lim>0
    t=t-lim*100*a;
end

it=floor(t/a)+1;
itn=it+1;

if it==100
    
    itn=1;
    
end

% extracting velocity values to do linear interpolation in time
ut1= U{it}.u2;
vt1= U{it}.v2;
wt1= U{it}.w2;
ut2= U{itn}.u2;
vt2= U{itn}.v2;
wt2= U{itn}.w2;


V1=wind_mex(x,y,z,ut1,vt1,wt1);
u1 = V1(1,1);
v1 = V1(1,2);
w1 = V1(1,3);
   

V2=wind_mex(x,y,z,ut2,vt2,wt2);
u2 = V2(1,1);
v2 = V2(1,2);
w2 = V2(1,3);

% interpolation by a weighting method
u= u1*(a*it-t)/a ...
   + u2*(a-a*it+t)/a ;
v= v1*(a*it-t)/a ...
   + v2*(a-a*it+t)/a;
w= w1*(a*it-t)/a ...
   + w2*(a-a*it+t)/a ;

% the negative in Z has been modified here

V = [u;-v;-w];
end