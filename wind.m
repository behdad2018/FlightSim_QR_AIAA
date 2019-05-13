function V = wind(x,y,z,u2,v2,w2)
u = 0;
v = 0;
w = 0;
% x and y can have any values but z is bounded to 0 to 319
% you should already load u_intp, v_intp W_intp just once in the begining
% out of this function


% applying periodic BCs

% priodicity in x

if x>398
    xbox=floor(x/398);
    x=x-398*xbox;
end

if x<=0
    xbox=ceil(abs(x)/398);
    x=x+398*xbox;
end

% priodicity in y

if y>398
    ybox=floor(y/398);
    y=y-398*ybox;
end

if y<=0
    ybox=ceil(abs(y)/398);
    y=y+398*ybox;
end

% velocity values
if z>=0 && z<1
    
    u=interp3(u2,(x+2)/2,(y+2)/2,1)*z;
    v=interp3(v2,(x+2)/2,(y+2)/2,1)*z;
    w=interp3(w2,(x+2)/2,(y+2)/2,1)*z;
    
% interpolation     
elseif z<=319 && z>=1
    
    u=interp3(u2,(x+2)/2,(y+2)/2,(z+1)/2);
    v=interp3(v2,(x+2)/2,(y+2)/2,(z+1)/2);
    w=interp3(w2,(x+2)/2,(y+2)/2,(z+1)/2);
    
end

V = [u,v,w];

end


% function [u,v,w]=wind(x,y,z,u_intp,v_intp,w_intp)
% 
% % x and y can have any values but z is bounded to 0 to 319
% % you should already load u_intp, v_intp W_intp just once in the begining
% % out of this function
% 
% 
% % applying periodic BCs
% 
% % priodicity in x
% 
% if x>398
%     xbox=floor(x/398);
%     x=x-398*xbox;
% end
% 
% if x<0
%     xbox=ceil(abs(x)/398);
%     x=x+398*xbox;
% end
% 
% % priodicity in y
% 
% if y>398
%     ybox=floor(y/398);
%     y=y-398*ybox;
% end
% 
% if y<0
%     ybox=ceil(abs(y)/398);
%     y=y+398*ybox;
% end
% 
% % velocity values
% 
% if z<=319 && z>=0
%     
%     u=u_intp(x,y,z);
%     v=v_intp(x,y,z);
%     w=w_intp(x,y,z);
%     
% end
% 
% end