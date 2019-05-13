function xmin=golden_search(a,b,fcn) %#codegen
%global geometry
% this function takes in the "start" and "end" intervals
% as well as the function in which we are looking to find
% the minimum value if the "fcn"

% ------------------------GOLDEN SECTION METHOD----------------------------
% -------------------------------------------------------------------------
% Copyright (c) 2009, Katarzyna Zarnowiec, all rights reserved 
% mailto: katarzyna.zarnowiec@gmail.com
% -------------------------------------------------------------------------
%close all
%figure; 
%hold on;


% a=0;                            % start of interval
% b=0.02;                          % end of interval
epsilon=0.0001;               % accuracy value
iter= 500;                       % maximum number of iterations
tau=double((sqrt(5)-1)/2);      % golden proportion coefficient, around 0.618
k=0;                            % number of iterations


x1=a+(1-tau)*(b-a);             % computing x values
x2=a+tau*(b-a);

f_x1=fcn(x1);                     % computing values in x points
f_x2=fcn(x2);

%plot(x1,f_x1,'rx')              % plotting x
%plot(x2,f_x2,'rx')

while ((abs(b-a)>epsilon) && (k<iter))
    k=k+1;
    if(f_x1<f_x2)
        b=x2;
        x2=x1;
        x1=a+(1-tau)*(b-a);
        
        f_x1=fcn(x1);
        f_x2=fcn(x2);
        
%       plot(x1,f_x1,'rx');
    else
        a=x1;
        x1=x2;
        x2=a+tau*(b-a);
        
        f_x1=fcn(x1);
        f_x2=fcn(x2);
        
%        plot(x2,f_x2,'rx')
    end
    
    k=k+1;
end


% chooses minimum point
if(f_x1<f_x2)
%    sprintf('x_min=%f', x1);
%    sprintf('f(x_min)=%f ', f_x1);
    xmin= x1;
%    plot(x1,f_x1,'ro')
else
%    sprintf('x_min=%f', x2);
%    sprintf('f(x_min)=%f ', f_x2);
%    plot(x2,f_x2,'ro');
    xmin= x2;
end

end