function [varargout]=inv_engine_new(rpm,varargin)
%#codegen

if nargin==3
    V_rel_B=varargin{1};
    T=varargin{2};
end


[T1,Q,~]=Propeller(rpm,V_rel_B);

er=abs((T1-T)/T);

varargout{1}=er;

if nargout==3
    varargout{2}=rpm;
    varargout{3}=Q;
end

end





