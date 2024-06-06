% Usage: g = Grad(funHandle, x0, varargin)
%     fun: handle to the multidimensional scalar function
%          This function takes a vector argument of
%          length n and possibly other arguments, returns a scalar.
%          It can return a vector too, but
%          in that case, the gradient is calculted fot the first return
%          parameter.
%     x0: point of interest (vector of length n)
%     g: column vector containing the gradient of fun at x0. The
%        size(g) = size(x)
%
function g = numericalGradient(funcHandle, x0, varargin)
% |delta(i)| is relative to |x0(i)|
delta = x0 / 1000;
for i = 1 : length(x0)
    if x0(i) == 0
        % avoids delta(i) = 0 (**arbitrary value**)
        delta(i) = 1e-12;
    end
    % recovers original x0
    u = x0;
    u(i) = x0(i) + delta(i);
    % fun(x0(i-1), x0(i)+delta(i), x0(i+1), ...)
    f1 = feval ( funcHandle, u, varargin{:});
    f1 = f1(1); %take first return parameter
    u(i) = x0(i) - delta(i);
    % fun(x0(i-1), x0(i)-delta(i), x0(i+1), ...)
    f2 = feval ( funcHandle, u , varargin{:});
    f2 = f2(1); %take first return parameter
    % partial derivatives in column vector
    
    g(i,1) = (f1 - f2) / (2 * delta(i));
end

end






