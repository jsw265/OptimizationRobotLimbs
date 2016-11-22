function hOut = hessianFunction(x,lambda)

% This hessian function assumes zero non-linear constraints and so it does
% not use lambda or load in non-linear constraint second derivatives

p = getGlobalP();
[~,~,hOut] = objFunc(x,p);