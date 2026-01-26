function [q, qd, qdd] = cubicPolynomial(qInit, qFin, vInit, vFin, tInit, tFin, t)
% calculate the polynomial coefficients
c = computePolynomialCoefficients(qInit, qFin, ...
    vInit, vFin, ...
    tFin-tInit);    %tF-t0 = delta time segment (for polynomial calculation)
ti = t-tInit;       %get time for polynomial evaluation
time = [ti.^3 ti.^2 ti 1];
q = time*c; % calculate q -> alternative to polyval

% compute optional velocity
if nargout >= 2
    coefV = [0; 3*c(1); 2*c(2); c(3)];
    qd = time*coefV;
end

% compute optional acceleration
if nargout == 3
    coefA = [0; 0; 6*c(1); 2*c(2)];
    qdd = time*coefA;
end
end

function c = computePolynomialCoefficients(q0, qf, v0, vf, t)
% Following result has been obtained by:
% >> syms q0 qf v0 vf t;
% >> c = [q0; v0; 0; 0];
% >> T0 = [1 t; 0 1];
% >> invTF = [3/t^2, -1/t;
%            -2/t^3,  1/t^2];
% >> c(3:4) = invTF * ([qf; vf] - T0*c(1:2));
a0 = q0; a1 = v0;
a2 = (v0 - vf)/t - (3*(q0 - qf + t*v0))/t^2;
a3 = (2*(q0 - qf + t*v0))/t^3 - (v0 - vf)/t^2;

c = [a3; a2; a1; a0];
end