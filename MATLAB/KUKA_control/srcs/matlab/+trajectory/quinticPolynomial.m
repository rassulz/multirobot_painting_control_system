function [q, qd, qdd] = quinticPolynomial(qInit, qFin, vInit, vFin, aInit, aFin, tInit, tFin, t)
% calculate the polynomial coefficients
c = computePolynomialCoefficients(qInit, qFin, ...
    vInit, vFin, ...
    aInit, aFin, ...
    tFin-tInit);    %tF-t0 = delta time segment (for polynomial calculation)
ti = t-tInit;       %get time for polynomial evaluation
time = [ti.^5 ti.^4 ti.^3 ti.^2 ti 1];
q = time*c; % calculate q -> alternative to polyval

% compute optional velocity
if nargout >= 2
    coefV = [0 5*c(1) 4*c(2) 3*c(3) 2*c(4) c(5)]';
    qd = time*coefV;
end

% compute optional acceleration
if nargout == 3
    coefA = [0 0 20*c(1) 12*c(2) 6*c(3) 2*c(4)]';
    qdd = time*coefA;
end
end


function c = computePolynomialCoefficients(q0, qf, v0, vf, a0, af, t)
% Following result has been obtained by:
% >> syms q0 qf v0 vf a0 af t;
% >> c = [q0; v0; a0; 0; 0; 0];
% >> T0 = [1 t t^2; 0 1 2*t; 0 0 2];
% >> invTF = [10/t^3, -4/t^2,  1/(2*t);
%        -15/t^4,  7/t^3, -1/t^2;
%          6/t^5, -3/t^4,  1/(2*t^3)];
% >> c(4:6) = invTF * ([qf; vf; af] - T0*c(1:3));
a1 = q0; a2 = v0; a3 = a0;
a4 = (4*(v0 - vf + 2*a0*t))/t^2 - (2*a0 - af)/(2*t) - (10*(a0*t^2 + v0*t + q0 - qf))/t^3;
a5 = (2*a0 - af)/t^2 - (7*(v0 - vf + 2*a0*t))/t^3 + (15*(a0*t^2 + v0*t + q0 - qf))/t^4;
a6 = (3*(v0 - vf + 2*a0*t))/t^4 - (2*a0 - af)/(2*t^3) - (6*(a0*t^2 + v0*t + q0 - qf))/t^5;

c = [a6; a5; a4; a3; a2; a1];
end

