function q = convertEulerToQuaternion(euler)
% convert euler angles (in radians) to quaternion.
%source link:
%https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

ct = cos(euler/2); c1 = ct(1); c2 = ct(2); c3 = ct(3);
st = sin(euler/2); s1 = st(1); s2 = st(2); s3 = st(3);
q = [c1*c2*c3 + s1*s2*s3, ...
    c1*c2*s3 - s1*s2*c3, ...
    c1*s2*c3 + s1*c2*s3, ...
    s1*c2*c3 - c1*s2*s3];
end