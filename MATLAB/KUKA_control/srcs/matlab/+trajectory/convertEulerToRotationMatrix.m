function R = convertEulerToRotationMatrix(euler)
% convert euler angles (in radians) to rotation matrix R[3x3].
%source link:
%https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm

ct = cos(euler); c1 = ct(1); c2 = ct(2); c3 = ct(3);
st = sin(euler); s1 = st(1); s2 = st(2); s3 = st(3);
R = [c2*c1, s3*s2*c1 - c3*s1, c3*s2*c1 + s3*s1;
    c2*s1,  s3*s2*s1 + c3*c1, c3*s2*s1 - s3*c1;
    -s2,    s3*c2,            c3*c2];
end