function T = computeT(linkDH, q)
q = q + linkDH.offset;
T = [cos(q)     -sin(q)*cos(linkDH.alpha)   sin(q)*sin(linkDH.alpha)    linkDH.a*cos(q);
    sin(q)      cos(q)*cos(linkDH.alpha)    -cos(q)*sin(linkDH.alpha)   linkDH.a*sin(q);
    0           sin(linkDH.alpha)           cos(linkDH.alpha)           linkDH.d;
    0           0                           0                           1];
end
