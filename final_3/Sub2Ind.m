
function ndx = Sub2Ind(a, i1, i2, i3)
ndx = i1 + (i2-1)*a(1) + (i3-1)*a(1)*a(2);
end
