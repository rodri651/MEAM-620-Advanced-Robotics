function [p0,p1,p2,p3,p4] = get_values(id)

num=length(id);
p4=zeros(2,num);
p3=zeros(2,num);
p2=zeros(2,num);
p1=zeros(2,num);
p0=zeros(2,num);
for i = 1:num
    x=floor(id(i)/12)*0.304;
    y=rem(id(i),12)*0.304;
    p4(2,i) = x;%x direction
    p1(2,i) = x;
    p2(2,i) = x+0.152;
    p3(2,i) = x+0.152;
    p0(2,i) = x+0.152/2;
    if floor(id(i)/12)>5
        p4(2,i) = p4(2,i) + 0.026*2;
        p1(2,i) = p1(2,i) + 0.026*2;
        p2(2,i) = p2(2,i) + 0.026*2;
        p3(2,i) = p3(2,i) + 0.026*2;
        p0(2,i) = p0(2,i) + 0.026*2;
    elseif floor(id(i)/12)>2
        p4(2,i) = p4(2,i) + 0.026;
        p1(2,i) = p1(2,i) + 0.026;
        p2(2,i) = p2(2,i) + 0.026;
        p3(2,i) = p3(2,i) + 0.026;
        p0(2,i) = p0(2,i) + 0.026;
    end
    p4(1,i) = y;
    p3(1,i) = y;
    p1(1,i) = y+0.152;%y direction
    p2(1,i) = y+0.152;
    p0(1,i) = y+0.152/2;
end

end
    
    