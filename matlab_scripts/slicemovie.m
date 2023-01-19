figure;
for j=1:length(slice)
k=1;
if(isempty(slice{j}))
    continue;
end
clear xx yy
for p=slice{j}
    xx(k)=p.x;
    yy(k)=p.y;
    k=k+1;
end
scatter(xx,yy);
hold on 
pause(0.5)
end