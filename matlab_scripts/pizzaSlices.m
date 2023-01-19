function slices=pizzaSlices(pointsVector,k)
slices=cell(1,floor(360/k));
for p=pointsVector
    angle=mod(rad2deg(atan2(p.y,p.x))+360,360);
    slicekey=floor(angle/k);
    slices{slicekey+1}=[slices{slicekey+1},p];
end
end