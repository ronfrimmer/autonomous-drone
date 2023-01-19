
opts = delimitedTextImportOptions("NumVariables", 7);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = (",");

% Specify column names and types
opts.VariableNames = ["x", "z", "y", "VarName4", "VarName5", "VarName6", "VarName7"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
pointData = readtable("C:\Users\ASUS\Documents\matlab_scripts\Y2.csv", opts);
datax=pointData.x;
datay=pointData.y;

%draw pointcloud
figure (12);
scatter(datax,-datay,10,'filled');
grid minor
% pause(3);

angle_res=2;
precent=0.05;

for i=1:length(datax)
    p1=point;
    p1.x=datax(i);
    p1.y=datay(i);
    p1.d=p1.distance;
    pointsVector(i)=p1;
end


%create pizza slices
slice=pizzaSlices(pointsVector,angle_res);


%ploting slices
 %slicemovie


%statistics analysis
MuSigma=table('Size',[length(slice) 6],'VariableTypes',{'double','double','double','double','double','double'},'VariableNames',{'mu','sigma','sliceNum','Xval','Yval','max_distance'});
for j=1:length(slice)
if((isempty(slice{j}))|| (length(slice{j})<2))
    MuSigma(j,:)={0,0,j,nan,nan,nan};
else
    k=1;
    for p=slice{j}
        d(k)=p.d;
        k=k+1;
    end
    normalParam=createFit(d);
    [x,y]=pol2cart(deg2rad((j-1)*angle_res+angle_res/2),normalParam.mu);
    MuSigma(j,:)={normalParam.mu,normalParam.sigma,j,x,y,max(d)};
    clear d
end 
end 
for i=1:height(MuSigma)
    if(MuSigma(i,:).mu==0 && i~=1)
        MuSigma(i,:).mu=MuSigma(i-1,:).mu;
%     else
%          MuSigma(i,:).mu=MuSigma(floor(360/angle_res)-1,:).mu; %still have problem when the last and first slices are empty
    end
end

MuSigma = sortrows(MuSigma,'sigma','descend');
MuSigma_to_draw = MuSigma(floor(360/angle_res*precent):end,:);


top_var_Slices = MuSigma(1:floor(360/angle_res*precent),:);
top_var_Slices = sortrows(top_var_Slices,'sliceNum','ascend');
i=1;
while(i<height(top_var_Slices)+1)
    flag=true;
    count=0;
    sum=0;
    while (flag&&i<height(top_var_Slices))
        if(top_var_Slices(i+count,:).sliceNum+1==top_var_Slices(i+1+count,:).sliceNum)
            sum=sum+top_var_Slices(i+count,:).max_distance;
            count=count+1;
        elseif(count~=0)
            sum=sum+top_var_Slices(i+count,:).max_distance;
            flag=false;
            for j=i:i+count
                top_var_Slices(j,:).mu=sum/(count+1);
                [top_var_Slices(j,:).Xval,top_var_Slices(j,:).Yval]=pol2cart(deg2rad((top_var_Slices(j,:).sliceNum-1)*angle_res+angle_res/2),top_var_Slices(j,:).mu);
            end
        else
            flag=false;
        end
        if((i+count)==height(top_var_Slices))
            sum=sum+top_var_Slices(i+count,:).max_distance;
            flag=false;
            for j=i:i+count
                top_var_Slices(j,:).mu=sum/(count+1);
                [top_var_Slices(j,:).Xval,top_var_Slices(j,:).Yval]=pol2cart(deg2rad((top_var_Slices(j,:).sliceNum-1)*angle_res+angle_res/2),top_var_Slices(j,:).mu);
            end
        end
    end
    i=i+count+1;
end

% while(top_var_Slices(i,:).sliceNum+1==top_var_Slices(i+1,:).sliceNum)
%     sum=sum+top_var_Slices(i,:).max_distance;
%     count=count+1;
% end



MuSigma2=complete_empty_slices(MuSigma_to_draw,MuSigma,angle_res);

figure (99)
%scatter(MuSigma.Xval,-MuSigma.Yval,70,'filled','square')
figure (99)
scatter(MuSigma_to_draw.Xval,-MuSigma_to_draw.Yval,70,'filled','square')
hold on
scatter(MuSigma2.Xval,-MuSigma2.Yval,70,'filled','square','r')
hold on
scatter(top_var_Slices.Xval,-top_var_Slices.Yval,70,'filled','square','g')




x_tot=[MuSigma2.Xval; MuSigma_to_draw.Xval;top_var_Slices.Xval];
y_tot=[MuSigma2.Yval; MuSigma_to_draw.Yval;top_var_Slices.Yval];
[theta_tot,r_tot]=cart2pol(x_tot,y_tot);
r_thete_mat=[r_tot*1000 mod(rad2deg(theta_tot)+360,360)];
r_thete_mat=rmmissing(r_thete_mat);
save 'C:\aliza\final_project\Tello code and data\scan_data.mat' r_thete_mat

avg=mean(MuSigma_to_draw.mu);

