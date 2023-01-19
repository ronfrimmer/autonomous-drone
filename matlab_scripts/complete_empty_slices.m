function MuSigma2=complete_empty_slices(MuSigma_to_draw,MuSigma,angle_res)
MuSigma2 = MuSigma_to_draw(MuSigma_to_draw.sigma==0,:);
MuSigma2 = sortrows(MuSigma2,'sliceNum','ascend');

for i=1:height(MuSigma2)
    mu_previos=MuSigma(MuSigma.sliceNum==(MuSigma2(i,:).sliceNum-1),:).mu; % have problem if the first slice num is 1
    if(~isempty(MuSigma2(MuSigma2.sliceNum==(MuSigma2(i,:).sliceNum-1),:)))
        mu_previos=MuSigma2(MuSigma2.sliceNum==(MuSigma2(i,:).sliceNum-1),:).mu;
    end
    angle=MuSigma2(i,:).sliceNum*angle_res;
    matching_angle=mod(angle+180-angle_res,360);
    if(matching_angle==0)
        matching_angle=360;
    end
    mu_complementry=MuSigma(MuSigma.sliceNum==(matching_angle/angle_res),:).mu;
    if(mu_complementry==0)
        MuSigma2(i,:).mu=mu_previos;
        [MuSigma2(i,:).Xval,MuSigma2(i,:).Yval]=pol2cart(deg2rad((MuSigma2(i,:).sliceNum-1)*angle_res+angle_res/2),MuSigma2(i,:).mu);
        continue
    end
    ratio_factor=mu_previos/mu_complementry;
    MuSigma2(i,:).mu=MuSigma(MuSigma.sliceNum==((mod(matching_angle,360)+angle_res)/angle_res),:).mu*ratio_factor;
    if(MuSigma2(i,:).mu==0)
        MuSigma2(i,:).mu=MuSigma2(MuSigma2.sliceNum==((mod(matching_angle,360)+angle_res)/angle_res),:).mu*ratio_factor;
    end
    [MuSigma2(i,:).Xval,MuSigma2(i,:).Yval]=pol2cart(deg2rad((MuSigma2(i,:).sliceNum-1)*angle_res+angle_res/2),MuSigma2(i,:).mu);
end
end


