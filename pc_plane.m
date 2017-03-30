% rosinit('http://localhost:11311')
sub = rossubscriber('/vslam/pc2',rostype.sensor_msgs_PointCloud2);
pub = rospublisher('/position','std_msgs/Float32MultiArray');
msg = rosmessage(pub);


while 1
    pc=receive(sub);
    ptCloud=pointCloud(readXYZ(pc));
    maxDistance=0.01;
    Data=[];
    density_threshold=200;
    h_min=0.4;
    h_max=1;
    wid_min=0.4;
    wid_max=1;
   %     scatter3(pc);
    % referenceVector = [0,0,1];
    % referenceVector = [-1,0,1];
    %  [model,inlierIndices,outlierIndices]=pcfitplane(ptCloud,maxDistance,referenceVector,5);

    % read the middle point and size of first plane
     [model,inlierIndices,outlierIndices]=pcfitplane(ptCloud,maxDistance,'Confidence',90);
     plane1 = select(ptCloud,inlierIndices);
     point_first=plane1.Location;
     size(point_first);
    %  find largest value
     point_sorted_down=sort(point_first,'descend');
     threshold=round(size(point_sorted_down,1)*0.005);
    %  threshold=5;
    %  point_max=max(point_first,[],1);
    point_max=point_sorted_down(threshold,:);
    % find smallest value
    point_sorted_up=sort(point_first);
     point_min=point_sorted_up(threshold,:);
     size_triangle=point_max-point_min;
     w=sqrt(size_triangle(1)^2+size_triangle(3)^2);
     h=size_triangle(2);
     middle_point=(point_max+point_min)./2;
     %calculate the orientation of first plane
     norm_1=model.Normal;
     norm=[norm_1(1),norm_1(3),-norm_1(2)];
     angle1=atan(norm(3)/(sqrt(norm(1)^2+norm(2)^2)));
     angle2 =atan(norm(1)/norm(2));
     plane1_position=[middle_point,w,h,angle1,angle2];
     size(point_first,1)/w/h;
    %data publishing
     %     pub = rospublisher('/position','std_msgs/Float32MultiArray');
     %     msg = rosmessage(pub);
     if  size(point_first,1)/w/h>density_threshold&w>wid_min&w<wid_max&h>h_min&h<h_max
        size(point_first,1)
        Data = [Data,plane1_position];
     end


     remainPtCloud = select(ptCloud,outlierIndices);
     [model2,inlierIndices,outlierIndices]=pcfitplane(remainPtCloud,0.01,'Confidence',90);
    %  pcshow(plane1)
    %  plot(model);
    % figure 
    plane2 = select(remainPtCloud,inlierIndices);
    % hold on
    pcshow(plane2);
    point_second=plane2.Location;
     %  find largest value
     point_sorted_down=sort(point_second,'descend');
     threshold=round(size(point_sorted_down,1)*0.005);
    threshold=1;
    %  point_max=max(point_first,[],1);
    point_max=point_sorted_down(threshold,:);
    % find smallest value
    point_sorted_up=sort(point_second);
     point_min=point_sorted_up(threshold,:);
     size2=point_max-point_min;
     w=sqrt(size2(1)^2+size2(3)^2);
     h=size2(2);
     middle_point2=(point_max+point_min)./2;
     %calculate the orientation of first plane
     norm_2=model2.Normal;
     norm=[norm_2(1),norm_2(3),-norm_2(2)];
     angle1=atan(norm(3)/(sqrt(norm(1)^2+norm(2)^2)));
     angle2 =atan(norm(1)/norm(2));
     plane2_position=[middle_point2,w,h,angle1,angle2];
     size(point_second,1)/w/h;
    %data publishing
     if  size(point_second,1)/w/h>density_threshold&w>wid_min&w<wid_max&h>h_min&h<h_max
         size(point_second,1)
        Data = [Data,plane2_position];
     end

    %-------------------------------------------------------------------------------
    % thrid plane
    remainPtCloud1 = select(remainPtCloud,outlierIndices);
     [model3,inlierIndices,outlierIndices]=pcfitplane(remainPtCloud1,0.01,'Confidence',90);
    %  pcshow(plane1)
    %  plot(model)
    plane3= select(remainPtCloud1,inlierIndices);
    point_third=plane3.Location;
     %  find largest value
     point_sorted_down=sort(point_third,'descend');
     threshold=round(size(point_sorted_down,1)*0.005)+1;
    % threshold=1;
    point_max=point_sorted_down(threshold,:);
    % find smallest value
    point_sorted_up=sort(point_third);
     point_min=point_sorted_up(threshold,:);
     size3=point_max-point_min;
     w=sqrt(size3(1)^2+size3(3)^2);
     h=size3(2);
     middle_point3=(point_max+point_min)./2;
     %calculate the orientation of first plane
     norm_3=model3.Normal;
     norm=[norm_3(1),norm_3(3),-norm_3(2)];
     angle1=atan(norm(3)/(sqrt(norm(1)^2+norm(2)^2)));
     angle2 =atan(norm(1)/norm(2));
     plane3_position=[middle_point3,w,h,angle1,angle2];
     size(point_third,1)/w/h;
    %data publishing
     if  size(point_third,1)/w/h>density_threshold&w>wid_min&w<wid_max&h>h_min&h<h_max
         size(point_third,1)
        Data = [Data,plane3_position];
     end

    %-------------------------------------------------------------------------------
    % fourth plane
    remainPtCloud2 = select(remainPtCloud1,outlierIndices);
     [model4,inlierIndices,outlierIndices]=pcfitplane(remainPtCloud2,0.01,'Confidence',90);
    %  pcshow(plane1)
    %  plot(model)
    plane4= select(remainPtCloud2,inlierIndices);
    point_fourth=plane4.Location;
     %  find largest value
     point_sorted_down=sort(point_fourth,'descend');
    %  threshold=round(size(point_sorted_down,1)*0.005);
    threshold=1;
    point_max=point_sorted_down(threshold,:);
    % find smallest value
    point_sorted_up=sort(point_fourth);
     point_min=point_sorted_up(threshold,:);
     size4=point_max-point_min;
     w=sqrt(size4(1)^2+size4(3)^2);
     h=size4(2);
     middle_point4=(point_max+point_min)./2;
     %calculate the orientation of first plane
     norm_4=model4.Normal;
     norm=[norm_3(1),norm_3(3),-norm_3(2)];
     angle1=atan(norm(3)/(sqrt(norm(1)^2+norm(2)^2)));
     angle2 =atan(norm(1)/norm(2));
     plane4_position=[middle_point4,w,h,angle1,angle2];
     size(point_fourth,1)/w/h;
    %data publishing
     if  size(point_fourth,1)/w/h>density_threshold&w>wid_min&w<wid_max&h>h_min&h<h_max
         size(point_fourth,1)
        Data = [Data,plane4_position];
     end
     msg.Data=Data;
     send(pub,msg);

end