clc
clear
addpath('/home/turtlebotmaster/Desktop/MATLAB/custom_msgs/matlab_gen/msggen')
addpath('/home/turtlebotmaster/Desktop/pozyx_tag_class');
addpath('/home/turtlebotmaster/Desktop/');
rosshutdown;
setenv('ROS_MASTER_URI','http://10.32.0.20:11311');
setenv('ROS_IP','10.32.0.78');
try
    rosinit
catch
    fprintf('\nWARNING: ROS not initalized\n');
end
fprintf('\n');

aID={'0x6937','0x6940','0x6935','0x6955'};
tID='0x0000';

%% Start 
tag_obj=pozyx_tag(tID,aID,'/pozyx_device_range');


%% Everything below here is for SLAM testing

disp('Subscribing to /odom')
try
    odom=rossubscriber('/odom');
catch
    fprintf('\nWARNING: odom subscirption FAILED\n');
end
anchors.names={...
    '0x6937';...
    '0x6940';...
    '0x6935';...
    '0x6955'}; 
anchors.coords=[...
    0,0;...
    6.7056,0;...
    6.7056,3.6576;...
    0,3.6577];
pause(5);

disp('Starting');
while(1)
   %Get set of four measurments,then multilaterate
   
   tag_obj.RemoveOldMeasurments(3);
   distances=[];
   badFlag=0;
   for(ii=1:length(aID))
       if(~isempty(tag_obj.youngDistanceMeasure{ii}))
        distances=[distances; tag_obj.youngDistanceMeasure{ii}(1,end)]; 
       else
           badFlag=1;
       end
       
   end
   
   if(~badFlag)
       [a]=trilat(anchors.coords,distances);
       x=a(1);
       y=a(2);
       
       figure(1)
       scatter(anchors.coords(:,1),anchors.coords(:,2))
       hold on
       scatter(x,y);
       xlim([-1 7]);
       ylim([-1 5]);
       hold off
   end
   
   pause(.1); 
   
   
end
