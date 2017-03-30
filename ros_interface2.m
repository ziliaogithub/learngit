ObjektSpeicherProgramm = ObjektSpeicherII ;
  rosinit('http://localhost:11311')

%subscribe to a topic and get the image
 sub = rossubscriber('/ardrone/image_raw',rostype.sensor_msgs_Image);
%  sub = rossubscriber('/usb_cam/image_raw',rostype.sensor_msgs_Image);
%initialize a publisher
chatpub = rospublisher('/people','std_msgs/Float32MultiArray');
msg = rosmessage(chatpub);
while 1
     %receive the data
     ObjektSpeicherProgramm.Id;
     picture = receive(sub);
     %convert the image to matlab format
    a=readImage(picture);
    
    ObjektSpeicherProgramm.Frame=a;
    ObjektSpeicherProgramm.FrameGray = rgb2gray(a);
    if ~isempty(ObjektSpeicherProgramm.Frame)
    % Objekterkennung 
%               ObjektSpeicherProgramm.Id
%               ObjektSpeicherProgramm.Objekt_Position
              if isempty(ObjektSpeicherProgramm.Id) 
              ObjektSpeicherProgramm.Objekterkennung(); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Objektverfolgung 
              else
              ObjektSpeicherProgramm.Verfolgung(); 
              end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
               if ~isempty(ObjektSpeicherProgramm.Id)
                   %send the data to ros topic
                   %msg.Data = ObjektSpeicherProgramm.Id;
                   %send(chatpub,msg);
                   b= ObjektSpeicherProgramm.Objekt_Position;
                   c=[ObjektSpeicherProgramm.Id;b'] ;
                   msg.Data = reshape(c,[1,numel(c)]) 
                   ObjektSpeicherProgramm.Objekt_Position;
                   send(chatpub,msg);
                   ObjektSpeicherProgramm.ObjektInVideoAnzeigen();
               end
    %              ObjektSpeicherProgramm.Datenaufbereitung(TCP_SEND); 
    % cv.imshow('sss', ObjektSpeicherProgramm.Frame);
    % pause(0.001)
    end
    %show the image
    image(ObjektSpeicherProgramm.Frame);
end
%use the function