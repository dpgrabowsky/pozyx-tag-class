classdef pozyx_tag < handle
    % pozyx_tag Class used for pozyx anchor data sampling and storage.
    %   Responsible for storing relavent information about pozyx anchors
    %   such as the number of anchors, their hex IDs, and data received 
    %   from the anchors. Also used to sample the data from the anchors.
    %
    % pozyx_tag Properties: 
    %   numAnchors;           - The number of anchors (1x1)
    %   anchorIDs;            - The hex identification of each anchor stored as a string in a cell array  (1xn) 
    %   dataRaw;              - The raw data received from an anchor (1xn) 
    %   timeStamps;           - The time each dataRaw was received at (1xn)
    %   numMeasurments;       - The number of measurments received from each anchor (1xn)
    %   sampleRate;           - The rate at which each anchor is sampled in Hz (1xn)
    %   tagID;                - The hex identification of the tag
    %   topicName;            - The name of the pozyx topic to subscribe to.
    %   pozyxSubsriber        - The subscriber subscribed to the pozyx measurment topic.
    %   startTime             - The time that the first message is received 
    %   youngDistanceMeasure  - A list of distance measurments that are younger than timeLimit seconds
    %   where, n is the number of anchors
    %
    % pozyx_anchors Methods: 
    %   SubscriberCallback   - Parses messges recieved by the subscriber from
    %                          the pozyx topic.
    %   PrintInformation     - Used to print descriptive infromation of object to the command window
    %   RemoveOldMeasurments - Remove distance measurments from youngDistanceMeasure variable that are 
    %                          older than three seconds
        
        
    properties

        numAnchors;     % The number of anchors (1x1)
        anchorIDs;      % The hex identification of each anchor stored as a string in a cell array  (1xn) 
        dataRaw;        % The raw data received from an anchor in a cell array (1xn) 
        timeStamps;     % The time each dataRaw was received at stored in cell array (1xn)
        distanceMeasure % Array of distance measurments stored in cell array (1xn)
        numMeasurments; % The number of measurments received from each sensor (1xn)
        sampleRate;     % The rate at which each anchor is sampled in Hz (1xn)
        anchorTimers;   % Used to automatically poll each sensor at sampleRate (1xn)
        tagID;          % The hex identification of the tag
        topicName;      % The name of the pozyx topic to subscribe to.
        pozyxSubsriber  % The subscriber subscribed to the pozyx measurment topic.
        startTime       % The time that the first message is received 
        youngDistanceMeasure % A list of distance measurments that are younger than timeLimit seconds
        sensorHeight
    end
    
    methods
        function obj = pozyx_tag(tagID,anchorIDs,varargin)
            %pozyx_tag Construct an instance of this class using the tag and anchor ids
            %   Constructor used to instantiate an object of the
            %   pozyx_anchors class.
            %   pozyx_tag(tagID,anchorIDs) Set the tagID and anchor IDs
            %   that the distance measurments are based off.
            %   
            %   pozyx_tag(tagID,anchorIDs,topicName) Set the tagID and
            %   anchor IDs that the distance measurments are based off and
            %   attempt to subsribe to topicName
            
            fprintf('Instantiating pozyx_tag object...\n');
            
            obj.tagID=tagID;
            fprintf('Tag ID:      %s \n',tagID);
            obj.anchorIDs=anchorIDs;
            fprintf('Anchor IDs: \n'); 
            for(ii=1:length(anchorIDs))
                fprintf('             %s \n',anchorIDs{ii});
            end
            obj.numAnchors = length(anchorIDs);
            obj.dataRaw=cell(1,obj.numAnchors);
            obj.timeStamps=cell(1,obj.numAnchors);
            obj.numMeasurments=zeros(1,obj.numAnchors);
            obj.anchorTimers={};
            obj.sampleRate=ones(1,obj.numAnchors)*10; % default sample rate of 10 Hz
            obj.topicName='';
            obj.startTime=[];
            obj.distanceMeasure=cell(1,obj.numAnchors);
            obj.youngDistanceMeasure=cell(1,obj.numAnchors);
            obj.sensorHeight=0.3084;
            if(length(varargin)==1)
                obj.topicName=varargin{1};
                fprintf('Attempting to subsribe to ''%s''\n',obj.topicName);
                fprintf('    Checking topics\n');
                topicList=rostopic('list');
                if(find(strcmp(obj.topicName,topicList)))
                    fprintf('    Topic found, creating subscriber.\n');
                    %obj.pozyxSubsriber=rossubscriber(obj.topicName,'pozyx_ros_examples/DeviceRange',@obj.SubscriberCallback); 
                    obj.pozyxSubsriber=rossubscriber(obj.topicName,@obj.SubscriberCallback);    
                else
                    fprintf('    WARNING: Topic does not exist, subsriber not set \n');
                end
            end
            
            fprintf('Instantiation complete. \n\n');
        end
        
        function obj=SubscriberCallback(obj, src, message)
            addpath('/home/turtlebotmaster/Desktop/MATLAB/custom_msgs/matlab_gen/msggen')
            % SubscriberCallback This function will be called whenever the
            % pozyx topic that is subscribed to receives a messege 
            % src
            % message: The most recenelty published message
            %           -MessageType (string)
            %           -Timestamp   (int)
            %           -Distance    (int)
            %           -RSS         (int)
            %           -Device      (string, device num in dec)
            
            
            %Determine the anchorIdx that the anchor the measurment belongs to. 
            anchorName=message.Device; 
            anchorName=strcat('0x',dec2hex(str2num(anchorName)));            
            anchorIdx=find(not(cellfun('isempty',strfind(obj.anchorIDs,anchorName))));
            
            
            %store the raw measurment
            temp=obj.dataRaw{anchorIdx};
            temp{end+1}=message;
            obj.dataRaw{anchorIdx}=temp;
            
            %increment the number of measurments
            obj.numMeasurments(1,anchorIdx)=obj.numMeasurments(1,anchorIdx)+1;
            
            
            %parse the message for easy of use
            
            %parse timestamps, if this is the first messege treat it as the
            %'start time' as well.
            if(isempty(obj.startTime))
                tic;
                obj.startTime=message.Timestamp;
            end
            %store the time stamps in seconds for ease of use
            temp=obj.timeStamps{anchorIdx};
            temp(end+1)=double(message.Timestamp-obj.startTime)*10^-3;
            obj.timeStamps{anchorIdx}=temp;
            
            %parse the distances
            temp=obj.distanceMeasure{anchorIdx};
            temp(end+1)=real(sqrt((double(message.Distance)*10^-3)^2-obj.sensorHeight^2));
            obj.distanceMeasure{anchorIdx}=temp;
            
            %parse the youngDistance
            Ttemp=obj.timeStamps{anchorIdx};
            if(isempty(obj.youngDistanceMeasure{anchorIdx}))
                temp=[double(message.Distance)*10^-3;Ttemp(end)];
            else
                temp=obj.youngDistanceMeasure{anchorIdx};
                temp(:,end+1)=[double(message.Distance)*10^-3;Ttemp(end)];
            end
            
            obj.youngDistanceMeasure{anchorIdx}=temp;
            
            
            verbose=0;
            if(verbose)
                fprintf('Message recieved on topic %s. \n',obj.topicName);
                message
                
            end
            
        end  
        
        function PrintInformation(obj)
            % PrintInformation Used to print descriptive infromation of
            % object to the command window
            %   Print the number of anchors, each anchor name, each anchors
            %   number of measurments, and the sample rate of each anchor.
            
            fprintf('Printing pozyx_tag object infromation...\n');
            fprintf('Pozyx Tag ID: %s \n',obj.tagID);
            if(~isempty(obj.pozyxSubsriber))
               fprintf('Subscribed to topic: ''%s''\n', obj.topicName);
            end
            
            fprintf('Number of anchors: %d \n',obj.numAnchors);

            T=table(obj.anchorIDs',obj.numMeasurments');
            T.Properties.VariableNames={'Anchor_IDs', 'Number_of_Measurments'};
            T
            fprintf('\n');
        end
        
        function obj=RemoveOldMeasurments(obj,timeLimit)
           % RemoveOldMeasurements removes all measurments for youngDistanceMeasurme that are older than timeLimit
           % timeLimit - the oldest time in seconds that will not be
           % deleted
           
           %temporarily store newDistanceMeasure in case it is accessed by
           %something else  
           youngDistanceMeasure=obj.youngDistanceMeasure;
           
           %find the row indexes for each nodse timestamps that are greater
           %than timeLimit 
           currentTime=toc;
           for(ii=1:length(youngDistanceMeasure))
               temp=youngDistanceMeasure{ii};
               if(~isempty(temp))
                   idxs=((currentTime-temp(2,:))>timeLimit);
                   temp(:,idxs)=[];
                   youngDistanceMeasure{ii}=temp;
               end
           end
            
           obj.youngDistanceMeasure=youngDistanceMeasure;
        end
        
    end
end

