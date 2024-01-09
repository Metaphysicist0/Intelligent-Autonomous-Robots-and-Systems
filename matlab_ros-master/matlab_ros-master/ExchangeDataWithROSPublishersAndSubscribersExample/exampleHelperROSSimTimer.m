function exampleHelperROSSimTimer(~, ~, handles)
    %exampleHelperROSSimTimer - Timer update function called in startExamples to publish
    %   messages at well-defined intervals
    %   exampleHelperROSSimTimer(~,~,HANDLES) publishes /pose and /scan messages at
    %   regular intervals. The /scan message value remains constant while
    %   the /pose message value changes continually.
    %   
    %   See also exampleHelperROSCreateSampleNetwork
    
    %   Copyright 2014-2021 The MathWorks, Inc.
    
    % Update the pose message values
    if isvalid(handles.twistPub)
        handles.twistPubmsg.Linear.X = handles.twistPubmsg.Linear.X + (rand(1)-0.5)./10;
        handles.twistPubmsg.Linear.Y = handles.twistPubmsg.Linear.Y + (rand(1)-0.5)./10;
        handles.twistPubmsg.Linear.Z = handles.twistPubmsg.Linear.Z + (rand(1)-0.5)./10;
        handles.twistPubmsg.Angular.X = handles.twistPubmsg.Angular.X + (rand(1)-0.5)./10;
        handles.twistPubmsg.Angular.Y = handles.twistPubmsg.Angular.Y + (rand(1)-0.5)./10;
        handles.twistPubmsg.Angular.Z = handles.twistPubmsg.Angular.Z + (rand(1)-0.5)./10;
    
        % Publish the scan and pose messages
        send(handles.twistPub,handles.twistPubmsg);
    end
       
    if isvalid(handles.scanPub)
        handles.scanPubmsg.Header.Stamp = rostime('now','system',...
            'DataFormat',handles.scanPub.DataFormat);
        send(handles.scanPub,handles.scanPubmsg);
    end
end