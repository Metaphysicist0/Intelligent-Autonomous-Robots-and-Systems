function odomCallback(~, message)
    global x
    global y
    global theta

    pose = message.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;

    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = angles(1);

    disp('x, y, theta: ');
    [x, y, theta]
end

