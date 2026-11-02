function pose = cnn_sensor(x) %#ok<INUSD>
%CNN_SENSOR Real CNN in the loop: Blender renders the camera view, a keypoint CNN + PnP estimates the pose.
%   Withheld in this public beta: the perception core will be released with v1.0.
error('docking:betaWithheld', ['The CNN-in-the-loop sensor is not part of the public beta. ' ...
    'Use sensor mode 0 (true pose) or 1 (statistical model of the CNN error).']);
end
