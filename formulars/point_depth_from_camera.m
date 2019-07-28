function w = point_depth_from_camera(K,R,camera_position_in_world_frame,X)

% Multiple view geometry in computer vision 2nd version, P162
% w = m_3_t*( X - C ), where X is the point position in the world frame and C is
% the camera position in the world frame, w is the depth from camera to the
% point in the world frame ( and the camera frame ).

% m_3 is the principal ray direction
% m_3_t is the third row of M, multiple view geometry in computer vision,
% P158
% M = K*R, multiple view geometry in computer vision 2nd version, P157

M = K*R;
m_3_t = M(3,:);
m_3 = m_3_t';

C = camera_position_in_world_frame';

w = m_3_t*( X - C );

end