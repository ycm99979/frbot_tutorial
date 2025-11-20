include "map_builder.lua" 
include "trjectory_builder.lua"

options = {
    -- frame settings
    map_builder = MAP_BUILDER,             -- Global SLAM 및 submap 관리
    trjectory_builder = TRAJECTORY_BUILDER,-- Local SLAM 및 sensor 
    map_frame = "map",                     
    tracking_frame = "base_link",          -- 현재 위치를 추정하는 기준 frame
    published_frame = "odom",              -- cartographer가 pub할 pose
    odom_frame = "odom",                   -- 이때 map -> odom으로 tf 변환

    -- tf publish settings
    provide_odom_frame = false,
    publish_frame_projected_to_2d = true,  -- footprint라는 이름으로 2d 공간 상에 투영
                                           -- 2d SLAM 진행 시 유용
    -- sensor settings
    use_odometry = true,                   -- nav2 사용 시 nav_msgs/Odometry 라는 topic이 나옴
    use_navsat = false,                    -- GPS (nav_msgs/NavSaFix) topic 사용 여부
    use_landmarks = false,                 -- landmarks 일반적으로 pose를 추정하는데 기준점이 되는데 여기서도 같은 의미인진 모름
    num_laser_scans = 1,                   -- 2d 평면 상의 scan을 사용 여부
    num_multi_echo_laser_scans = 0,        -- 다중 에코 레이너 스캐너 사용 여부
    num_subdivisions_per_laser_scan = 1,   -- LaserScan 메세지를 분할할지 여부 1이면 사용 X
    num_point_clouds = 0,                  -- sensor_msgs/PointCloud2 수신 여부

    -- data publish and sampling settings
    lookup_transform_timeout_sec = 0.2,    
    submap_publish_period_sec = 0.3,       -- submap pub 주기 0.3s
    pose_publish_period_sec = 5e-3,        -- 현재 pose를 pub하는 주기 (현재 0.005s = 200Hz)
    trajectory_publish_period_sec = 30e-3, -- 궤적 발행 주기 (0.03s = 33Hz)
    rangefinder_sampling_ratio = 1.,       -- 데이터의 샘플링 비율 
    odometry_sampling_ratio = 0.1,         -- odom 메세지 중 10%만 사용
    fixed_frame_pose_sampling_ratio = 1.,  -- fixed pose 발생 비율
    imu_sampling_ratio = 1.,               -- imu data 사용 비율
    landmarks_sampling_ratio = 1.,         -- landmark (사용X)
}
