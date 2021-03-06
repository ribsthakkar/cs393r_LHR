loc_noise = 0.5
angle_noise = 0.1
naive_motion_model = {
    k1 = 0.5,
    k2 = 0.5,
    k3 = 0.1,
    k4 = 0.1
}
motion_model = {
    k1 = 0.15,
    k2 = 0.01,
    k3 = 0.8
}
gamma = 0.8
lidar_stddev = 0.15
rays = 50
distance_observe_threshold = 0.05
angle_observe_threshold = 0.1
robust_observation_likelihood_multiple = 13.8629 -- 20log(2)
min_update_before_resample_count = 6
