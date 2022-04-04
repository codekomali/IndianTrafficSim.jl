# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Parameters

const SIGNAL_POS_X_MARGIN = 80.0
const SIGNAL_POS_Y_MARGIN = 80.0
const SIGNAL_RED_TIME = 1500
const SIGNAL_GREEN_TIME = 1200
const SIGNAL_YELLOW_TIME = 300
const SIGNAL_MS = 5

const H_NUM_LANES = 2
const H_LANE_WIDTH = 50
const H_MEDIAN_WIDTH = 2

const V_NUM_LANES = 2
const V_LANE_WIDTH = 50
const V_MEDIAN_WIDTH = 2

const ROAD_BOUNDARY_COLOR = :red
const ROAD_BOUNDARY_LW = 2
const LANE_MARKER_COLOR = :blue
const LANE_MARKER_LS = :dashed
const MEDIAN_COLOR = :green
const MEDIAN_LW = 2

const PEDESTRIAN_WALKWAY_WIDTH = 28

# cars 10 km/h ≈ 2.5 m/s
# lets use milliseconds instead of seconds
# for smooth animation we use 24 frames/sec ≈ 25 f/s
# 25 frames take 1 sec (1000 msec) so 1 frame takes 1000/25 = 40 ms
# if a car moves 2.5 m/1000ms then it moves (2.5/1000) * 40 = 0.1m in 40 ms
const VEHICLE_INITIAL_SPEED = 0.1

end