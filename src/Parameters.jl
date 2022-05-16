# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Parameters

# SIGNALS RELATED
const SIGNAL_POS_X_MARGIN = 80.0
const SIGNAL_POS_Y_MARGIN = 80.0
const SIGNAL_RED_TIME = 1500
const SIGNAL_GREEN_TIME = 1200
const SIGNAL_YELLOW_TIME = 300
const SIGNAL_MS = 5

# LANE DIMENSIONS
const H_NUM_LANES = 2
const H_LANE_WIDTH = 50
const H_MEDIAN_WIDTH = 2
const V_NUM_LANES = 2
const V_LANE_WIDTH = 50
const V_MEDIAN_WIDTH = 2

# PEDESTRIAN WALKWAYS AND CROSSWALKS RELATED
const PEDESTRIAN_WALKWAY_WIDTH = 28
# Average CrossPath width 2M i.e 2 * 15.5 = 31
const CROSS_PATH_WIDTH = 31
# Instead of right at the edge of the road (near intersection)
# this margin dictates how much away the cross path is located
const CROSS_PATH_MARGIN = 10

# VEHICLE RELATED
# cars 10 km/h ≈ 2.5 m/s
# lets use milliseconds instead of seconds
# for smooth animation we use 24 frames/sec ≈ 25 f/s
# 25 frames take 1 sec (1000 msec) so 1 frame takes 1000/25 = 40 ms
# ∴ 1 unit of time in our simulation = 40 milliseconds
# if a car moves 2.5 m/1000ms then it moves (2.5/1000) * 40 = 0.1m in 40 ms
const VEHICLE_INITIAL_SPEED = 0.1
# Average Car width 1.8 m
# ∴ Car width = 1.8 * 15.5 ≈ 27.9 units (rounded to 28)
# Average Car length 4.5 m
# ∴ Car length = 4.5 * 15.5 ≈ 68.75 units (rounded to 69)
# source : https://measuringstuff.com/car-length-and-width-measured-in-feet/
const VEHICLE_LENGTH = 69
const VEHICLE_WIDTH = 28

# VEHICLE SPAWN RATE (ONCE EVERY 'N' TICKS)
const VSR_MIN = 100
const VSR_INC = 100
const VSR_MAX = 5000

# ENVIRONMENT RELATED
const EXTENT_WIDTH = 4100
const EXTENT_HEIGHT = 4100

# PLOT AND AESTHETICS RELATED
# road
const ROAD_BOUNDARY_COLOR = :red
const ROAD_BOUNDARY_LW = 2
# lane
const LANE_MARKER_COLOR = :blue
const LANE_MARKER_LS = :dash
# median
const MEDIAN_COLOR = :green
const MEDIAN_LW = 2
# cross path
const CP_COLOR = :darkorange
const CP_LW = 2
# pedestrian walk way boundary (aka platform)
const PW_COLOR = :black
# plot title and alignment
const PLOT_TITLE = "Indian Traffic Simulator"
const PLOT_TITLE_ALIGN = :center

end