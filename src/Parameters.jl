# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Parameters

# Scale
# Typical lane width = 3.7 m
# Current lane width = 50 units
# ∴ 1 m = (50/3.7) ≈ 13.5 units

# SIGNALS RELATED
const SIGNAL_POS_X_MARGIN = 80.0
const SIGNAL_POS_Y_MARGIN = 80.0
const SIGNAL_RED_TIME = 1500 # 60 Seconds
const SIGNAL_GREEN_TIME = 1200 # 48 Seconds
const SIGNAL_YELLOW_TIME = 300 # 12 Seconds
const SIGNAL_MS = 5

# LANE DIMENSIONS
const H_NUM_LANES = 2
const H_LANE_WIDTH = 50
const H_MEDIAN_WIDTH = 2
const V_NUM_LANES = 2
const V_LANE_WIDTH = 50
const V_MEDIAN_WIDTH = 2

# PEDESTRIAN WALKWAYS AND CROSSWALKS RELATED
const PEDESTRIAN_WALKWAY_WIDTH = 24
# Average CrossPath width 2M i.e 2 * 13.5 = 27
const CROSS_PATH_WIDTH = 27
# Instead of right at the edge of the road (near intersection)
# this margin dictates how much away the cross path is located
const CROSS_PATH_MARGIN = 10

# VEHICLE RELATED
# cars 10 km/h ≈ 2.5 m/s
# lets use milliseconds instead of seconds
# for smooth animation we use 24 frames/sec ≈ 25 f/s
# 25 frames take 1 sec (1000 msec) so 1 frame takes 1000/25 = 40 ms
# ∴ 1 unit of time in our simulation = 40 milliseconds
# if a car moves 2.5 m/1000ms then it moves (2.5/1000) * 40 = 0.1m in 40 ms = 0.1m/tick
# 0.1m/tick = 0.1 * 13.5 = 1.35 units/tick
const VEHICLE_INITIAL_SPEED = 1.35
# Average Car width 1.8 m
# ∴ Car width = 1.8 * 13.5 ≈ 24.3 units (rounded to 24)
# Average Car length 4.5 m
# ∴ Car length = 4.5 * 13.5 ≈ 60.8 units (rounded to 61)
# source : https://measuringstuff.com/car-length-and-width-measured-in-feet/
const VEHICLE_LENGTH = 61
const VEHICLE_WIDTH = 24

# VEHICLE SPAWN RATE (ONCE EVERY 'N' TICKS)
const VSR_MIN = 100
const VSR_INC = 100
const VSR_MAX = 5000

# ENVIRONMENT RELATED
# 3380 unit / 13.5 unit ≈ 250.37m [roughly quarter km](rounded to 3380)
const EXTENT_WIDTH = 3380
const EXTENT_HEIGHT = 3380

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

const L2R_ORIENTATION = (1.0, 0.0)
const R2L_ORIENTATION = (-1.0, 0.0)
const B2T_ORIENTATION = (0.0, 1.0)
const T2B_ORIENTATION = (0.0, -1.0)
# IDM Parameters from Treiber 2000 (congested traffic states)

# 0.73 m/s^2
# 0.00000073 m/ms^2 ≈ 0.0000292 m/tick^2 ≈ 0.00039 units/tick^2 ≈ 0.0004 units/tick^2 
const A_max = 0.0004

# 1.6 seconds
# 1600 ms = 40 ticks
const Safe_T = 40

# 1.67 m/s^2
# 0.00000167 m/ms^2 ≈ 0.0000668 m/tick^2 ≈ 0.0009 units/tick^2
const B_dec = 0.0009

# same as vehicle initial speed (10 km/h ≈ 2.5 m/s)
const V0_pref = VEHICLE_INITIAL_SPEED

# Jam distance
const S0_jam = VEHICLE_LENGTH + 10

# DEBUGGING PARAMETERS
const DEBUG_MODE = true
const AGENT_NEARBY_THRESH = 100

end