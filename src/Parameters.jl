# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Parameters

include("Utils.jl")
import .Utils as U

# TIME SCALE INFORMATION - UNIT OF TIME
# lets use milliseconds instead of seconds
# for smooth animation we use 24 frames/sec ≈ 25 f/s
# 25 frames take 1 sec (1000 msec) so 1 frame takes 1000/25 = 40 ms
# ∴ 1 unit of time in our simulation = 40 milliseconds

# DEBUGGING PARAMETER
const DEBUG_MODE = true

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

# INTERACTIVE INTERFACE RELATED

# VEHICLE SPAWN RATE (ONCE EVERY 'N' TICKS)
const VSR_MIN = 100
const VSR_INC = 100
const VSR_MAX = 5000

# VEHICLE COMMON PARAMS

const L2R_ORIENTATION = (1.0, 0.0)
const R2L_ORIENTATION = (-1.0, 0.0)
const B2T_ORIENTATION = (0.0, 1.0)
const T2B_ORIENTATION = (0.0, -1.0)

const AGENT_NEARBY_THRESH = 100

# INDIVIDUAL VEHICLES RELATED

# CARS RELATED

# cars 10 km/h ≈ 2.5 m/s
# lets use milliseconds instead of seconds
# for smooth animation we use 24 frames/sec ≈ 25 f/s
# 25 frames take 1 sec (1000 msec) so 1 frame takes 1000/25 = 40 ms
# ∴ 1 unit of time in our simulation = 40 milliseconds
# if a car moves 2.5 m/1000ms then it moves (2.5/1000) * 40 = 0.1m in 40 ms = 0.1m/tick
# 0.1m/tick = 0.1 * 13.5 = 1.35 units/tick
const CAR_INITIAL_SPEED = 1.35
# Average Car width 1.8 m
# ∴ Car width = 1.8 * 13.5 ≈ 24.3 units (rounded to 24)
# Average Car length 4.5 m
# ∴ Car length = 4.5 * 13.5 ≈ 60.8 units (rounded to 61)
# source : https://measuringstuff.com/car-length-and-width-measured-in-feet/
const CAR_LENGTH = 61
const CAR_WIDTH = 24

# Value from Bokare and Maurya (2016) [As explained in Ramireddy et al 2020]
# The maximum acceleration rate
# for truck, three-wheeler, two-wheeler, diesel car and petrol
# car were found to be 1.0, 0.64, 1.96, 2.23, and 2.87 m/s2 respectively
# We assume petrol cars (so 2.87 m/s^2)

const CAR_A_max = U.ms2_to_ut2(2.87)

# 1.6 seconds
# 1600 ms = 40 ticks
const CAR_Safe_T = 40

# Source
# TITLE: STUDY OF DECELERATION BEHAVIOUR OF DIFFERENT VEHICLE TYPES
# Authors: Maurya and Bokare 2012
# Car 1.62 m/s2
const CAR_B_dec = U.ms2_to_ut2(1.62)

# SOURCE: Discharge Characteristics of Heterogeneous Traffic at Signalized Intersections 
# AUTHORS: Maini and Khan 2000
# Signal Clearing speed
# Car 5.43 (smallest in table 5)
const CAR_V0_pref = U.mps_to_upt(5.43)

# Jam distance
const CAR_S0_jam = CAR_LENGTH + 10

#color
const CAR_COLOR = :green

# TRUCKS RELATED

# 10 km/h ≈ 2.5 m/s
# if a truck moves 2.5 m/1000ms then it moves (2.5/1000) * 40 = 0.1m in 40 ms = 0.1m/tick
# 0.1m/tick = 0.1 * 13.5 = 1.35 units/tick
const TRUCK_INITIAL_SPEED = 1.35
# Truck Length 13.6 m Width x 2.45 m (source: http://fess.su/news/dimensions-and-sizes-of-trucks)
# ∴ Truck width = 2.45 * 13.5 ≈ 33.075 units (rounded to 33)
# Truck length 13.6 m
# ∴ Truck length = 13.6 * 13.5 ≈ 183.6 units (rounded to 180)
# source : https://measuringstuff.com/car-length-and-width-measured-in-feet/
const TRUCK_LENGTH = 180
const TRUCK_WIDTH = 33

# Value from Bokare and Maurya (2016) [As explained in Ramireddy et al 2020]
# The maximum acceleration rate
# for truck, three-wheeler, two-wheeler, diesel car and petrol
# car were found to be 1.0, 0.64, 1.96, 2.23, and 2.87 m/s2 respectively
const TRUCK_A_max = U.ms2_to_ut2(1)


# 1.6 seconds
# 1600 ms = 40 ticks
const TRUCK_Safe_T = 40

# Source
# TITLE: STUDY OF DECELERATION BEHAVIOUR OF DIFFERENT VEHICLE TYPES
# Authors: Maurya and Bokare 2012
# Truck 0.88 m/s2
const TRUCK_B_dec = U.ms2_to_ut2(0.88)

# SOURCE: Discharge Characteristics of Heterogeneous Traffic at Signalized Intersections 
# AUTHORS: Maini and Khan 2000
# Signal Clearing speed
# Truck/Bus 5.05 (smallest in table 5)
const TRUCK_V0_pref = U.mps_to_upt(5.05)

# Jam distance 
const TRUCK_S0_jam = TRUCK_LENGTH + 10

#color
const TRUCK_COLOR = :deepskyblue

# MOTORCYCLE RELATED

# 10 km/h ≈ 2.5 m/s (2.77 m/s)
# if a truck moves 2.5 m/1000ms then it moves (2.5/1000) * 40 = 0.1m in 40 ms = 0.1m/tick
# 0.1m/tick = 0.1 * 13.5 = 1.35 units/tick
const MC_INITIAL_SPEED = 1.35

# SOURCE (https://powersportsguide.com/average-motorcycle-dimensions/#:~:text=In%20a%20nutshell%2C%20the%20average,Height%3A%2040%2D60%20inches)
# Length: 75-100 inches and Width: 25-40 inches
# Width: 40 inches = 1.016m (rounded to 1m)
# ∴ MC width = 1 * 13.5 = 13.5 units (rounded to 14)
# MC length 100 inches = 2.54m (rounded to 2.5)
# ∴ MC length = 2.5 * 13.5 ≈ 33.7 units (rounded to 38)

const MC_LENGTH = 38
const MC_WIDTH = 14

# IDM Parameters from Treiber 2000 (congested traffic states)

# Value from Bokare and Maurya (2016) [As explained in Ramireddy et al 2020]
# The maximum acceleration rate
# for truck, three-wheeler, two-wheeler, diesel car and petrol
# car were found to be 1.0, 0.64, 1.96, 2.23, and 2.87 m/s2 respectively
const MC_A_max = U.ms2_to_ut2(1.96)

# 1.6 seconds
# 1600 ms = 40 ticks
const MC_Safe_T = 40

# Source
# TITLE: STUDY OF DECELERATION BEHAVIOUR OF DIFFERENT VEHICLE TYPES
# Authors: Maurya and Bokare 2012
# MC 1.6 m/s2
const MC_B_dec = U.ms2_to_ut2(1.6)

# SOURCE: Discharge Characteristics of Heterogeneous Traffic at Signalized Intersections 
# AUTHORS: Maini and Khan 2000
# Signal Clearing speed
# MC 5.71 (smallest in table 5)
const MC_V0_pref = U.mps_to_upt(5.71)

# Jam distance 
const MC_S0_jam = MC_LENGTH + 10

#color
const MC_COLOR = :blue

end