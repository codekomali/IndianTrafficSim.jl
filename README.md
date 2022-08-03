*Warning: This is an ongoing project. Keep visiting this site for more updates. The features listed below are wishlist and are being currently implemented. Upon first release, this warning will be removed. Meanwhile, If you plan to contribute, please contact author at code.komali@gmail.com*
# IndianTrafficSim 
Indian Traffic Simulator a traffic simulator built entirely in Julia. It is an effort to leverage the growing Julia ecosystem to support researchers working on Microscopic Traffic Simulation (Agent Based Traffic Simulation). It uses the following packages:
* [Agents](https://juliadynamics.github.io/Agents.jl/stable/) for the ABM Simulation. 
* [InteractiveDynamics](https://juliadynamics.github.io/InteractiveDynamics.jl/stable/) for ABM visualization, interaction and plotting.
* [Makie](https://makie.juliaplots.org/stable/) is used for some custom visualization and plotting.

![Typical Indian traffic junction](https://upload.wikimedia.org/wikipedia/commons/a/a6/Karol_Bagh%2C_2008_%2814%29.JPG)

Traffic in India and other developing nations are not the same as traffic in many Western countries. While IndianTrafficSim can be used for simulating traffic in western nations, it will include significant support for simulating Indian traffic including poor lane discipline, higher hetrogenity (two wheelers, tuk-tuks, etc.).

Models Supported by IndianTrafficSim
* Intelligent Driver Model (IDM) - Treiber, Martin; Hennecke, Ansgar; Helbing, Dirk (2000), "Congested traffic states in empirical observations and microscopic simulations", Physical Review E, 62 (2): 1805–182
* Minimizing Overall Braking Induced by Lane Changes (MOBIL) - Treiber, M., & Helbing, D. (2016). Mobil: General lane-changing model for car-following models. Disponıvel Acesso Dezembro. (in progress)

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://codekomali.github.io/IndianTrafficSim.jl/stable)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://codekomali.github.io/IndianTrafficSim.jl/dev)
[![Build Status](https://github.com/codekomali/IndianTrafficSim.jl/actions/workflows/CI.yml/badge.svg?branch=master)](https://github.com/codekomali/IndianTrafficSim.jl/actions/workflows/CI.yml?query=branch%3Amaster)
[![Coverage](https://codecov.io/gh/codekomali/IndianTrafficSim.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/codekomali/IndianTrafficSim.jl)
