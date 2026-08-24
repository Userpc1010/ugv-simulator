TEMPLATE = subdirs

SUBDIRS = \
    Simulation \
    nav2_smac_planner \
    MPPI-Generic \
    Regulated-Pure-Pursuit \
    Vector-Pursuit-Controller

Simulation.depends += nav2_smac_planner MPPI-Generic Regulated-Pure-Pursuit Vector-Pursuit-Controller
Simulation.file = Simulation/Simulation.pro
