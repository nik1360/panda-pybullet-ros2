
import numpy as np

# Panda Denavit-Hartenberg parameters according to modified 
# convention (Craig, "Introduction to Robotics: Mechanics 
# and Control (3rd Edition)") 
dh_parameters = [
    {"a":0, "d":0.333, "alpha":0, "theta":None, "details":"joint_1"},       # theta is the joint angle at a given instant
    {"a":0, "d":0, "alpha":-np.pi/2, "theta":None, "details":"joint_2"},
    {"a":0, "d":0.316, "alpha":np.pi/2, "theta":None, "details":"joint_3"},
    {"a":0.0825, "d":0, "alpha":np.pi/2, "theta":None, "details":"joint_4"},
    {"a":-0.0825, "d":0.384, "alpha":-np.pi/2, "theta":None, "details":"joint_5"},
    {"a":0, "d":0, "alpha":np.pi/2, "theta":None, "details":"joint_6"},
    {"a":0.088, "d":0, "alpha":np.pi/2, "theta":None, "details":"joint_7"},
    {"a":0, "d":0.107, "alpha":0, "theta":0, "details":"joint_7"}
]