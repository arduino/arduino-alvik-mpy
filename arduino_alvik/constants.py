"""
HW0: Alvik carrier board with APDS9960
HW1: Alvik carrier board with APDS9999

HW_REVISION_1_3  0       // apds9960 1.3 - 1.5
HW_REVISION_1_6  1       // apds9999 1.6

"""

HW_REVISION_1_3 = HW0 = 0
HW_REVISION_1_6 = HW1 = 1

# *** HW_REVISION_1_3 DEFINITIONS
# COLOR SENSOR
hw0_color_thresholds = {
    "COLOR_FULL_SCALE": 4097,
    "WHITE_CAL": [450, 500, 510],
    "BLACK_CAL": [160, 200, 190],

    "MIN_SATURATION": 0.1,
    "BLACK_VALUE": 0.05,
    "GREY_VALUE": 0.15,
    "LIGHT_GREY_VALUE": 0.8,
    "MIN_COLOR_VALUE": 0.1,

    "HSV_LIMITS": {"thresholds": [20, 90, 140, 170, 210, 250, 280],
                       "high_h_v_thre": [0.5, 0.77],
                       "high_h_s_thre": [0.45]}
}

# *** HW_REVISION_1_6 DEFINITIONS
# COLOR SENSOR
hw1_color_thresholds = {
    "COLOR_FULL_SCALE": 4097,
    "WHITE_CAL": [726, 1478, 775],
    "BLACK_CAL": [385, 809, 411],

    "MIN_SATURATION": 0.1,
    "BLACK_VALUE": 0.05,
    "GREY_VALUE": 0.15,
    "LIGHT_GREY_VALUE": 0.17,
    "MIN_COLOR_VALUE": 0.1,

    "HSV_LIMITS": {"thresholds": [46, 46, 129, 150, 220, 235, 257],
                       "high_h_v_thre": [0.27, 0.83],
                       "high_h_s_thre": [0.64]}
}
