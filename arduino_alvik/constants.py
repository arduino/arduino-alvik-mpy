"""
HW0: Alvik carrier board with APDS9960
HW1: Alvik carrier board with APDS9999
"""
USES_HARDWARE = "HW0"  # Change to "HW1" for hardware version 1


# COLOR SENSOR

if USES_HARDWARE == "HW0":
    COLOR_FULL_SCALE = 4097
    WHITE_CAL = [450, 500, 510]
    BLACK_CAL = [160, 200, 190]

    min_saturation = 0.1
    black_value = 0.05
    grey_value = 0.15
    light_grey_value = 0.8
    min_color_value = 0.1

    hsv_limits = {"thresholds": [20, 90, 140, 170, 210, 250, 280],
                      "high_h_v_thre": [0.5, 0.77],
                      "high_h_s_thre": [0.45]}

elif USES_HARDWARE == "HW1":
    COLOR_FULL_SCALE = 4097
    WHITE_CAL = [726, 1478, 775]
    BLACK_CAL = [385, 809, 411]

    min_saturation = 0.1
    black_value = 0.05
    grey_value = 0.15
    light_grey_value = 0.17
    min_color_value = 0.1

    hsv_limits = {"thresholds": [46, 46, 129, 150, 220, 235, 257],
                      "high_h_v_thre": [0.27, 0.83],
                      "high_h_s_thre": [0.64]}
