def pixel_to_spatial(pixel_x ,pixel_y):
    """
    Converts pixel coordinates from a 640x480 webcam to spatial coordinates of Hubert

    Parameters:
    - pixel_x: pixel x-coordinate to be converted to spatial y-coordinate
    - pixel_y: pixel y-coordinate to be converted to spatial z-coordinate

    Returns:
    - spatial_x: spatial x-coordinate estimated as distance from Hubert to wall
    - spatial_y: spatial y-coordinate corresponding to pixel x-coordinate
    - spatial_z: spatial z-coordinate corresponding to pixel y-coordinate
    """

    pixel_to_meter_ratio = 0.0011228070175

    #origin of spatial_y is estimated as pixel_x = 320px
    pixel_x0 = 640
    #origin of spatial_z is estimated as pixel_y = 808.64 (pixel y coordinates is inverted)
    pixel_y0 = 360

    spatial_y = (pixel_x-pixel_x0) * pixel_to_meter_ratio
    spatial_z = (pixel_y0-pixel_y) * pixel_to_meter_ratio

    spatial_x = 1.69 #distance from Hubert to wall

    return spatial_x, spatial_y, spatial_z