def pixel_to_spatial(pixel_x ,pixel_y, pixel_to_meter_ratio, camera_offset):
    """
    Converts pixel coordinates to spatial coordinates of Hubert

    Parameters:
    - pixel_x: pixel x-coordinate to be converted to spatial y-coordinate
    - pixel_y: pixel y-coordinate to be converted to spatial z-coordinate
    - pixel_to_meter_ratio: pixel to spatial meter ratio from calibration
    - camera_offset: distance in meter from bottom of frame to reference z=0

    Returns:
    - spatial_x: spatial x-coordinate estimated as distance from Hubert to wall
    - spatial_y: spatial y-coordinate corresponding to pixel x-coordinate
    - spatial_z: spatial z-coordinate corresponding to pixel y-coordinate
    """

    pixel_x0 = 1280/2
    pixel_y0 = 720

    spatial_y = (pixel_x0 - pixel_x) * pixel_to_meter_ratio - 0.03
    spatial_z = (pixel_y0 - pixel_y) * pixel_to_meter_ratio + camera_offset

    spatial_x = 1.69 #distance from Hubert to wall

    return spatial_x, spatial_y, spatial_z