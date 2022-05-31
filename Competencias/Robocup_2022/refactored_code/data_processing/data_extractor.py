import numpy as np
import cv2 as cv

import utilities
from data_processing import camera_processing
from data_processing import point_cloud_processor

floor_color_ranges = {
            "normal":   
                {   
                    "range":   ((0, 0, 0), (255, 255, 255)), 
                    "threshold":0.5},

            "swamp":
                {
                    "range":((0, 0, 0), (255, 255, 255)),
                    "threshold":0.5},
            }
    
def get_square_color(square):
    color_counts = {}
    for row in square:
        for color in row:
            for key, value in floor_color_ranges.items():
                if utilities.is_color_in_range(color, value["range"]):
                    if key in color_counts:
                        color_counts[key] += 1
                    else:
                        color_counts[key] = 1
    
    for key, value in color_counts.items():
        if value < floor_color_ranges[key]["threshold"] * len(square) ** 2:
            del color_counts[key]
    
    if len(color_counts) == 0:
        return None
    else:
        return max(color_counts, key=color_counts.get)



def get_floor_colors(camera_images:dict, lidar_point_cloud, robot_rotation:float, robot_position:list):
    # Flatten, correct and join camera images
    camera_final_image = camera_processing.get_floor_image(camera_images, robot_rotation)

    #offsets = (int((robot.position[0] * 850) % 50), int((robot.position[1] * 850) % 50))
    #utilities.draw_grid(camera_final_image, 50, offsets)
    #cv.imshow("camera_final_image", camera_final_image)

    # Get point clouds
    point_cloud, out_of_bounds_point_cloud = lidar_point_cloud

    # Scale point clouds to camera image resolution
    camera_out_of_bounds_point_cloud = point_cloud_processor.processPointCloudForCamera(out_of_bounds_point_cloud, robotPos=robot_position)
    camera_point_cloud = point_cloud_processor.processPointCloudForCamera(point_cloud, robotPos=robot_position)

    # Join point clouds and delete duplicates
    total_camera_point_cloud = np.vstack((camera_point_cloud, camera_out_of_bounds_point_cloud))
    #total_camera_point_cloud = np.unique(total_camera_point_cloud)

    # Get lines from robot to point cloud
    seen_points = point_cloud_processor.get_intermediate_points(total_camera_point_cloud, (350, 350))
    
    # Draw the floor image where the floor is visible
    #final_image = np.zeros(camera_final_image.shape, np.uint8)
    final_image = np.zeros((700, 700, 3), np.uint8)
    utilities.draw_poses(final_image, seen_points, back_image=camera_final_image)

    # Draw the grid on the final image
    offsets = (int((robot_position[0] * 850) % 50), int((robot_position[1] * 850) % 50))

    utilities.draw_squares_where_not_zero(final_image, 50, offsets)
    utilities.draw_poses(final_image, camera_point_cloud, 255)
    utilities.draw_grid(final_image, 50, offsets)
    
    print("FINAL IMG SHAPE", final_image.shape)
    
    cv.imshow("final_image", final_image.astype(np.uint8))
    cv.waitKey(1)

    """
    squares = utilities.get_squares_where_not_zero(final_image, 50, offsets)
    for square in squares:
        print(get_square_color(square))

    return squares
    """