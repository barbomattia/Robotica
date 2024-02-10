import time
import open3d as o3d
import cv2
import numpy as np
import os
# import detection as dc
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from scipy.spatial.transform import Rotation as R

classNames = [
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y3-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2",
    "X2-Y2-Z2-FILLET"
]

##
#  @brief Load the meshes of the blocks as pointclouds.
#  
#  This function loads the meshes of the blocks from the predefined folder and converts them to point clouds.
#  
#  @return A dictionary containing the point clouds of the meshes
#  
#  @note This is a hard-coded setup, hence the function will only work if the folder structure is as follows:
#  - models
#    - block_1
#      - model.config
#      - model.sdf
#      - mesh
#        - block_1.stl
#    - block_2
#      - model.config
#      - model.sdf
#      - mesh
#        - block_2.stl
#    - ...
#  
#  Example usage:
#  @code
#  mesh_folder = "./models/"
#  meshes = load_meshes(mesh_folder)
#  @endcode
#  
#  @see open3d.geometry.PointCloud
#  @see open3d.io.read_triangle_mesh
#  @see open3d.geometry.PointCloud.sample_points_uniformly
#  @see open3d.geometry.PointCloud.rotate
#  @see open3d.geometry.PointCloud.translate
#  @see open3d.geometry.PointCloud.get_center
#  @see open3d.geometry.PointCloud.points
#  @see open3d.utility.Vector3dVector
#  @see os.listdir
#  @see os.path
#  @see os.path.isdir
#  @see os.path.exists
#  @see os.path.join
def load_meshes(mesh_folder):
    meshes = {}
    # Loop through all the folders in the models folder
    for block_folder in os.listdir(mesh_folder):
        block_path = os.path.join(mesh_folder, block_folder)
        if os.path.isdir(block_path):
            config_path = os.path.join(block_path, "model.config")

            sdf_path = os.path.join(block_path, "model.sdf")
            mesh_folder_path = os.path.join(block_path, "mesh")

            if os.path.exists(config_path) and os.path.exists(sdf_path) and os.path.exists(mesh_folder_path):
                stl_file = [file for file in os.listdir(mesh_folder_path) if file.endswith(".stl")]
                if stl_file:
                    stl_path = os.path.join(mesh_folder_path, stl_file[0])
                    mesh = o3d.io.read_triangle_mesh(stl_path)
                    # Transform the mesh to a point cloud
                    mesh = mesh.sample_points_uniformly(number_of_points=3000)
                    # Create a rotation of -60 degrees around the x-axis
                    rotation_matrix = np.array([[1, 0, 0], [0, np.cos(np.radians(-60)), -np.sin(np.radians(-60))], [0, np.sin(np.radians(-60)), np.cos(np.radians(-60))]])
                    # Apply the rotation to the mesh
                    mesh.rotate(rotation_matrix, center=(0, 0, 0))

                    # Calculate the centroid
                    centroid = mesh.get_center()
                    #print("Mesh centroid:", centroid)
                    # Move the mesh to the origin
                    mesh.translate(-centroid)

                    meshes[block_folder] = mesh
                else:
                    print(f"Warning: No STL file found in the mesh folder for block {block_folder}")
            else:
                print(f"Warning: Incomplete files for block {block_folder}")

    return meshes

##
#  @brief Read the YOLO detections from a file. 
#
#  This function reads the YOLO detections from a file and returns them as a list of strings.
#
#  @param file_path The path to the file containing the YOLO detections
#
#  @return A list of strings containing the YOLO detections
#
#  @note The file should contain the YOLO detections in the following format:
#  @code
#  block_1: (x, y, w, h)
#  block_2: (x, y, w, h)
#  ...
#  @endcode
#
#  Example usage:
#  @code
#  detections_file_path = "./detections.txt"
#  yolo_detections = read_yolo_detections(detections_file_path)
#  @endcode
def read_yolo_detections(file_path):
    # Create an empty list to store the detections
    yolo_detections = []
    # Open the file containing the detections
    with open(file_path, "r") as file:
        # Iterate over each line
        for line in file:
            # check if the line is empty, if not, add it to the list as a string
            if line.strip():
                yolo_detections.append(line.strip())
    # Print all the the detections
    print(yolo_detections)
    return yolo_detections

##
#  @brief Scale a point cloud by a given factor.
#
#  This function scales a point cloud by a given factor and returns the scaled point cloud.
#
#  @param point_cloud The point cloud to be scaled
#  @param scale_factor The factor by which the point cloud should be scaled
#
#  @return The scaled point cloud
#
#  @note The scaling factor should be a positive float value.
#
#  Example usage:
#  @code
#  scaled_point_cloud = scale_point_cloud(point_cloud, 0.5)
#  @endcode
#
#  @see open3d.geometry.PointCloud
#  @see open3d.utility.Vector3dVector
def scale_point_cloud(point_cloud, scale_factor):
    # Get the current points from the point cloud
    points = np.asarray(point_cloud.points)
    
    # Apply the scaling factor to the points
    scaled_points = points * scale_factor
    
    # Create a new point cloud with the scaled points
    scaled_point_cloud = o3d.geometry.PointCloud()
    scaled_point_cloud.points = o3d.utility.Vector3dVector(scaled_points)
    
    return scaled_point_cloud

##
#  @brief Rotate a point cloud by 180 degrees around the x-axis and flip it horizontally.
#
#  This function rotates a point cloud by 180 degrees around the x-axis and flips it horizontally.
#
#  @param point_cloud The point cloud to be rotated
#
#  @return The rotated point cloud
#
#  Example usage:
#  @code
#  rotated_point_cloud = rotate_point_cloud(point_cloud)
#  @endcode
#
#  @see open3d.geometry.PointCloud
#  @see open3d.utility.Vector3dVector
def rotate_point_cloud(point_cloud):
    # Create a rotation matrix for a 180-degree rotation around the x-axis
    rotation_matrixA = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    # Create a rotation to flip the image horizontally
    rotation_matrixB = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # Apply the rotation to the point cloud by creating a new point cloud
    rotated_points = np.dot(np.asarray(point_cloud.points), rotation_matrixA.T)
    rotated_points = np.dot(np.asarray(rotated_points), rotation_matrixB.T)

    rotated_point_cloud = o3d.geometry.PointCloud()
    rotated_point_cloud.points = o3d.utility.Vector3dVector(rotated_points)

    return rotated_point_cloud

##
#  @brief Create a depth image from a point cloud.
#
#  This function projects a point cloud onto a depth image using the camera intrinsics.
#
#  @param point_cloud The point cloud to be projected
#  @param intrinsics The camera intrinsics
#
#  @return The depth image
#
#  Example usage:
#  @code
#  depth_image = create_depth_image_from_point_cloud(point_cloud, intrinsics)
#  @endcode
#
#  @see open3d.camera.PinholeCameraIntrinsic
#  @see open3d.geometry.PointCloud
#  @see open3d.utility.Vector3dVector
def create_depth_image_from_point_cloud(point_cloud, intrinsics):
    # Initialize an empty depth image
    depth_image = np.zeros((intrinsics.height, intrinsics.width))

    # Get the intrinsic parameters from the intrinsic_matrix
    fx = intrinsics.intrinsic_matrix[0, 0]
    fy = intrinsics.intrinsic_matrix[1, 1]
    cx = intrinsics.intrinsic_matrix[0, 2]
    cy = intrinsics.intrinsic_matrix[1, 2]

    # Project each point in the point cloud onto the image plane
    for point in np.asarray(point_cloud.points):
        # Apply the camera intrinsics
        x = int((point[0] * fx / point[2]) + cx)
        y = int((point[1] * fy / point[2]) + cy)

        # Check if the projected point falls within the image bounds
        if 0 <= x < intrinsics.width and 0 <= y < intrinsics.height:   
            depth_image[y, x] = point[2]
    # Visualize the depth image
    #visualize_depth_frame(depth_image)

    return depth_image

##
#  @brief Crop the depth image to the bounding box of each block.
#
#  This function crops the depth image to the bounding box of each block detected by YOLO.
#  It also prints the center of each block in the pointcloud coordinates.
#  It also converts the cropped depth image to a point cloud by calling the depth_image_to_point_cloud function.
#
#  @param yolo_detections The YOLO detections
#  @param depth_image The depth image
#  @param intrinsics The camera intrinsics
#
#  @return A list of tuples containing the block identifier, the cropped depth image, and the cropped point cloud
#
#  Example usage:
#  @code
#  blocks = crop_depth_image(yolo_detections, depth_image, intrinsics)
#  @endcode
#
#  @see open3d.camera.PinholeCameraIntrinsic
#  @see open3d.geometry.PointCloud
#  @see open3d.utility.Vector3dVector
def crop_depth_image(yolo_detections, depth_image, intrinsics):

    #create a list of tuples to store the cropped depth images together with the block identifier
    blocks = []
    
    # Define the transformation matrix
    rotation_matrixA = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    rotation_matrixB = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
    rotation = np.dot(rotation_matrixA, rotation_matrixB).T
    
    # Given orientation quaternion
    x, y, z, w = 0.612267, -0.612267, 0.353737, -0.353737

    # Calculate elements of the rotation matrix
    r11 = 1 - 2 * y**2 - 2 * z**2
    r12 = 2 * x * y - 2 * z * w
    r13 = 2 * x * z + 2 * y * w
    r21 = 2 * x * y + 2 * z * w
    r22 = 1 - 2 * x**2 - 2 * z**2
    r23 = 2 * y * z - 2 * x * w
    r31 = 2 * x * z - 2 * y * w
    r32 = 2 * y * z + 2 * x * w
    r33 = 1 - 2 * x**2 - 2 * y**2
    
    # Create the rotation matrix
    world_to_camera = np.array([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ])
    
    # Iterate over all blocks detected by YOLO
    for line in yolo_detections:
        # Remove the last char from the line
        line = line[:-1]
        # Divide split the line at ":", take the first element as "identifier" and the second element as "bbox"
        block_identifier, bboxS = line.split(": (")
        # Initialize an array of 4 ints to store the bbox
        bbox = np.zeros(4)
        # fill bbox with the values from splitting bboxS at ","
        bbox[0], bbox[1], bbox[2], bbox[3] = bboxS.split(", ")

        # Extract bounding box coordinates
        xmin, ymin, width, height = bbox
        xmax = xmin + width
        ymax = ymin + height
        
        # crop the depth image to the bounding box
        cropped_depth_image = depth_image[int(ymin):int(ymax), int(xmin):int(xmax)]

        # Transform the cropped depth image to a point cloud
        cropped_point_cloud = depth_image_to_point_cloud(cropped_depth_image)

        # Visualize the cropped point cloud
        # visualize(cropped_point_cloud)

### center
        fx = intrinsics.intrinsic_matrix[0, 0]
        fy = intrinsics.intrinsic_matrix[1, 1]
        cx = intrinsics.intrinsic_matrix[0, 2]
        cy = intrinsics.intrinsic_matrix[1, 2]
        # Get the center of the cropped depth image
        norm_X = (int(xmin) + int(xmax)) / 2
        norm_Y = (int(ymin) + int(ymax)) / 2

        # Get depth value of the center of the cropped depth image
        depth_value = depth_image[int(norm_Y), int(norm_X)]

        pc_z = depth_value
        pc_x =  (norm_X - cx) * pc_z / fx
        pc_y = -(norm_Y - cy) * pc_z / fy

        print("Center:", pc_x, pc_y, pc_z-0.023)

        pc = np.array([pc_x, -pc_y, pc_z - 0.023])
        
        pc = np.dot(rotation, pc)
        
        pc = np.dot(world_to_camera, pc)
        pc = pc - np.array([0.4, -0.59, -1.4])
        
        print(pc)
        # Append the cropped depth image together with the block identifier to the list
        blocks.append((block_identifier, [pc[0], pc[1], pc[2]], cropped_depth_image, cropped_point_cloud))      
    
    return blocks

##
#  @brief Convert a depth image to a point cloud.
#
#  This function converts a depth image to a point cloud by projecting its depth value into a new space. 
#  This is achieved by multiplying the depth value by a constant factor, so that the pixels with depth value close to 0 will remain o a plane, while the other will "emerge".
#
#  @param depth_frame The depth image
#
#  @return The point cloud
#
#  Example usage:
#  @code
#  point_cloud = depth_image_to_point_cloud(depth_frame)
#  @endcode
#
#  @see open3d.geometry.PointCloud
#  @see open3d.utility.Vector3dVector
def depth_image_to_point_cloud(depth_frame):
    # Generate 3D coordinates without camera intrinsics
    rows, cols = depth_frame.shape
    u, v = np.meshgrid(np.arange(cols), np.arange(rows))
    x = u
    y = -v  # Invert Y-axis
    z = depth_frame*1100

    #Print min and max depth values
    #print("Min depth:", np.min(z))
    #print("Max depth:", np.max(z))

    # Create point cloud
    points = np.stack([x, y, z], axis=-1)
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(points.reshape(-1, 3))

    #print("Min distance between points:", min_distance_between_points(pointcloud))

    # Remove points in the point cloud corresponding to the blue color
    point_cloud = remove_blue_points_from_point_cloud(pointcloud)
    pointcloud = point_cloud

    xyz = np.asarray(pointcloud.points)

    # DBSCAN Clustering
    clustering = DBSCAN(eps=10.0, min_samples=8)
    labels = clustering.fit_predict(xyz)

    # Check if any clusters were found
    if len(labels[labels>=0]) > 0:
        # Get the largest cluster
        counts = np.bincount(labels[labels>=0])
        largest_cluster_label = np.argmax(counts)
        xyz_filtered = xyz[labels == largest_cluster_label]
    else:
        print("No clusters found.")
        xyz_filtered = xyz  # or however you want to handle this case

    # Create a new point cloud based on the filtered results
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(xyz_filtered)

    pointcloud = pcd_filtered
    # Calculate the centroid
    centroid = pointcloud.get_center()
    #print("PC centroid:", centroid)

    # Visualize the point cloud and the coordinate frame
    #o3d.visualization.draw_geometries([pointcloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])])

    # Move the point cloud to the origin
    pointcloud.translate(-centroid)

    return pointcloud

##
#  @brief Remove points from a point cloud based on a custom criterion.
#
#  This function removes points from a point cloud based on a custom criterion.
#  In this example, we remove points with a normalized depth value less than 155.
#
#  @param point_cloud The point cloud
#
#  @return The cropped point cloud
#
#  Example usage:
#  @code
#  cropped_point_cloud = remove_blue_points_from_point_cloud(point_cloud)
#  @endcode
#
#  @see open3d.geometry.PointCloud
#  @see open3d.utility.Vector3dVector
def remove_blue_points_from_point_cloud(point_cloud):
    # Convert Open3D PointCloud to a NumPy array
    point_cloud_array = np.asarray(point_cloud.points)
    
    # Extract depth values from the third column
    depth_values = point_cloud_array[:, 2]
    
    # Normalize the depth values to the range [0, 255]
    normalized_depth = np.interp(depth_values, (depth_values.min(), depth_values.max()), (0, 255))
    
    # Convert normalized depth to an 8-bit unsigned integer (uint8) array
    normalized_depth_uint8 = normalized_depth.astype(np.uint8)
    
    # Use normalized_depth_uint8 for further processing or visualization
    
    # If needed, you can apply a custom criterion to remove points based on the normalized_depth_uint8 array
    # For example, keep points where normalized depth is within a specific range
    
    # Example: Keep points with normalized depth bigger than 155
    valid_points_mask = normalized_depth_uint8 > 105
    
    # Apply the valid points mask to the original point cloud
    cropped_point_cloud = point_cloud.select_by_index(np.where(valid_points_mask)[0])
    
    return cropped_point_cloud

##
#  @brief Visualize a depth image.
#
#  This function visualizes a depth image using a color map.
#  It is used mainly for debugging purposes.
#
#  @param depth_image The depth image
#
#  Example usage:
#  @code
#  visualize_depth_frame(depth_image)
#  @endcode
#
#  @see cv2.normalize
#  @see cv2.applyColorMap
#  @see cv2.imshow
def visualize_depth_frame(depth_image):
    # Normalize the depth image
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # Apply a color map
    depth_image_color = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
    # Show the image
    cv2.imshow("Depth Image", depth_image_color)
    cv2.waitKey(0)

##
#  @brief Visualize a point cloud.
#
#  This function visualizes a point cloud with the coordinate frame.
#  It is used mainly for debugging purposes.
#
#  @param point_cloud The point cloud
#
#  Example usage:
#  @code
#  visualize(point_cloud)
#  @endcode
#
#  @see open3d.visualization.draw_geometries
#  @see open3d.geometry.TriangleMesh.create_coordinate_frame
def visualize(element):
    # Get centroid of the point cloud
    centroid = element.get_center()

    # Translate the point cloud to the origin
    element.translate(-centroid)

    # Visualize the point cloud with the coordinate frame
    o3d.visualization.draw_geometries([element])

##
#  @brief Computes minimum distance between points in a point cloud.
#
#  This function computes the minimum distance between points in a point cloud.
#
#  @param point_cloud The point cloud
#
#  @return The minimum distance between points
#
#  Example usage:
#  @code
#  min_distance = min_distance_between_points(point_cloud)
#  @endcode
#
#  @see numpy.linalg.norm
def min_distance_between_points(point_cloud):
    # Get the array of points from the point cloud
    points = np.asarray(point_cloud.points)

    # Initialize minimum distance
    min_distance = float('inf')

    # Iterate over all pairs of points
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            # Calculate Euclidean distance between points i and j
            distance = np.linalg.norm(points[i] - points[j])

            # Update minimum distance if needed
            min_distance = min(min_distance, distance)

    return min_distance

##
#  @brief Check if a point cloud is a plane.
#
#  This function checks if a point cloud is a plane by fitting a plane to the point cloud using RANSAC.
#  It is used to filter out some possible false positives.
#
#  @param point_cloud The point cloud
#
#  @return True if the point cloud is a plane, False otherwise
#
#  Example usage:
#  @code
#  is_plane = is_plane(point_cloud)
#  @endcode
#
#  @see numpy.asarray
#  @see sklearn.linear_model.RANSACRegressor
#  @see numpy.sum
def is_plane(point_cloud):
    # Get the array of points from the point cloud
    points = np.asarray(point_cloud.points)

    # Apply RANSAC to fit a plane to the point cloud
    ransac = RANSACRegressor()
    ransac.fit(points[:, :2], points[:, 2])

    # Get the inlier mask
    inlier_mask = ransac.inlier_mask_

    # Get the number of inliers
    num_inliers = np.sum(inlier_mask)

    # If all the points are inliers, the point cloud is a plane
    if num_inliers == (len(points))*0.85:
        return True
    else:
        return False


def find_best(meshes, blocks):
    # classNames = dc.classNames
    detected = []
    criticals = []
    final = []
    
    for block_identifier, center, _, cropped_point_cloud in blocks:
        classes = [classType for classType in classNames if classType.startswith(block_identifier.split("Z", 1)[0] + "Z")]
        results = []
        
        for block in classes:
            if block not in detected:
                block_mesh = meshes[block]
                results.append((block, compute_pose(block_mesh, cropped_point_cloud)))
                
        if not results: #empty
            criticals.append((center, cropped_point_cloud))
        else:
            min_error = min(res[1][0] for res in results)
            for block, result in results:
                if min_error == result[0]:
                    detected.append(block) # yay we know the class
                    rpy_angles = R.from_matrix(result[1][:3, :3]).as_euler('xyz', True)
                    rpy_angles[:-1] = [0, 0]
                    final.append((block, center, rpy_angles))
                    break
    
    remaining = list(set(classNames) - set(detected))
    for critical in criticals:
        results = []
        for block in remaining:
            if block not in detected:
                block_mesh = meshes[block]
                results.append((block, compute_pose(block_mesh, critical[1])))
        
        min_error = min(res[1][0] for res in results)
        for block, result in results:
            if min_error == result[0]:
                detected.append(block)
                rpy_angles = R.from_matrix(result[1][:3, :3]).as_euler('xyz', True)
                rpy_angles[:-1] = [0, 0]
                final.append((block, critical[0], rpy_angles))
                break
                
    for l in final:
        print(l[-1])
        
    return final
    #print(list(set(classNames) - set(detected)))
    #print(f'Number of criticals: {len(critical)}')
    #critical[0][1].paint_uniform_color([0, 0, 1])
    #o3d.visualization.draw_geometries([critical[0][1], o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])])

        

##
#  @brief Compute the pose of the blocks.
#
#  This function computes the pose of the blocks by performing a three step iterative ICP between the point cloud of the block and the mesh of the block, where each iteration is performed with a different pre-rotation around the z-axis.
#  It also performs some pre-processing steps such as scaling and filtering the point cloud.
#
#  @param meshes A dictionary containing the point clouds of the meshes
#  @param blocks A list of tuples containing the block identifier, the cropped depth image, and the cropped point cloud
#
#  Example usage:
#  @code
#  compute_pose(meshes, blocks)
#  @endcode
#
#  @see open3d.geometry.PointCloud
#  @see open3d.geometry.PointCloud.rotate
#  @see open3d.geometry.PointCloud.translate
#  @see open3d.geometry.PointCloud.get_center
#  @see open3d.geometry.PointCloud.points
#  @see open3d.utility.Vector3dVector
def compute_pose(block_mesh, cropped_point_cloud): 

    if(is_plane(cropped_point_cloud)):
        print("Plane detected")
        return
        
    # Create a rotation matrix RA around the x-axis
    angle = 60
    RA = np.array([[1, 0, 0], [0, np.cos(np.radians(angle)), -np.sin(np.radians(angle))], [0, np.sin(np.radians(angle)), np.cos(np.radians(angle))]])

    # Apply the rotation to the point cloud and the block mesh
    cropped_point_cloud.rotate(RA, center=(0, 0, 0))
    block_mesh.rotate(RA, center=(0, 0, 0))

    # Get min z value
    min_z = np.min(np.asarray(cropped_point_cloud.points)[:, 2])

    #print("Min z:", min_z)

    # Remove the lowest layer of the point cloud
    points = np.asarray(cropped_point_cloud.points)
    mask = points[:, 2] > min_z + 4
    cropped_points = points[mask]
    cropped_point_cloud = o3d.geometry.PointCloud()
    cropped_point_cloud.points = o3d.utility.Vector3dVector(cropped_points)
    
    points = np.asarray(cropped_point_cloud.points)
    # DBSCAN Clustering
    clustering = DBSCAN(eps=10.0, min_samples=8)
    labels = clustering.fit_predict(points)

    # Check if any clusters were found
    if len(labels[labels>=0]) > 0:
        # Get the largest cluster
        counts = np.bincount(labels[labels>=0])
        largest_cluster_label = np.argmax(counts)
        xyz_filtered = points[labels == largest_cluster_label]
    else:
        print("No clusters found.")
        xyz_filtered = points  # or however you want to handle this case

    # Create a new point cloud based on the filtered results
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(xyz_filtered)

    cropped_point_cloud = pcd_filtered

    #o3d.visualization.draw_geometries([cropped_point_cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])])

## rescaling
    # Move the block mesh and the point cloud to the origin
    block_mesh.translate(-block_mesh.get_center())
    cropped_point_cloud.translate(-cropped_point_cloud.get_center())

    # Translate the pointcloud up in order to align the bottom to z=0
    pc_min_z = np.min(np.asarray(cropped_point_cloud.points)[:, 2])
    cropped_point_cloud.translate([0, 0, -pc_min_z])

    # Translate the block mesh up in order to align the bottom to z=0
    m_min_z = np.min(np.asarray(block_mesh.points)[:, 2])
    block_mesh.translate([0, 0, -m_min_z])

    # Get the max z values to calculate the scaling factor
    pc_max_z = np.max(np.asarray(cropped_point_cloud.points)[:, 2])
    pc_min_z = np.min(np.asarray(cropped_point_cloud.points)[:, 2])
    mesh_max_z = np.max(np.asarray(block_mesh.points)[:, 2])
    mesh_min_z = np.min(np.asarray(block_mesh.points)[:, 2])

    #print("PC-Min z:", pc_min_z, "PC-Max z:", pc_max_z)
    #print("M-Min z:", mesh_min_z, "M-Max z:", mesh_max_z)

    pc_dimension = pc_max_z - pc_min_z
    mesh_dimension = mesh_max_z - mesh_min_z

    #print("PC dimension:", pc_dimension)
    #print("Mesh dimension:", mesh_dimension)

    # Calculate the scaling factor for the block mesh
    scale_factor = pc_dimension / mesh_dimension

    #print("Scale factor:", scale_factor)
    if scale_factor < 970:
        scale_factor = 950
    if scale_factor > 1100:
        scale_factor = 1100

    # Scale the block mesh
    scaled_block_mesh = scale_point_cloud(block_mesh, scale_factor)

    # Move the scaled block mesh and the point cloud to the origin again
    scaled_block_mesh.translate(-scaled_block_mesh.get_center())
    cropped_point_cloud.translate(-cropped_point_cloud.get_center())

    # Get the new min and max z values of the scaled mesh and the point cloud
    min_z = np.min(np.asarray(cropped_point_cloud.points)[:, 2])
    max_z = np.max(np.asarray(cropped_point_cloud.points)[:, 2])

    #print("PC-Min z:", min_z, "PC-Max z:", max_z)

    m_min_z = np.min(np.asarray(scaled_block_mesh.points)[:, 2])
    m_max_z = np.max(np.asarray(scaled_block_mesh.points)[:, 2])

    #print("M-Min z:", m_min_z, "M-Max z:", m_max_z)

## rescaling

    # Perform ICP to get the transformation
    return multi_start_pose_detection(scaled_block_mesh, cropped_point_cloud, rotations=3)
        
    # Visualize the cropped point cloud and the scaled block mesh, also add the coordinate system axis
    #o3d.visualization.draw_geometries([scaled_block_mesh, cropped_point_cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])])

    # Transform the block mesh
    #transformed_block_mesh = scaled_block_mesh.transform(transformation)

    # Paint the transformed block mesh yellow
    #transformed_block_mesh.paint_uniform_color([1, 1, 0])  # RGB values range from 0 to 1

    # Paint the cropped point cloud blue
    #cropped_point_cloud.paint_uniform_color([0, 0, 1])

    # Visualize the point clouds and a coordinate frame
    #o3d.visualization.draw_geometries([transformed_block_mesh, cropped_point_cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])])


##
#  @brief Perform multi-start pose detection using ICP.
#
#  This function performs multi-start pose detection using ICP to align a mesh with a point cloud.
#  It rotates the mesh around the z-axis and performs ICP for each rotation.
#  It prints the best transformation and the corresponding error.
#
#  @param mesh The mesh
#  @param point_cloud The point cloud
#  @param rotations The number of rotations to perform
#
#  @return The best transformation
#
#  Example usage:
#  @code
#  transformation = multi_start_pose_detection(mesh, point_cloud, rotations=3)
#  @endcode
#
#  @see open3d.geometry.PointCloud
#  @see open3d.geometry.PointCloud.get_rotation_matrix_from_xyz
#  @see open3d.geometry.PointCloud.estimate_normals
#  @see open3d.geometry.PointCloud.rotate
#  @see open3d.geometry.PointCloud.transform
#  @see open3d.pipelines.registration.registration_icp
#  @see open3d.pipelines.registration.TransformationEstimationPointToPlane
#  @see open3d.pipelines.registration.ICPConvergenceCriteria
#  @see numpy.eye
#  @see numpy.dot
#  @see numpy.linalg.inv
#  @see time.sleep
def multi_start_pose_detection(mesh, point_cloud, rotations=3):
    rotation_angle = 0
    rotation_matrix = mesh.get_rotation_matrix_from_xyz((0, 0, 0))
    rotation_matrix_4x4 = np.eye(4)
    best_transformation = None
    best_error = float('10')
    best_iteration = -1

    # Estimate normals for the target point cloud
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.4, max_nn=50))
    # Estimate normals for the rotated mesh
    mesh.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.1, max_nn=50))

    for i in range(rotations):        
        # Rotate the mesh
        rotation_angle = (np.pi) * i * 0.5
        rotation_matrix = mesh.get_rotation_matrix_from_xyz((0, 0, rotation_angle))

        # Visualize the two point clouds and the coordinate frame and close the window after 1 second
        #o3d.visualization.draw_geometries([mesh, point_cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])], window_name="Open3D", width=800, height=600, left=50, top=50, point_show_normal=False)    

        # Create a new pointcloud to copy the rotated mesh
        C_mesh = o3d.geometry.PointCloud()
        C_mesh.points = mesh.points
        C_mesh.normals = mesh.normals
        C_mesh.rotate(rotation_matrix, center=mesh.get_center())

        # Convert the 3x3 rotation matrix to a 4x4 matrix
        rotation_matrix_4x4[:3, :3] = rotation_matrix

        # Perform Point-to-Plane ICP
        icp_result = o3d.pipelines.registration.registration_icp(
            C_mesh, point_cloud, 6, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
        )

        #print("Rotation:", rotation_angle, "Error:", icp_result.inlier_rmse)


        # Visualize the result 
        # transform the copy of the mesh with the transformation
        C_mesh.transform(icp_result.transformation)
        #o3d.visualization.draw_geometries([C_mesh, point_cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])], window_name="Open3D", width=800, height=600, left=50, top=50, point_show_normal=False)
        # Add a small delay after visualization
        #time.sleep(0.6)
        # If this alignment is better than the previous best, update the best transformation and error
        if icp_result.inlier_rmse < best_error:
            best_error = icp_result.inlier_rmse
            best_transformation = np.dot(icp_result.transformation, rotation_matrix_4x4)
            best_iteration = i
            
        # Reverse the transformation
        C_mesh.transform(np.linalg.inv(icp_result.transformation))
        # Reverse the initial rotation
        C_mesh.rotate(rotation_matrix.T, center=mesh.get_center())
        # Visualize the result
        #visualize(C_mesh)
        #print("cicle ", i, " done")

    print("Best error:", best_error)
    print("Best transformation:", best_transformation)
    #print("Best iteration:", best_iteration)
    
    return (best_error, best_transformation)



if __name__ == "__main__":

    # Get camera intrinsic parameters
    fx = 790.94585984335442  # focal length in x
    fy = 790.94585984335442  # focal length in y
    cx = 960                 # principal point in x
    cy = 540                 # principal point in y
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width=1920, height=1080, fx=fx, fy=fy, cx=cx, cy=cy)

    # Load the files scene point cloud
    point_cloud = o3d.io.read_point_cloud("./mostro.ply")
    detections_file_path = "./detections.txt"
    mesh_folder = "./models/"

    # Rotate the point cloud to align it with the image frame
    point_cloud = rotate_point_cloud(point_cloud)

    # Project 3D points to depth image within the specified range
    depth_image = create_depth_image_from_point_cloud(point_cloud, intrinsics)
    
    # Load meshes for each block
    meshes = load_meshes(mesh_folder)

    # Load YOLO detections from detections.txt
    yolo_detections = read_yolo_detections(detections_file_path)

    # Get the final blocks' info triplets
    blocks = crop_depth_image(yolo_detections, depth_image, intrinsics)

    # Perform ICP to get the pose of the block respectivelly to the mesh of the first block
    #compute_pose(meshes, blocks)
    
    results = find_best(meshes, blocks)
    
    i = 0
    with open('centers.txt', 'w') as file:
        for result in results:
            file.write(f'{i}: {result[1]}\n')
            i += 1
