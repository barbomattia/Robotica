import os
import xml.etree.ElementTree as ET
import random

def save_sdf_to_file(tree, filename):
    # Ensure the directory exists, create it if not
    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)

    # Get the root element of the XML tree
    root = tree.getroot()

    # Write the formatted XML to the file with indentation
    with open(os.path.abspath(filename), "wb") as file:
        tree_str = ET.tostring(root, encoding="unicode", method="xml")
        file.write(tree_str.encode("utf-8"))
    # Ensure the directory exists, create it if not
    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)

    # Get the root element of the XML tree
    root = tree.getroot()

    # Call the indent function to add newlines and indentation
    indent(root)

    # Write the formatted XML to the file
    with open(os.path.abspath(filename), "w") as file:
        file.write(minidom.parseString(ET.tostring(root)).toprettyxml(indent="  "))

def generate_random_pose(x_range, y_range, z_value):
    x = random.uniform(x_range[0], x_range[1])
    y = random.uniform(y_range[0], y_range[1])
    z = z_value
    # Random rotation about the Z-axis
    yaw = random.uniform(0, 2 * 3.14159)
    return f"{x} {y} {z} 0 0 {yaw}"

def generate_world_sdf():
    world_sdf = ET.Element("sdf", version="1.4")
    world = ET.SubElement(world_sdf, "world", name="default")

    # Add physics parameters
    physics = ET.SubElement(world, "physics", type="ode")
    gravity = ET.SubElement(physics, "gravity")
    gravity.text = "0 0 -9.81"
    max_step_size = ET.SubElement(physics, "max_step_size")
    max_step_size.text = "0.001"
    real_time_factor = ET.SubElement(physics, "real_time_factor")
    real_time_factor.text = "1"

    # Include sun and ground_plane
    sun_include = ET.SubElement(world, "include")
    sun_uri = ET.SubElement(sun_include, "uri")
    sun_uri.text = "model://sun"

    ground_plane_include = ET.SubElement(world, "include")
    ground_plane_uri = ET.SubElement(ground_plane_include, "uri")
    ground_plane_uri.text = "model://ground_plane"

    # Include the desk model
    tavolo_include = ET.SubElement(world, "include")
    tavolo_name = ET.SubElement(tavolo_include, "name")
    tavolo_name.text = "tavolo"
    tavolo_uri = ET.SubElement(tavolo_include, "uri")
    tavolo_uri.text = "model://tavolo"
    tavolo_pose = ET.SubElement(tavolo_include, "pose")
    tavolo_pose.text = "0.0 0.0 0.0 0 0 0"

    # Generate random poses for 11 blocks
    block_classes = [
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
        "X2-Y2-Z2-FILLET",
    ]

    x_range = [0.25, 0.85]
    y_range = [0.25, 0.75]
    z_value = 1.05  # Assuming a common z value for all blocks

    # Generate random poses for 11 blocks with random rotation about Z-axis
    for block_class in block_classes:
        block_include = ET.SubElement(world, "include")
        block_name = ET.SubElement(block_include, "name")
        block_name.text = block_class
        block_uri = ET.SubElement(block_include, "uri")
        block_uri.text = f"model://{block_class}"
        block_pose = ET.SubElement(block_include, "pose")

        # Generate a random pose with rotation about Z-axis for each block, ensuring they don't overlap
        overlap = True
        while overlap:
            block_pose.text = generate_random_pose(x_range, y_range, z_value)
            overlap = any(
                (
                    abs(float(other_block_pose.split()[0]) - float(block_pose.text.split()[0])) < 0.1
                    and abs(float(other_block_pose.split()[1]) - float(block_pose.text.split()[1])) < 0.1
                )
                for other_block_pose in [tavolo_pose.text] + [include.find("pose").text if include.find("pose") is not None else "" for include in world.findall("include") if include != block_include and include.find("pose") is not None]
            )

    # Add camera configuration
    gui = ET.SubElement(world, "gui")
    camera = ET.SubElement(gui, "camera", name="gzclient_camera")
    camera_pose = ET.SubElement(camera, "pose")
    camera_pose.text = "1. 3.2 2.2 0. 0.4 -1.75"

    return ET.ElementTree(ET.ElementTree(world_sdf).getroot())  # Include XML declaration

def save_sdf_to_file(tree, filename):
    tree.write(filename)

if __name__ == "__main__":
    world_tree = generate_world_sdf()
    save_sdf_to_file(world_tree, "./locosim/ros_impedance_controller/worlds/bricks.world")
