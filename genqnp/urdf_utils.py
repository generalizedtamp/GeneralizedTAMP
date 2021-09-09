
import trimesh
from pybullet_tools.utils import CLIENT
import os


gelatin_box = "../models/ycb_urdf/YcbGelatinBox/model.urdf"
tennis_ball = "../models/ycb_urdf/YcbTennisBall/model.urdf"
foam_brick = "../models/ycb_urdf/YcbFoamBrick/model.urdf"
potted_meat_can = "../models/ycb_urdf/YcbPottedMeatCan/model.urdf"
soup_can = "../models/ycb_urdf/YcbTomatoSoupCan/model.urdf"
master_chef_can = "../models/ycb_urdf/YcbMasterChefCan/model.urdf"


RED_CUBE_URDF = "../models/recycle/red_cube.urdf"
YELLOW_CUBE_URDF = "../models/recycle/yellow_cube.urdf"
BLUE_CUBE_URDF = "../models/recycle/blue_cube.urdf"
GREEN_CUBE_URDF = "../models/recycle/green_cube.urdf"
PURPLE_CUBE_URDF = "../models/recycle/purple_cube.urdf"
ORANGE_CUBE_URDF = "../models/recycle/orange_cube.urdf"
PINK_CUBE_URDF = "../models/recycle/pink_cube.urdf"
BLUE_CAN_URDF = "../models/recycle/blue_can.urdf"



scales = {
    gelatin_box: 1,
    tennis_ball: 1,
    foam_brick: 1,
    potted_meat_can: 1,
    soup_can: 1,
    master_chef_can: 1
}

block_heights = {
    gelatin_box: 0.08,
    tennis_ball: 0.06,
    foam_brick: 0.04,
    potted_meat_can: 0.09,
    soup_can: 0.11,
    master_chef_can: 0.12
}


def get_ycb_objects():
    # TODO rotations
    return [gelatin_box, tennis_ball, foam_brick, potted_meat_can, soup_can, master_chef_can]


def get_color_blocks():
    # TODO rotations
    return [RED_CUBE_URDF, YELLOW_CUBE_URDF, BLUE_CUBE_URDF, GREEN_CUBE_URDF, PURPLE_CUBE_URDF, ORANGE_CUBE_URDF, PINK_CUBE_URDF]