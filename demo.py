import Sofa.SofaGL as SG
import Sofa.Core as SC
import Sofa.Simulation as SS
import SofaRuntime
import os
import time
sofa_directory = os.environ["SOFA_ROOT"]
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from typing import List
# import pyOpenHaptics.hd as hd
# from pyOpenHaptics.hd_callback import hd_callback
# from pyOpenHaptics.hd_device import HapticDevice
from dataclasses import dataclass, field
import numpy as np
from utils import ImageLoader, create_sofa_window

@dataclass
class DeviceState:
    button: bool = False
    joints: list = field(default_factory=list)
    gimbals: list = field(default_factory=list)
    transform: list = field(default_factory=list)
    translation: list = field(default_factory=list)

# @hd_callback
# def state_callback():
#     global device_state
#     transform = hd.get_transform()
#     device_state.transform = [[transform[0][0], transform[1][0], transform[2][0], 0.],
#                               [transform[0][1], transform[1][1], transform[2][1], 0.],
#                               [transform[0][2], transform[1][2], transform[2][2], 0.],
#                               [transform[0][3], transform[1][3], transform[2][3], transform[3][3]]]
    
#     device_state.translation = [transform[3][0], transform[3][1], transform[3][2]]
#     joints = hd.get_joints()
#     gimbals = hd.get_gimbals()
#     device_state.joints = [joints[0], joints[1], joints[2]]
#     device_state.gimbals = [gimbals[0], gimbals[1], gimbals[2]]
#     button = hd.get_buttons()
#     device_state.button = True if button==1 else False

# Directory to the different logos
logo_dir = "logos/kings-logo.png"

# Create the pygame widnow size and flags for debugging and final demo
big_display_size = (1920, 1080)
big_display_center = (big_display_size[0] // 2, big_display_size[1] // 2)
big_position = [0, 0]
small_display_size = (300, 300)
small_display_center = (1620 + small_display_size[0] // 2, 780 + small_display_size[1] // 2)
small_position = [1620, 780]
deb_flags = pygame.DOUBLEBUF | pygame.OPENGL
flags = pygame.DOUBLEBUF | pygame.OPENGL | pygame.FULLSCREEN
up_down_angle = 0
in_out_zoom = 1
left_right_angle = 0
around_angle = 0
translation = [0.0, 0.0, 0.0]
init_view = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]



def init_display(node: SC.Node, im_loader: ImageLoader):
    global init_view
    """
    Define the initial window for the pygame rendering

    Args:
        node (SC.Node): Root node for a Sofa simulation scene
    """
    pygame.display.init()
    pygame.display.set_mode(big_display_size, deb_flags)
    pygame.display.set_caption("Colon simulation")
    pygame.mouse.set_visible(False)
    glClearColor(1, 1, 1, 1)
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    create_sofa_window(big_position, big_display_size, node)
    create_sofa_window(small_position, small_display_size, node)

    pygame.display.flip()


def simple_render(rootNode: SC.Node, im_loader: ImageLoader, mouse_move: List[int], zoom_mouse: int, change_view: bool):
    global up_down_angle, in_out_zoom, left_right_angle, around_angle, translation
    """
    Get the OpenGL context to render an image of the simulation state

    Args:
        rootNode (SC.Node): Sofa root node 
    """
    keypress = pygame.key.get_pressed()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    glViewport(0, 0, big_display_size[0], big_display_size[1])
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, big_display_size[0], big_display_size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    # Draw the logo
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (big_display_size[0] / big_display_size[1]), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    # Get the projection from the Sofa scene
    cameraMVM = rootNode.detailed_camera.getOpenGLModelViewMatrix()
    glMultMatrixf(cameraMVM)
    glMatrixMode(GL_MODELVIEW)  
    view_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)

    glPushMatrix()
    glTranslatef(8., 0., 0.)
    glRotatef(90., 0., 0., 1.)

    glMultMatrixf(view_matrix)
    view_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)
    
    SG.draw(rootNode)
    glPopMatrix()
    
    glClear(GL_DEPTH_BUFFER_BIT)
    # glEnable(GL_SCISSOR_TEST)
    # glScissor(small_position[0], small_position[1], small_display_size[0], small_display_size[1])
    # glClearColor(1, 1, 1, 1)
    glViewport(small_position[0], small_position[1], small_display_size[0], small_display_size[1])
    glDepthMask(False)
    # glColor(1, 1, 1, 1)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, small_display_size[0], small_display_size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    # Draw the logo
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (small_display_size[0] / small_display_size[1]), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    # Get the projection from the Sofa scene
    cameraMVM = rootNode.global_camera.getOpenGLModelViewMatrix()
    glMultMatrixf(cameraMVM)
    glMatrixMode(GL_MODELVIEW)  
    view_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)

    
    #### Start the camera movement ####
    glPushMatrix()
    # glMultMatrixf(cur_view)
    # glLoadIdentity()

    # Move the object around
    if keypress[pygame.K_w]:
        translation[2] += 0.1
    if keypress[pygame.K_s]:
        translation[2] -= 0.1
    if keypress[pygame.K_d]:
        translation[0] -= 0.1
    if keypress[pygame.K_a]:
        translation[0] += 0.1

    # # Zoom the object with the mouse wheel
    in_out_zoom += zoom_mouse * 0.5
    glTranslatef(in_out_zoom, translation[2], translation[0])

    # Move the object with the haptic device
    # if device_state.button:
    #     translation = device_state.translation

    # Rotate the object from left to right
    left_right_angle += mouse_move[0]*0.05
    glRotatef(left_right_angle, 0.0, 1.0, 0.0)

    # Rotate the object up and down
    up_down_angle -= mouse_move[1]*0.05
    glRotatef(up_down_angle, 0.0, 0.0, 1.0)

    glMultMatrixf(view_matrix)
    view_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)
    
    SG.draw(rootNode)
    glPopMatrix()
    glDepthMask(True)

    pygame.display.flip()


def createScene(root: SC.Node):
    """
    This function is necessary to run Sofa from both the runSofa executable and the python interpreter (IDE)

    Args:
        root (SC.Node): Root node for a Sofa simulation scene
    """
    # Register all the common component in the factory.
    SofaRuntime.PluginRepository.addFirstPath(os.path.join(sofa_directory, 'bin'))
    root.addObject("RequiredPlugin", name="Sofa.Component.IO.Mesh")
    root.addObject("RequiredPlugin", name="Sofa.Component.Engine.Transform")
    root.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Direct")
    root.addObject("RequiredPlugin", name="Sofa.Component.Mass")
    root.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Backward")
    root.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.Spring")
    root.addObject("RequiredPlugin", name="Sofa.Component.StateContainer")
    root.addObject("RequiredPlugin", name="Sofa.Component.Topology.Container.Constant")
    root.addObject("RequiredPlugin", name="Sofa.Component.Visual")
    root.addObject("RequiredPlugin", name="Sofa.GL.Component.Rendering3D")
    root.addObject("RequiredPlugin", name="Sofa.GL.Component.Shader")
    root.addObject("RequiredPlugin", name="Sofa.Component.Mapping.MappedMatrix")
    root.addObject("RequiredPlugin", name="Sofa.Component.Haptics")
    root.addObject("RequiredPlugin", name="SofaMeshCollision")
    root.addObject("RequiredPlugin", name="SofaUserInteraction")
    root.addObject("RequiredPlugin", name="Sofa.Component.SceneUtility")
    root.addObject("RequiredPlugin", name="SofaPython3")
    root.addObject("RequiredPlugin", name="BeamAdapter")

    root.addObject("VisualStyle", displayFlags="hideCollisionModels showVisualModels")

    # place light and a camera
    root.addObject("LightManager")
    root.addObject("DirectionalLight", name="spotlight", direction=[0,0,-1])
    root.addObject("InteractiveCamera", name="global_camera", position=[10, 0, 0],
                            lookAt=[0,0,0], distance=15,
                            fieldOfView=45, zNear=0.63, zFar=100)
    
    root.addObject("InteractiveCamera", name="detailed_camera", position=[10.0, 0., 0.],
                   lookAt=[0.1,0.1,0], distance=0,
                   fieldOfView=45, zNear=0.63, zFar=100)
    
    # root.addObject(SpotlightController(node=root))

    colon = root.addChild("Colon")
    colon.addObject("MeshObjLoader", name="loader", filename="mesh/partial-colon-decimate.obj")
    colon.addObject("MeshTopology", src="@loader")
    colon.addObject("MechanicalObject", src="@loader", template="Vec3d", rx=-90, ry=30, rz=0, dx=12, dy=1, scale=0.0275)
    colon.addObject("TriangleCollisionModel", group=1)
    colon.addObject("LineCollisionModel", group=1)
    colon.addObject("PointCollisionModel", group=1)
    visu_colon = colon.addChild("Visu")
    visu_colon.addObject("MeshObjLoader", name="loader", filename="mesh/partial-colon.obj")
    visu_colon.addObject("OglModel", src="@loader", color="red")
    visu_colon.addObject("BarycentricMapping")



def main():
    global pre_transform
    im_loader=ImageLoader(10, 10)
    root = SC.Node("root")
    createScene(root)
    SS.init(root)
    init_display(root, im_loader)
    done = False
    mouse_move = [0, 0]
    zoom_mouse = 0.0
    move_camera = False
    paused = False
    change_view = False
    clock = pygame.time.Clock()

    pygame.mouse.set_pos(small_display_center)

    while not done:
        clock.tick(100)
        SS.animate(root, root.getDt())
        SS.updateVisual(root)
        if not paused:
            simple_render(root, im_loader, mouse_move, zoom_mouse, move_camera)
        
        zoom_mouse = 0.0
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_RETURN:
                    done = True
                if event.key == pygame.K_PAUSE or event.key == pygame.K_p:
                    paused = not paused
                    pygame.mouse.set_pos(small_display_center)
            if not paused:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        move_camera = True
                    elif event.button == 3:
                        change_view = not change_view
                    
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        move_camera = False
                if event.type == pygame.MOUSEWHEEL:
                    zoom_mouse = event.y

                if not move_camera:
                    continue

                if event.type == pygame.MOUSEMOTION:
                    mouse_move = [event.pos[i] - small_display_center[i] for i in range(2)]
                    pygame.mouse.set_pos(small_display_center)
                
        time.sleep(root.getDt())

    pygame.quit()


if __name__ == "__main__":
    # device_state = DeviceState()
    # device = HapticDevice(callback=state_callback)
    time.sleep(0.4)
    # cur_state = device_state.transform
    main()
    # device.close()