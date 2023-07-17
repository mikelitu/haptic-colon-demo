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
import pyOpenHaptics.hd as hd
from pyOpenHaptics.hd_callback import hd_callback
from pyOpenHaptics.hd_device import HapticDevice
from dataclasses import dataclass, field
import numpy as np

@dataclass
class DeviceState:
    button: bool = False
    joints: list = field(default_factory=list)
    gimbals: list = field(default_factory=list)
    transform: list = field(default_factory=list)

@hd_callback
def state_callback():
    global device_state
    transform = hd.get_transform()
    device_state.transform = [[transform[0][0], transform[1][0], transform[2][0], 0.],
                              [transform[0][1], transform[1][1], transform[2][1], 0.],
                              [transform[0][2], transform[1][2], transform[2][2], 0.],
                              [transform[0][3], transform[1][3], transform[2][3], transform[3][3]]]
    joints = hd.get_joints()
    gimbals = hd.get_gimbals()
    device_state.joints = [joints[0], joints[1], joints[2]]
    device_state.gimbals = [gimbals[0], gimbals[1], gimbals[2]]
    button = hd.get_buttons()
    device_state.button = True if button==1 else False

# Directory to the different logos
logo_dir = "logos/kings-logo.png"

# Create the pygame widnow size and flags for debugging and final demo
display_size = (1920, 1080)
display_center = (display_size[0] // 2, display_size[1] // 2)
deb_flags = pygame.DOUBLEBUF | pygame.OPENGL
flags = pygame.DOUBLEBUF | pygame.OPENGL | pygame.FULLSCREEN
up_down_angle = 0
in_out_zoom = 1
left_right_angle = 0
around_angle = 0
translation = [0.0, 0.0, 0.0]

class ImageLoader:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 0
        self.height = 0
        self.img_data = 0
    
    def load(self, im_dir):
        image = pygame.image.load(im_dir).convert_alpha()
        img_data = pygame.image.tostring(image, 'RGBA')
        self.width = image.get_width()
        self.height = image.get_height()

        self.texID = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.texID)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.width, self.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img_data)
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)     

    def draw(self):

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslate(self.x, self.y, 0)

        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.texID)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)
        glVertex2f(0, 0)
        glTexCoord2f(1, 0)
        glVertex2f(self.width, 0)
        glTexCoord2f(1, 1)
        glVertex2f(self.width, self.height)
        glTexCoord2f(0, 1)
        glVertex2f(0, self.height)
        glEnd()
        glDisable(GL_TEXTURE_2D)


def init_display(node: SC.Node, im_loader: ImageLoader):
    """
    Define the initial window for the pygame rendering

    Args:
        node (SC.Node): Root node for a Sofa simulation scene
    """
    pygame.display.init()
    pygame.display.set_mode(display_size, deb_flags)
    pygame.display.set_caption("Pygame logo")
    pygame.mouse.set_visible(False)
    glClearColor(1, 1, 1, 1)
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, display_size[0], display_size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    # Draw the logo
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    im_loader.load(logo_dir)
    im_loader.draw()

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    SG.glewInit()
    SS.initVisual(node)
    SS.initTextures(node)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (display_size[0] / display_size[1]), 0.1, 50.0)
    
    # Set the background to white
    # glClearColor(1, 1, 1, 1)
    # glClear(GL_COLOR_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    pygame.display.flip()

def simple_render(rootNode: SC.Node, im_loader: ImageLoader, mouse_move: List[int], zoom_mouse: int):
    global up_down_angle, in_out_zoom, left_right_angle, around_angle, cur_state
    """
    Get the OpenGL context to render an image of the simulation state

    Args:
        rootNode (SC.Node): Sofa root node 
    """
    keypress = pygame.key.get_pressed()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, display_size[0], display_size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    # Draw the logo
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    im_loader.draw()

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (display_size[0] / display_size[1]), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    # Get the projection from the Sofa scene
    cameraMVM = rootNode.camera.getOpenGLModelViewMatrix()
    glMultMatrixf(cameraMVM)
    glMatrixMode(GL_MODELVIEW)  
    view_matrix= glGetFloatv(GL_MODELVIEW_MATRIX)

    #### Start the camera movement ####
    glPushMatrix()
    # glLoadIdentity()

    # Move the object around
    if keypress[pygame.K_w]:
        translation[2] += 0.1
    if keypress[pygame.K_s]:
        translation[2] -= 0.1
    if keypress[pygame.K_d]:
        translation[0] += 0.1
    if keypress[pygame.K_a]:
        translation[0] -= 0.1

    # Zoom the object with the mouse wheel
    in_out_zoom -= zoom_mouse * 0.5
    glTranslatef(translation[0], in_out_zoom, translation[2])

    # Rotate the object from left to right
    left_right_angle += mouse_move[0]*0.05
    glRotatef(left_right_angle, 0.0, 1.0, 0.0)

    # Rotate the object up and down
    up_down_angle += mouse_move[1]*0.05
    glRotatef(up_down_angle, 1.0, 0.0, 0.0)

    # Transform the original view
    # print(device_state.transform)
    if keypress[pygame.K_SPACE]:
        glMultMatrixf(device_state.transform)
        cur_state = device_state.transform
    else:
        glMultMatrixf(cur_state)

    glMultMatrixf(view_matrix)
    glGetFloatv(GL_MODELVIEW_MATRIX, view_matrix)

    SG.draw(rootNode)
    glPopMatrix()

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
    root.addObject("RequiredPlugin", name="SofaConstraint")
    root.addObject("RequiredPlugin", name="SofaHaptics")
    root.addObject("RequiredPlugin", name="SofaMeshCollision")
    root.addObject("RequiredPlugin", name="SofaUserInteraction")
    root.addObject("RequiredPlugin", name="Sofa.Component.SceneUtility")
    root.addObject("RequiredPlugin", name="SofaPython3")

    # place light and a camera
    root.addObject("LightManager")
    root.addObject("DirectionalLight", name="spotlight", direction=[0,1,0])
    root.addObject("InteractiveCamera", name="camera", position=[-31.899, -10.3206, 29.9103],
                            lookAt=[0,0,0], distance=10,
                            fieldOfView=45, zNear=0.63, zFar=100)
    
    # root.addObject(SpotlightController(node=root))

    sphere = root.addChild("Liver")
    sphere.addObject("MeshObjLoader", name="loader", filename="mesh/partial-colon.obj")
    sphere.addObject("OglModel", src="@loader", color="red", dy=-10, ry=-90)


def main():
    SofaRuntime.importPlugin("SofaComponentAll")
    im_loader=ImageLoader(10, 10)
    root = SC.Node("root")
    createScene(root)
    SS.init(root)
    init_display(root, im_loader)
    done = False
    mouse_move = [0, 0]
    zoom_mouse = 0.0
    paused = False

    pygame.mouse.set_pos(display_center)

    while not done:
        SS.animate(root, root.getDt())
        SS.updateVisual(root)
        if not paused:
            simple_render(root, im_loader, mouse_move, zoom_mouse)
        zoom_mouse = 0.0
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_RETURN:
                    done = True
                if event.key == pygame.K_PAUSE or event.key == pygame.K_p:
                    paused = not paused
                    pygame.mouse.set_pos(display_center)

            if not paused:
                if event.type == pygame.MOUSEMOTION:
                    mouse_move = [event.pos[i] - display_center[i] for i in range(2)]
                    pygame.mouse.set_pos(display_center)
                if event.type == pygame.MOUSEWHEEL:
                    zoom_mouse = event.y
        time.sleep(root.getDt())

    pygame.quit()
if __name__ == "__main__":
    device_state = DeviceState()
    device = HapticDevice(callback=state_callback)
    time.sleep(0.4)
    cur_state = device_state.transform
    main()
    device.close()