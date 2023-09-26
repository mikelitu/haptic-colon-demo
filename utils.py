from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
import Sofa.SofaGL as SG
import Sofa.Simulation as SS
from scipy.spatial.transform import Rotation
import math


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


class SofaPygameWindow(object):
    def __init__(self, position, size, node, camera) -> None:
        self.position = position
        self.size = size
        self.node = node
        self.camera = camera
        self.create_window()

    def update(self) -> None:
        pass

    def create_window(self, main_display: bool):
        if main_display:
            glClearColor(1, 1, 1, 1)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glViewport(self.position[0], self.position[1], self.size[0], self.size[1])
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(0, self.size[0], self.size[1], 1)
        glMatrixMode(GL_MODELVIEW)

        # Draw the logo
        glLoadIdentity()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glEnable(GL_LIGHTING)
        glEnable(GL_DEPTH_TEST)
        SG.glewInit()
        SS.initVisual(self.node)
        SS.initTextures(self.node)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (self.size[0] / self.size[1]), 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


def create_sofa_window(position, size, node):
    glViewport(position[0], position[1], size[0], size[1])
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, size[0], size[1], 1)
    glMatrixMode(GL_MODELVIEW)

    # Draw the logo
    glLoadIdentity()
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    SG.glewInit()
    SS.initVisual(node)
    SS.initTextures(node)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (size[0] / size[1]), 0.1, 100.0)
    
    # Set the background to white
    # glClearColor(1, 1, 1, 1)
    # glClear(GL_COLOR_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    view_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)


def get_device_orientation():
    Rx = Rotation.from_quat([0.707, 0.0, 0.0, 0.707]).as_matrix()
    Ry = Rotation.from_quat([0.0, 1.0, 0.0, 0.0]).as_matrix()
    return Rotation.from_matrix(Ry @ Rx).as_quat().tolist()
