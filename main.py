import sys
import pygame as pg
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from settings import *
import ctypes

# Sıcrama olmasını önlemek için mause'u ortaya al
ctypes.windll.user32.SetCursorPos(HALF_RESOLUTION[0], HALF_RESOLUTION[1])

# Kupe ait köşe ve kenar layout
vertices = ((-2, -2, -2), (-2, 2, -2), (-2, 2, 2), (-2, -2, 2), (2, -2, -2), (2, 2, -2), (2, 2, 2), (2, -2, 2))
edges = ((0, 1), (0, 3), (0, 4), (1, 2), (1, 5), (2, 3), (2, 6), (3, 7), (4, 5), (4, 7), (5, 6), (6, 7))


def Ropes(arr):
    x, y, z = 0, 0, 0
    glBegin(GL_LINES)
    for i in range(OBJECT_COUNT):
        glVertex3fv((OBJECT_INIT_POS_LAYOUT[i][0], 0, 2))
        glVertex3fv([arr[i][0]+x, arr[i][1]+y, arr[i][2]+z])
        x += arr[i][0]
        y += arr[i][1]
        z += arr[i][2]
    glEnd()


def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def createPlane(viewMatrix):
    # multiply the current matrix by the get the new view matrix and store the final vie matrix
    glMultMatrixf(viewMatrix)
    viewMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)

    # apply view matrix
    glPopMatrix()
    glMultMatrixf(viewMatrix)

    glLightfv(GL_LIGHT0, GL_POSITION, [1, -1, 1, 0])

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glPushMatrix()

    glColor4f(0, 1, 0, 1)
    glBegin(GL_QUADS)
    glVertex3f(-10, -10, -2)
    glVertex3f(10, -10, -2)
    glVertex3f(10, 10, -2)
    glVertex3f(-10, 10, -2)
    glEnd()
    return viewMatrix


def CreateSphere():
    sphere = gluNewQuadric()

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (RESOLUTION[0] / RESOLUTION[1]), 0.1, 50.0)

    glMatrixMode(GL_MODELVIEW)
    gluLookAt(0, -8, 0, 0, 0, 0, 0, 0, 1)
    viewMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)
    glLoadIdentity()
    return sphere, viewMatrix

def UpdateSphere(sphere, posArray):

    for i in range(OBJECT_COUNT):
        glTranslatef(posArray[i][0], posArray[i][1], posArray[i][2])
        glColor4f(1, 0, 0, 1)
        gluSphere(sphere, SIZE_RATIO*BALL_SIZE, 32, 16)

    glPopMatrix()


############# PHYSICSSSS
# https://physics.stackexchange.com/questions/278433/number-of-degrees-of-freedom-of-the-coupled-pendulum-problem
# d-l(θ1−θ2)= d0   => d0 burda yayın sıkışması
# f = k * d0 = k*d-l*(θ1−θ2)  => f burda anlık kuvvet
# f*dt = j  => j burda itme
# dv = j/m  => dv burda hızdaki değişim
# dv*dt = dx   => dx burda konumdaki değişim

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_with_axes(v1):
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = (0, 1, 0)
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

class Object3D:
    def __init__(self, initPos, initVel, mass):
        self.pos = initPos
        self.vel = initVel
        self.mass = mass
        self.angle = 0

    def accelerate(self, dv):
        self.vel += dv

    def move(self):
        self.pos += self.vel * DT

class Space():

    def __init__(self):
        self.objectList = []
        self.dvList = []

        for i in range(OBJECT_COUNT):
            self.objectList.append(Object3D(np.array(OBJECT_INIT_POS_LAYOUT[i], np.float64),
                                            np.array(OBJECT_INIT_VEL_LAYOUT[i], np.float64),
                                            OBJECT_MASS_LAYOUT[i]))

    def iteration(self):
        # calculate degrees
        for i in range(OBJECT_COUNT):
            self.objectList[i].angle = angle_with_axes(self.objectList[i].pos)

        # calculate d0 >> force >> itme >> velocity change >> append
        counter = 0
        for i in SPRING_LAYOUT:
            angle = self.objectList[i[0]].angle - self.objectList[i[1]].angle
            angle *= PENDULUM_ROPE_LENGTH
            angle = SPRING_RELAXED_LEN_LAYOUT[counter] - angle

            f1mag = -angle * SPRING_K_LAYOUT[counter] * DT / self.objectList[i[0]].mass
            f2mag = angle * SPRING_K_LAYOUT[counter] * DT / self.objectList[i[1]].mass

            # kuvvetin büyüklüğünü biliyoruz yönünü hesaplayalım
            way = self.objectList[i[1]].pos - self.objectList[i[0]].pos
            way /= np.linalg.norm(way)

            self.dvList.append((f1mag * way, f2mag * way))
            counter += 1

        counter = 0
        # apply velocity change
        for i in SPRING_LAYOUT:
            self.objectList[i[0]].accelerate(self.dvList[counter][0])
            self.objectList[i[1]].accelerate(self.dvList[counter][1])

        # move the object
        for obj in self.objectList:
            obj.move()


posArray = [[-0.6, 0, 0],
            [0.6, 0, 0],
            [0.6, 0, 0]]

# UZAYI YARATTTTTT
spaceTest = Space()

# Pygame i baslat
pg.init()
# Pygame ekranını hazırla resolution boutunda ekrana görüntüleri verirken double buffering yap
# Ayrıca pyOpengl kullanacağız onon için flag
screen = pg.display.set_mode(RESOLUTION, pg.DOUBLEBUF | pg.OPENGL)
# mause u gizle
pg.mouse.set_visible(False)  # Hide cursor here

# Derinlik işiklandırma shader modelleri
glEnable(GL_DEPTH_TEST)
glEnable(GL_LIGHTING)
glShadeModel(GL_SMOOTH)
glEnable(GL_COLOR_MATERIAL)
glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

# ortam ışıklandırması
glEnable(GL_LIGHT0)
glLightfv(GL_LIGHT0, GL_AMBIENT, [0.5, 0.5, 0.5, 1])
glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1])

sphere, viewMatrix = CreateSphere()

# init mouse movement and center mouse on screen
displayCenter = [screen.get_size()[i] // 2 for i in range(2)]
mouseMove = [0, 0]
pg.mouse.set_pos(displayCenter)

up_down_angle = 0.0
paused = False
run = True
while run:
    for event in pg.event.get():
        if event.type == pg.QUIT:  # Sağ üst köşedeki kırmızı tuş
            run = False
        if event.type == pg.KEYDOWN:  # TUŞA BASILMASI
            if event.key == pg.K_ESCAPE or event.key == pg.K_RETURN:
                run = False
            if event.key == pg.K_PAUSE or event.key == pg.K_p:
                paused = not paused
                pg.mouse.set_pos(displayCenter)
        if not paused:
            if event.type == pg.MOUSEMOTION:  # MAUSE HAREKETLERİ
                mouseMove = [event.pos[i] - displayCenter[i] for i in range(2)]
            pg.mouse.set_pos(displayCenter)

    if not paused:
        # get keys
        keypress = pg.key.get_pressed()
        # mouseMove = pygame.mouse.get_rel()

        # init model view matrix
        glLoadIdentity()

        # apply the look up and down
        up_down_angle += mouseMove[1] * 0.1
        glRotatef(up_down_angle, 1.0, 0.0, 0.0)

        # init the view matrix
        glPushMatrix()
        glLoadIdentity()

        # apply the movment
        if keypress[pg.K_w]:
            glTranslatef(0, 0, 0.1)
        if keypress[pg.K_s]:
            glTranslatef(0, 0, -0.1)
        if keypress[pg.K_d]:
            glTranslatef(-0.1, 0, 0)
        if keypress[pg.K_a]:
            glTranslatef(0.1, 0, 0)

        # apply the left and right rotations
        glRotatef(mouseMove[0] * 0.1, 0.0, 1.0, 0.0)
        viewMatrix = createPlane(viewMatrix)
        UpdateSphere(sphere, posArray)
        Cube()
        Ropes(posArray)

        ##### DO MATH HERE
        # pyOpenGl hesaplamaları yaptıktan sonra burda bitmiyor düzgün bir layoutta yerleştirmek lazım
        # mantık şu pozisyon sıfırlanır
        # birinci cismin 1. cismin pozisyonu yerleştir
        # 2. - 1. poz
        # 3. - 2.  ve sırasıyla n. - (n-1). cismin pozisyonu arraye yüklenir
        #for i in range(OBJECT_COUNT):
        #    posArray[i] =

        spaceTest.iteration()
        print(spaceTest.objectList[0].pos)
        x, y, z = 0, 0, 0
        for i in range(OBJECT_COUNT):
            posArray[i][0] = spaceTest.objectList[i].pos[0] - x
            posArray[i][1] = spaceTest.objectList[i].pos[1] - y
            posArray[i][2] = spaceTest.objectList[i].pos[2] - z
            x = spaceTest.objectList[i].pos[0]
            y = spaceTest.objectList[i].pos[1]
            z = spaceTest.objectList[i].pos[2]


        ###########3


        pg.display.flip()
        pg.time.wait(10)

pg.quit()