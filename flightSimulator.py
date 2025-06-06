import FreeSimpleGUI as sg
import matplotlib.pyplot as plt
import numpy as np
import os
import pygame
import pyglet
import sqlite3
import time
import win32gui

from pygame.locals import *

fontList = sg.Text.fonts_installed_list()

if "Roboto" not in fontList:
    try:
        pyglet.options['win32_gdi_font'] = True
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Black.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-BlackItalic.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Bold.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-BoldItalic.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Italic.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Light.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-LightItalic.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Medium.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-MediumItalic.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Regular.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-Thin.ttf'))))
        pyglet.font.add_file(str(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Fonts/Roboto-ThinItalic.ttf'))))
    except:
        pass

bgColor = '#202225'
boxColor = '#313338'
textColor = '#f3f4f5'
baseFont = ("Roboto",11,"bold")

theme_definition = {'BACKGROUND': boxColor,
                    'TEXT': textColor,
                    'INPUT': bgColor,
                    'TEXT_INPUT': textColor,
                    'SCROLL': bgColor,
                    'BUTTON': ('#f3f4f5', '#202225'),
                    'PROGRESS': ('#01826B', '#D0D0D0'),
                    'BORDER': 1,
                    'SLIDER_DEPTH': 0,
                    'PROGRESS_DEPTH' : 0}

sg.theme_add_new('Discord_Dark', theme_definition)

sg.theme('Discord_Dark')

def enumHandler(hwnd, lParam):
    if win32gui.IsWindowVisible(hwnd):
        if 'Explore!!' in win32gui.GetWindowText(hwnd):
            win32gui.SetFocus(hwnd)

def tryFloat(x):
    try:
        return float(x)
    except:
        return 0

def sign(x):
    if x >= 0:
        return 1
    else:
        return -1
    
def rotateIntoVector(vec1,vec2,*rotAxis):
    if rotAxis == tuple([]):
        rotAxis = np.cross(vec1,vec2)
        rotAxis = rotAxis/np.linalg.norm(rotAxis)
    else:
        rotAxis = rotAxis[0]
    theta = np.arccos(np.dot(vec1,vec2))

    kMatrix = np.array([[0,-rotAxis[2],rotAxis[1]],[rotAxis[2],0,-rotAxis[0]],[-rotAxis[1],rotAxis[0],0]])
    identity = np.array([[1,0,0],[0,1,0],[0,0,1]])
    return identity + kMatrix*np.sin(theta) + np.matmul(kMatrix,kMatrix)*(1-np.cos(theta))

def isUnitVector(vec):

    if np.linalg.norm(vec) == 1:
        return True
    else:
        return False
    
def isOrthogonal(vec1,vec2):

    if np.dot(vec1,vec2) == 0:
        return True
    else:
        return False

def normalize(vec):
    mag = np.linalg.norm(vec)
    return [vec[0]/mag, vec[1]/mag, vec[2]/mag]

def reorthonormalize(matrix):
    xVector = [matrix[0][0],matrix[1][0],matrix[2][0]]
    xVector = xVector/np.linalg.norm(xVector)
    zVector = [matrix[0][2],matrix[1][2],matrix[2][2]]
    zVector = zVector/np.linalg.norm(zVector)

    newY = np.cross(zVector,xVector)
    newX = np.cross(newY,zVector)
    newZ = np.cross(newX,newY)

    orthoMatrix = [[newX[0],newY[0],newZ[0]],[newX[1],newY[1],newZ[1]],[newX[2],newY[2],newZ[2]]]

    return orthoMatrix

def getThrottleMod(currentSpeed,maxSpeed,minThrottle,optThrottle,maxThrottle):

    currentSpeed = tryFloat(currentSpeed)
    maxSpeed = tryFloat(maxSpeed)
    minThrottle = tryFloat(minThrottle)
    optThrottle = tryFloat(optThrottle)
    maxThrottle = tryFloat(maxThrottle)

    if maxSpeed > 0:
        speedPercent = currentSpeed / maxSpeed
    else:
        speedPercent = 0

    if speedPercent < optThrottle or optThrottle >= 1:
        percentToOptimal = speedPercent / optThrottle
        throttleMod = percentToOptimal * (1 - minThrottle) + minThrottle
    else:
        percentFromOptimal = (speedPercent - optThrottle)/(1 - optThrottle)
        throttleMod = ((maxThrottle - 1) * percentFromOptimal + 1)

    throttleMod = np.floor(10*throttleMod+0.5)/10
    
    return throttleMod

def updateAxis(current, controlPosition, maxRate, accel, decel, dt):
    target = controlPosition * maxRate
    if current < target:
        current += accel * dt
        if current > target:
            current = target
    elif current > target:
        current -= decel * dt
        if current < target:
            current = target
    else:
        current = target

    return current

def transformPitch(matrix,degrees):
    rads = degrees * np.pi/180

    sine = np.sin(rads)
    cosine = np.cos(rads)

    b = matrix[0][1]
    c = matrix[0][2]
    f = matrix[1][1]
    g = matrix[1][2]
    j = matrix[2][1]
    k = matrix[2][2]

    matrix[0][1] = b * cosine + c * sine
    matrix[0][2] = b * -sine + c * cosine
    matrix[1][1] = f * cosine + g * sine
    matrix[1][2] = f * -sine + g * cosine
    matrix[2][1] = j * cosine + k * sine
    matrix[2][2] = j * -sine + k * cosine

    return matrix

def transformYaw(matrix,degrees):
    rads = degrees * np.pi/180

    sine = np.sin(rads)
    cosine = np.cos(rads)

    a = matrix[0][0]
    c = matrix[0][2]
    e = matrix[1][0]
    g = matrix[1][2]
    i = matrix[2][0]
    k = matrix[2][2]

    matrix[0][0] = a * cosine + c * -sine
    matrix[0][2] = a * sine + c * cosine
    matrix[1][0] = e * cosine + g * -sine
    matrix[1][2] = e * sine + g * cosine
    matrix[2][0] = i * cosine + k * -sine
    matrix[2][2] = i * sine + k * cosine

    return matrix

def transformRoll(matrix,degrees):
    rads = degrees * np.pi/180

    sine = np.sin(rads)
    cosine = np.cos(rads)

    a = matrix[0][0]
    b = matrix[0][1]
    e = matrix[1][0]
    f = matrix[1][1]
    i = matrix[2][0]
    j = matrix[2][1]

    matrix[0][0] = a * cosine + b * sine
    matrix[0][1] = a * -sine + b * cosine
    matrix[1][0] = e * cosine + f * sine
    matrix[1][1] = e * -sine + f * cosine
    matrix[2][0] = i * cosine + j * sine
    matrix[2][1] = i * -sine + j * cosine

    return matrix

def renderPoints(position,trans,ax):

    #So to rework this:
    #Create rotation matrix for Facing onto Z, then apply to the point grid
    #Create viewspace bounds by pitching and yawing the facing vector
    #Apply camera matrix to point grid to project points into the viewplane (I think?)
    #Could alternately take the point vectors and find the rotation angle from facing and if the pitch/yaw angles are greater than the viewspace bound angles, exclude
    #Filter points using the viewspace bounds
    #Get distances to remaining points and return

    offset = [np.mod(position[0],2000/9),np.mod(position[1],2000/9),np.mod(position[2],2000/9)]

    coordinateSpace = np.linspace(-1000,1000,10)

    xCoords = [x-offset[0] for x in coordinateSpace]
    yCoords = [y-offset[1] for y in coordinateSpace]
    zCoords = [z-offset[2] for z in coordinateSpace]

    #Rotate facing into Z:

    xAxis = [1,0,0]
    yAxis = [0,1,0]
    zAxis = [0,0,1]

    xTrans = [trans[0][0],trans[1][0],trans[2][0]]
    yTrans = [trans[0][1],trans[1][1],trans[2][1]]
    zTrans = [trans[0][2],trans[1][2],trans[2][2]]

    #So to do this we just need to rotate the coordinate system into our facing direction (Z) and then rotate about Z by some angle to align the X and Y axes.
    zRot = rotateIntoVector(zTrans,zAxis)
    yAngle = np.arccos(np.dot(np.matmul(zRot,yTrans),yAxis))

    kMatrix = np.array([[0,-1,0],[1,0,0],[0,0,0]])
    identity = np.array([[1,0,0],[0,1,0],[0,0,1]])
    yRot = identity + kMatrix*np.sin(yAngle) + np.matmul(kMatrix,kMatrix)*(1-np.cos(yAngle))
    
    fullRotation = np.matmul(yRot,zRot)

    if abs(np.cross(np.dot(fullRotation,xTrans),xAxis)[2]) > 10**-6: #make sure the transform X-axis is pointing in the correct direction (read: aligned with the correct x-axis) and if not, flip the y-rotation angle. Without this you can only rotate halfway around.
        yAngle = -np.arccos(np.dot(np.matmul(zRot,yTrans),yAxis))

        kMatrix = np.array([[0,-1,0],[1,0,0],[0,0,0]])
        identity = np.array([[1,0,0],[0,1,0],[0,0,1]])
        yRot = identity + kMatrix*np.sin(yAngle) + np.matmul(kMatrix,kMatrix)*(1-np.cos(yAngle))
        fullRotation = np.matmul(yRot,zRot)

    #Define pitch and yaw angles in terms of focal length such that the frustum has the correct dimensions, i.e.:
    f = 512 #focal length. Not sure what's ideal here since it's a function of FOV so play with it maybe until you get something that works.
    yawAngle = np.arctan(1024/2 / f) #45 degrees for f = 512
    pitchAngle = np.arctan(768/2 / f) #~36.9 degrees

    yawRight = [[np.cos(yawAngle),0,np.sin(yawAngle)],[0,1,0],[-np.sin(yawAngle),0,np.cos(yawAngle)]]
    yawLeft = np.transpose(yawRight)
    pitchUp = [[1,0,0],[0,np.cos(pitchAngle),np.sin(pitchAngle)],[0,-np.sin(pitchAngle),np.cos(pitchAngle)]]
    pitchDown = np.transpose(pitchUp)

    topRight = np.matmul(pitchUp,np.matmul(yawRight,zAxis))
    bottomLeft = np.matmul(pitchDown,np.matmul(yawLeft,zAxis))

    allPoints = []
    points = []
    dists = []

    for x in xCoords:
        for y in yCoords:
            for z in zCoords:
                #Rotate point into the viewplane
                translatedPoint = [x-offset[0],y-offset[1],z-offset[2]]
                rotatedPoint = np.matmul(fullRotation,translatedPoint)
                allPoints.append(rotatedPoint)
                distance = np.linalg.norm(rotatedPoint)
                #image point onto viewplane
                focusScale = f/rotatedPoint[2]
                imagedPoint = [focusScale*rotatedPoint[0],focusScale*rotatedPoint[1]]
                if np.dot(rotatedPoint,zAxis) >= 0 and imagedPoint[0] <= topRight[0]*rotatedPoint[2]/topRight[2] and imagedPoint[0] >= bottomLeft[0]*rotatedPoint[2]/bottomLeft[2] and imagedPoint[1] <= topRight[1]*rotatedPoint[2]/topRight[2] and imagedPoint[1] >= bottomLeft[1]*rotatedPoint[2]/bottomLeft[2]: #Check if point is in front of the viewplane and then if it's within the bounds of the window
                    points.append([imagedPoint[0]+1024/2,imagedPoint[1]+768/2])
                    dists.append(f/distance * 5)

    return points, dists

def main():

    fig = plt.figure(figsize=(24,12))
    ax = fig.add_subplot(projection='3d')
    mgr = plt.get_current_fig_manager()
    mgr.window.setGeometry(2230,336,1024,768)
    plt.ion()
    plt.show()

    if os.path.exists('Data/tables.db') and os.path.exists('Data/savedata.db'):

        data = sqlite3.connect("file:Data/tables.db?mode=rw", uri=True)
        cur = data.cursor()

        data2 = sqlite3.connect("file:Data/savedata.db?mode=rw", uri=True)
        cur2 = data2.cursor()

        chassisData = cur.execute("SELECT name, accel, decel, pitchaccel, yawaccel, rollaccel, speedmod, speedfoils, minthrottle, optthrottle, maxthrottle, slide FROM chassis").fetchall()
        chassisNames = [chassis[0] for chassis in chassisData]

        engineData = cur2.execute("SELECT * FROM engine").fetchall()
        engineNames = [engine[0] for engine in engineData]

        selectionLeftCol = [
            [sg.Push(),sg.Text('Select Chassis:',font=baseFont)],
            [sg.Push(),sg.Text('Select Engine:',font=baseFont)]
        ]

        selectionRightCol = [
            [sg.Combo(chassisNames,'',s=(25,20),font=baseFont,key='chassis'),sg.Push()],
            [sg.Combo(engineNames,'',s=(25,10),font=baseFont,key='engine'),sg.Push()]
        ]

        selectionLayout = [
            [sg.Frame('',selectionLeftCol,border_width=0,p=0),sg.Frame('',selectionRightCol,border_width=0,p=0)],
            [sg.VPush()],
            [sg.Push(),sg.Button('Continue',font=('Roboto',14,'bold')),sg.Push()]
        ]

        window = sg.Window('Select Chassis and Engine',selectionLayout,finalize=True,size=(400,150))

        while True:
            event, values = window.read()

            if event == 'Continue':
                chassis = values['chassis']
                engine = values['engine']
                
                if chassis != '' and engine != '':
                    chassisStats = list([data for data in chassisData if data[0] == chassis][0])
                    engineStats = list([data for data in engineData if data[0] == engine][0])
                    overloadMod = 1.4
                    accel = tryFloat(chassisStats[1]) * overloadMod
                    decel = tryFloat(chassisStats[2]) * overloadMod
                    pAccel = tryFloat(chassisStats[3]) * overloadMod
                    yAccel = tryFloat(chassisStats[4]) * overloadMod
                    rAccel = tryFloat(chassisStats[5]) * overloadMod
                    if chassisStats[7] != 'N/A':
                        speedMod = tryFloat(chassisStats[7])
                    else:
                        speedMod = tryFloat(chassisStats[6])
                    chassisMinThrottleMod = tryFloat(chassisStats[8])
                    chassisOptThrottleMod = tryFloat(chassisStats[9])
                    chassisMaxThrottleMod = tryFloat(chassisStats[10])
                    slideMod = tryFloat(chassisStats[11])
                    maxPitch = engineStats[3] * overloadMod
                    maxYaw = engineStats[4] * overloadMod
                    maxRoll = engineStats[5] * overloadMod
                    topSpeed = engineStats[6] * overloadMod * speedMod
                    break

            if event == sg.WIN_CLOSED:
                break
        
        window.close()

    else:
        overloadMod = 1.4
        accel = 25 * overloadMod
        decel = 30 * overloadMod
        pAccel = 300 * overloadMod
        yAccel = 200 * overloadMod
        rAccel = 150 * overloadMod
        speedMod = 0.9025
        chassisMinThrottleMod = 0.2
        chassisOptThrottleMod = 0.4
        chassisMaxThrottleMod = 0.5
        slideMod = 1.5
        maxPitch = 90 * overloadMod
        maxYaw = 90 * overloadMod
        maxRoll = 90 * overloadMod
        topSpeed = 125 * overloadMod * speedMod

    dataLayoutLeft = [
        [sg.Push(),sg.Text('Speed: ',font=baseFont)],
        [sg.Push(),sg.Text('Pitch: ',font=baseFont)],
        [sg.Push(),sg.Text('Yaw: ',font=baseFont)],
        [sg.Push(),sg.Text('Roll: ',font=baseFont)],
        [sg.Push(),sg.Text('Coordinates: ',font=baseFont)],
        [sg.Push(),sg.Text('Facing: ',font=baseFont)],
        [sg.Push(),sg.Text('Show K/V Arrows: ',font=baseFont)]
    ]

    dataLayoutRight = [
        [sg.Text('',key='speedoutput',font=baseFont),sg.Push()],
        [sg.Text('',key='pitchoutput',font=baseFont),sg.Push()],
        [sg.Text('',key='yawoutput',font=baseFont),sg.Push()],
        [sg.Text('',key='rolloutput',font=baseFont),sg.Push()],
        [sg.Text('',key='coordoutput',font=baseFont),sg.Push()],
        [sg.Text('',key='facingoutput',font=baseFont),sg.Push()],
        [sg.Checkbox('',False,key='quivertoggle'),sg.Push()]
    ]

    dataLayout = [
        [sg.Frame('',dataLayoutLeft,border_width=0,p=0),sg.Frame('',dataLayoutRight,border_width=0,p=0)],
        [sg.Push(),sg.Button('Center on Self',font=baseFont),sg.Button('Center on Origin',font=baseFont),sg.Push()],
        [sg.Push(),sg.Button('Pause',font=baseFont),sg.Button('Reset',font=baseFont),sg.Push()]
    ]

    infoWindow = sg.Window('Flight Info',dataLayout,finalize=True,size=(400,300),relative_location=(-712,0))

    pygame.init()
    screen = pygame.display.set_mode((1024,768))
    pygame.display.set_caption('Explore!!')
    pygame.mouse.set_visible(1)

    mousePos = [0,0]

    screenRes = [1024, 768]

    deadZoneOuter = 1/3
    deadZoneInner = 1/12

    deadZoneInnerEnd = [screenRes[0]/2 * deadZoneInner,screenRes[1]/2 * deadZoneInner]
    deadZoneOuterStart = [screenRes[0]/2 * deadZoneOuter,screenRes[1]/2 * deadZoneOuter] #since we're splitting the screen into +- half the dimensions already, and we're making the deadzone half of that

    throttle = 0
    pitchThrottle = 0
    yawThrottle = 0
    yawThrottleKey = 0
    yawThrottleRetained = 0
    rollThrottle = 0
    lastThrottle = 0
    throttleUp = False
    throttleDown = False
    throttleYawRight = False
    throttleYawLeft = False
    throttleRollRight = False
    throttleRollLeft = False

    speed = 0
    velocity = [0,0,0]
    pitch = 0
    yaw = 0
    roll = 0

    position = [0, 0, 0]
    positionTrack = []
    positionTrackQuiver = []
    velocityTrack = []
    facingTrack = []
    trackCounter = 10
    transform = np.identity(3)
    facing = [transform[0][2],transform[1][2],transform[2][2]]

    timeStep = 1/30

    lastTick = 0
    togglePause = False
    plotToggle = 'origin'

    while True:

        infoEvents, infoValues = infoWindow.read(timeout=0)

        if infoEvents == 'Pause':
            if togglePause == False:
                togglePause = True
            else:
                togglePause = False

        if infoEvents == 'Center on Self':
            plotToggle = 'self'
        elif infoEvents == 'Center on Origin':
            plotToggle = 'origin'

        for event in pygame.event.get():
            if event.type == MOUSEMOTION:
                mousePos = list(event.pos)
                mousePos[0] -= screenRes[0]/2
                mousePos[1] -= screenRes[1]/2
                mousePos[1] *= -1
                if abs(mousePos[0]) >= deadZoneInnerEnd[0]:
                    yawThrottle = sign(mousePos[0]) * (abs(mousePos[0])-deadZoneInnerEnd[0])/deadZoneOuterStart[0]
                    if yawThrottle > 1:
                        yawThrottle = 1
                    elif yawThrottle < -1:
                        yawThrottle = -1
                    yawThrottleRetained = yawThrottle
                else:
                    yawThrottle = 0
                if abs(mousePos[1]) >= deadZoneInnerEnd[1]:
                    pitchThrottle = sign(mousePos[1]) * (abs(mousePos[1])-deadZoneInnerEnd[1])/deadZoneOuterStart[1]
                    if pitchThrottle > 1:
                        pitchThrottle = 1
                    elif pitchThrottle < -1:
                        pitchThrottle = -1
                else:
                    pitchThrottle = 0
            elif event.type == KEYUP:
                keyPress = event.unicode
                if keyPress == 'w' and throttleUp:
                    throttleUp = False
                elif keyPress == 's' and throttleDown:
                    throttleDown = False
                elif keyPress == 'q' and throttleRollLeft:
                    throttleRollLeft = False
                    rollThrottle = 0
                elif keyPress == 'e' and throttleRollRight:
                    throttleRollRight = False
                    rollThrottle = 0
                elif keyPress == 'a' and throttleYawLeft:
                    throttleYawLeft = False
                    yawThrottleKey = 0
                    yawThrottle = yawThrottleRetained
                elif keyPress == 'd' and throttleYawRight:
                    throttleYawRight = False
                    yawThrottleKey = 0
                    yawThrottle = yawThrottleRetained
            elif event.type == KEYDOWN or event.type == TEXTINPUT:
                try:
                    keyPress = event.unicode
                except:
                    keyPress = event.text
                if keyPress == 'w':
                    throttleUp = True
                    if time.time() - lastThrottle > 0.1:
                        throttle += 0.05
                        throttle = min(throttle,1)
                        lastThrottle = time.time()
                elif keyPress == 's':
                    throttleDown = True
                    if time.time() - lastThrottle > 0.1:
                        throttle -= 0.05
                        throttle = max(throttle,0)
                        lastThrottle = time.time()
                elif keyPress == 'q':
                    throttleRollLeft = True
                    rollThrottle = -1
                elif keyPress == 'e':
                    throttleRollRight = True
                    rollThrottle = 1
                elif keyPress == 'a':
                    throttleYawLeft = True
                    yawThrottleKey = -1
                elif keyPress == 'd':
                    throttleYawRight = True
                    yawThrottleKey = 1
            else:
                pass
            yawThrottle = yawThrottleKey + yawThrottleRetained
            if yawThrottle > 1:
                yawThrottle = 1
            elif yawThrottle < -1:
                yawThrottle = -1

        if time.time() - lastTick > timeStep and togglePause == False:
            if lastTick != 0:
                dt = timeStep
            else:
                dt = timeStep
            speed = updateAxis(speed,throttle,topSpeed,accel,decel,dt)
            throttleMod = getThrottleMod(speed,topSpeed,chassisMinThrottleMod,chassisOptThrottleMod,chassisMaxThrottleMod)
            pitch = updateAxis(pitch,pitchThrottle,maxPitch*throttleMod,pAccel,pAccel,dt)
            yaw = updateAxis(yaw,yawThrottle,maxYaw*throttleMod,yAccel,yAccel,dt)
            roll = updateAxis(roll,rollThrottle,maxRoll,rAccel,rAccel,dt)

            transform = transformYaw(transform,yaw*dt)
            transform = transformPitch(transform,pitch*dt)
            transform = transformRoll(transform,roll*dt)

            transform = reorthonormalize(transform)
            
            facing = [transform[0][2],transform[1][2],transform[2][2]]

            if slideMod == 0:
                for i in range(0,3):
                    velocity[i] = facing[i] * speed
            else:
                for i in range(0,3):
                    velocity[i] += facing[i] * speed * slideMod * dt

                velocityMag = np.linalg.norm(velocity)
                if velocityMag > speed:
                    velocityMag = speed
                    for i in range(0,3):
                        velocity[i] = normalize(velocity)[i] * speed

            position = [position[0]+velocity[0]*dt, position[1]+velocity[1]*dt, position[2]+velocity[2]*dt]

            positionTrack.append(position)

            if trackCounter == 10:
                trackCounter = 0
                positionTrackQuiver.append(position)
                velocityTrack.append([velocity[0]/np.linalg.norm(velocity),velocity[1]/np.linalg.norm(velocity),velocity[2]/np.linalg.norm(velocity)])
                facingTrack.append(facing)
            else:
                trackCounter += 1

            displayedPoints, radii = renderPoints(position,transform,ax)

            screen.fill((0,0,0))

            for i in range(len(displayedPoints)):
                pygame.draw.circle(screen,tuple([0,255,255]),displayedPoints[i],radii[i])
                
            pygame.display.flip()

            lastTick = time.time()

        infoWindow['speedoutput'].update(round(velocityMag*10,0))
        infoWindow['pitchoutput'].update(round(pitch,0))
        infoWindow['yawoutput'].update(round(yaw,0))
        infoWindow['rolloutput'].update(round(roll,0))
        infoWindow['coordoutput'].update('X ' + str(round(position[0],0)) + ' Y ' + str(round(position[1],0)) + ' Z ' + str(round(position[2],0)))
        infoWindow['facingoutput'].update('X ' + str(round(facing[0],2)) + ' Y ' + str(round(facing[1],2)) + ' Z ' + str(round(facing[2],2)))

        if len(positionTrack) > 1000:
            positionTrack = positionTrack[1:]
        if len(facingTrack) > 100:
            positionTrackQuiver = positionTrackQuiver[1:]
            facingTrack = facingTrack[1:]
            velocityTrack = velocityTrack[1:]

        ax.cla()
        ax.set_aspect('equal')
        maxDim = max(100,max([max(x) for x in positionTrack] + [-min(x) for x in positionTrack]))
        if plotToggle == 'self':
            ax.set_xlim(position[0]-250,position[0]+250)
            ax.set_ylim(position[1]-250,position[1]+250)
            ax.set_zlim(position[2]-250,position[2]+250)
        else:
            ax.set_xlim(-maxDim,maxDim)
            ax.set_ylim(-maxDim,maxDim)
            ax.set_zlim(-maxDim,maxDim)
        ax.scatter([0],[0],[0],c='Red',s=10)
        if positionTrack != []:
            ax.plot([pos[0] for pos in positionTrack],[pos[1] for pos in positionTrack],[pos[2] for pos in positionTrack])
            if infoValues['quivertoggle']:
                if maxDim < 250:
                    aLength = maxDim/5
                else:
                    aLength = 50
                ax.quiver([pos[0] for pos in positionTrackQuiver],[pos[1] for pos in positionTrackQuiver],[pos[2] for pos in positionTrackQuiver],[fac[0] for fac in facingTrack],[fac[1] for fac in facingTrack],[fac[2] for fac in facingTrack],length=aLength,color='red')
                ax.quiver([pos[0] for pos in positionTrackQuiver],[pos[1] for pos in positionTrackQuiver],[pos[2] for pos in positionTrackQuiver],[vel[0] for vel in velocityTrack],[vel[1] for vel in velocityTrack],[vel[2] for vel in velocityTrack],length=aLength,color='green')

        if infoEvents == 'Reset':
            speed = 0
            velocity = 0
            throttleMod = chassisMinThrottleMod
            pitch = 0
            yaw = 0
            roll = 0
            transform = np.identity(3)
            lastTick = 0
            position = [0, 0, 0]
            positionTrack = []
            positionTrackQuiver = []
            velocityTrack = []
            facingTrack = []

            throttle = 0
            pitchThrottle = 0
            yawThrottle = 0
            yawThrottleKey = 0
            yawThrottleRetained = 0
            rollThrottle = 0
            lastThrottle = 0
            throttleUp = False
            throttleDown = False
            throttleYawRight = False
            throttleYawLeft = False
            throttleRollRight = False
            throttleRollLeft = False

        if infoEvents == sg.WIN_CLOSED:
            break

        win32gui.EnumWindows(enumHandler,None)

main()
