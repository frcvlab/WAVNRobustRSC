#
# SIMPLE DRAWING FUNCTIONS
# for WAVN simulation
# dml Feb 2024
# 
#
import cv2

# Global for turn on/off drawing the map
gRender=True

# Global
# color codes for drawing various things
#
robotMark =(100,0,0)
robotMark1=(150,0,0)
robotMark2=(200,0,200)
landMark = (0,80,0)
lineMark = (0,150,150)
pathMark1 = (120,50,150)
pathMark2 = (50,120,220)
pathMark3 = (120,150,50)
pathMark4 = (150,150,150)
wallMark = (150,150,0)

#
#
#worldsize globals
worldX,worldY=None,None

#
# draw a line
def doLine(s,e,map,mark=lineMark,offset=(0,0)):
    if not gRender:
        return
    if isinstance(e,list):
        for ep in e:
            doLine(s,ep,map,mark,offset)
        return
    if isinstance(s,list):
        for sp in s:
            doLine(sp,e,map,mark,offset)
        return
    #   
    xdiff = float(e[0]-s[0])
    ydiff = float(e[1]-s[1])
    res=100.0
    for i in range(0,int(res)):
        x=s[0]+i*float(xdiff/res)+offset[0]
        y=s[1]+i*float(ydiff/res)+offset[1]
        if 0<=x<worldX and 0<=y<worldY:
            map[int(x)][int(y)]=mark
    return

#draw a wall
def drawWall(w,map,mark=wallMark):
    if not gRender:
        return
    doLine(w[0:2],w[2:4],map,mark)
    return

#draw a robot
def drawRobot(rp,map,mark=robotMark):
    if not gRender:
        return
    u,v=rp
    p=(int(u),int(v))
    for x in [p[0]-1,p[0],p[0]+1]:     
        map[x][p[1]]=mark
    for y in [p[1]-1,p[1],p[1]+1]:
        map[p[0]][y]=mark
    return

#draw a landmark
def drawLandmark(p,map,mark):
    if not gRender:
        return
    for x in [p[0]-1,p[0],p[0]+1]:
        for y in [p[1]-1,p[1],p[1]+1]:
            map[x][y]=mark
    map[p[0]][p[1]]=0
    return

# draw all the lines of a path from RCS or BFS
def drawPath(path,lmark,map,mark=pathMark1,offset=(0,0)):
    if not gRender:
        return
    for pi in range(0,len(path)-1):
        l1 = path[pi]
        l2 = path[pi+1]
        doLine( l1,l2,map,mark,offset)
    return

# Draw the world: robots, landmarks, walls
def drawWorld(robots,landmarks,walls, common, cansee, map, clflag=True, rs=-1, re=-1):
    if not gRender:
        return
    map.fill(255) # kill old map
    for r in robots:
        if clflag :
            for m in cansee[robots.index(r)]:
                doLine(r,m,map)
    #
    for l in landmarks:
        li = landmarks.index(l)
        mark=landMark
        if li==re:
            mark=robotMark2
        drawLandmark(l, map,mark)
    #
    for w in walls:
        drawWall(w, map)
    #
    #
    for r in robots:
        mark=robotMark
        ri=robots.index(r)
        if ri==rs:
            mark=robotMark2
        drawRobot(r, map, mark)
    return

#
#
def showWorld(map,wflag=200):
    if not gRender:
        return
    image=cv2.resize(map, (500,500))
    cv2.imshow('World',image)
    cv2.waitKey(wflag)
    return
#
#
def initDraw(wX,wY):
    global worldX,worldY
    worldX,worldY=wX,wY
    if gRender:
        cv2.namedWindow('World',cv2.WINDOW_AUTOSIZE)
    return
