#
#
# SIMPLE GEOMETRY FUNCTIONS
# For WAVN simulation 
# for checking to see if a wall intersects a line of sight
# dml Feb 2024

import numpy as np

# will two walls intersect each other
#
def intersect(w1,w2):
    global debug
    x11,y11,x12,y12,m1,c1=w1
    x21,y21,x22,y22,m2,c2=w2
    if m1==m2:
        m1=m2+0.001
    xi = (c2-c1)/(m1-m2)
    yi = m1*xi+c1
    flag= insegmentrange(xi,yi,x11,y11,x12,y12) and \
        insegmentrange(xi,yi,x21,y21,x22,y22)
    return flag

#will a list of walls and a wall intersect each other
def anyintersect(wlist,w):
    for w1 in wlist:
        if intersect(w1,w):
            return True
    return False

#is a point in the bounding box of a line segment
def insegmentrange(x,y,x1,y1,x2,y2):
    global debug
    if debug:
        print("isr ",x,y,x1,y1,x2,y2)
        debug=False
    return ( min(x1,x2)-2 <= x <= max(x1,x2)+2 ) and \
           ( min(y1,y2)-2 <= y <= max(y1,y2)+2 )

#is the point p on or close to the line w      
def online(w,p):
    x,y=p
    x1,y1,x2,y2,m,c=w
    if insegmentrange(x,y,x1,y1,x2,y2):
        m=w[4]
        c=w[5]
        res=m*x+c-y
        return abs(res)<=10
    return False

#is this point on any of the lines in lW
def pointonanyline(lW,p):
    if lW==[]:
        return False
    for w in lW:
        if online(w,p):
            return True
    return False    

#are any of the points in lP on this line
def anyonline(w,lP):
    for p in lP:
        if online(w,p):
            return True
    return False

#are two points close to one another
def close(p1,p2):
    d=np.hypot(p1[0]-p2[0],p1[1]-p2[1])
    return d<2.0

#is anypoint in the list close to this point
def anyclose(r,lP):
    for p in lP:
        if close(r,p):
            return True
    return False
#
