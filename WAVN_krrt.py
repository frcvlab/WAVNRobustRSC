#
# RRT* and interface to ledger
# dml Jan 2025
#

import numpy as np
import random as rand

#
# robust ledger format ledger=[ [R,[L1,L2,...], [R,L1,L2,....], ...]
# where R are robots and L are visible landmarks
# make an adjacency matrix for the equivalend robot-to-robot
# common landmark visibility adjacency matrix
#
def ledger2graph(ledger,N,M): # returns AdjMat
    VisibMat=np.zeros( (N,M) )
    AdjMat=np.zeros((N,N))
    # First make a NxM robot to landmark binary visibility matrix
    # VisibMat[r][l]==1 if robot r can see landmark l
    prev_lmarks=None
    for entry in ledger:
        robot,lmarks=entry
        vmr = VisibMat[robot]
        if isinstance(lmarks,list):
            for lm in lmarks:
                vmr[lm]=1
            if not prev_lmarks is None:
                for lm in prev_lmarks:
                    vmr[lm]=1
        elif not lmarks is None:
            vmr[-lmarks]=1
            if not prev_lmarks is None:
                vmr[-prev_lmarks]=1
        prev_lmarks=lmarks
        
    # next, make the robot-robot adjacency matrix
    for i in range(N): # each robot
        adji = AdjMat[i]
        for j in range(M):
            for k in range(N):
                if VisibMat[i][j]==1 and VisibMat[k][j]==1:
                    AdjMat[i][k],AdjMat[k][i]=1,1 # symmetric
    for i in range(N):
        AdjMat[i][i]=0 # for convenience
    return AdjMat

# RRT for WAVN
# Qgoal is a set of final robots that could see the goal
# start is the robot to move
# A is the visibility by common landmark adjacency matrix
# ledger is the ledger
# n is number of trials
# return is graph of vertices (robot numbers) and edges (robot,robot)
#
def rrt(Qgoal,start,A,ledger,n=300):
    # Qgoal //region that identifies success
    counter = 0 #keeps track of iterations
    V,E=[],[] # Graph G
    V.append(start)
    L = len(ledger)
    while counter < n:
        xNew = samplefree(ledger,V)
        xNearest = nearest(xNew,V,E,A)
        # don't need to check for inObstacle, but we instead check for
        # did mutual visibility allow such a point
        if not xNearest is None: # equivalent to not inObstacle test
            link=(xNearest,xNew)
            V.append(xNew)
            E.append(link)
            if xNew in Qgoal:
                path = extractPath(E,start,xNew)
                return path
        counter += 1
    return None

# k-RRT* for WAVN
# Qgoal is a set of final robots that could see the goal
# start is the robot to move
# A is the visibility by common landmark adjacency matrix
# ledger is the ledger
# n is number of trials
# k is the neighborhood to check/rewire
# default values for n,k chosen to get >90% success in path finding
# return is path as a sequence of robots that viewed common landmarks
# p=[r1,r2,r3] etc where r1 is the start, and the next motion is for
# r1 to move to a  common landmark seen by r1 and r2, then to one see
# by r2 and r3 etc.
#
def rrtstar(Qgoal,start,A,ledger,n=500,k=10):
    i = 0 #keeps track of iterations
    V,E=[],[] # Graph G, edges=(v1,v2,cost)
    V.append(start)
    L = len(ledger)

    for i in range(n):
        xNew = samplefree(ledger,V)

        if len(E)==0: # first one gets a pass
            E.append( (start,xNew,1) ) # cost is one
            if xNew in Qgoal:
                path = extractPath(E,start,xNew)
                return path
            continue

        xNearest = nearest(xNew,V,E,A)

        # don't need to check for inObstacle, but we instead check for
        # did mutual visibility allow such a point
        
        if xNearest is None: # equivalent to inObstacle for this case
            continue
        
        cost_xnew = A[xNearest][xNew]
        XNear = knear(V,E,xNew,A,k*np.log(i))
        V.append(xNew)

        xMin = xNearest
        cMin = cost(xNearest,E)+cost_xnew

        for xNear in XNear:
            if cost(xNear,E)+A[xNear][xNew] < cMin:
                xMin = xNear
                cMin = cost(xNear,E)+A[xNear][xNew]

        E.append( (xMin,xNew,cMin) )

        for xNear in XNear: # rewiring
            if cost(xNew,E)+A[xNew][xNear]<cost(xNear,E):
                e = edge(xNear,E)
                E = removeEdge(e,E)
                E.append( (xNew,xNear,cost(xNew,E)+A[xNew][xNear]) )

        if xNew in Qgoal: # success
            path = extractPath(E,start,xNew)
            return path
    return None



# return a random sample of the robots in the ledger
# slight danger of infinite loop of all robots
# already in V
# 
def samplefree(ledger,V):
    L = len(ledger)
    while True:
        xIndex = rand.randrange(0,L)
        xNew = ledger[xIndex][0] # robot index from ledger
        if not xNew in V:
            break
    return xNew

# find the nearest vertex to xNew in
# the graph V,E
#
def nearest(xNew,V,E,A):
    for edge in E:
        vertex=edge[1]
        if vertex!=xNew and A[vertex][xNew]==1:
            return vertex
    return None
#
# Give the graph V,E made by RRT, this function
# extracts the path from rs to E 
#
def extractPath(E,rs,rg):
    path=[]
    r = rg
    while r!=rs:
        link=[]
        for l in E:
            if l[1]==r:
                link=l
                break
        if link==[]:
            print("RRT: Extract path - no path exists!")
            exit()
        path.append( (link[0],link[1]) )
        r = link[0]
    path.reverse()
    return path

#
# returns k nodes in the neighborhood of xNew
# or less if there are fewer than k
#
def knear(V,E,xNew,A,k):
    neighbors=[]
    count = 0
    for edge in E:
        vertex=edge[1]
        if vertex!=xNew and A[vertex][xNew]==1:
            neighbors.append(vertex)
            count += 1
            if count >= k:
                return neighbors
    return neighbors

#
# return the cost for this OUT vertex
# xNearest, where 
# edge = (v1,v2,c) in E edge from v1 to v2
#
def cost(xNearest,E):
    #print("xNearest ",xNearest," E ",E)
    for edge in E:
        if edge[1]==xNearest:
            return edge[2]
    return None

# return the edge for this out vertex
# xNearest, where 
# edge = (v1,v2,c) in E edge from v1 to v2
#
def edge(xNearest,E):
    for edge in E:
        if edge[1]==xNearest:
            return edge
    return None

# remove an edge e from the edge list E
# return the new edge list
#
def removeEdge(e,E):
    newE=[]
    for edge in E:
        if edge != e:
            newE.append(edge)
    return newE

# Translate an RRT path to the standard form that
# the other methods use for drawing
# last landmark needs to be added, since RRT just does Robot to Robot
# common landmark steps.
# le is the last landmark, and robots,landmarks are the robot
# and landmark position lists from WAVN
#
def RRTpath2path(rrtpath,le,robots,landmarks):
    path = []
    for link in rrtpath:
        path.append(robots[link[0]])
        path.append(robots[link[1]])
    path.append(landmarks[le])
    return path
    
#EOF

