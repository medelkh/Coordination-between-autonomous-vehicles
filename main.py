import numpy as np
from random import seed
from random import random
from turtle import *
import turtle
import sys
import time

trt = turtle.Turtle()
scale = 100
trt.speed(10)
INF = (1e16,-1)
samples = 250
screen = Screen()
screen.tracer(False)

class Point:
    def __init__(pt,x,y):
        pt.x=x
        pt.y=y

    def __gt__(pt,other):
        if pt.x==other.x: return pt.y>=other.y
        else: return pt.x>pt.y

    def dist(pt,pt2):
        return np.sqrt((pt2.y-pt.y)**2+(pt2.x-pt.x)**2)


def ccw(A,B,C): #counterclockwise
    return (C.y-A.y)*(B.x-A.x) >= (B.y-A.y)*(C.x-A.x)

def intersect(A,B,C,D): #intersection des segments AB et CD
    return ccw(A,B,C)!=ccw(A,B,D) and ccw(C,D,A)!=ccw(C,D,B)

def f(T,pt):
    T.setpos(pt.x*scale-230,pt.y*scale-230)

def draw(pt1,pt2):
    trt.pencolor("black")
    trt.up()
    f(trt,pt1)
    trt.down()
    f(trt,pt2)

class Obstacle:
    def __init__(obs,sommets):
        obs.V = sommets #liste de sommets du polygone
        obs.N = len(sommets)
        for i in range(obs.N-1):
            draw(sommets[i],sommets[i+1])
        draw(sommets[0],sommets[-1])

    def is_inside(obs,P):
        point_inf = Point(-0.215153,0.165442)
        intersection_count = 0
        for i in range(obs.N-1):
            intersection_count += intersect(point_inf,P,obs.V[i],obs.V[i+1])
        intersection_count += intersect(point_inf,P,obs.V[0],obs.V[-1])
        return intersection_count % 2 == 1

    def intersects(obs,A,B):
        if obs.is_inside(A) or obs.is_inside(B): return True
        for i in range(obs.N-1):
            if intersect(A,B,obs.V[i],obs.V[i+1]): return True
        if intersect(A,B,obs.V[0],obs.V[-1]): return True
        return False

class Segtree:
    def __init__(self,sz):
        self.seg = [INF for i in range(4*sz)]
        self.n = sz
        self.ptr = 0
        self.minpos = 0
    def upd(self, p, l, r, id, val):
        if(l>id or r<id): return None
        if l==r:
            self.seg[p]=val
        else:
            md = (l+r)//2
            self.upd(p*2,l,md,id,val)
            self.upd(p*2+1,md+1,r,id,val)
            self.seg[p] = min(self.seg[p*2],self.seg[p*2+1])
    def update(self,id,val):
        self.upd(1,0,self.n-1,id,val)
    def get_mn(self, p, l, r):
        if l==r:
            ret = self.seg[p]
            self.seg[p] = INF
            return ret
        else:
            md = (l+r)//2
            ret = 0
            if self.seg[p*2]<=self.seg[p*2+1]:
                ret = self.get_mn(p*2,l,md)
            else:
                ret = self.get_mn(p*2+1,md+1,r)
            self.seg[p] = min(self.seg[p*2],self.seg[p*2+1])
        return ret
    def get(self):
        return self.get_mn(1,0,self.n-1)
    def push(self,item):
        self.update(self.ptr,item)
        self.ptr+=1
    def empty(self):
        return self.seg[1]==INF

N,M=5,5

#INPUT: list of Obstacle(list of Point(x,y), the vertices of the polygon)
Obstacles = [Obstacle([Point(1,0),Point(2,0),Point(2,1)]),
            Obstacle([Point(1,1.5),Point(2,1.5),Point(2,3),Point(1,3)]),
            Obstacle([Point(3,5),Point(2,4),Point(3,3),Point(4,3),Point(4,4)]),
            Obstacle([Point(2.7,1),Point(2.3,2),Point(3,2.5),Point(4,0.5)]),
            Obstacle([Point(4,1.5),Point(4,2.7),Point(4.8,2.7),Point(4.8,1.5)])]


rayon = 2.5*np.sqrt(N*M*np.log(samples)/(samples*np.pi))
queue = [] #(left,A,B,start,finish)
car_size = 0.1
shape = ((car_size*scale,car_size*scale),(-car_size*scale,car_size*scale),(-car_size*scale,-car_size*scale),(car_size*scale,-car_size*scale))
turtle.register_shape('car', shape)

def valid_point(pt): #verifier si le point n'est dans aucun obstacle
    for Obs in Obstacles:
        if Obs.is_inside(pt):
            return False
    return True

def valid_segment(A,B):
    vecx, vecy = B.x-A.x, B.y-A.y
    vecx, vecy = vecx/np.sqrt(vecx**2+vecy**2), vecy/np.sqrt(vecx**2+vecy**2)
    orthx, orthy = vecy,-vecx
    orthx*=car_size
    orthy*=car_size
    return sub_valid_segment(Point(A.x+orthx,A.y+orthy),Point(B.x+orthx,B.y+orthy)) and sub_valid_segment(Point(A.x-orthx,A.y-orthy),Point(B.x-orthx,B.y-orthy))

def sub_valid_segment(A,B):
    for Obs in Obstacles:
        if Obs.intersects(A,B): return False
    return True

class Robot:
    def __init__(rbt,s,e,l,clr):
        rbt.car = turtle.Turtle()
        rbt.car.color(clr)
        rbt.car.shape('car')
        rbt.start = s
        rbt.end = e
        rbt.xblocks,rbt.yblocks = int(np.ceil(M/rayon)),int(np.ceil(N/rayon))
        rbt.grid = [[[] for j in range(rbt.xblocks)] for i in range(rbt.yblocks)]
        rbt.V = []
        rbt.G = [[] for i in range(samples+2)]
        rbt.edges = 0
        rbt.path = []
        rbt.lim = l
        rbt.trajectory = []
        rbt.dur = []
        rbt.cur = 0
        rbt.car.pencolor(clr)
        rbt.car.up()
        f(rbt.car,s)
        rbt.car.down()
        rbt.car.dot(13)
        rbt.car.up()
        f(rbt.car,e)
        rbt.car.down()
        rbt.car.dot(13)

    def generer_noeuds(rbt):
        init,but = rbt.start, rbt.end
        ret = [init]
        rbt.grid[int(init.y//rayon)][int(init.x//rayon)].append(0)
        for i in range(samples):
            pt = Point(random()*M, random()*N)
            while(valid_point(pt)==False):
                pt = Point(random()*M, random()*N)
            ret.append(pt)
            rbt.grid[int(pt.y//rayon)][int(pt.x//rayon)].append(i+1)
            '''trt.pencolor("green")
            trt.up()
            f(pt)
            trt.down()
            trt.dot(8)'''
        ret.append(but)
        rbt.grid[int(but.y//rayon)][int(but.x//rayon)].append(samples+1)
        return ret

    def voisins(rbt,u):
        i,j = int(rbt.V[u].y//rayon), int(rbt.V[u].x//rayon)
        steps = [(0,0),(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,-1),(-1,1)]
        for step in steps:
            y,x=step
            if(i+y<0 or j+x<0 or i+y>=rbt.yblocks or j+x>=rbt.xblocks): continue
            for v in rbt.grid[i+y][j+x]:
                if(u<v and rbt.V[u].dist(rbt.V[v])<=rayon and valid_segment(rbt.V[u],rbt.V[v])):
                    D = rbt.V[u].dist(rbt.V[v])
                    rbt.G[u].append((v,D))
                    rbt.G[v].append((u,D))
                    rbt.edges+=1
                    '''trt.pencolor("blue")
                    trt.up()
                    f(rbt.V[u])
                    trt.down()
                    f(rbt.V[v])
                    trt.down()'''
    def dijkstra(rbt):
        n = len(rbt.G)
        dist, vis = [1e16 for i in range(n)], [False for i in range(n)]
        dist[0] = 0
        to = [-1 for i in range(n)]
        dij = Segtree(rbt.edges)
        dij.push((0,0))
        while(dij.empty()==False):
            D,u = dij.get()
            if vis[u]: continue
            vis[u] = True
            for v in rbt.G[u]:
                if vis[v[0]]: continue
                new_dist = dist[u]+v[1]
                if new_dist<dist[v[0]]:
                    dist[v[0]] = new_dist
                    dij.push((new_dist,v[0]))
                    to[v[0]] = u
        if dist[n-1]==1e16: return "Aucun trajet trouve"
        x, ret = n-1, []
        while(x!=0):
            ret.append(x)
            x=to[x]
        ret.append(0)
        return ret

    def process(rbt):
        rbt.V = rbt.generer_noeuds()
        for i in range(samples+2):
            rbt.voisins(i)
        rbt.path = rbt.dijkstra()
        if rbt.path == "Aucun trajet trouve":
            print("Aucun trajet trouve")
        else:
            tmp = 0
            for i in range(len(rbt.path)-1):
                A = rbt.V[rbt.path[i]]
                B = rbt.V[rbt.path[i+1]]
                top = max(A.y,B.y)+car_size
                bottom = min(A.y,B.y)-car_size
                right = max(A.x,B.x)+car_size
                left = min(A.x,B.x)-car_size
                rng = (left-car_size-rayon-0.1,right+car_size+0.1)
                dis = A.dist(B)
                duration = dis/rbt.lim
                L,R = 0,len(queue)
                while L<R:
                    md = (L+R)//2
                    if queue[md][0]<rng[0]: L=md+1
                    else: R=md
                j1=L+0
                L,R = -1,len(queue)-1
                while L<R:
                    md = (L+R+1)//2
                    if queue[md][0] > rng[1]: R=md-1
                    else: L=md
                j2=L+0
                tmp_list = []
                for k in range(j1,j2+1):
                    C,D = queue[k][1],queue[k][2]
                    top2 = max(C.y,D.y)+car_size
                    bottom2 = min(C.y,D.y)-car_size
                    right2 = max(C.x,D.x)+car_size
                    left2 = min(C.x,D.x)-car_size
                    if left > right2 or left2 > right or top < bottom2 or top2 < bottom: continue
                    tmp_list.append((queue[k][3],queue[k][4]))
                tmp_list.sort()
                for pair in tmp_list:
                    if tmp>pair[1]: continue
                    elif not(tmp<pair[0] and tmp+duration<pair[0]): tmp = max(tmp,pair[1]+0.01)
                    else:
                        break
                rbt.trajectory.append(tmp)
                tmp+=duration
                print (str(i)+' '+str(tmp-duration)+' '+str(tmp))
            for i in range(len(rbt.path)-1):
                A = rbt.V[rbt.path[i]]
                B = rbt.V[rbt.path[i+1]]
                dis = A.dist(B)
                duration = dis/rbt.lim
                if i!=len(rbt.trajectory)-1 :
                    queue.append((min(A.x,B.x),A,B,rbt.trajectory[i],rbt.trajectory[i+1]))
                    rbt.dur.append(rbt.trajectory[i+1]-rbt.trajectory[i])
                else:
                    queue.append((min(A.x,B.x),A,B,rbt.trajectory[i],rbt.trajectory[i]+rayon/rbt.lim))
                    rbt.dur.append(rayon/rbt.lim)
            queue.sort()


#INPUT: list of Robot(start pt, finish pt, speed, color)
robots = [Robot(Point(0.5,0.5),Point(4.5,4.5),1.6,"red"), Robot(Point(4.3,0.1),Point(0.05,3.5),1,"blue"), Robot(Point(0.8,3), Point(0.8,4.7), 1.5, "green")]
 #Robot(Point(1,1),Point(3,0.5),0.7,"green")]

duration=0

for R in robots:
    R.process()
    duration = max(duration,R.trajectory[-1]+1)

#print(duration)
tm0 = time.time()

anim = [(-1,-1,-1,-1) for i in range(len(robots))] #(A,B,started,duration)

while(time.time()-tm0<duration):
    tm = time.time()-tm0
    for i in range(len(robots)):
        r=robots[i]
        if r.cur == len(r.path)-1: continue
        if r.trajectory[r.cur]<=tm:
            #print("done")
            A = r.V[r.path[r.cur]]
            B = r.V[r.path[r.cur+1]]
            d = r.dur[r.cur]
            anim[i] = (A,B,r.trajectory[r.cur],r.dur[r.cur])
            r.cur+=1
    for i in range(len(robots)):
        r = robots[i]
        if tm>anim[i][2]+anim[i][3]: continue
        X=anim[i][0].x+(anim[i][1].x-anim[i][0].x)*(tm-anim[i][2])/anim[i][3]
        Y=anim[i][0].y+(anim[i][1].y-anim[i][0].y)*(tm-anim[i][2])/anim[i][3]
        r.car.setpos(X*scale-230,Y*scale-230)
    screen.update()
