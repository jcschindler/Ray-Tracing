import math
import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from matplotlib import cm
import seaborn as sns


a = 1
xpix = 200
ypix = 200

class cube:

    def __init__(self,sideLength,x,y,z):
        cube.s = sideLength
        cube.loc = np.array([[x,y,z],[x+a*sideLength,y+sideLength,z+sideLength]])
        cube.sides = np.array([[x, x+a*sideLength],[y, y+sideLength],[z, z+sideLength]])
        cube.x = np.array([[a*sideLength,0,0],[0,sideLength,0],[0,0,sideLength]])   #position vec of 3planes x facing y facing z facing
        cube.n = np.array([cross(cube.x[1],cube.x[2]),cross(cube.x[2],cube.x[0]),cross(cube.x[0],cube.x[1])])
        cube.n  = np.array([unit(cube.n[0]), unit(cube.n[1]), unit(cube.n[2])])

class cam: #camera

    def __init__(self,x_position,y_position,z_position,x_facing,y_facing,z_facing):
        cam.fov = np.array([math.pi/4,math.pi/4])
        cam.pix = np.zeros([xpix,ypix])
        da = np.array([cam.fov[0]/(np.shape(cam.pix)[1]), cam.fov[1]/(np.shape(cam.pix)[0])]) # differential angle

        cam.loc = np.array([x_position, y_position, z_position])
        cam.dir = np.array([x_facing-x_position, y_facing-y_position, z_facing-z_position])# center pix
        # cam.dir = unit(self.dir) ##as of now top left pixel
        B=1 #arbitrary

        cam.nj = unit(np.array([1,1,0]))#unit(np.array([(-B*cam.dir[1])/cam.dir[0],B,0])) # parallel to xy plane
        cam.ni = unit(cross(cam.dir,cam.nj))
        cam.dir = unit(np.add(Quaternion(axis = cam.ni,angle = da[0]*(np.shape(cam.pix)[1]/2)).rotate(cam.dir),Quaternion(axis = cam.nj,angle = -da[1]*(np.shape(cam.pix)[0]/2)).rotate(cam.dir)))
        cam.angles = np.zeros([np.shape(cam.pix)[0],np.shape(cam.pix)[1],3])
        self.pixelAngle(da=da)

    # def checkPlane(self):
    def step(self):
        for i in range(np.shape(cam.pix)[0]): #num of rows
            for j in range(np.shape(cam.pix)[1]):
                cam.pix[i][j] = self.checkParallel(cube,self.angles[i][j])
                if cam.pix[i][j]<.01:
                    cam.pix[i][j] = np.nan
        # print(cam.pix)

    def pixelAngle(self,da):
        for i in range(np.shape(cam.pix)[0]): #num of rows
            for j in range(np.shape(cam.pix)[1]):
                self.angles[i][j] = unit(np.add(Quaternion(axis = cam.ni,angle = -da[0]*(i)).rotate(cam.dir),Quaternion(axis = cam.nj,angle = da[1]*(j)).rotate(cam.dir))) # add x and y vectors componentwise, then make unit vector

    def checkParallel(self,cube,dir): #run through for each angle
        testsides=np.array([0,0,0,0,0,0])
        for i in range(3):
            if dot(cube.n[i],dir)==0:
                testsides[i] = 0
                testsides[i+3] = 0
            else:
                testsides[i] = 1 # order x1, y1 ,z1, x2, y2, z2
                testsides[i+3] = 1
        dist = self.findIntersection(cube,dir,testsides)
        DIST = self.Shortest(dist)# list to int
        # print(DIST)
        return DIST[0]

    def findIntersection(self,cube,dir,testsides):
        w = np.array([np.add(self.loc,-cube.loc[0]),np.add(self.loc,-cube.loc[1])]) #1-3 use 1, 4-6 use 2
        inters = np.array([])
        dist = np.zeros(6)
        for j in range(2):#back/front plane
            for i in range(3):#xyz
                if testsides[i+(3*j)]==1:
                    s = -dot(cube.n[i],w[j])/dot(cube.n[i],dir)
                    inters = np.add(cam.loc,dir*s) #
                    if self.in_bounds(cube,inters,i+(3*j))==1:
                        dist[i+(3*j)] = self.distance(inters) # inters [1 by 3]
                # elif i ==3:
        return dist

    def in_bounds(self,cube,inters,i): # test 6 sides of cube
        if i == 0 or i == 3:#yz plane
            if inters[1]>cube.sides[1][0] and inters[1]<cube.sides[1][1] and inters[2]>cube.sides[2][0] and inters[2]<cube.sides[2][1]:
                return 1
            else:
                return 0

        if i == 1 or i == 4:#xz plane
            if inters[0]>cube.sides[0][0] and inters[0]<cube.sides[0][1] and inters[2]>cube.sides[2][0] and inters[2]<cube.sides[2][1]:
                return 1
            else:
                return 0

        if i == 2 or i == 5:#xy plane
            if inters[0]>cube.sides[0][0] and inters[0]<cube.sides[0][1] and inters[1]>cube.sides[1][0] and inters[1]<cube.sides[1][1]:
                return 1
            else:
                return 0

    def distance(self,inters):
        return math.sqrt(np.sum(np.add(self.loc,-inters)*np.add(self.loc,-inters)))

    def Shortest(self,dist):
        z = np.sort(dist,axis=0) #order by distance ;;we dont care which plane is hit, just dist
        a = 0
        stop = 0
        i = 0
        while stop == 0:
            if z[i]>0.01:
                a = i
                stop = 1
            i+=1
            if i == 6:
                stop = 1
        return[z[a]]


# def display(data):
def cross(a,b):
    return np.array([(a[1]*b[2])-(a[2]*b[1]),  (a[2]*b[0])-(a[0]*b[2]),  (a[0]*b[1])-(a[1]*b[0])])

def dot(a,b):
    return (a[0]*b[0] + a[1]*b[1] + a[2]*b[2])

def mag(a):
    return math.sqrt(np.sum(a*a))#math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])

def unit(a):
    # for i in range(3):
    #     a[i] = a[i]/mag(a)
    b = np.array([a[0]/mag(a),a[1]/mag(a),a[2]/mag(a)])
    return b

# room1 = room(10)

cube1 = cube(1,0,0,0)
# cam1 = cam(5,5,5,0,0,0)
cam1 = cam(6,-4,3,0,0,.5)
cam1.step()
# a=np.array([[math.pi/2,math.pi/2],[7,8]])
# print(cam1.pix)
# np.size(cam.pix)[1]
# plt.pcolor(cam1.pix)
# plt.colorbar()

fig,ax2 = plt.subplots()
plt.contour(cam1.pix);
mesh = ax2.pcolormesh(cam1.pix)
cbar = fig.colorbar(mesh);
sns.set_palette("husl",8)
ax2.set_aspect('equal');
plt.show()
