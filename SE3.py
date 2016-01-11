import sys
import os
import math
import numpy as np

class SE3:
    __M= np.eye(4)
    
    #Can be given a 3x3 R matrix and a
    #3x1 d matrix, neither, or just one of the matrices
    #constructor will make matrices as needed
    def __init__(self,R=None,d=None): #default values of rotation and translation matrix
        if R is not None and d is None: #if no translation matrix create it
            d=np.zeros((3,1))
                if R is None and d is not None: #if no rotation matrix create it
            R=np.eye(3)
        if R is not None and d is not None: #create special euclideon matrix
            self.__M= np.concatenate((np.concatenate((R,d),axis=1),np.array([[0,0,0,1]])),axis=0)
        self.__M  

    def inv(self): #returns the inverse of a rotation matrix
        #outputs the objects inverse as a SE3 object
        d=SE3.gettranslation(self)
        r=SE3.getrotation(self)
        m=r.T
        d2=-1*m.dot(d)
        g=SE3(m,d2)
        return g
    def getMatrix(self): #prints the SE3 mattrix's visual form
        #outputs the SE3 matrix
        return self.__M

    def mtimes(g1,g2): #multiply two SE3 matrixes by each other and return a new SE3 object
        #given two SE3 objects returns
        #the product of two with as a SE3 object
        g3=SE3()
        m=g1.__M.dot(g2.__M)
        g3.__M=m
        return g3

    def leftact(self,pv):
        #multiplies the SE3 matrix with a given 3x1 or 4x1 vector
        x=None
        if pv.shape[0]==3 and pv.shape[1]==1:
            tw=np.array([[pv[0,0]],
                         [pv[1,0]],
                         [pv[2,0]],
                         [1]])
            y=self.__M.dot(tw)
            x=np.array([[y[0,0]],
                        [y[1,0]],
                        [y[2,0]]])
        elif pv.shape[0]==4 and pv.shape[1]==1:
            if pv[3,0]==0:
                R=SE3.getrotation(self)
                k=np.eye(4)
                for i in range(3):
                    for j in range(3):
                        k[i,j]==R[i,j]

                x=k.dot(pv)
            else:
                x=self.__M.dot(pv)
        return x

    def adjoint(self,h):
        #given h computes the adjoint(g*h*g^-1)
        #h can be also be a twist in vector form(6x1) or hat form(4x4)
        z=None
        if isinstance(h,SE3)==True:
            y=SE3.inv(self)
            x=SE3.mtimes(self,h)
            z=SE3.mtimes(x,y)
        elif h.shape[0]== 6 and h.shape[1]==1:
            R=SE3.getrotation(self)
            v=np.array([[h[0,0]],
                        [h[1,0]],
                        [h[2,0]]])
            w=np.array([[h[3,0]],
                        [h[4,0]],
                        [h[5,0]]])
            d=SE3.gettranslation(self)
            print d
            dhat=np.array([[0,-1*d[2,0],d[1,0]],
                           [d[2,0],0,-1*d[0,0]],
                           [-1*d[1,0],d[0,0],0]])
            u=R.dot(v)
            vf=u+dhat.dot(R).dot(w)
            wf=R.dot(w)
            z=np.r_[vf,wf]
    
        elif h.shape[0]== 4 and h.shape[1]==4:
            x=SE3.inv(self).dot(h)
            z=x.dot(self)
        return z

    def log(self,tau=None):
        #returns a 6x1 vector(twist) tau default value is 1
        if tau is None:
            tau=1
        r=SE3.getrotation(self)
        d=SE3.gettranslation(self)
        tracer=((r[0,0]+r[1,1]+r[2,2])-1)/2
        norw=math.acos(tracer)/tau
        rtrans=r.T
        if norw==0:
            wvec=np.zeros((3,1))
            v=d/tau
        else:
            try:
                w=norw*(r-rtrans)/(2*math.sin(norw*tau))
                v1=math.pow(norw,2)
                v2=np.eye(3)-r
                v3=v2.dot(w)
                wvec=np.array([[w[2,1]],[w[0,2]],[w[1,0]]])
                v4=np.linalg.inv(v3+tau*wvec.dot(wvec.T))
                v=v1*v4.dot(d)
            except LinAlgError:
                print "Matrix not invertible outputting useless vector"
                v=np.zeros((3,1))
                wvec=np.zeros((3,1))
            else:
                 print "Inverted Matrix"
        xi=np.r_[v,wvec]
        return xi

    def getrotation(self):
        #returns the SE3 objects rotation matrix
        r=self.__M[0:3,0:3]
        return r
    def gettranslation(self):
        #returns the SE3 obects translation vector
        d=self.__M[0:3,[3]]
        return d

    @staticmethod
    def hat(xiVec):
        #hat the a 6x1 vector(returns a 4x4 vector)
        if xiVec.shape[0]==6:
            xiHat=np.array([[0,-1*xiVec[5,0],xiVec[4,0],xiVec[0,0]],
                            [xiVec[5,0],0,-1*xiVec[3,0],xiVec[1,0]],
                            [-1*xiVec[4,0],xiVec[3,0],0,xiVec[2,0]],
                            [0,0,0,0]])

        return xiHat
    @staticmethod
    def unhat(xiHat):
        #given a 4x4 matrix returns a 6x1 vector
        xiVec=np.array([[xiHat[0,3]],
                        [xiHat[1,3]],
                        [xiHat[2,3]],
                        [xiHat[2,1]],
                        [xiHat[0,2]],
                        [xiHat[1,0]]])
        return xiVec

    @staticmethod
    def exp(xi,tau=None):
        #given a 6x1 vector returns a SE3 object
        #may output garbage if matrix not invertible
        c=np.zeros((3,1))
        xiHat=SE3.hat(xi)
        v=np.array([[xiHat[0,3]],
                    [xiHat[1,3]],
                    [xiHat[2,3]]])
        w=np.array([[xiHat[2,1]],
                    [xiHat[0,2]],
                    [xiHat[1,0]]])
        wtrans=w.T
        what=xiHat[0:3,0:3]
        normw=np.linalg.norm(w)
        w2=w.dot(wtrans)-math.pow(normw,2)*np.eye(3)
        if tau==None:
            tau=1
            print tau
        if np.array_equiv(w,c)==False:
            ewt=np.eye(3)+(what/normw)*math.sin(tau*normw)+w2*(1-math.cos(normw*tau))/math.pow(normw,2)
            d1=np.eye(3)-ewt
            d2=d1.dot(what)/math.pow(normw,2)
            d3=d2.dot(v)
            d32=(w.dot(wtrans).dot(v)*tau)/math.pow(normw,2)
            d=d3+d32
        else:
            ewt=np.eye(3)
            d=v*tau
        expXi=np.concatenate((np.concatenate((ewt,d),axis=1),np.array([[0,0,0,1]])),axis=0)
        omegatau=SE3()
        omegatau.__M=expXi
        return omegatau

    @staticmethod
    def RtoEuler(R):
        #given a 3x3 rotaiton matrix
        #gives you the angles that make up the rotation matrix
        #in a 1x3 matrix
        if math.abs(R[2,0])!=1:
            theta1=-1*math.asin(R[2,0])
            theta2=math.pi-theta1
            cos1=math.cos(theta1)
            cos2=math.cos(theta2)
            zai1=math.atan2(R[2,1]/cos1,R[2,2]/cos1)
            zai2=math.atan2(R[2,1]/cos2,R[2,2]/cos2)
            phi1=math.atan2(R[1,0]/cos1,R[0,0]/cos1)
            phi2=math.atan2(R[1,0]/cos2,R[0,0]/cos2)
            euler=np.array([[theta1,zai1,phi1],
                           [theta2,zai2,phi2]])
        else:
            phi=0
            if R[2,0]==-1:
                theta=math.pi/2
                zai= math.atan2(R[0,1],R[0,2])
            else:
                theta=-1*math.pi/2
                zai= math.atan2(-1*R[0,1],-1*R[0,2])
            euler=np.array([theta,zai,phi])
        return euler

#example test code for the SE3 class
k=math.sqrt(2)
p=math.pi
r4=np.array([[1 ,0, 0],[0, 0, -1],[0, 1, 0]])
d4=np.array([[1],[2],[3]])
d5=np.array([[2],[-1],[-1]])
r5=np.array([[k/2, 0, -1*k/2],[0, 1, 0],[k/2, 0, k/2]])
p1 = np.array([[4],[5],[6]])
p2 = np.array([[4],[5],[6],[1]])
g4 = SE3(r4,d4)
g5 = SE3(r5,d5)
g6 = g4.mtimes(g5)
kk=g6.inv()
j=kk.getMatrix()
l=g6.getMatrix()
jj=g6.leftact(p1)
ll=g6.leftact(p2)

r1=np.array([[1, 0, 0],[0, 0, -1],[0,1,0]])
d1=np.array([[-5],[6],[-1]])
d2=np.array([[0.],[0.],[0.]])
r2=np.array([[ 1./k, 0, 1./k],[0, 1., 0],[-1./k,0,1./k]])
g1=SE3( r1,d1)
g2 = SE3(r2 , d2)
g2.adjoint(g1)
g1.adjoint(g2)
d3=np.array([[1],[0],[-1]])
r3=np.array([[k/2., 0 ,-1.*k/2.],[0, 1., 0],[k/2., 0 ,k/2.]])
g = SE3(r3,d3)
xi = np.array([[0],[ 1],[0], [p/10.], [0], [p/12.]])
jj=g.adjoint(xi)
rr=SE3.exp(xi)










