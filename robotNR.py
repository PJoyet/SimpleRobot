# -*- coding: utf-8 -*-
"""
Module utilisé pour le cours de simulation physique
Master 1 Système Avancé et Robotique
@author: Pierre Joyet

Ce module contient:
   la classe:       - RobotNR(position=[0,0,0], liaisons=[pi/2,0,0], segments=[0.5,0.5,0.5],
                              nom="robot",couleur=[0,0,0], DH_d=None,DH_alpha=None)
   
   les fonctions:   - MatriceRot2angles(R)
                    - proche(x, y, rtol=1.e-5, atol=1.e-8)
                    - TH(theta,d,a,alpha)
     
"""

import numpy as np
from math import pi,cos,sin,asin,atan2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def proche(x, y, rtol=1.e-5, atol=1.e-8):
    """Fonction qui détermine si deux valeurs sont proches"""
    return abs(x-y) <= atol + rtol * abs(y)

def MatriceRot2angles(R):
    """"Fonction qui calcule les angles depuis la matrice de rotation"""
    phi = 0.0
    if proche(R[2,0],-1.0):
        theta = pi/2.0
        psi = atan2(R[0,1],R[0,2])
    elif proche(R[2,0],1.0):
        theta = -pi/2.0
        psi = atan2(-R[0,1],-R[0,2])
    else:
        theta = -asin(R[2,0])
        cos_theta = cos(theta)
        psi = atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return psi, theta, phi

def TH(theta,d,a,alpha):
    """Fonction de calcul de la matrice de la transformée homogène"""
    TH =np.array([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha) ,a*cos(theta)],\
                  [sin(theta),cos(theta)*cos(alpha) ,-cos(theta)*sin(alpha),a*sin(theta)],\
                  [0         ,sin(alpha)            ,cos(alpha)            ,d],\
                  [0         ,0                     ,0                     ,1]])
    return TH

class RobotNR(object):
    """
--> RobotNR(position=[0,0,0], liaisons=[pi/2,0,0], segments=[0.5,0.5,0.5],
                       nom="robot",couleur=[0,0,0],DH_d=None,DH_alpha=None)

La classe RobotNR est une classe créant un robot à N Rotoïdes

Arguments: 
    - position (réels): [X,Y,Z] Position 3D de l'origine du robot
    - liaisons (réels): [theta1,theta2,...,thetaN] Rotation des N liaisons du robot
    - segments (réels): [a1,a2,...,aN] Longueur des N segments du robot
    - nom (string): Nom du robot
    - couleur (entiers): [R,G,B] Couleur RB du robot (entre 0 et 255)
    - DH_d (optionnel): [d1,d2,...,dN] Paramètre DH de la longueur par rapport a Z
    - DH_alpha (optionnel): [alpha1,alpha2,...,alphaN] Paramètre DH de la rotation par rapport à X
    
Fonctions:
    - Calc_TH(start,stop): calcul de la transformée homogène entre deux liaisons
    - getPosO(): renvoie les angles des segments
    - getPosA(): renvoie le x,y,z de l'extrémité du robot
    - getAllPosA(): renvoie les x,y,z de chaque liaison
    - MGD(angle): modification des angles des segments
    - MGI(pos,angle,eps): calcul des angles des segments
    - jacobienne(): calcul de la jacobienne de l'extrémité du robot
    - graph_plot_2D(): affiche le robot en 2D sur matplotlib
    - graph_plot_3D(): affiche le robot en 3D sur matplotlib
    - simule(): calcul le déplacement du robot sur un pas de temps dt
    
    """
    
    def __init__(self, position=[0,0,0], liaisons=[pi/2,0,0], segments=[0.5,0.5,0.5],
                       nom="robot",couleur=[0,0,0],DH_d=None,DH_alpha=None):
        """Attributs de RobotNR"""
        
        if DH_d is None:
            DH_d = [0]*len(segments)
        if DH_alpha is None:
            DH_alpha = [0]*len(liaisons)
            
        if not len(set(map(len,[liaisons,segments,DH_d,DH_alpha])))==1:
            raise TypeError("les arguments du DH doivent avoir les memes dimensions (defaut = 3)")
            
        self.nom = nom
        self.position = position
        self.couleur = couleur
        self.DH = {"theta":liaisons,"d":DH_d,"a":segments,"alpha":DH_alpha}
        
        
    def __str__(self):
        """Fonction d'affichage de RobotNR"""
        with np.printoptions(precision=2,suppress=True):
            dh_keys = np.array(list(self.DH.keys()))
            dh_values = np.transpose(np.array(list(self.DH.values())))
            return "Robot \""+self.nom+"\" d'origine: "+str(self.position)+"\n"+\
                "Parametre DH: \n"+str(dh_keys)+"\n"+str(dh_values)
                
    def __repr__(self):
        """Fonction d'affichage de RobotNR"""
        with np.printoptions(precision=2,suppress=True):
            dh_keys = np.array(list(self.DH.keys()))
            dh_values = np.transpose(np.array(list(self.DH.values())))
            return "Robot \""+self.nom+"\" d'origine: "+str(self.position)+"\n"+\
                "Parametre DH: \n"+str(dh_keys)+"\n"+str(dh_values)
                

    
    def Calc_TH(self,start,stop):
        """Fonction de calcul de la transformée homogène"""
        T = np.eye(4)
        T[:3,3]=self.position
        for i in range(start,stop):
            T = np.dot(T,TH(self.DH["theta"][i],self.DH["d"][i],self.DH["a"][i],self.DH["alpha"][i]))
        return T
    
    def getPosO(self):
        """Fonction qui renvoie la rotation des liaisons"""
        return (np.array(self.DH["theta"])+2*pi)%(2*pi)

    def getPosA(self):
        """Fonction qui renvoie la position de l'effecteur"""
        return self.Calc_TH(0,len(self.DH["theta"]))[:3,3]
    
    def getAllPosA(self):
        """Fonction qui renvoie la position de toutes les liaisons en X, Y et Z"""
        X = []
        Y = []
        Z = []
        for i in range(len(self.DH["theta"])+1):
            x,y,z=self.Calc_TH(0,i)[:3,3]
            X.append(x)
            Y.append(y)
            Z.append(z)
        return X,Y,Z    
    
    def MGD(self,angle):
        """Fonction de calcul du modèle géométrique direct"""
        self.DH["theta"]=angle
        #print("Nouvelles coordonnees articulaires de \""+self.nom+"\":\n",np.round(self.getPosA(),decimals = 3))    

    def jacobienne(self):
        """Fonction de calcul de la jacobienne"""
        jacobienne = np.zeros((6, len(self.DH["theta"])))
        vec = np.array([0,0,1])
        endEffecteur = self.getPosA()
        for i in range(len(self.DH["theta"])):
            LiaisCoords = self.Calc_TH(0,i)[:3,3]
            LiaisRot = self.Calc_TH(0,i)[:3,:3]
            V = np.dot(LiaisRot,vec)
            omega = np.cross(V,(endEffecteur-LiaisCoords))
            jacobienne[:3,i]=omega
            jacobienne[3:6,i]=V
        return jacobienne
    
    def MGI(self,pos,angle=None,eps=1e-6):
        """Fonction de calcul du modèle géométrique inverse"""
        maxiter = 0
        if angle == None:
            while True:
                E = pos-self.Calc_TH(0,len(self.DH["theta"]))[:3,3]
                J = self.jacobienne()[:3,:]
                piJ = np.linalg.pinv(J)
                dtheta=np.dot(piJ,E)
                self.DH["theta"]=self.DH["theta"]+dtheta
                maxiter +=1
                if  np.linalg.norm(E)<=eps or maxiter ==1000:
                    break
        else:
            while True:
                E=[]
                H = self.Calc_TH(0,len(self.DH["theta"]))
                val = MatriceRot2angles(H[:3,:3])
                E[:3] = pos-H[:3,3]
                E[3:]=[angle[0]-val[0],angle[1]-val[1],angle[2]-val[2]]
                J = self.jacobienne()
                piJ = np.linalg.pinv(J)
                dtheta=np.dot(piJ,E)
                self.DH["theta"]=self.DH["theta"]+dtheta
                maxiter +=1
                if  np.linalg.norm(E)<=eps or maxiter ==1000:
                    break
                
    def graph_plot_2D(self,legend="",color=None,fig=''):
        """Fonction d'affichage 2D du robot"""
        if color == None:
            color = (self.couleur[0]/255,self.couleur[1]/255,self.couleur[2]/255)
        f = plt.figure(fig)
        ax=f.gca()    
        X,Y,Z = self.getAllPosA()
        ax.axis('equal')
        ax.plot(X,Y,color=color, marker='.',label=self.nom+" "+legend)
        ax.plot(X[0],Y[0],color=color,marker='s',markersize=10)
        ax.plot(X[-1],Y[-1],color=color,marker='.',markersize=10)
        ax.legend()
        
    def graph_plot_3D(self,legend="",color=None,fig=''):
        """Fonction d'affichage 3D du robot"""
        if color == None:
            color = (self.couleur[0]/255,self.couleur[1]/255,self.couleur[2]/255)
        f = plt.figure(fig)
        ax=f.gca(projection='3d')
        X,Y,Z = np.round(self.getAllPosA(),decimals=3)
        ax.plot(X,Y,Z,color=color,marker='.',label=self.nom+" "+legend)
        ax.scatter(X[0],Y[0],Z[0],color=color,marker='s')
        ax.legend()
        
    def simule(self,pos,dt=0.1):
        """Fonction de simulation pour un pas de temps dt"""
        E = pos-self.Calc_TH(0,len(self.DH["theta"]))[:3,3]
        J = self.jacobienne()[:3,:]
        piJ = np.linalg.pinv(J)
        dtheta=np.dot(piJ,E)*dt
        self.DH["theta"]=self.DH["theta"]+dtheta
                
if __name__=="__main__":
    rob = RobotNR(segments=[1,1,1])
    rob.simule([1,2,0])
    print(rob.getPosA())
    rob.simule([1,2,0])
    print(rob.getPosA())
    print(rob.DH["theta"])
    # J = rob.jacobienne()
    # pinv = np.ndarray.transpose(J) * np.linalg.inv(np.dot(J,np.ndarray.transpose(J)))
    # print(rob.jacobienne())
    # print(np.linalg.pinv(rob.jacobienne()))
    # print(np.linalg.inv(np.dot(J,np.ndarray.transpose(J)))*np.ndarray.transpose(J))
    #  rob = RobotNR(position=[0,0,0])
    #  print(rob)
    #  print()
    #  print("print(rob.getPosO())")
    #  print(rob.getPosO())
    #  print()
    #  print("print(rob.getAllPosA())")
    #  print(rob.getAllPosA())
    #  print()
    #  print("print(rob.Calc_TH(0,len(rob.DH[\"theta\"])))   # Calc_TH(0,3)")
    #  print(rob.Calc_TH(0,len(rob.DH["theta"])))
    #  rob.MGD([pi/3,pi/3,-pi/3])
    #  print()
    #  print("print(rob.jacobienne())")
    #  print(rob.jacobienne())
    #  print()
    #  print("print(rob.getPosA())")
    #  print(rob.getPosA())
    #  print()
    #  rob.MGI([-0.9,-0.4,0])
    #  rob.MGD([pi/3,pi/3,-pi/3])
    #  rob.MGI([-0.9,-0.4,0],[0,0,pi])
    #  print("print(rob.getPosA())")
    #  print(rob.getPosO())
    #  rob.graph_plot_2D("MGI angle contraint par pi","k",fig="3RPlan - 2D")
    #  rob.graph_plot_3D("MGI angle contraint par pi","k",fig="3RPlan - 3D")

    #  print()
    #  print("---")
    #  print()

    #   #test robot3D
    #  theta = [0,0,0]
    #  d = [0,0,0]
    #  a = [0.50,0.50,0.50]
    #  alpha = [pi/2,0,0]
    #  couleur = [24,138,125]
    #  rob3D = RobotNR(position=[0,0,0], liaisons=theta, segments=a,DH_d=d,DH_alpha=alpha,nom="robot3D",couleur=couleur)
    #  rob3D.MGD([pi/3,pi/3,-pi/3])
    #  print(rob3D)
    #  rob3D.MGI([0.6,1,-0.2])
    #  rob3D.graph_plot_3D(fig="3R3D")
            
    