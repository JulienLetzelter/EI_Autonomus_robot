import numpy as np
import copy
from graph import set_graph, dijkstra
from raspi_serial import *
from picamera import PiCamera
from picamera.array import PiRGBArray

camera = PiCamera()
rawcapture = PiRGBArray(camera, size=(80, 60))
time.sleep(0.1)

class robot:
    def __init__(self, position, objectif, reseau):
        self.base = position
        self.direction = 0  # direction de la voiture 0:haut, 180:bas, 90:gauche, -90, droite
        self.noeud_prec = position
        self.objectif = objectif
        self.reseau = reseau
        self.autres = Mobile(reseau)
        self.arret = False
        self.Chemin = self.pcc(position, objectif)
        self.actif = True
        self.livraison = True
        self.noeud_obj = self.Chemin[-1]
        self.arete = (self.noeud_prec, self.noeud_obj)
        self.changement = False

    def intersection(self, noeud):
        print(noeud)
        self.autres.mise_a_jour()
        self.noeud_prec = noeud
        if self.changement:
            self.Chemin = self.pcc(noeud, self.objectif)
        else:
            self.Chemin.pop()
        if not self.Chemin:
            if self.livraison:
                self.livraison = False
                self.objectif = self.base
                self.Chemin = self.pcc(noeud, self.objectif)
                print("colis délivré")
            else:
                self.actif = False
                return
        self.noeud_obj = self.Chemin[-1]
        self.arete = (self.noeud_prec, self.noeud_obj)

    def pcc(self, depart, arrive):
        self.autres.mise_a_jour()
        distance, Chemin = dijkstra(self.reseau, depart, arrive)
        if distance == np.inf:
            if self.test(depart, arrive):
                if self.livraison:
                    print("le colis ne peut pas être délivré")
                    self.livraison = False
                    self.objectif = self.base
                    _, Chemin = dijkstra(self.reseau, depart, self.objectif)
                else:
                    print("la voiture ne peut pas rentrer à sa base")
                    self.actif = False
            else:
                self.arret = True
        elif self.arret:
            self.arret = False
            self.Chemin = Chemin
            self.noeud_obj = Chemin[-1]
            self.arete = (self.noeud_prec, self.noeud_obj)
        return Chemin

    def test(self, depart, arrive):
        reseau_temp = copy.deepcopy(self.reseau)
        for p in self.autres.Position:
            _, valeur = self.autres.Position[p]
            reseau_temp[p] = valeur
        distance, _ = dijkstra(reseau_temp, depart, arrive)
        return distance == np.inf

    def obstacle(self, Position):
        """Ajoute un obstacle sur le reseau"""
        for i in Position:
            self.reseau[i[0], i[1]] = np.inf
            self.reseau[i[1], i[0]] = np.inf
            if (i[0], i[1]) in self.autres.Position:
                self.autres.Position.pop((i[0], i[1]))
            if (i[1], i[0]) in self.autres.Position:
                self.autres.Position.pop((i[1], i[0]))
    
    def rotation(self, angle):
        self.direction = (self.direction + angle + 90) % 360 - 90

class Mobile:
    def __init__(self, reseau):
        self.Position = {}
        self.attente = 3
        self.reseau = reseau

    def detection(self, positions):
        for p in positions:
            if not p in self.Position:
                self.Position[p[0], p[1]] = [
                    time.time(), self.reseau[p[0], p[1]]]
                self.Position[p[1], p[0]] = [
                    time.time(), self.reseau[p[1], p[0]]]
            else:
                self.Position[p[0], p[1]][0] = time.time()
                self.Position[p[1], p[0]][0] = time.time()
            self.reseau[p[0], p[1]] = np.inf
            self.reseau[p[1], p[0]] = np.inf

    def mise_a_jour(self):
        t = time.time()
        Fini = []
        for p in self.Position:
            if t - self.Position[p][0] > self.attente:
                Fini.append(p)

        for p in Fini:
            _, valeur = self.Position.pop(p)
            self.reseau[p] = valeur
        


def get_angle2(arete):
    if arete[0] == arete[1] + 1:
        return -90
    elif arete[0] == arete[1] - 1:
        return 90
    elif arete[0] > arete[1] + 1:
        return 180
    else:
        return 0


def main():
    #### Variables globales ###
    ##   Définition du réseau  ##
    n = 5
    m = n
    reseau = set_graph(n, m)

    ##  Définition du robot  ##
    position_init = 0
    objectif = n*m//2
    #objectif = 25
    voiture = robot(position_init, objectif, reseau)
    connect_to_arduino()
    print("Welcome to raspi_serial.py")
    ### Boucle de traitement ###
    while voiture.actif:
        
        direction_arete = get_angle2(voiture.arete)
        angle = direction_arete - voiture.direction
        angle = (angle + 180)%360 -180
        voiture.rotation(angle)
        tourner_sur_place(np.sign(angle)*(abs(angle)*0.237037 - 1.333333))
        voiture.changement = avancer(camera,rawcapture)
        if voiture.changement:
            point_arr = voiture.noeud_prec
            voiture.autres.detection([voiture.arete])
            voiture.rotation(180)
        else:
            point_arr = voiture.noeud_obj
        voiture.intersection(point_arr)


if __name__ == '__main__':
    main()
