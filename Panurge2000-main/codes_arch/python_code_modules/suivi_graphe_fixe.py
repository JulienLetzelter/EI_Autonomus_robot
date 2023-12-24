import numpy as np
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
        self.Chemin = self.pcc(position, objectif)
        self.actif = True
        self.livraison = True
        self.noeud_obj = self.Chemin[-1]
        self.arete = (self.noeud_prec, self.noeud_obj)

    def intersection(self, noeud, changement):
        print(noeud)
        self.noeud_prec = noeud
        if changement:
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
        distance, Chemin = dijkstra(self.reseau, depart, arrive)
        if distance == np.inf:

            if self.livraison:
                print("le colis ne peut pas être délivré")
                self.livraison = False
                self.objectif = self.base
                _, Chemin = dijkstra(self.reseau, depart, self.objectif)
            else:
                print("la voiture ne peut pas rentrer à sa base")
                self.actif = False

        return Chemin

def obstacle(Position, voiture):
    """Ajoute un obstacle sur le reseau"""
    for i in Position:
        voiture.reseau[i[0], i[1]] = np.inf
        voiture.reseau[i[1], i[0]] = np.inf
        


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
        voiture.direction += angle
        angle = (angle + 180)%360 -180
        print("angle" + str(angle))
        tourner_sur_place(np.sign(angle)*(abs(angle)*0.237037 - 1))
        """
        if lecture_distance() < 60:
            obstacle([voiture.arete], voiture)
            point_arr = voiture.noeud_prec
            voiture.intersection(point_arr, True)
            continue
        """
        changement = avancer(camera,rawcapture)
        if changement:
            point_arr = voiture.noeud_prec
            obstacle([voiture.arete], voiture)
            camera.capture(rawcapture, use_video_port=True, resize=(80, 60), format="bgr")
            frame = rawcapture.array
            angle_corr,is_inter = return_angle(frame)
            tourner_sur_place(np.sign(angle_corr)*(abs(angle_corr)*0.237037 - 1))
            voiture.direction = (voiture.direction + 180 + 90) % 360 - 90
        else:
            point_arr = voiture.noeud_obj
        voiture.intersection(point_arr, changement)


if __name__ == '__main__':
    main()
