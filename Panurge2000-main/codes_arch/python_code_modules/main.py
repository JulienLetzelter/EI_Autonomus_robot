import numpy as np
from graph import set_graph, dijkstra
from raspi_serial import *

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
        if [i[0], i[1]] in voiture.autres.Position:
            voiture.autres.Position.pop([i[0], i[1]])
        if [i[1], i[0]] in voiture.autres.Position:
            voiture.autres.Position.pop([i[1], i[0]])


def angle(arete):
    if arete[0] == arete[1] + 1:
        return 90
    elif arete[0] == arete[1] - 1:
        return -90
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

    ### Boucle de traitement ###
    while voiture.actif:
        connect_to_arduino()
        print("Welcome to raspi_serial.py")
        direction_arete = angle(voiture.arete)
        angle = direction_arete - voiture.direction
        voiture.direction += angle
        tourner(np.sign(angle)*(abs(angle)*0,237037 - 1.333333))
        changement = avancer()
        if changement:
            point_arr = voiture.noeud_prec
            obstacle(voiture.arete, voiture)
        else:
            point_arr = voiture.noeud_obj
        voiture.intersection(point_arr, changement)


if __name__ == '__main__':
    main()
