#include <exception>
#include <string>
#include <iostream>
#include "pointList.h"
#include <cstdlib>
#include <fstream>
#include <math.h>


PointList::PointList(){
    points = NULL;
    taille = 0;
}

PointList::PointList(int taille2){
    if(taille2 < 0){
        std::cout << "Impossible de créer une listes de point de longueur negative\n";
        return;
    }
    else if(taille2==0){
        points = NULL;
    }
    else{
        points = new Point[taille2];
    }
    for(int i(0); i<taille2; i++){
        points[i].set(0.,0.,0.);
    }
    taille = taille2;
}

PointList::PointList(PointList const& copie){
    taille = copie.taille;
    points = new Point[taille];
    for(int i=0; i<taille; i++){
        points[i] = copie.points[i];
    }
}

PointList::PointList(std::string file){
    std::ifstream fluxPoint(file.c_str());
    if(fluxPoint){
        char a;
        bool firstLine(true), flag(true), lecNb(false);
        double valeur(0.);
        int composante(0), compteurDec(0), compteurPoint(0), negativite(1), tailleList(0);
        bool decimal(false);

        while(firstLine && flag){
            fluxPoint.get(a);
            if(a == 10){
                firstLine = false;
            }
            else if(a>47 && a<58){
                tailleList = tailleList * 10 + (a-48);
            }
            else{
                std::cout << "Le fichier du nuage de points n'est pas au bon format\n";
                taille = 0;
                points = NULL;
                return ;
            }
        }
        points = new Point[tailleList];
        taille = tailleList;
        while(fluxPoint.get(a) && flag){
            if(a>47 && a<58){
                lecNb = true;
                if(decimal){
                    compteurDec = compteurDec+1;
                }
                if(compteurDec == 0){
                    valeur = valeur*10 + (a-48);
                }
                else{
                    valeur = valeur + pow(10,-compteurDec)*(a-48);
                }
            }
            else if(a == 32){
                if(lecNb){
                    if(composante == 0){
                        points[compteurPoint].setX(negativite * valeur);
                    }
                    else if(composante == 1){
                        points[compteurPoint].setY(negativite * valeur);
                    }
                    else{
                        std::cout << "Il y a trop d'argument sur la ligne " << compteurPoint << "\n";
                        return ;
                    }
                    composante = (composante + 1);
                    decimal = false;
                    compteurDec = 0;
                    valeur = 0.;
                    negativite = 1;
                }
                lecNb = false;
            }
            else if(a == 10){
                points[compteurPoint].setZ(negativite * valeur);
                composante = 0;
                valeur = 0.;
                decimal = false;
                compteurDec = 0;
                compteurPoint ++;
                negativite = 1;
                lecNb = false;
                if(compteurPoint >= taille){
                    if(fluxPoint.get(a)){
                        std::cout << "Le fichier est mal calibre\n";
                    }
                    flag = false;
                }
            }
            else if(a == 46){
                decimal = true;
            }
            else if(a == 45){
                negativite = -1;
            }
            else{
                std::cout << "Un caractere du fichier est illisible" << a << "\n";
            }
        }
        if(compteurPoint != taille){
            std::cout << "Le fichier est mal calibre\n";
        }
    }
    else{
        std::cout << "Impossible de créer une liste de Point a partir de " << file << "\n";
        taille = 0;
        points = NULL;
    }
}

PointList::~PointList(){
    delete points;
}

void PointList::display(std::ostream& str){
    int i;
    for(i=0; i<taille; i++){
        points[i].display(str);
    }
}

PointList PointList::getNbhd(int nbNb){
    PointList neighboors(nbNb);

    return neighboors;
}

Point PointList::getPoint(int i){
    return points[i];
}

void PointList::setPoint(Point point2, int i){
    points[i].setX(point2.getX());
    points[i].setY(point2.getY());
    points[i].setZ(point2.getZ());
}


//Insère un point à la position i et décale tous les autres points (le dernier sort de la liste)
/*void PointList::putPoint(Point addPoint, int i){
    if(i>=taille || i<0){
        std::cout << "L'indexe du point a ajouter n'es pas conforme";
    }
    else{
        int j(taille);
        for(j = taille-1; j>i-1 ; j--){
*/
