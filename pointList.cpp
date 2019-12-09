#include <exception>
#include <string>
#include <iostream>
#include "pointList.h"
#include <cstdlib>
#include <fstream>


pointList::pointList(){ 
    points = NULL;
    taille = 0;
}

pointList::pointList(int taille2){
    if(taille2 < 0){
        std::cout << "Impossible de créer une liste de point de longueur negative\n";
        return;
    }
    else if(taille2==0){
        points = NULL;
    }
    else{
        points = new Point[taille2];
    }
    taille = taille2;
}

pointList::pointList(pointList const& copie){
    taille = copie.taille;
    points = new Point[taille];
    for(int i=0; i<taille; i++){
        points[i] = copie.points[i];
    }
}

pointList::pointList(std::string file){
    std::ifstream fluxPoint(file.c_str());
    if(fluxPoint){
        char a;
        bool firstLine(true), flag(true), lecNb(false);
        double valeur(0.);
        int composante(0), compteurDec(1), compteurPoint(0), negativite(1);
        bool decimal(false);

        while(firstLine && flag){
            fluxPoint.get(a);
            if(a == 10){
                firstLine = false;
            }
            else if(a>47 && a<58){
                taille = taille * 10 + (a-48);
            }
            else{
                std::cout << "Le fichier du nuage de points n'est pas au bon format\n";
                taille = 0;
                points = NULL;
                return ;
            }
        }
        points = new Point[taille];
        while(fluxPoint.get(a) && flag){
            if(a>47 && a<58){
                lecNb = true;
                if(decimal){
                    compteurDec = compteurDec*10;
                }
                valeur = valeur*10 + (a-48);
            }
            else if(a == 32){
                if(lecNb){
                    if(composante == 0){
                        points[compteurPoint].setX(negativite * valeur/compteurDec);
                    }
                    else if(composante == 1){
                        points[compteurPoint].setY(negativite * valeur/compteurDec);
                    }
                    else{
                        std::cout << "Il y a trop d'argument sur la ligne " << compteurPoint << "\n";
                        return ;
                    }
                    composante = (composante + 1);
                    decimal = false;
                    compteurDec = 1;
                    valeur = 0.;
                    negativite = 1;
                }
                lecNb = false;
            }
            else if(a == 10){
                points[compteurPoint].setZ(negativite * valeur/compteurDec);
                composante = 0;
                valeur = 0.;
                decimal = false;
                compteurDec = 1;
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

pointList::~pointList(){
    free(points);
}

void pointList::display(std::ostream& str){
    int i;
    for(i=0; i<taille; i++){
        points[i].display(str);
    }
}
