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
        composante = NULL;
        int b(0);
        int compteur(0);
        double* newComposante(NULL);
        int i(0);
        while(fluxVector.get(a)){
            if(a>47 && a<58){
                b=10*b+(a-48);
            }
            else if(a==32){
                compteur++;
                newComposante=composante;
                composante = (double*)malloc(compteur*sizeof(double));
                for(i=0;i<compteur-1;i++){
                    composante[i]=newComposante[i];
                }
                composante[compteur-1]=b;
                free(newComposante);
                b=0;
            }
            else{
                compteur++;
                newComposante=composante;
                composante = (double*)malloc(compteur*sizeof(double));
                for(i=0;i<compteur-1;i++){
                    composante[i]=newComposante[i];
                }
                composante[compteur-1]=b;
                free(newComposante);
                b=0;
            }
        }
        taille=compteur;
    }
    else{
        std::cout << "Impossible de créer une liste de Point a partir de " << file;
        taille = 0;
        points = NULL;
    }
}


pointList::~pointList(){
    free(points);
}


