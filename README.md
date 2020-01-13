# ProjetModelSurf

Création des Makefile nécessaires à la compilation du projet :

cd ProjetModelSurf/
mkdir build
cd build/
cmake ..


Compilation du projet :

make


Execution du projet :

-Creation de Surface : 


    ./build/tests/test_Mesh fichierEntree.off fichierSortie2.obj 

            crée la surface présente dans fichierEntree.off dans fichierSortie.obj. 

ou

    ./build/tests/test_Mesh fichierEntree.off


            crée la surface présente dans fichier.off dans ./build/test_implfunc.obj.

           Différents fichiers .off sont disponibles dans ./models/


Une interface permet de visualiser le maillage obtenue : 

cd InterfaceGraphique/
make
./viewer/myViewer

On peut ensuite charger les différents modèles que l'on veut.

Les objets .obj peuvent aussi être ouvert à partir de MeshLab.
