To run the program:

Open your terminal, go to the directory where the executable file stays: cd: <WORKING DIR> For example: `cd /Users/usr/Downloads/`

run `./voxelizer pathToInput.obj pathtoOutput.obj` For instance: `./voxelizer /usr/Downloads/sphere.obj /usr/Downloads/sphere64.obj`

For compiling:

from source folder run: `g++ -std=c++11 main.cpp Mesh.cpp CompFab.cpp -o a1`