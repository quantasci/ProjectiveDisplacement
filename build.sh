
cd ..
git clone https://github.com/quantasci/libmin.git
cd libmin
git pull
cd ..

cmake -S libmin -B build/libmin -DBUILD_OPENGL=TRUE -DBUILD_GLEW=TRUE
cd build/libmin
make

cd ../../ProjectiveDisplacement
git pull
cd ..

cmake -S ProjectiveDisplacement -B build/pdm -DLIBMIN_INSTALL=build/libmin -DBUILD_OPENGL=TRUE -DBUILD_GLEW=TRUE
cd build/pdm
make 

