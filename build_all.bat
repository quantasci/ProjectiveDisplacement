@echo off

echo ### This script requires Git, Cmake and Visual Studio to be install before proceeding.
echo ### Stop and edit this .bat file to set your own Visual Studio path (if not using VS2019)
echo.
path=%path%;C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin

echo ### Cloning or updating \libmin repository
cd ..
git clone https://github.com/quantasci/libmin.git
cd libmin
git pull
cd ..

echo ### Compiling libmin to \build\libmin
@echo on
cmake -S libmin -B build/libmin -DBUILD_OPENGL=TRUE -DBUILD_GLEW=TRUE
cd build/libmin
msbuild libmin.sln /p:Configuration=Debug
msbuild libmin.sln /p:Configuration=Release
@echo off

echo ### Cloning or updating \ProjectiveDisplacement repository
echo.
cd ../../ProjectiveDisplacement
git pull
cd ..

echo ### Compiling ProjectiveDisplacement to \build\pdm
echo.

@echo on
cmake -S ProjectiveDisplacement -B build/pdm -DLIBMIN_INSTALL=build/libmin -DBUILD_OPENGL=TRUE -DBUILD_GLEW=TRUE
cd build/pdm
msbuild proj_displace_mesh.sln /p:Configuration=Debug
msbuild proj_displace_mesh.sln /p:Configuration=Release
@echo off

echo ######## DONE
echo ### Update and build complete.
echo ### Your current directory is now \build\pdm
echo ### Type proj_displace_meshd.exe to run it!
echo.



