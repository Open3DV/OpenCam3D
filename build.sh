cd version
./build.sh

cd ../sdk
rm -r build
mkdir build
cd build 
cmake ..
make
cp libopen_cam3d_sdk.so ../../example

cd ../../example
rm -r build
mkdir build
cd build 
cmake ..
make

cd ../../cmd
rm -r build
mkdir build
cd build 
cmake ..
make

cd ../../calibration
rm -r build
mkdir build
cd build 
cmake ..
make

cd ../../test
rm -r build
mkdir build
cd build 
cmake ..
make

cd ../../gui
rm -r build
mkdir build
cd build 
cmake ..
make
cp ../../tools/pack.sh ./bin
cp ../../tools/open_cam3d_gui.sh ./bin
cd ./bin
./pack.sh
rm pack.sh
rm open_cam3d_gui.sh

cd ../../../
mkdir release_camera
cd release_camera
cp ../gui/build/bin -r ./
cp ../cmd/build/open_cam3d ./bin/
cp ../calibration/build/calibration ./bin/
cp ../example/build/example ./bin/
cp ../test/build/open_cam3d_test ./bin/



