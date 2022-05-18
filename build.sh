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
