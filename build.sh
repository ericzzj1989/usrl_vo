echo "Configuring and building Thirdparty/g2o ..."

cd Thirdparty/g2o

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../../

echo "Configuring and building usrl_vo ..."

mkdir build
cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
cmake ..
make -j