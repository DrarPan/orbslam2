echo "Configuring and building Thirdparty/DBoW2 ..."
cd DBoW2
if [ ! -d "./build" ]; then
 mkdir build
fi

cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

if [ ! -d "./build" ]; then
 mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j