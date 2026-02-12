### mlx90640 jetson driver

this is a c++ driver for the mlx90640 thermal sensor on jetson nano. it uses opencv for live display and libtiff to save high-precision temperature data.

#### setup

1. install dependencies: sudo apt-get install libopencv-dev libtiff-dev libi2c-dev
2. make

#### usage

./mlxget -l : live view only (no saving)

./mlxget -s 10 : record raw 32-bit float tiffs for 10 seconds

./mlxget -s 30 -c : record colorized 8-bit tiffs for 30 seconds

./mlxget : continuous capture until ctrl+c
