CC = gcc
CXX = g++

CFLAGS = -Wall -O3 -I.
CXXFLAGS = -Wall -O3 -I. `pkg-config --cflags opencv4`
LIBS = -lm -ltiff `pkg-config --libs opencv4`

TARGET = mlxget
OBJS = tool.o MLX90640_API.o MLX90640_I2C_Driver.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LIBS)

MLX90640_API.o: MLX90640_API.c
	$(CC) $(CFLAGS) -c MLX90640_API.c

MLX90640_I2C_Driver.o: MLX90640_I2C_Driver.c
	$(CC) $(CFLAGS) -c MLX90640_I2C_Driver.c

tool.o: tool.cpp
	$(CXX) $(CXXFLAGS) -c tool.cpp

clean:
	rm -f *.o $(TARGET)
