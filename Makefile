CXX := g++ -std=c++11
CFLAGS := -Wall
INCLUDES := -I/usr/local/include/wpilibc
LDFLAGS := -lm -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching  -lopencv_flann -lopencv_photo -lopencv_superres -lopencv_video -lopencv_videostab -lpthread

test: test.cc Timer.cc
	$(CXX) $(CFLAGS) $(INCLUDES) test.cc Timer.cc -o $@ $(LDFLAGS)

cube: cube.cc Timer.cc
	$(CXX) $(CFLAGS) $(INCLUDES) cube.cc Timer.cc -o $@ $(LDFLAGS)
