CXX := g++ -std=c++11
CFLAGS := -Wall
LDFLAGS := -lm -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_dnn -lopencv_flann -lopencv_imgcodecs -lopencv_photo -lopencv_shape -lopencv_superres -lopencv_videoio -lopencv_video -lopencv_videostab

test: test.cc Timer.cc
	$(CXX) $(CFLAGS) test.cc Timer.cc -o $@ $(LDFLAGS)