#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <python2.7/Python.h>
//#include <ntcore.h>
//#include <networktables/NetworkTable.h>
#include <opencv2/opencv.hpp>
#include "Timer.hh"

#define FPS                 30
#define FRAME_WIDTH         320
#define FRAME_HEIGHT        240
#define CAMERA_PATH         0
#define MAX_CAPTURE_PATH    10
#define MIN_CONTOUR_AREA    50
#define MIN_CONTOUR_FRAC    0.0
#define EPS                 0.04
#define HUE_LBOUND          30
#define HUE_UBOUND          40
#define SAT_LBOUND          90
#define SAT_UBOUND          190
#define VALUE_LBOUND        150
#define VALUE_UBOUND        255
#define CONNECT_SLEEP_TIME  30000   // us
#define CONNECT_LOG_TIME    1000    // ms

#define X_ENTRY             "camera_x"
#define Y_ENTRY             "camera_y"
#define ROBORIO_IP          "10.67.31.2"
#define TABLE_NAME          "SmartDashboard"

//#define DEBUG
//#define LOOP_TIME

// Matrices must be dense and differently allocated
void applyKernel(cv::Mat& dest, const cv::Mat& src)
{
    if(src.data == dest.data) {
        printf("Matrices must be different\n");
        return;
    }

    if(src.size != dest.size) {
        printf("Matrices must be the same size\n");
        return;
    }

    dest = cv::Scalar::all(0);
    int rows = src.rows;
    int cols = src.cols;
    const uint8_t *sdata = (uint8_t*)src.data;
    uint8_t *ddata = (uint8_t*)dest.data;

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            const uint8_t *src_pixel = &sdata[i * src.step + j * 3];
            uint8_t *dest_pixel = &ddata[i * dest.step + j];

            if(src_pixel[0] >= HUE_LBOUND && src_pixel[0] <= HUE_UBOUND &&
               src_pixel[1] >= SAT_LBOUND && src_pixel[1] <= SAT_UBOUND &&
               src_pixel[2] >= VALUE_LBOUND && src_pixel[2] <= VALUE_UBOUND)
            {
                dest_pixel[0] = 255;
            }
            else
            {
                dest_pixel[0] = 0;
            }
        }
    }
}

void setProps(cv::VideoCapture *cap)
{
    cap->set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap->set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap->set(CV_CAP_PROP_FPS, FPS);
}

void connect(cv::VideoCapture *cap, cv::Mat *image, PyObject **send_func, PyObject *init_func, PyObject *gettable_func)
{
	PyObject *init_args = PyTuple_New(1);
	PyObject *ivalue = PyString_FromString(ROBORIO_IP);
	PyTuple_SetItem(init_args, 0, ivalue);

	PyObject *gettable_args = PyTuple_New(1);
	PyObject *tvalue = PyString_FromString(TABLE_NAME);
	PyTuple_SetItem(gettable_args, 0, tvalue);

    ms_t logTime = Timer::getMs();
    while(true)
    {
        cap->open(CAMERA_PATH);
        setProps(cap);

		if(!*send_func || PyCallable_Check(*send_func)) {
			Py_XDECREF(*send_func);
			*send_func = NULL;
			ivalue = PyObject_CallObject(init_func, init_args);
    		tvalue = PyObject_CallObject(gettable_func, gettable_args);

			if(tvalue)
				*send_func = PyObject_GetAttrString(tvalue, "putNumber");
		}

		// Check if ivalue and tvalue are correct
        if(cap->grab() && cap->retrieve(*image) && !image->empty())
        {
            cv::Size s = image->size();

            if(s.width == FRAME_WIDTH && s.height == FRAME_HEIGHT)
                break;
        }

		usleep(CONNECT_SLEEP_TIME);

        ms_t time = Timer::getMs();

        if((time - logTime) > CONNECT_LOG_TIME) {
            printf("Trying to connect...\n");
            logTime = time;
        }
	}

	Py_DECREF(init_args);
	Py_DECREF(gettable_args);

    printf("Connected to camera\n");
}

bool initpy(PyObject **init_func, PyObject **gettable_func)
{
	Py_Initialize();
	PyObject *module_name = PyString_FromString("networktables");
	PyObject *module = PyImport_Import(module_name);
	Py_DECREF(module_name);

	if(!module) {
		PyErr_Print();
		printf("PYTHON ERROR: Could not import networktables!\n");
		return false;
	}

	*init_func = PyObject_GetAttrString(module, "initialize");
	if(!*init_func || !PyCallable_Check(*init_func)) {
		if(PyErr_Occurred())
			PyErr_Print();
        printf("PYTHON ERROR: Could not get initialize function!\n");
		Py_DECREF(module);
        return false;
    }

	*gettable_func = PyObject_GetAttrString(module, "getTable");
    if(!*gettable_func || !PyCallable_Check(*gettable_func)) {
		if(PyErr_Occurred())
            PyErr_Print();
        printf("PYTHON ERROR: Could not get getTable function!\n");
        Py_DECREF(module);
		Py_DECREF(*init_func);
		return false;
    }

	return true;
}

int main(int argc, char **argv)
{
    Timer::init();

    cv::VideoCapture cap(CAMERA_PATH);
    setProps(&cap);

    cv::Mat image;

	PyObject *send_x_args = NULL, *send_y_args = NULL, *send_func = NULL, *init_func = NULL, *gettable_func = NULL, *s = NULL;
	initpy(&init_func, &gettable_func);
	send_x_args = PyTuple_New(2);
	send_y_args = PyTuple_New(2);
	s = PyString_FromString(X_ENTRY);
	if(s && send_x_args)
		PyTuple_SetItem(send_x_args, 0, s);
	s = PyString_FromString(Y_ENTRY);
	if(s && send_y_args)
		PyTuple_SetItem(send_y_args, 0, s);

	connect(&cap, &image, &send_func, init_func, gettable_func);

    cv::Mat hsv_image(image.size(), CV_8UC3, cv::Scalar::all(0));
    cv::Mat temp(image.size(), CV_8UC1, cv::Scalar::all(0));

    std::vector<std::vector<cv::Point>> contours;

#ifdef LOOP_TIME
    ms_t lastTime = Timer::getMs();
#endif

#ifdef DEBUG
    while(cv::waitKey(1) != 27)
    {
#else
    while(true)
    {
#endif
        if(cap.grab())
        {
            if(cap.retrieve(image))
            {
                cv::Size s = image.size();

                if(image.empty())
                    printf("Retrieved image in empty\n");
                else if(s.width != FRAME_WIDTH || s.height != FRAME_HEIGHT)
                    printf("Retrieved image size is incorrect: %d, %d instead of %d, %d\n", s.width, s.height, FRAME_WIDTH, FRAME_HEIGHT);
                else
                {
                    try
                    {
                        cv::cvtColor(image, hsv_image, CV_BGR2HSV);

                        applyKernel(temp, hsv_image);

                        cv::GaussianBlur(temp, temp, cv::Size(3, 3), 1.0, 1.2);

#ifdef DEBUG
                        cv::imshow("Mask", temp);
#endif

                        findContours(temp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

                        ssize_t opt_idx = -1;
                        double m_a = 0.0, a = 0.0;
                        size_t l = contours.size();
                        for(size_t i = 0; i < l; i++) {
                            if((a = contourArea(contours[i])) > m_a) {
                                m_a = a;
                                opt_idx = i;
                            }
                        }

                        if(l < 1 || opt_idx == -1)
                            printf("Too few countours were found\n");
                        else
                        {
                            cv::Moments m = cv::moments(contours[opt_idx]);
                            cv::Point centroid(m.m10 / m.m00, m.m01 / m.m00);

							if(send_x_args) {
								PyObject *val = PyInt_FromLong(centroid.x);

								if(val) {
									PyTuple_SetItem(send_x_args, 1, val);
									if(send_x_args) {
										PyObject_CallObject(send_func, send_x_args);
										// Handle output of call
									}
								}
								else
									printf("PYTHON ERROR: Could not create int from long for centroid.x\n");
							}
							else
								printf("PYTHON ERROR: send_x_args is NULL in main loop\n");

							if(send_y_args) {
                                PyObject *val = PyInt_FromLong(centroid.y);

                                if(val) {
									PyTuple_SetItem(send_y_args, 1, val);
                                    if(send_y_args) {
										PyObject_CallObject(send_func, send_y_args);
                                    	// Handle output of call
                                	}
								}
                                else
                                    printf("PYTHON ERROR: Could not create int from long for centroid.y\n");
                            }
							else
								printf("PYTHON ERROR: send_y_args is NULL in main loop\n");

#ifdef DEBUG
                            cv::circle(image, centroid, 10, cv::Scalar(0, 0, 250), 2);
                        }

                        //cv::circle(temp, cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 10, cv::Scalar(250), 2);
                        //cv::imshow("Image", hsv_image);
                        cv::imshow("Original", image);
						//cv::imshow("Edges", edges);
#else
                        }
#endif
                    }
                    catch(cv::Exception e)
                    {
                        printf("\nException encountered: %s\n", e.what());
                    }
                }
            }
            else {
                printf("Could not retrieve image\n");
                connect(&cap, &image, &send_func, init_func, gettable_func);
            }
        }
        else {
            printf("Could not grab image\n");
            connect(&cap, &image, &send_func, init_func, gettable_func);
        }

#ifdef LOOP_TIME
        ms_t time = Timer::getMs();
        ms_t dt = time - lastTime;
        lastTime = time;

        printf("Loop time: %lld ms\n", dt);
#endif
    }

	Py_XDECREF(send_x_args);
	Py_XDECREF(send_y_args);
	Py_XDECREF(send_func);
	Py_XDECREF(init_func);
	Py_XDECREF(gettable_func);

	Py_Finalize();

    return 0;
}
