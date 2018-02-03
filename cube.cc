#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
//#include <ntcore.h>
//#include <networktables/NetworkTable.h>
#include <opencv2/opencv.hpp>
#include "Timer.hh"

#define FPS                 30
#define FRAME_WIDTH         320
#define FRAME_HEIGHT        240
#define CAPTURE_PATH        0
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

#define BUFSZ               1024

#define DEBUG
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

    printf("%d %d\n", src.step, dest.step);

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            const uint8_t *src_pixel = &sdata[3 * (i * src.step + j)];
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

            if(i == (rows / 2) && j == (cols / 2))
                printf("Center color (HSV): (%d, %d, %d)\n", src_pixel[0], src_pixel[1], src_pixel[2]);
        }
    }
}

void setProps(cv::VideoCapture *cap)
{
    cap->set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap->set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap->set(CV_CAP_PROP_FPS, FPS);
}

void connect(cv::VideoCapture *cap, cv::Mat *image)
{
    int path = 0;

    cap->open(path);
    setProps(cap);

    ms_t logTime = Timer::getMs();
    while(true)
    {
        if(/*exists(path) &&*/ cap->grab() && cap->retrieve(*image) && !image->empty())
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

        if(++path > MAX_CAPTURE_PATH)
            path = 0;

        cap->open(path);
        setProps(cap);
    }

    printf("Connected to camera\n");
}

int main(int argc, char **argv)
{
    Timer::init();

    cv::VideoCapture cap(CAPTURE_PATH);
    setProps(&cap);

    cv::Mat image;

    connect(&cap, &image);

    cv::Mat hsv_image(image.size(), CV_8UC3, cv::Scalar::all(0));
    cv::Mat result(image.size(), CV_8UC1, cv::Scalar::all(0));
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

                        //cv::GaussianBlur(temp, temp, cv::Size(7, 7), 1.0, 1.2);

                        findContours(temp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

                        ssize_t opt_idx = -1;
                        double m_a = 0.0;
                        size_t l = contours.size();
                        for(size_t i = 0; i < l; i++)
                            if(contourArea(contours[i]) > m_a)
                                opt_idx = i;

                        if(l < 1 || opt_idx == -1)
                            printf("Too few countours were found\n");
                        else
                        {
                            cv::Moments m = cv::moments(contours[opt_idx]);
                            cv::Point centroid(m.m10 / m.m00, m.m01 / m.m00);

                            //printf("Opt: %d %d\n", centroid.x, centroid.y);

#ifdef DEBUG
                            cv::circle(image, centroid, 10, cv::Scalar(0, 0, 250), 2);

                            cv::circle(temp, cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 10, cv::Scalar(250), 2);
                        }


                        //cv::imshow("Image", hsv_image);
                        cv::imshow("Mask", temp);
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
                connect(&cap, &image);
            }
        }
        else {
            printf("Could not grab image\n");
            connect(&cap, &image);
        }

#ifdef LOOP_TIME
        ms_t time = Timer::getMs();
        ms_t dt = time - lastTime;
        lastTime = time;

        printf("Loop time: %lld ms\n", dt);
#endif
    }

    return 0;
}
