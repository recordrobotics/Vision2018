#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Timer.hh"

#define FPS                 30
#define FRAME_WIDTH         640
#define FRAME_HEIGHT        480
#define CAPTURE_ID          "/dev/video1"
#define MIN_CONTOUR_AREA    160
#define THRESH_SENS         150
#define HUE_LBOUND          80
#define HUE_UBOUND          91
#define CONNECT_SLEEP_TIME  30000   // us
#define CONNECT_LOG_TIME    1000    // ms
#define CAM_BUF_SIZE        1

#ifndef DEBUG
#define DEBUG
#endif


int main(int argc, char **argv)
{
    Timer::init();

    cv::VideoCapture cap(CAPTURE_ID);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    //cap.set(CV_CAP_PROP_BUFFERSIZE, CAM_BUF_SIZE);
    cap.set(CV_CAP_PROP_FPS, FPS);

    cv::Mat image;

    ms_t startTime = Timer::getMs();
    while(true)
    {
        if(cap.grab() && cap.retrieve(image) && !image.empty())
        {
            cv::Size s = image.size();

            if(s.width == FRAME_WIDTH && s.height == FRAME_HEIGHT)
                break;
        }

        usleep(CONNECT_SLEEP_TIME);

        if((Timer::getMs() - startTime) % CONNECT_LOG_TIME)
            printf("Trying to connect...\n");
    }

    printf("Connected to camera\n");

    cv::Mat result(image.size(), CV_8UC3, cv::Scalar::all(0));

    cv::Scalar lowerb(HUE_LBOUND, 0, 255 - THRESH_SENS);
    cv::Scalar upperb(HUE_UBOUND, THRESH_SENS, 255);

    std::vector<std::vector<cv::Point>> contours;

    ms_t lastTime = Timer::getMs();

    while(cv::waitKey(1) != 27)
    {
        ms_t t1 = Timer::getMs();
        if(cap.grab())
        {
#ifdef DEBUG
            printf("Grab time: %lld ms\n", Timer::getMs() - t1);
#endif
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
                        cv::cvtColor(image, result, CV_BGR2HSV);
                        cv::inRange(result, lowerb, upperb, result);
                        //cv::GaussianBlur(result, result, cv::Size(3, 3), 1.0, 1.2);

                        findContours(result, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

                        ssize_t l = contours.size();
#ifdef DEBUG
                        printf("Initial number of contours: %ld\n", l);
#endif

                        for(ssize_t i = 0; i < l; i++)
                        {
                            if(contourArea(contours[i]) < MIN_CONTOUR_AREA)
                            {
                                contours.erase(contours.begin() + i);
                                i--;
                                l--;
                            }
                        }

                        size_t l2 = contours.size();
                        std::vector<cv::Point2f> centroids(l2);

                        for(size_t i = 0; i < l2; i++)
                        {
                            cv::Moments m = cv::moments(contours[i], false);
                            centroids.push_back(cv::Point2f(m.m10 / m.m00, m.m01 / m.m00));

#ifdef DEBUG
                            cv::circle(image, centroids.back(), 10, cv::Scalar(0, 0, 250), 2);
#endif
                        }

#ifdef DEBUG
                        //drawContours(image, contours, -1, cv::Scalar(0, 0, 250), 1);
                        cv::imshow("Result", image);
#endif
                    }
                    catch(cv::Exception e)
                    {
                        printf("\nException encountered: %s\n", e.what());
                    }
                }
            }
            else
                printf("Could not retrieve image\n");
        }
        else
            printf("Could not grab image\n");

#ifdef DEBUG
        ms_t time = Timer::getMs();
        ms_t dt = time - lastTime;
        lastTime = time;

        printf("Loop time: %lld ms\n", dt);
#endif
    }

    return 0;
}
