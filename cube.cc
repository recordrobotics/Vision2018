#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
//#include <ntcore.h>
//#include <networktables/NetworkTable.h>
#include <opencv2/opencv.hpp>
#include "Timer.hh"

#define FPS                 30
#define FRAME_WIDTH         640
#define FRAME_HEIGHT        480
#define CAPTURE_PATH        "/dev/video1"
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

#ifndef DEBUG
#define DEBUG
#endif

// Matrices must be dense and differently allocated
void applyKernel(cv::Mat& dest, const cv::Mat& src, void (*kernel)(uint8_t *dest, const uint8_t *src))
{
    if(src.data == dest.data) {
        printf("Matrices must be different\n");
        return;
    }

    dest = cv::Scalar::all(0);

    int rows = src.rows;
    int cols = src.cols;
    int schannels = src.channels();
    int dchannels = dest.channels();
    const uint8_t *sdata = (uint8_t*)src.data;
    uint8_t *ddata = (uint8_t*)dest.data;

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            kernel(&ddata[dchannels * (i * cols + j)], &sdata[schannels * (i * cols + j)]);

            if(i == (rows / 2) && j == (cols / 2))
                printf("(%d, %d, %d)\n", sdata[schannels * (i * cols + j)], sdata[schannels * (i * cols + j) + 1], sdata[schannels * (i * cols + j) + 2]);
        }
    }
}

void grayConvolution(cv::Mat& dest, const cv::Mat& src, const cv::Size& s)
{
    if(src.data == dest.data) {
        printf("Matrices must be different\n");
        return;
    }

    if(s.width % 2 != 1 || s.height % 2 != 1) {
        printf("Size must be odd numbers\n");
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
            bool invalid = false;

            for(int i2 = 0; i2 < s.width; i2++)
            {
                for(int j2 = 0; j2 < s.height; j2++)
                {
                    int idx = i * cols + j - cols * (s.height / 2 - j2) + i2 - s.width / 2;

                    if(idx > -1 && sdata[idx] == 0)
                    {
                        invalid = true;
                        break;
                    }
                }

                if(invalid)
                    break;
            }

            if(!invalid)
                ddata[i * cols + j] = sdata[i * cols + j];
        }
    }
}

void HSVKernel(uint8_t *dest_pixel, const uint8_t *src_pixel)
{
    if(src_pixel[0] >= HUE_LBOUND && src_pixel[0] <= HUE_UBOUND &&
       src_pixel[1] >= SAT_LBOUND && src_pixel[1] <= SAT_UBOUND &&
       src_pixel[2] >= VALUE_LBOUND && src_pixel[2] <= VALUE_UBOUND)
    {
        dest_pixel[0] = 255; //src_pixel[0] - 0.4 * (src_pixel[1] - src_pixel[2]);
        //dest_pixel[1] = src_pixel[1];
        //dest_pixel[2] = src_pixel[2];
    }
    else
        dest_pixel[0] = 0;
}

bool exists(const char *name)
{
    struct stat buffer;
    return (stat(name, &buffer) == 0);
}

void connect(cv::VideoCapture *cap, cv::Mat *image)
{
    cap->open(CAPTURE_PATH);

    ms_t logTime = Timer::getMs();
    while(true)
    {
        if(exists(CAPTURE_PATH) && cap->grab() && cap->retrieve(*image) && !image->empty())
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

        cap->open(CAPTURE_PATH);
    }

    printf("Connected to camera\n");
}

typedef struct contour {
    std::vector<cv::Point> points;
    double area;
    cv::Rect bbox;
    cv::Point centroid;
} contour_t;

bool contour_compare(const contour_t& a, const contour_t& b)
{
    return (a.area < b.area);
}

int main(int argc, char **argv)
{
    Timer::init();

    cv::VideoCapture cap(CAPTURE_PATH);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(CV_CAP_PROP_FPS, FPS);

    cv::Mat image;

    connect(&cap, &image);

    cv::Mat hsv_image(image.size(), CV_8UC3, cv::Scalar::all(0));
    cv::Mat result(image.size(), CV_8UC1, cv::Scalar::all(0));
    cv::Mat temp(image.size(), CV_8UC1, cv::Scalar::all(0));
	cv::Mat edges(image.size(), CV_8UC1, cv::Scalar::all(0));
//    cv::Mat color_mask(image.size(), CV_8UC3, cv::Scalar::all(0));

//    cv::Scalar lowerb(HUE_LBOUND, 0, 255 - THRESH_SENS);
//    cv::Scalar upperb(HUE_UBOUND, THRESH_SENS, 255);

    std::vector<contour_t> contours;
    std::vector<std::vector<cv::Point>> contours_temp;

    ms_t lastTime = Timer::getMs();

#ifdef DEBUG
    while(cv::waitKey(1) != 27)
    {
        ms_t t1 = Timer::getMs();
#else
    while(true)
    {
#endif
        if(cap.grab())
        {
#ifdef DEBUG
            //printf("Grab time: %lld ms\n", Timer::getMs() - t1);
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
                        cv::cvtColor(image, hsv_image, CV_BGR2HSV);
                        //cv::adaptiveThreshold(result, result, 200, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 1);
                        //cv::inRange(result, lowerb, upperb, result);
                        //image.copyTo(color_mask, result);

                        applyKernel(temp, hsv_image, HSVKernel);

                        cv::GaussianBlur(temp, temp, cv::Size(7, 7), 1.0, 1.2);

//                        grayConvolution(result, temp, cv::Size(3, 5));

//                        cv::GaussianBlur(result, result, cv::Size(5, 9), 1.0, 1.2);

                        //cv::Canny(result, result, 50, 60);

                        //cv::cvtColor(result, result, CV_HSV2BGR);

                        findContours(temp, contours_temp, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

                        ssize_t l = contours_temp.size();
#ifdef DEBUG
                        //printf("Initial number of contours: %ld\n", l);
#endif

                        contours.clear();

                        for(ssize_t i = 0; i < l; i++)
                            contours.push_back({ contours_temp[i], cv::contourArea(contours_temp[i]), cv::boundingRect(contours_temp[i]), cv::Point(0, 0) });

                        std::sort(contours.begin(), contours.end(), contour_compare);

                        for(ssize_t i = 0; i < l * MIN_CONTOUR_FRAC; i++)
                            contours.erase(contours.begin());

                        if(contours.size() && contours[0].area < MIN_CONTOUR_AREA)
                        {
                            ssize_t ll = contours.size();

                            for(ssize_t i = 0; i < ll; i++)
                            {
                                if(contours[0].area < MIN_CONTOUR_AREA)
                                    contours.erase(contours.begin());
                                else
                                    break;
                            }
                        }

                        size_t l2 = contours.size();

                        if(l2 < 1) {
                            //drawContours(image, contours, -1, cv::Scalar(0, 0, 250), 1);
                            printf("Too few countours were found\n");
                        }
                        else
                        {
                            for(size_t i = 0; i < l2; i++)
                            {
                                cv::Moments m = cv::moments(contours[i].points, false);
                                contours[i].centroid.x = m.m10 / m.m00;
                                contours[i].centroid.y = m.m01 / m.m00;
                            }

                            size_t opt = l2 - 1;

                            for(size_t i = 0; i < l2; i++)
                            {

                            }

#ifdef DEBUG
                            cv::circle(image, contours[opt].centroid, 10, cv::Scalar(0, 0, 250), 2);

                            cv::circle(hsv_image, cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 10, cv::Scalar(0, 250, 0), 2);
                        }


						cv::Canny(image, edges, 50, 200);

                        //cv::imshow("Image", hsv_image);
                        cv::imshow("Mask", temp);
                        //cv::imshow("Original", image);
						cv::imshow("Edges", edges);

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
            else
                printf("Could not retrieve image\n");
        }
        else
            printf("Could not grab image\n");

        if(!exists(CAPTURE_PATH))
            connect(&cap, &image);

        ms_t time = Timer::getMs();
        ms_t dt = time - lastTime;
        lastTime = time;

        //printf("Loop time: %lld ms\n", dt);
    }

    return 0;
}
