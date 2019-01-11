#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <linux/if_packet.h>
#include <unistd.h>
#include <vector>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "Timer.hh"
#include "pack2.hh"

#define FPS                 30
#define FRAME_WIDTH         176
#define FRAME_HEIGHT        144
#define STREAM_FRAME_WIDTH  160
#define STREAM_FRAME_HEIGHT 120
#define CAMERA_MAX_PATH     10
#define MAX_CAPTURE_PATH    10
#define MIN_CONTOUR_AREA    50
#define MIN_CONTOUR_FRAC    0.0
#define EPS                 0.04

// These are the values for the cube

#define C_HUE_LBOUND          19
#define C_HUE_UBOUND          41
#define C_SAT_LBOUND          159
#define C_SAT_UBOUND          256
#define C_VALUE_LBOUND        119
#define C_VALUE_UBOUND        256

// These are the values for the retroreflective tape

#define T_HUE_LBOUND          69
#define T_HUE_UBOUND          101
#define T_SAT_LBOUND          9
#define T_SAT_UBOUND          141
#define T_VALUE_LBOUND        239
#define T_VALUE_UBOUND        256

#define CONNECT_SLEEP_TIME  30000   // us
#define CONNECT_LOG_TIME    1000    // ms
#define SEND_SIG            "q{A#;"
#define SEND_MODE_SIG       "j@9."
#define ROBORIO_DEFAULT_IP  "10.67.31.2"
#define ROBORIO_MDNS        "roboRIO-6731-FRC.local"
#define ROBORIO_PORT        1024
#define LOCAL_PORT          1024
#define DEST_PORT           1024
#define RECEIVE_TIMEOUT_S   0
#define RECEIVE_TIMEOUT_US  10000

typedef uint32_t ip_t;

typedef struct threshold {
	int16_t hlb, hub, slb, sub, vlb, vub;
} threshold_t;

enum E_SEARCH_MODE {
	ESM_CUBE = 0,
	ESM_TAPE,
	ESM_COUNT
};

volatile bool should_exit;

void exit_handler(int signum)
{
	should_exit = true;
}

// Matrices must be dense and differently allocated
void applyKernel(cv::Mat& dest, const cv::Mat& src, threshold_t *thresh)
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

    //printf("%d %d\n", src.rows, dest.cols);

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            const uint8_t *src_pixel = &sdata[i * src.step + j * 3];
            uint8_t *dest_pixel = &ddata[i * dest.step + j];

            if(src_pixel[0] > thresh->hlb && src_pixel[0] < thresh->hub &&
               src_pixel[1] >= thresh->slb && src_pixel[1] <= thresh->sub &&
               src_pixel[2] >= thresh->vlb && src_pixel[2] <= thresh->vub)
            {
                dest_pixel[0] = 255;
                //dest_pixel[1] = 255;
                //dest_pixel[2] = 255;
				//printf("%d %d\n", cols, rows);
				//bottom_curve[j] = i;
            }
            else
            {
                dest_pixel[0] = 0;
                //dest_pixel[1] = 0;
                //dest_pixel[2] = 0;
            }

            //if(i == (rows / 2) && j == (cols / 2))
            //    printf("Center color (HSV): (%d, %d, %d)\n", src_pixel[0], src_pixel[1], src_pixel[2]);
        }
    }
}

void setProps(cv::VideoCapture *cap)
{
    cap->set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap->set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap->set(CV_CAP_PROP_FPS, FPS);
}

void getRoborioIP(struct sockaddr_in *dest_addr)
{
	//FILE *p = popen("getent hosts " ROBORIO_MDNS, "r");
	FILE *p = NULL;

	if(!p) {
		printf("Could not create getent process, falling back to default roborio ip '%s'\n", ROBORIO_DEFAULT_IP);
		inet_pton(AF_INET, ROBORIO_DEFAULT_IP, &(dest_addr->sin_addr));
	}
	else {
	    const int bufsz = 128;
	    char buf[bufsz];
	    size_t r_l = fread(buf, 1, bufsz, p);
	    buf[r_l] = '\0';
	    char *temp = strtok(buf, " ");
	    if(!temp || inet_pton(AF_INET, temp, &(dest_addr->sin_addr)) != 1) {
		    printf("Could not get a valid roboRIO ip, falling back to default roborio ip '%s'\n", ROBORIO_DEFAULT_IP);
		    inet_pton(AF_INET, ROBORIO_DEFAULT_IP, &(dest_addr->sin_addr));
	    }

        pclose(p);

        printf("Roborio ip: %s\n", temp);
	}
}	

void connect(cv::VideoCapture *cap, cv::Mat *image)
{
	int path = 0;
    cap->open(path);
    setProps(cap);

    ms_t logTime = Timer::getMs();
    while(true)
    {
		if(should_exit)
			return;

        if(cap->grab())
		{
		 	if(cap->retrieve(*image))
			{
				if(!image->empty())
				{
            		cv::Size s = image->size();

            		if(s.width == FRAME_WIDTH && s.height == FRAME_HEIGHT)
						break;
					else
						printf("Retrieved image is wrong size: (%d, %d) instead of (%d, %d)\n", s.width, s.height, FRAME_WIDTH, FRAME_HEIGHT);
				}
				else
					printf("Retrieved image is empty\n");
			}
			else
				printf("Could not retrieve image\n");
		}
		else
			printf("Could not grab image\n");

        usleep(CONNECT_SLEEP_TIME);

        ms_t time = Timer::getMs();

        if((time - logTime) > CONNECT_LOG_TIME) {
            printf("Trying to connect...\n");
            logTime = time;
        }

//		if(++path == CAMERA_MAX_PATH)
//			path = 0;

        cap->open(path);
        setProps(cap);
    }

    printf("Connected to camera\n");
}

threshold_t cube_thresh;
threshold_t tape_thresh;

threshold_t *curr_thresh;

int sock;
E_SEARCH_MODE mode;

void *recvLoop(void *args)
{
    pthread_mutex_lock(&mutex);
	thread_running = true;
	
	size_t sig_len = strlen(SEND_MODE_SIG);
	unsigned char buf[sig_len + 1];

	struct sockaddr_in remote_addr;
	socklen_t addr_len = sizeof(remote_addr);

	uint8_t m;

    bool done = thread_should_close;
    pthread_mutex_unlock(&mutex);

	while(!done) {
        pthread_mutex_lock(&mutex);
		ssize_t r = recvfrom(sock, buf, sig_len + 1, 0, (struct sockaddr*)&remote_addr, &addr_len);
        pthread_mutex_unlock(&mutex);

		unsigned short port = ntohs(remote_addr.sin_port);

		if((size_t)r == (sig_len + 1) && strncmp((char*)buf, SEND_MODE_SIG, sig_len) && port == ROBORIO_PORT) {
			unpack(buf + sig_len, "C", &m);

			printf("%d mode received\n", m);

            pthread_mutex_lock(&mutex);
			mode = (E_SEARCH_MODE)m;
            pthread_mutex_unlock(&mutex);
		}

        pthread_mutex_lock(&mutex);
        done = thread_should_close;
        pthread_mutex_unlock(&mutex);
	}

    pthread_mutex_lock(&mutex);
	thread_running = false;
    pthread_mutex_unlock(&mutex);

	pthread_exit(NULL);
}

int main(int argc, char **argv)
{
	should_exit = false;

	signal(SIGINT, exit_handler);

	cube_thresh.hlb = C_HUE_LBOUND;
	cube_thresh.hub = C_HUE_UBOUND;
	cube_thresh.slb = C_SAT_LBOUND;
	cube_thresh.sub = C_SAT_UBOUND;
	cube_thresh.vlb = C_VALUE_LBOUND;
	cube_thresh.vub = C_VALUE_UBOUND;

	tape_thresh.hlb = T_HUE_LBOUND;
	tape_thresh.hub = T_HUE_UBOUND;
	tape_thresh.slb = T_SAT_LBOUND;
	tape_thresh.sub = T_SAT_UBOUND;
	tape_thresh.vlb = T_VALUE_LBOUND;
	tape_thresh.vub = T_VALUE_UBOUND;

	mode = ESM_CUBE;
	curr_thresh = &cube_thresh;

    Timer::init();

    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if(sock < 0) {
        printf("Could not create socket\n");
        //return 1;
    }

    struct sockaddr_in n_addr;

    memset((char*)&n_addr, 0, sizeof(n_addr));

    n_addr.sin_family = AF_INET;
    n_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    n_addr.sin_port = htons(LOCAL_PORT);

    if(bind(sock, (struct sockaddr*)&n_addr, sizeof(n_addr)) < 0) {
        printf("Socket binding failed\n");
        //return 1;
    }

	struct timeval tv;
	tv.tv_sec = RECEIVE_TIMEOUT_S;
	tv.tv_usec = RECEIVE_TIMEOUT_US;
	if(setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
		printf("Could not set timeout\n");
	}

	size_t send_t_len = 4;
	const char *send_format = "dd";
    size_t send_sig_len = strlen(SEND_SIG);
    unsigned char mes[2 * send_t_len + send_sig_len + 1];
    strcpy((char*)mes, SEND_SIG);
	mes[send_sig_len] = 'R';

    struct sockaddr_in dest_addr;
    memset((char*)&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DEST_PORT);

	pthread_t thread;
	thread_running = false;

    if(pthread_mutex_init(&mutex, NULL) != 0) {
        printf("Could not initialize mutex\n");
    }
	else if(pthread_create(&thread, NULL, recvLoop, (void*)NULL) != 0) {
		printf("Could not create receive thread\n");
		thread_should_close = true;
	}
	else {
		thread_should_close = false;
    }

    cv::VideoCapture cap(0);
    setProps(&cap);

	cv::Size image_size(FRAME_WIDTH, FRAME_HEIGHT);
	cv::Mat image(image_size, CV_8UC3, cv::Scalar::all(0));

	cv::Size stream_size(STREAM_FRAME_WIDTH, STREAM_FRAME_HEIGHT);
	cv::Mat stream_image(stream_size, CV_8UC3, cv::Scalar::all(0));

    connect(&cap, &image);

	if(!should_exit)
		getRoborioIP(&dest_addr);

    cv::Mat hsv_image(image_size, CV_8UC3, cv::Scalar::all(0));
    cv::Mat temp(image_size, CV_8UC1, cv::Scalar::all(0));

    std::vector<std::vector<cv::Point>> contours;

    ms_t lastTime = Timer::getMs();

	ms_t startTime = Timer::getMs();

	char c;

	//int bottom_curve[FRAME_WIDTH];
	//int bottom_curve_smooth[FRAME_WIDTH];

    while(!should_exit)
	{
        pthread_mutex_lock(&mutex);
		if(mode == ESM_CUBE)
			curr_thresh = &cube_thresh;
		else if(mode == ESM_TAPE)
			curr_thresh = &tape_thresh;
        pthread_mutex_unlock(&mutex);

		c = cv::waitKey(1);
		
		if(c == 27)
			break;

		switch(c) {
		case 'w':
			if(curr_thresh->hlb < 256)
				curr_thresh->hlb++;
			break;
		case 's':
			if(curr_thresh->hlb > -1)
				curr_thresh->hlb--;
			break;
		case 'e':
			if(curr_thresh->hub < 256)
				curr_thresh->hub++;
			break;
		case 'd':
			if(curr_thresh->hub > -1)
				curr_thresh->hub--;
			break;
		case 'r':
			if(curr_thresh->slb < 256)
				curr_thresh->slb++;
			break;
		case 'f':
			if(curr_thresh->slb > -1)
				curr_thresh->slb--;
			break;
		case 't':
			if(curr_thresh->sub < 256)
				curr_thresh->sub++;
			break;
		case 'g':
			if(curr_thresh->sub > -1)
				curr_thresh->sub--;
			break;
		case 'y':
			if(curr_thresh->vlb < 256)
				curr_thresh->vlb++;
			break;
		case 'h':
			if(curr_thresh->vlb > -1)
				curr_thresh->vlb--;
			break;
		case 'u':
			if(curr_thresh->vub < 256)
				curr_thresh->vub++;
			break;
		case 'j':
			if(curr_thresh->vub > -1)
				curr_thresh->vub--;
			break;
		default:
			//printf("c: %c\n", c);
			break;
		};
		
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

						//for(int i = 0; i < FRAME_WIDTH; i++)
						//	bottom_curve[i] = -1;

                        applyKernel(temp, hsv_image, curr_thresh);

						/*double mean = 0.0;
						int num = 0;
						for(int i = 0; i < FRAME_WIDTH; i++) {
							if(bottom_curve[i] != -1) {
								mean += (double)bottom_curve[i];
								num++;
							}
						}

						if(num)
							mean /= (double)num;

						//printf("mean: %f\n", mean);

						double stddev = 0.0;

						for(int i = 0; i < FRAME_WIDTH; i++)
							stddev += (bottom_curve[i] - mean) * (bottom_curve[i] - mean);

						if(num)
							stddev /= (double)num;

						//printf("stddev: %f\n", stddev);*/

						printf("hlb: %d hub: %d slb: %d sub: %d vlb: %d vub: %d\n", curr_thresh->hlb, curr_thresh->hub, curr_thresh->slb, curr_thresh->sub, curr_thresh->vlb, curr_thresh->vub);

                        cv::GaussianBlur(temp, temp, cv::Size(3, 3), 1.0, 1.2);

						//cv::imshow("Mask", temp);

                        findContours(temp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

                        ssize_t opt_idx = -1;
						ssize_t opt_idx2 = -1;
                        double m_a = 0.0, m_a2 = 0.0, a = 0.0;
                        size_t l = contours.size();
						//printf("Initial number of contours: %d\n", l);
                        for(size_t i = 0; i < l; i++) {
                            if((a = contourArea(contours[i])) > m_a) {
                                m_a = a;
                                opt_idx = i;
                            }
							else if(a > m_a2) {
                                m_a2 = a;
								opt_idx2 = i;
                            }
                        }

                        double x = 2.0, y = 2.0;

                        pthread_mutex_lock(&mutex);
						if(mode == ESM_CUBE && opt_idx != -1)
						{
                            pthread_mutex_unlock(&mutex);
							//std::vector<cv::Point> simple_contour;
							//cv::approxPolyDP(contours[opt_idx], simple_contour, 10.0, true);
							//cv::drawContours(image, std::vector<std::vector<cv::Point>>{simple_contour}, 0, cv::Scalar(0, 0, 255));
                            cv::Moments m = cv::moments(contours[opt_idx]);
                            cv::Point centroid(m.m10 / m.m00, m.m01 / m.m00);

							x = (double)(centroid.x - FRAME_WIDTH / 2) / (double)(FRAME_WIDTH / 2);
							y = -(double)(centroid.y - FRAME_HEIGHT / 2) / (double)(FRAME_HEIGHT / 2);
						}
						else if(mode == ESM_TAPE && opt_idx != -1 && opt_idx2 != -1)
                        {
                            pthread_mutex_unlock(&mutex);
                            cv::Moments m1 = cv::moments(contours[opt_idx]);
                            cv::Moments m2 = cv::moments(contours[opt_idx2]);

                            cv::Point centroid1(m1.m10 / m1.m00, m1.m01 / m1.m00);
                            cv::Point centroid2(m2.m10 / m2.m00, m2.m01 / m2.m00);

                            double avgx = 0.5 * (centroid1.x + centroid2.x);
                            double avgy = 0.5 * (centroid1.y + centroid2.y);

                            x = (double)(avgx - FRAME_WIDTH / 2) / (double)(FRAME_WIDTH / 2);
                            y = -(double)(avgy - FRAME_HEIGHT / 2) / (double)(FRAME_HEIGHT / 2);
                        }
                        else {
                            pthread_mutex_unlock(&mutex);
                            printf("Too few contours found\n");
                        }

						if(stream_size == image_size) {
//							cv::imshow("Image", image);
						}
						else {
							//cv::resize(image, stream_image, stream_size, 0, 0, cv::INTER_LINEAR);
//							cv::imshow("Image", stream_image);
						}

                        //printf("%f %f\n", x, y);

                        pack(mes + send_sig_len + 1, send_format, x, y);
                        pthread_mutex_lock(&mutex);
                        ssize_t r = sendto(sock, mes, 2 * send_t_len + send_sig_len + 1, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                        pthread_mutex_unlock(&mutex);

                        if(r == -1) {
                            printf("Error sending packet!\n");
							getRoborioIP(&dest_addr);
						}
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

        ms_t time = Timer::getMs();
        ms_t dt = time - lastTime;
        lastTime = time;

        printf("Loop time: %lld ms\n", dt);
    
		if((Timer::getMs() - startTime) > 30000)
		    break;
	}

    printf("Exit received\n");

	cap.release();

    printf("Closing...");

    return 0;
}
