// cv_code.cpp : Objective Tracking and Recognition Subsystem
// cvui.h : GUI support lib. 

#include "stdafx.h"
#include "PSEyeDemo.h"

#define CVUI_IMPLEMENTATION

#include <Windows.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/video/tracking.hpp>
//#include <opencv2/ocl/ocl.hpp>
//#include <winsock2.h>
#include "xPCUDPSock.h"




//A  :254*152.4 = 38709.6
//B :480*640 = 307200
//A/B = 7.94 px 


#include "cvui.h"


#include <thread>
#include <mutex>

//#define CVUI_IMPLEMENTATION
#define _TRUE 1
#define _FALSE 0
#define COLOR_TARGET Scalar(0, 255, 0)
#define COLOR_DECOY Scalar(0, 0, 255)
#define USE_LAPTOP_CAM FALSE
#define PIXEL_DISTENCE 1000
#define TRACKMODE 1
#define SQR(X) ((X) * (X))

#pragma pack(push,1) // Important! Tell the compiler to pack things up tightly 



using namespace std;

struct PACKOUT
{
	float flt1;
	float flt2;
};

#pragma pack(pop) // Fall back to previous setting




using namespace cv; // allows ... without using namespace cv::

#define FRAME_RATE 999
#define RESOLUTION CLEYE_VGA
// QVGA or VGA

typedef struct{
	CLEyeCameraInstance CameraInstance;
	Mat *Frame;
	unsigned char *FramePtr;
	int Threshold;
}CAMERA_AND_FRAME;
typedef vector<Point2f> Point2fVector;


typedef struct {
	int still;
	double x;
	double y;
	
}Position;

typedef struct {
	Mat rgb;
	Mat gray;
}processing_frame;

//Mat dst, img, hsv, mask;
Mat h;
Mat s;
Mat v;
//For screenshot
//h 35-78
//s 43 - 255
//v 46 - 255
//For real target
Mat show_img = Mat(480, 640, CV_8UC4);

int hmin = 51; int hmin_Max = 360; int hmax = 95; int hmax_Max = 360;
int smin = 56; int smin_Max = 255; int smax = 255; int smax_Max = 255;
int vmin = 28; int vmin_Max = 255; int vmax = 255; int vmax_Max = 255;

int flag_frame_obt = 0;
int is_init_coordination = _FALSE;
int is_camera_close = _FALSE;
int flag_show_process_window = _FALSE;
int FramerCounter = 0;
int curr_fps = 0;

bool flag_track = FALSE;
bool send_pkt = FALSE;
bool button_cal_distance = FALSE;
bool flag_show_homograph = FALSE;
bool switch_to_gray = FALSE;
bool is_find_homo = FALSE;
bool flag_use_laptop_cam = USE_LAPTOP_CAM;
Point2fVector points;
Point2fVector points2;
Mat_<double> H;

mutex mtx;
mutex mtx_camera;

vector<Position> vec_target_pos, vec_tmp_target_pos, vec_moving_tg1, vec_moving_tg2;
vector<Position> vec_decoy_pos, vec_tmp_decoy_pos, vec_tg_angle;


Vec4f predict_line;

double cur_x = 0;
double cur_y = 0;


char num_target[100];
char num_decoy[100];
char _fps[100];

//vector<Mat> planes;

/*Functions*/
void hsv_tres();
Mat get_erosion(Mat frame);
Mat get_dilation(Mat frame);
processing_frame detect(Mat img);
Mat get_homography(Mat img);
void get_angle();

void track_mode();



void printMatrix(const Mat_<double>& C)
{
	cout << setprecision(3) << right << fixed;

	for (int row = 0; row < C.rows; ++row)
	{
		for (int col = 0; col < C.cols; ++col)
		{
			cout << setw(5) << C(row, col) << " ";
		}
		cout << endl;
	}
}
bool calculate_distance(Position A, Position B, int val);


int frame_counter = 0;


//get mouse position.
void MousCallback(int mEvent, int x, int y, int flags, void* param)
{
	Point2fVector* pPointVec = (Point2fVector*)param;
	if (mEvent == CV_EVENT_LBUTTONDOWN)
	{
		pPointVec->push_back(Point2f(float(x), float(y)));
	}
}


//track bar
static void on_trackbar(int, void*)
{
	
	
	

}



void blurCallback(int state, void* userData) {
}


//thread
int camera_thread(Mat *Frame);
void window_thread();


double ho_val=0.0;
double ve_val = 0.0;
int main(int argc, _TCHAR* argv[])
{
	
	int nRetCode = 0;
	if (!InitUDPLib())
	{
		nRetCode = 2;
	}
	else
	{
		CUDPSender sender(sizeof(PACKOUT), 12302, "127.0.0.1");
		PACKOUT pkout;


		int sample_rate = 2; //unit = frame
		Mat hsv, mask;
		Mat Frame = Mat(480, 640, CV_8UC4);
		Mat img = Mat(480, 640, CV_8UC4); //cur_frame
		Mat warpped_img = Mat(480, 640, CV_8UC4); //cur_frame
		thread video_cap_handle = thread(camera_thread, &Frame);
		//thread window_thread_handle = thread(window_thread);
		Mat unwarp;
		hsv_tres();

		processing_frame cur_frame;

		namedWindow("ctrl");
		cvui::init("ctrl");
		namedWindow("Camera");
		cv::Mat panel = cv::Mat(cv::Size(400, 600), CV_8UC3);

		Point point0, point1, point2;
		double k = 0.0;


		Position pred_pos, v;
		int count = 2;

		//Position vx, vy;
		//vx.x = 0;vx.y = 0;
		//vy.x = 0;vy.y = 0;


		//Frame = get_homography(Frame);
		while (1) {
			panel = cv::Scalar(49, 52, 49);
			frame_counter++;

			cout << frame_counter << endl;


			img = Frame;

			if (is_find_homo == TRUE) {
				img = get_homography(img);
			}
			cur_frame = detect(img);

			get_angle();
			

			if (frame_counter == sample_rate) {
				//char v_info[100];
				if (vec_moving_tg1.empty() == true) {
					v.x = 0.0; v.y = 0.0;
				}
				else if (vec_moving_tg1.size() == sample_rate) {

					v.x = (vec_moving_tg1[sample_rate - 1].x - vec_moving_tg1[0].x) / (sample_rate);
					v.y = (vec_moving_tg1[sample_rate - 1].y - vec_moving_tg1[0].y) / (sample_rate);

					//pred_pos.x = vec_moving_tg1[sample_rate - 1].x + 20 * v.x;
					//pred_pos.y = vec_moving_tg1[sample_rate - 1].y + 20 * v.y;



				}
				vec_moving_tg1.clear();
				frame_counter = 0;
			}

			pred_pos.x = cur_x + 20 * v.x;
			pred_pos.y = cur_y + 20 * v.y;


			
			pkout.flt1 = (float)ho_val;
			pkout.flt2 = (float)ve_val;
			sender.SendData(&pkout);
		



		



			cvui::counter(panel, 90, 50, &count);

			cvui::trackbar(panel, 40, 250, 360, &ho_val, (double)-45.0, (double)45.0);
			cvui::trackbar(panel, 40, 300, 360, &ve_val, (double)-45.0, (double)45.0);
			pkout.flt1 = (float)ho_val;
			pkout.flt2 = (float)ve_val;
			sender.SendData(&pkout);

			/*
			if (cvui::button(panel, 40, 350, "send packet")) {
				send_pkt = TRUE;
			}

			if (send_pkt == TRUE) {
				sender.SendData(&pkout);
				send_pkt = FALSE;
			}
			*/

			
			if (cvui::button(panel, 10, 160, "aim targets")) {
				flag_track = TRUE;
			}

			if (flag_track == TRUE) {
				

				for (int i = 0; i < vec_tg_angle.size(); i++) {
					pkout.flt1 = (float)vec_tg_angle.at(i).x;
					pkout.flt2 = (float)vec_tg_angle.at(i).y;
					sender.SendData(&pkout);
					Sleep(5000);
				}
				pkout.flt1 = 0;
				pkout.flt2 = 0;
				sender.SendData(&pkout);
				flag_track = FALSE;
				Sleep(2000);
			}


			if (cvui::button(panel, 10, 15, "Gray")) {
				switch_to_gray = TRUE;
			}
			if (cvui::button(panel, 10, 70, "RGB")) {
				switch_to_gray = FALSE;
			}
			if (cvui::button(panel, 10, 70+70-15, "no display")) {
				switch_to_gray = FALSE;
			}
			if (cvui::button(panel, 10, 150+50, "calibrate")) {
				is_find_homo = TRUE;
				is_init_coordination = 0;
			}

			if (cvui::button(panel, 100+30, 150+50, "original img")) {
				is_find_homo = FALSE;
				//is_init_coordination = 0;
			}
			if (cvui::button(panel, 300, 450, "QUIT")) {
				is_camera_close = 1;
				break;
				//is_init_coordination = 0;
			}
			if (vec_target_pos.size() > 0) {
				char tg_info[100];
				for (int i = 0; i < vec_target_pos.size(); i++) {
					//printf("pos:(%f,%f)\n",vec_target_pos[i].x,vec_target_pos[i].y);
					sprintf(tg_info, "tg:%i (%f , %f)mm", i, vec_target_pos[i].x, vec_target_pos[i].y);
					cvui::text(panel, 100, 15 + i * 30, tg_info);
				}

			}


			char vec_v_info[100];
			for (int i = 0; i < vec_moving_tg1.size(); i++) {
				//printf("pos:(%f,%f)\n",vec_target_pos[i].x,vec_target_pos[i].y);
				sprintf(vec_v_info, "v:%i (%f , %f)mm", i, vec_moving_tg1[i].x, vec_moving_tg1[i].y);
				cvui::text(panel, 100, 450 + i * 30, vec_v_info);

				printf("%s\n", vec_v_info);
			}



			char angle_info[100];
			for (int i = 0; i < vec_tg_angle.size(); i++) {
				//printf("pos:(%f,%f)\n",vec_target_pos[i].x,vec_target_pos[i].y);
				sprintf(angle_info, "angle tg:%i (%f , %f)mm", i, vec_tg_angle[i].x, vec_tg_angle[i].y);
				cvui::text(panel, 100, 500 + i * 30, angle_info);

				//printf("%s\n", vec_v_info);
			}




			char v_info[100];
			sprintf(v_info, "vx=%f, vy=%f", v.x, v.y);
			cvui::text(panel, 100, 15 + 3 * 30, v_info);
			circle(cur_frame.rgb, Point(pred_pos.x, pred_pos.y), 1 / 32.0, COLOR_TARGET, 5, 8);




			cvui::imshow("ctrl", panel);
			cvui::update();
			if (switch_to_gray == TRUE) {
				imshow("Camera", cur_frame.gray);
			}
			else {
				imshow("Camera", cur_frame.rgb);
			}

			if (waitKey(1) == 27 || waitKey(1) == 'q') {
				is_camera_close = 1;
				break;
			}


		}

		video_cap_handle.join();
		//destroyAllWindows();

	}
	return nRetCode;
}







void hsv_tres() {
	namedWindow("Trackbar",CV_WINDOW_AUTOSIZE);
	// H
	createTrackbar("hmin", "Trackbar", &hmin, hmin_Max, on_trackbar);
	createTrackbar("hmax", "Trackbar", &hmax, hmax_Max, on_trackbar);
	// S
	createTrackbar("smin", "Trackbar", &smin, smin_Max, on_trackbar);
	createTrackbar("smax", "Trackbar", &smax, smax_Max, on_trackbar);
	// V
	createTrackbar("vmin", "Trackbar", &vmin, vmin_Max, on_trackbar);
	createTrackbar("vmax", "Trackbar", &vmax, vmax_Max, on_trackbar);


}


Mat get_erosion(Mat frame)
{
	int erosion_elem = 2;
	int erosion_size = 3;
	int erosion_type;
	if (erosion_elem == 0) { erosion_type = MORPH_RECT; }
	else if (erosion_elem == 1) { erosion_type = MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(erosion_type,
		Size(1 * erosion_size + 1, 1 * erosion_size + 1),
		Point(erosion_size, erosion_size));

	/// Apply the erosion operation
	Mat erosion_dst;
	erode(frame, erosion_dst, element);
	if (flag_show_process_window == 1) {
		imshow("Erosion Demo", erosion_dst);
		//waitKey(1);

	}
	return erosion_dst;
}


Mat get_dilation(Mat frame)
{
    int	dilation_elem = 2;
	int dilation_type;
	int dilation_size = 5;

	if (dilation_elem == 0) { dilation_type = MORPH_RECT; }
	else if (dilation_elem == 1) { dilation_type = MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(dilation_type,
		Size(1 * dilation_size +1 , 1 * dilation_size +1 ),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	Mat dilation_dst;
	dilate(frame, dilation_dst, element);
	if (flag_show_process_window) {
		imshow("Dilation Demo", dilation_dst);
		//waitKey(1);

	}
	return dilation_dst;
}


/* Get homograhy for each frame, which required lots of calculations.
 * Return warpped image.
 */
Mat get_homography(Mat img) {
	//10*6
	//A  :3048*1828.8 = 5574182.4
//B :480*640 = 307200
//A/B =18.145 px 



	double real_h = 480.0, real_w = 640.0;
	double scale = 1.0;

	if (is_init_coordination == 0) {
        
		

		points.clear();
		points2.clear();
		//Sleep(2000);
		//imshow("Camera", img);
		MessageBoxA(NULL, "Please click four corners of the plane.\n"
			"bottom Left -> bottom right -> upper right->upper left",
			"Click", MB_OK);



		cvSetMouseCallback("Camera", MousCallback, &points);

		while (1)
		{
			// wait for mouse clicks
			waitKey(10);
			if (points.size() == 4) {
				MessageBoxA(NULL, "Done!", "Click", MB_OK);
				break;
			}



		}



		vec_target_pos.clear();
		vec_tmp_target_pos.clear();


		cvSetMouseCallback("Camera", NULL, NULL);
		is_init_coordination = 1;

		cout << "pos:" << points << endl;
		// HERE I ASSUME EACH PIXEL WILL BE 5 mm
		
		//Point2fVector points2;
		points2.push_back(Point2f(0.0, real_h / scale));
		points2.push_back(Point2f(real_w / scale, real_h / scale));
		points2.push_back(Point2f(real_w / scale, 0.0));
		points2.push_back(Point2f(0.0, 0.0));

		//Mat_<double> H = findHomography(Mat(points), Mat(points2));
		H = findHomography(Mat(points), Mat(points2));
		cout << "The transformation Matrix is :" << endl;
		printMatrix(H);
		cout << endl;
	}
	Mat unwarpImage;
	//warpPerspective(img, unwarpImage, H, Size(1200.0 / scale, 2000.0/ scale));
	warpPerspective(img, unwarpImage, H, Size(real_w /scale, real_h /scale));


	return unwarpImage;
}

/* detect: get the position of all targets and save in to vector.
 *
 *
 *
 * return detection image.
 */
processing_frame detect(Mat img) {
	//dst = Mat::zeros(img.size(), img.type());
	Position tmp_decoy_pos, tmp_target_pos;
	double scale = 1.0;
	Mat mask,hsv;
	processing_frame cur_frame;
	
	cvtColor(img, hsv, COLOR_BGR2HSV);

	inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);


	medianBlur(mask,mask,13);
	mask = get_erosion(mask);
	//GaussianBlur(mask, mask, Size(5, 5), 1, 1);
	mask = get_dilation(mask);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat dist_8u;
	mask.convertTo(dist_8u, CV_8U);
	findContours(dist_8u, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);


	vec_tmp_target_pos.clear();
	int target = 0, decoy = 0, total = 0;
	for (size_t i = 0; i < contours.size(); i++)
	{

		Moments moment = moments((Mat)contours[i]);
		double area = moment.m00;
		//target
		if (area >= 50 && area <= 250) {
			target++;
			//drawContours(img, contours, static_cast<int>(i), Scalar(0, 255, (i + i * 20) % 255), -1);

			Point pt;
			pt.x = int(moment.m10 / moment.m00);
			pt.y = int(moment.m01 / moment.m00);

			tmp_target_pos.x = pt.x;
			tmp_target_pos.y = pt.y;
			vec_tmp_target_pos.push_back(tmp_target_pos);

			
			
			//char pos[100];
			//sprintf(pos, "(%i,%i)mm", pt.x, pt.y);
			//putText(img, pos, Point(pt.x + 5, pt.y), FONT_HERSHEY_DUPLEX, 1, COLOR_TARGET, 1);
			circle(img, pt, 1 / 32.0, COLOR_TARGET, 5, 8);
			

		}

		//decoy
		else if (area >= 350 && area <= 1000) {
			decoy++;
			//drawContours(img, contours, static_cast<int>(i), Scalar(0, 255, (i + i * 20) % 255), -1);
			Point pt2;
			char pos2[100];
			pt2.x = int(moment.m10 / moment.m00);
			pt2.y = int(moment.m01 / moment.m00);

			
			circle(img, pt2, 1 / 32.0, COLOR_DECOY, 5, 8);

			//sprintf(pos2, "(%i,%i)mm", pt2.x, pt2.y);
			//putText(img, pos2, Point(pt2.x + 5, pt2.y), FONT_HERSHEY_DUPLEX, 1, COLOR_DECOY, 1);

		}

		
	}
    
	track_mode();

	for (int i = 0;i < vec_target_pos.size();i++) {
		
		//cout << "(" << vec_target_pos[i].x<<",";
		//cout << vec_target_pos[i].y<<")" << endl;
		char tg_pos[100];
		sprintf(tg_pos, "tg%i", i);
		putText(img, tg_pos, Point(vec_target_pos[i].x, vec_target_pos[i].y), FONT_HERSHEY_DUPLEX, 1, COLOR_TARGET, 1);
		 
	}



	sprintf(num_decoy, "#decoy=%i", decoy);
	putText(img, num_decoy, Point(5, 100), FONT_HERSHEY_DUPLEX, 1, COLOR_DECOY, 1);

	sprintf(num_target, "#target=%i", target);
	putText(img, num_target, Point(5, 50), FONT_HERSHEY_DUPLEX, 1, COLOR_TARGET, 1);

    
	sprintf(_fps, "fps=%i", curr_fps);
	putText(img, _fps, Point(500, 50), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 255), 1);
	
	
    
	cur_frame.rgb = img;
	cur_frame.gray = mask;
	
	return cur_frame;

}



///////////////////////SUB THREAD///////////////////////////
//for high frame rates you will process images here the main function will allow interactions and display only
int camera_thread(Mat *Frame){
	
	
	//GUID CameraID;
	clock_t StartTime, EndTime;
	bool CamParam = 0;
	CLEyeCameraInstance EyeCamera;
	VideoCapture cap;
	
	Mat cur_frame = Mat(480, 640, CV_8UC4);


	if (flag_use_laptop_cam == FALSE) {
		EyeCamera = CLEyeCreateCamera(CLEyeGetCameraUUID(0), CLEYE_COLOR_RAW, RESOLUTION, FRAME_RATE);
		CLEyeCameraStart(EyeCamera);
	}
	else {
		cap.open(0 + CV_CAP_ANY);
		cap.set(CV_CAP_PROP_FPS, FRAME_RATE);

	}

	while (1) {

		if (is_camera_close == 1) break;
		if (flag_use_laptop_cam == FALSE) {
			CLEyeCameraGetFrame(EyeCamera, cur_frame.data);
		}
		else {
			cap.read(cur_frame);
		}



		// Track FPS

		if (FramerCounter == 0)StartTime = clock();
		FramerCounter++;
		EndTime = clock();

		//if (FramerCounter == 5) {
			*Frame = cur_frame;
		//}

		if ((EndTime - StartTime) / CLOCKS_PER_SEC >= 1) {
			curr_fps = FramerCounter;
			cout << "FPS:" << FramerCounter << endl;
			FramerCounter = 0;
		}

	}

	if (flag_use_laptop_cam = FALSE) {
		CLEyeCameraStop(EyeCamera);
		CLEyeDestroyCamera(EyeCamera);
		EyeCamera = NULL;
	}


		
	return 0;
}

void window_thread() {
	/*
	Mat cur_frame;
	Sleep(2000);
	cout << "done wait" << endl;
	cv::namedWindow("Camera", CV_WINDOW_AUTOSIZE);

	while (1) {
		if (is_camera_close == 1) break;



		cur_frame = show_img;
		
			imshow("Camera",cur_frame);
		

    

	}

	*/
}

/* Compare bewteen each frame. Determine which target is moving.
 */
void track_mode() {
	int moving = 0;
	//int k = 0;
	//int target = 0;
	Position cur_pos;
	//if empty
	if(vec_target_pos.empty()==true && vec_tmp_target_pos.empty()==false){
		vec_target_pos = vec_tmp_target_pos;
    
    }
	//if not.
	//ASSUME that # of target will NOT changed.
	else {
		//Check px distence bw each target 
		for (int i = 0;i < vec_target_pos.size();i++) vec_target_pos[i].still = 1;

		/*
		for (int i = 0; i < vec_tmp_target_pos.size();i++) {
			for (int j = 0;j < vec_target_pos.size();j++) {
				printf("---%f--\n", SQR(vec_tmp_target_pos.at(i).x - vec_target_pos.at(j).x) + SQR(vec_tmp_target_pos.at(i).y - vec_target_pos.at(j).y));
				if ((SQR(vec_tmp_target_pos.at(i).x - vec_target_pos.at(j).x) +
					SQR(vec_tmp_target_pos.at(i).y - vec_target_pos.at(j).y) ) >=20    ) {
				}
				vec_target_pos[j].still = 0;
				cout << "tg:" << j << " moving-----" << endl;
				continue;

			}
		}
		*/
		

		
		for (int i = 0; i < vec_tmp_target_pos.size();i++) {

			for (int j = 0;j < vec_target_pos.size();j++) {

				if ( 
					( (SQR(vec_tmp_target_pos.at(i).x - vec_target_pos.at(j).x) +
					   SQR(vec_tmp_target_pos.at(i).y - vec_target_pos.at(j).y) ) <= PIXEL_DISTENCE) &&
						
						(SQR(vec_tmp_target_pos.at(i).x - vec_target_pos.at(j).x) +
						   SQR(vec_tmp_target_pos.at(i).y - vec_target_pos.at(j).y)) >=30
					){
					vec_target_pos[j] = vec_tmp_target_pos.at(i);	
					vec_moving_tg1.push_back(vec_target_pos[j]);
					cur_x = vec_tmp_target_pos.at(i).x;
					cur_y = vec_tmp_target_pos.at(i).y;

					moving++;
					printf("--%i--pushed %f,%f\n", moving,vec_target_pos[j].x, vec_target_pos[j].y);					
					break;
				}
			}
		}

		
	
	}
	
	
}



void get_angle() {
	vec_tg_angle.clear();
	double scalar_x = 366;
	double scalar_y = 196;

	Position angle;
	for (int i = 0; i < vec_target_pos.size(); i++) {
		angle.x = (float)(vec_target_pos.at(i).x - scalar_x) / 14.417;
		angle.y = (float)(scalar_y - vec_target_pos.at(i).y ) / 13.63;
		vec_tg_angle.push_back(angle);
	}



}