#pragma once
#include "trajectory.h"
#include "opencv2\opencv.hpp"

using namespace std;
using namespace cv;

#define ROI_WIDTH 60
#define MOTION_THRESH 10
#define MOTION_MIN_COUNT 10
#define LEFT_BALL_X_INIT 362
#define LEFT_BALL_Y_INIT 114
#define RIGHT_BALL_X_INIT 280
#define RIGHT_BALL_Y_INIT 120
#define CATCHER_X_OFFSET -12
#define CATCHER_Y_OFFSET -23
#define CATCHER_X_SCALE -1.5
#define CATCHER_Y_SCALE -1.09
#define Y_TRACKING_BOUND 220
class ball_tracker
{
private:
	string left_xml;
	string right_xml;
	string stereo_xml;
	string rect_xml;
	int num_frames_to_track;
	double catcher_z;
	Point2d left_center = Point2d(LEFT_BALL_X_INIT, LEFT_BALL_Y_INIT);
	Point2d right_center = Point2d(RIGHT_BALL_X_INIT, RIGHT_BALL_Y_INIT);
	Point2d prediction;
	vector<vector<Point2d>> points;
	vector<vector<Point2d>> clicked_points;
	bool ball_in_flight = false;
	Mat initial_right_image;
	Mat initial_left_image;
	bool initilized = false;
	int wait_after_motion = 0;
	Point2d get_ball_center(Mat frame, int which);
	bool is_ball_in_flight(Mat left_image,Mat right_image);
	bool is_there_motion(Mat image, Mat prev_image, Point point);
	Mat get_region_of_interest(Mat image, Point2d center);
	void set_next_points(Mat left_image, Mat right_image);
	Point2d camera_2d_to_catcher(double x, double y);
	vector<vector<Point2d>> combine_point_vectors(vector<Point2d> points1, vector<Point2d> points2);
	Point2d get_image_mouse_point(Mat image, string window_name);
	vector<Point2d> click_points(vector<Mat> images);
	vector<Mat> right_tracking_images;
	vector<Mat> left_tracking_images;
	vector<Mat> left_flight_images;
	vector<Mat> right_flight_images;
	bool show_tracking_images_flag;
	bool plot_points_flag;
	void draw_points();
	void show_tracking_images();
	void click_flight_images();
	int x_poly_order;
	int y_poly_order;
	bool click_flight_images_flag;
	int num_flight_frames;
	int num_flight_frames_countdown;


public:
	ball_tracker(string left_xml, string right_xml, string stereo_xml, string rect_xml) :
		left_xml(left_xml), right_xml(right_xml), stereo_xml(stereo_xml), rect_xml(rect_xml){}
	void set_num_frames_to_track(int num_frames){ num_frames_to_track = num_frames; }
	void set_catcher_z(double z){ catcher_z = z; }
	void feed_next_images(Mat left_image, Mat right_image);
	void set_wait_after_motion(int wait){ wait_after_motion = wait; }
	Point2d get_prediction();
	bool is_tracking_done();
	bool is_flight_done(){ return num_flight_frames_countdown < 0; }
	void reset();
	void plot_points();
	void set_plot_points_flag(bool plot){ plot_points_flag = plot; }
	void set_show_tracking_images_flag(bool show){ show_tracking_images_flag = show; }
	void show_results();
	void set_poly_order(int x, int y){ x_poly_order = x; y_poly_order = y; }
	void set_click_flight_images_flag(bool show){ click_flight_images_flag = show; }
	void set_num_flight_frames(int num){ num_flight_frames = num; num_flight_frames_countdown = num; }

};
void onMouse(int evt, int x, int y, int flags, void *param);



