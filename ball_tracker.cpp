#include "stdafx.h"
#include "ball_tracker.h"
#include "trajectory.h"

char key;
volatile double point_x;
Point2d ball_tracker::get_prediction(){
	Point3d camera_3d = get_z_plane_intercept(points,left_xml,right_xml,rect_xml,stereo_xml,1,catcher_z,x_poly_order,y_poly_order);
	return camera_2d_to_catcher(camera_3d.x, camera_3d.y);
}
void ball_tracker::plot_points(){
	if (plot_points_flag){
		plot_points_3d(points, left_xml, right_xml, rect_xml, stereo_xml, 1, x_poly_order, y_poly_order, catcher_z);
		if (click_flight_images_flag && clicked_points.size() > 0){
			plot_points_3d(clicked_points, left_xml, right_xml, rect_xml, stereo_xml, 1, 1, -1, catcher_z);
		}
	}
}

void ball_tracker::show_results(){
	show_tracking_images();
	click_flight_images();
	plot_points();

}

void ball_tracker::reset(){
	left_center = Point2d(LEFT_BALL_X_INIT, LEFT_BALL_Y_INIT);
	right_center = Point2d(RIGHT_BALL_X_INIT, RIGHT_BALL_Y_INIT);
	points.clear();
	ball_in_flight = false;
	initilized = false;
	left_tracking_images.clear();
	right_tracking_images.clear();
	left_flight_images.clear();
	right_flight_images.clear();
	num_flight_frames_countdown = num_flight_frames;
	wait_after_motion = 0;
}
bool ball_tracker::is_tracking_done(){
	if (points.size() >= num_frames_to_track){
		return true;
	}
	if (left_center.y > Y_TRACKING_BOUND || right_center.y > Y_TRACKING_BOUND){
		return true;
	}
	else{
		return false;
	}
}

Point2d ball_tracker::get_ball_center(Mat frame, int which){
	Point2d *prev_center;

	if (which == 1){
		prev_center = &left_center;
	}
	else{
		prev_center = &right_center;
	}
	Mat frame_gray;
	if (frame.channels() == 3){
		cvtColor(frame, frame_gray, CV_RGB2GRAY);
	}
	else{
		frame_gray = frame;
	}
	Mat sub_mat = get_region_of_interest(frame_gray, *prev_center);
	vector<Vec3f> circles;
	HoughCircles(sub_mat, circles, CV_HOUGH_GRADIENT, 1, 100, 100, 3, 0, 100);

	Point center;
	int radius;
	int min_dist = 100000;
	for (size_t i = 0; i < circles.size(); i++)
	{
		int dist = norm(*prev_center - Point2d(cvRound(circles[i][0]), cvRound(circles[i][1])));
		if (dist < min_dist){
			center.x = cvRound(circles[i][0]);
			center.y = cvRound(circles[i][1]);
			radius = cvRound(circles[i][2]);
			min_dist = dist;
		}
	}
	//circle(frame, *prev_center, 3, Scalar(255, 255, 0), -1, 8, 0);

	*prev_center = center;
	//line(frame, Point2d(0, Y_TRACKING_BOUND), Point2d(frame.size().width, Y_TRACKING_BOUND), Scalar(255, 0, 0));
	//cvNamedWindow("center");
	//imshow("center", frame);
	//waitKey(0);
	if (show_tracking_images_flag){
		circle(frame, *prev_center, 3, Scalar(0, 255, 0), -1, 8, 0);
		if (which == 1){
			left_tracking_images.push_back(frame.clone());
		}
		else{
			right_tracking_images.push_back(frame.clone());
		}
	}
	return center;
}

void ball_tracker::show_tracking_images(){
	if (show_tracking_images_flag == false){
		return;
	}
	namedWindow("left_image");
	namedWindow("right_image");
	for (int i = 0; i < left_tracking_images.size(); i++){
		imshow("left_image", left_tracking_images.at(i));
		imshow("right_image", right_tracking_images.at(i));
		if (cvWaitKey(0) == '\33') {
			break;
		}
	}
	destroyAllWindows();
}

void ball_tracker::feed_next_images(Mat left_image, Mat right_image){
	if (!this->initilized){
		initial_right_image = right_image.clone();
		initial_left_image = left_image.clone();
		this->initilized = true;
		return;
	}
	if (is_ball_in_flight(left_image, right_image)){
		if (wait_after_motion-- <= 0){
			if (num_flight_frames_countdown-- > 0 && click_flight_images_flag == true){
				left_flight_images.push_back(left_image.clone());
				right_flight_images.push_back(right_image.clone());
			}
			if (!is_tracking_done()){
				set_next_points(left_image, right_image);
			}
		}
	}
}

void ball_tracker::click_flight_images(){
	if (click_flight_images_flag == false){
		return;
	}
	vector<Point2d> left_clicked_points = click_points(left_flight_images);
	vector<Point2d> right_clicked_points = click_points(right_flight_images);
	clicked_points = combine_point_vectors(left_clicked_points, right_clicked_points);
}

vector<Point2d> ball_tracker::click_points(vector<Mat> images){
	vector<Point2d> points;
	for (int i = 0; i < images.size(); i++){
		putText(images.at(i), "Click point " + to_string(i + 1), Point(120, 20), FONT_HERSHEY_SIMPLEX, .5, Scalar(255, 0, 0));
		Point2d point = get_image_mouse_point(images.at(i), "Click_ball");
		if (point.x == -1){
			break;
		}
		points.push_back(point);
	}
	destroyAllWindows();
	return points;
}


Point2d ball_tracker::get_image_mouse_point(Mat image, string window_name){
	Point2d ret_point(-1, -1);
	namedWindow(window_name);
	setMouseCallback(window_name, onMouse, (void *)&ret_point);
	while (waitKey(30) != '\33'){
		imshow(window_name, image);
		point_x = ret_point.x;
		if (point_x != -1){
			break;
		}
	}
	return ret_point;
}

void onMouse(int evt, int x, int y, int flags, void *param){
	if (evt == CV_EVENT_LBUTTONDOWN){
		Point2d * point = (Point2d*)param;
		point->x = x;
		point->y = y;
	}
}

vector<vector<Point2d>> ball_tracker::combine_point_vectors(vector<Point2d> points1, vector<Point2d> points2){
	vector<vector<Point2d>> ret_points;
	int min_size = (points1.size() < points2.size()) ? points1.size() : points2.size();
	for (int i = 0; i < min_size; i++){
		vector<Point2d> pair;
		pair.push_back(points1.at(i));
		pair.push_back(points2.at(i));
		ret_points.push_back(pair);
	}
	return ret_points;
}

bool ball_tracker::is_ball_in_flight(Mat left_image, Mat right_image) {
	if (ball_in_flight == false){
		ball_in_flight = (is_there_motion(right_image, initial_right_image, right_center));
	}
	return ball_in_flight;
}

bool ball_tracker::is_there_motion(Mat image, Mat prev_image, Point point){
	if (image.channels() == 3){
		cvtColor(image, image, CV_RGB2GRAY);
		cvtColor(prev_image, prev_image, CV_RGB2GRAY);

	}
	Mat image_roi = get_region_of_interest(image, point);
	Mat prev_roi = get_region_of_interest(prev_image, point);
	Mat diff;
	absdiff(image_roi, prev_roi,diff);
	threshold(diff, diff, MOTION_THRESH, 255, THRESH_BINARY);
	//if (countNonZero(diff) > MOTION_MIN_COUNT){
	//	namedWindow("prev");
	//	imshow("prev", prev_roi);
	//	namedWindow("image");
	//	imshow("image", image_roi);
	//	namedWindow("diff");
	//	imshow("diff", diff);
	//	waitKey(0);
	//}
	return countNonZero(diff) > MOTION_MIN_COUNT;
}

void ball_tracker::set_next_points(Mat left_image, Mat right_image){
	vector<Point2d> point_pair;
	point_pair.push_back(get_ball_center(left_image, 1));
	point_pair.push_back(get_ball_center(right_image, 2));
	points.push_back(point_pair);
}

Mat ball_tracker::get_region_of_interest(Mat image, Point2d center){
	Mat zeros = Mat(image.size().height, image.size().width, image.type(), double(0));
	int rect_x = center.x - (ROI_WIDTH / 2);
	if (rect_x < 0){
		rect_x = 0;
	}
	else if (rect_x + ROI_WIDTH > image.size().width){
		rect_x = image.size().width - ROI_WIDTH;
	}
	int rect_y = center.y - (ROI_WIDTH / 2);
	if (rect_y < 0){
		rect_y = 0;
	}
	else if (rect_y + ROI_WIDTH > image.size().height){
		rect_y = image.size().height - ROI_WIDTH;
	}
	Mat image_sub_mat = image(Rect(rect_x, rect_y, ROI_WIDTH, ROI_WIDTH));
	Mat zeros_sub_mat = zeros(Rect(rect_x, rect_y, ROI_WIDTH, ROI_WIDTH));
	image_sub_mat.copyTo(zeros_sub_mat);
	return zeros;

}

Point2d ball_tracker::camera_2d_to_catcher(double x, double y){
	double catcher_x = (x + CATCHER_X_OFFSET) / CATCHER_X_SCALE;
	double catcher_y = (y + CATCHER_Y_OFFSET) / CATCHER_Y_SCALE;
	return Point2d(catcher_x, catcher_y);
}
