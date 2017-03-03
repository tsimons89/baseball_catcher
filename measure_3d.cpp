#include "stdafx.h"
#include "measure_3d.h"

vector<Point2d> get_4_corners(Mat image, Size board_size){
	vector<Point2d> all_corners,four_corners;
	vector<Point2f> float_corners;
	TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
	bool pattern_found = findChessboardCorners(image, board_size, all_corners);
	if (pattern_found){
		Mat(all_corners).convertTo(float_corners,CV_32F);
		cornerSubPix(image, float_corners, Size(5, 5), Size(-1, -1), criteria);
		four_corners.push_back(all_corners.at(0));
		four_corners.push_back(all_corners.at(board_size.width - 1));
		four_corners.push_back(all_corners.at(all_corners.size() - 1));
		four_corners.push_back(all_corners.at(all_corners.size() - board_size.width));
	}
	else{
		all_corners.clear();
	}
	return four_corners;
}

vector<Point2d> get_4_corners_from_file(string filename,Size board_size){
	Mat image = imread(filename);
	if (image.channels()==3)
		cvtColor(image, image, CV_RGB2GRAY);
	return get_4_corners(image, board_size);
}

void show_4_corners_from_file(string filename, Size board_size){
	vector<Point2d> corners = get_4_corners_from_file(filename, board_size);
	show_points(filename, corners);
}

void show_rectified_corners_from_file(string filename, Size board_size, string clib_filename, string rect_xml_filename, int which_cam){
	vector<Point2d> corners = get_4_corners_from_file(filename, board_size);
	vector<Point2d> rectified_corners = my_undistort_points(corners, clib_filename, rect_xml_filename,which_cam);
	show_points(filename, rectified_corners);
}



Mat draw_points(string filename, vector<Point2d> points){
	Mat image = imread(filename);
	for (Point2d point : points)
		circle(image, point, 2, Scalar(0, 255, 0), 2);
	return image;
}

void show_points(string filename, vector<Point2d> points){
	namedWindow("points");
	imshow("points", draw_points(filename,points));
	waitKey(0);
}

vector<Point2d> my_undistort_points(vector<Point2d> points, string calib_filename, string rect_xml_filename, int which_cam){
	vector<Point2d> output;
	undistortPoints(points, output, get_xml_object<Mat>(calib_filename, "intrinsic"), get_xml_object<Mat>(calib_filename, "distortion"),
		get_xml_object<Mat>(rect_xml_filename, "r" + to_string(which_cam)), get_xml_object<Mat>(rect_xml_filename, "p" + to_string(which_cam)));
	return output;
}



vector<Point3d> get_3d_points(vector<Point2d> points_l, vector<Point2d> points_r, string left_xml, string right_xml,string rect_xml, string stereo_xml, int which){
	vector<Point3d> ret;
	vector<Point2d> left_points_rectified = my_undistort_points(points_l, left_xml, rect_xml, 1);
	vector<Point2d> right_points_rectified = my_undistort_points(points_r, right_xml, rect_xml, 2);
	for (int i = 0; i < points_l.size(); i++)
		ret.push_back(transform_to_3d(left_points_rectified.at(i), right_points_rectified.at(i), rect_xml,1));
	return ret;
}

vector<Point3d> get_3d_points(vector<vector<Point2d>> points, string left_xml, string right_xml, string rect_xml, string stereo_xml, int which){
	vector<Point2d> left_points = extract_camera_points(points, 1);
	vector<Point2d> right_points = extract_camera_points(points, 2);
	return get_3d_points(left_points, right_points, left_xml, right_xml, rect_xml, stereo_xml, which);
}

vector<Point2d> extract_camera_points(vector<vector<Point2d>> comb_points, int which){
	vector<Point2d> ext_points;
	for (auto point_pair : comb_points){
		ext_points.push_back(point_pair.at(which - 1));
	}
	return ext_points;
}

Point3d transform_to_3d(Point2d point_l, Point2d point_r,string rect_xml,int which){
	Mat q = get_xml_object<Mat>(rect_xml, "q");
	vector<Point3d> point_disparity_vec;
	point_disparity_vec.push_back(Point3d(point_l.x, point_l.y, point_l.x - point_r.x));
	vector<Point3d> ret_point_vec;
	perspectiveTransform(point_disparity_vec, ret_point_vec, q);
	return ret_point_vec.at(0);
}



Point3d coefs_to_point_3d(Mat a_coef, Mat b_coef, Mat c_coef,Mat rotation, Mat translation,int which){
	Mat A = combine_mats(a_coef, -b_coef, c_coef);
	Mat B = -rotation.t()*translation;
	Mat X;
	solve(A, B, X);
	return x_to_point_3d(a_coef, b_coef, X,which);
}

Point3d x_to_point_3d(Mat a_coef, Mat b_coef, Mat X,int which){
	Point3d ret_point;
	double a = X.at<double>(0, 0);
	double b = X.at<double>(1, 0);
	ret_point.x = (a*a_coef.at<double>(0, 0) + b*b_coef.at<double>(0, 0)) / 2;
	ret_point.y = (a*a_coef.at<double>(1, 0) + b*b_coef.at<double>(1, 0)) / 2;
	ret_point.z = (a*a_coef.at<double>(2, 0) + b*b_coef.at<double>(2, 0)) / 2;
	return ret_point;
}

Mat mat_dot_point(Mat mat,Point2d point){
	vector<Point3d> vec;
	vec.push_back(Point3d(point.x, point.y, 1));
	Mat point_mat = Mat(vec).reshape(1).t();
	Mat ret = mat*point_mat;
	return ret;
}

Mat combine_mats(Mat a, Mat b, Mat c){
	vector<Mat> comb_vec = { a, b, c };
	Mat ret;
	hconcat(comb_vec, ret);
	return ret;
}

Point3d mat_to_Point3d(Mat m){
	Point3d ret;
	ret.x = m.at<double>(0,0);
	ret.y = m.at<double>(1, 0);
	ret.z = m.at<double>(2, 0);
	return ret;
}

vector<Point3d> get_3d_corners_from_file(string left_file, string right_file, string left_xml, string right_xml, string rect_xml, string stereo_xml,Size board_size,int which){
	vector<Point2d> left_points = get_4_corners_from_file(left_file,board_size);
	vector<Point2d> right_points = get_4_corners_from_file(right_file, board_size);
	return get_3d_points(left_points, right_points, left_xml, right_xml, rect_xml,stereo_xml, which);
}


