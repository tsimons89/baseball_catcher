#pragma once
#include "opencv2\opencv.hpp"

using namespace std;
using namespace cv;


vector<Point2d> get_4_corners(Mat image, Size board_size);
vector<Point2d> get_4_corners_from_file(string filename, Size board_size);
void show_4_corners_from_file(string filename, Size board_size);
void show_rectified_corners_from_file(string filename, Size board_size, string clib_filename, string rect_xml_filename, int which_cam);
Mat draw_points(string filename, vector<Point2d> points);
void show_points(string filename, vector<Point2d> points);
vector<Point2d> my_undistort_points(vector<Point2d> points, string calib_filename, string rect_xml_filename,int which_cam);
vector<Point3d> get_3d_points(vector<Point2d> points_l, vector<Point2d> points_r, string left_xml, string right_xml, string rect_xml, string stereo_xml, int which);
vector<Point3d> get_3d_points(vector<vector<Point2d>> points, string left_xml, string right_xml, string rect_xml, string stereo_xml, int which);
vector<Point2d> extract_camera_points(vector<vector<Point2d>> comb_points, int which);
Point3d transform_to_3d(Point2d point_l, Point2d point_r, string rect_xml,int which);
//Point3d transform_to_3d(Point2d point_l, Point2d point_r, string left_xml, string right_xml, string stereo_xml, int which);
Point3d coefs_to_point_3d(Mat a_coef, Mat b_coef, Mat c_coef, Mat rotation, Mat translation,int which);
Point3d x_to_point_3d(Mat a_coef, Mat b_coef, Mat X,int which);
Mat mat_dot_point(Mat mat,Point2d point);
Mat combine_mats(Mat a, Mat b, Mat c);
Point3d mat_to_Point3d(Mat m);
vector<Point3d> get_3d_corners_from_file(string left_file, string right_file, string left_xml, string right_xml, string rect_xml, string stereo_xml, Size board_size,int which);

template<class T>
T get_xml_object(string xml_filename, string object_name){
	T object;
	FileStorage file(xml_filename, FileStorage::READ);
	file[object_name] >> object;
	return object;
}
template<class T>
void write_xml_object(string xml_filename, string object_name, T object){
	FileStorage file(xml_filename, FileStorage::WRITE);
	file << object_name << object;
}
