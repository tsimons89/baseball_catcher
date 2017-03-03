#pragma once
#include "opencv2\opencv.hpp"
#define CATCHER_Z_OFFSET 8
#define Y_POLY_ORDER 3
#define X_POLY_ORDER 1

using namespace std;
using namespace cv;
#define PLOT_SCALE 3

void plot_points_3d(vector<vector<Point2d>> points, string left_xml, string right_xml, string rect_xml, string stereo_xml, int which, int x_poly_order, int y_poly_order);
void show_points_3d(vector<Point3d> points, int x_poly_order, int y_poly_order);
Point2d get_cord_range(vector<Point3d>points, char dim);
double get_max_cord(vector<Point3d>points, char dim);
double get_min_cord(vector<Point3d>points, char dim);
double get_dim_value(Point3d point, char dim);
int get_range_mag(Point2d range);
vector<Point2d> get_2d_normal_points(vector<Point3d> points, char dim_1, char dim_2, Point2d range_1, Point2d range_2);
vector<Point2d> get_2d_points(vector<Point3d> points, char dim_1, char dim_2);
Point2d get_2d_normal_point(Point3d point, char dim_1, char dim_2, Point2d range_1, Point2d range_2);
Mat get_plot(vector<Point3d> points, char dim_1, char dim_2, int x_poly_order, int y_poly_order);
Point2d get_2d_point(Point3d point, char dim_1, char dim_2);
double get_normalize_dim(Point3d point, char dim, Point2d range);
Mat plot_point(Point2d point, Mat plot);
double get_dim_line_point(vector<Point3d> points, char dim_1, char dim_2, double dim_1_value, int poly_order);
Point3d get_line_z_point(vector<Point3d> points, double end_value, int x_poly_order, int y_poly_order);
Point3d get_z_plane_intercept(vector<vector<Point2d>> points, string left_xml, string right_xml, string rect_xml, string stereo_xml, int which, int plane_point, int x_poly_order, int y_poly_order);
vector<double> least_squares_poly_fit(vector<Point2d> line_points, int poly_order);
Mat get_xv_vec(vector<Point2d> line_points, int poly_order);
Mat get_x_mat(vector<Point2d> line_points, int poly_order);
double xy_sum(vector<Point2d> line_points, int exp);
double x_sum(vector<Point2d> line_points, int exp);
vector<double> mat_to_vec(Mat m);
Mat draw_line(vector<double> line_coefs, Mat image, Scalar color);
double calc_line_y(vector<double> line_coefs, double x);
