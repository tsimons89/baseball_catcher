#include "stdafx.h"
#include "trajectory.h"
#include "measure_3d.h"
#include <Filter.h>

void plot_points_3d(vector<vector<Point2d>> points, string left_xml, string right_xml, string rect_xml,string stereo_xml, int which,int x_poly_order,int y_poly_order,double catcher_z){
	vector<Point3d> points_3d = get_3d_points(points, left_xml, right_xml, rect_xml,stereo_xml, which);
	points_3d.push_back(get_line_z_point(points_3d, catcher_z, x_poly_order, y_poly_order));
	show_points_3d(points_3d,x_poly_order,y_poly_order);
}

void show_points_3d(vector<Point3d> points,int x_poly_order,int y_poly_order){
	Mat z_x_plot = get_plot(points, 'z', 'x', x_poly_order,y_poly_order);
	Mat z_y_plot = get_plot(points, 'z', 'y', x_poly_order, y_poly_order);
	Point3d catch_point = points.back();
	putText(z_y_plot, "(" + to_string(catch_point.x) + "," + to_string(catch_point.y) + "," + to_string(catch_point.z) + ")", Point(20, 20), FONT_HERSHEY_SIMPLEX, .75, Scalar(255, 0, 0));

	namedWindow("z_x_plot");
	namedWindow("z_y_plot");
	imshow("z_x_plot", z_x_plot);
	imshow("z_y_plot", z_y_plot);
	waitKey(0);
	destroyAllWindows();
}

Point2d get_cord_range(vector<Point3d>points, char dim){
	double max = get_max_cord(points, dim);
	double min = get_min_cord(points, dim);
	return Point2d(min, max);
}	

double get_max_cord(vector<Point3d>points, char dim){
	int max = INT_MIN;
	for (auto point : points){
		double dim_value = get_dim_value(point, dim);
		if (dim_value > max)
			max = dim_value;
	}
	return max;
}
double get_min_cord(vector<Point3d>points, char dim){
	int min = INT_MAX;
	for (auto point:points){
		double dim_value = get_dim_value(point, dim);
		if (dim_value < min)
			min = dim_value;
	}
	return min;
}

double get_dim_value(Point3d point, char dim){
	switch (dim)
	{
	case 'x':
		return point.x;
	case'y':
		return point.y;
	case 'z':
		return point.z;
	default:
		return 0;
	}
}

Mat get_plot(vector<Point3d> points, char dim_1, char dim_2,int x_poly_order,int y_poly_order){
	Point2d range_1 = get_cord_range(points, dim_1);
	Point2d range_2 = get_cord_range(points, dim_2);
	Mat plot(get_range_mag(range_2), get_range_mag(range_1), CV_64FC3, CV_RGB(255,255,255));
	Point3d intercept = get_line_z_point(points, 0,x_poly_order, y_poly_order);
	Point3d end = get_line_z_point(points, range_1.y, x_poly_order, y_poly_order);
	vector<Point3d> line_points;
	line_points.push_back(intercept);
	line_points.push_back(end);
	vector<Point2d> line_2d = get_2d_normal_points(line_points, dim_1, dim_2, range_1, range_2);
	vector<Point2d> points_2d = get_2d_normal_points(points, dim_1, dim_2, range_1, range_2);
	vector<double> line_coefs;
	if (dim_2 == 'x')
		line_coefs = least_squares_poly_fit(points_2d, x_poly_order);
	else
		line_coefs = least_squares_poly_fit(points_2d, y_poly_order);

	for (auto point : points_2d){
		plot = plot_point(point, plot);
	}
	return draw_line(line_coefs, plot, Scalar(0, 0, 255));
	//cv::line(plot, line_2d.at(0), line_2d.at(1),Scalar(0,0,255));
	return plot;
}

int get_range_mag(Point2d range){
	return (range.y - range.x + 1)*PLOT_SCALE;
}

vector<Point2d> get_2d_normal_points(vector<Point3d> points, char dim_1, char dim_2, Point2d range_1, Point2d range_2){
	vector<Point2d> norm_2d_points;
	for (auto point : points){
		norm_2d_points.push_back(get_2d_normal_point(point, dim_1, dim_2, range_1, range_2));
	}
	return norm_2d_points;
}

vector<Point2d> get_2d_points(vector<Point3d> points, char dim_1, char dim_2){
	vector<Point2d> norm_2d_points;
	for (auto point : points){
		norm_2d_points.push_back(get_2d_point(point, dim_1, dim_2));
	}
	return norm_2d_points;
}


Point2d get_2d_normal_point(Point3d point, char dim_1, char dim_2, Point2d range_1, Point2d range_2){
	return Point2d(get_normalize_dim(point, dim_1, range_1), get_normalize_dim(point, dim_2, range_2));
}

Point2d get_2d_point(Point3d point, char dim_1, char dim_2){
	return Point2d(get_dim_value(point, dim_1), get_dim_value(point, dim_2));
}


double get_normalize_dim(Point3d point, char dim, Point2d range){
	return (get_dim_value(point, dim) - range.x)*PLOT_SCALE;
}



Mat plot_point(Point2d point, Mat plot){
	circle(plot, point, 3, Scalar(255, 0, 0));
	return plot;
}

double get_dim_line_point(vector<Point3d> points, char dim_1, char dim_2, double dim_1_value,int poly_order){
	vector<Point2d> points_2d = get_2d_points(points, dim_1, dim_2);
	vector<double> line_params = least_squares_poly_fit(points_2d, poly_order);
	return calc_line_y(line_params, dim_1_value);
}

Point3d get_line_z_point(vector<Point3d> points,double end_value,int x_poly_order,int y_poly_order){
	double y = get_dim_line_point(points, 'z', 'y', end_value, y_poly_order);
	double x = get_dim_line_point(points, 'z', 'x', end_value, x_poly_order);
	return Point3d(x, y, end_value);
}

Point3d get_z_plane_intercept(vector<vector<Point2d>> points, string left_xml, string right_xml, string rect_xml, string stereo_xml, int which,int plane_point,int x_poly_order,int y_poly_order){
	vector<Point3d> points_3d = get_3d_points(points, left_xml, right_xml, rect_xml, stereo_xml, which);
	return get_line_z_point(points_3d, plane_point,x_poly_order,y_poly_order);
}

double calc_line_error(vector<Point2d> line_points, vector<double> line_params){
	double error_sum = 0;
	for (auto point : line_points){
		error_sum += abs(point.y - calc_line_y(line_params, point.x));
	}
	return (error_sum / (double)line_points.size());
}

vector<double> least_squares_poly_fit(vector<Point2d> line_points,int poly_order){
	if (poly_order < 0){
		return least_squares_poly_best_fit(line_points, BEST_FIT_MIN_ORDER, BEST_FIT_MAX_ORDER, ALLOWED_ERROR);
	}
	Mat xy_vec = get_xv_vec(line_points, poly_order);
	Mat x_mat = get_x_mat(line_points, poly_order);
	Mat coefs = x_mat.inv()*xy_vec;
	vector<double> line_coefs = mat_to_vec(coefs);
	return line_coefs;
}

vector<double> least_squares_poly_best_fit(vector<Point2d> line_points, int min_order, int max_order, double allowed_avg_error){
	vector<double> best_params;
	double min_error = 10000;
	for (int order = min_order; order <= max_order; order++){
		vector<double> line_params = least_squares_poly_fit(line_points, order);
		double error = calc_line_error(line_points, line_params);
		if (error < allowed_avg_error){
			return line_params;
		}
		if (error < min_error){
			min_error = error;
			best_params = line_params;
		}
	}
	return best_params;
}


Mat get_xv_vec(vector<Point2d> line_points, int poly_order){
	Mat xy_vec = Mat::zeros(Size(1,poly_order + 1), CV_64F);
	for (int i = 0; i <= poly_order; i++){
		xy_vec.at<double>(i,0) = xy_sum(line_points, i);
	}
	return xy_vec;
}
double xy_sum(vector<Point2d> line_points, int exp){
	double sum = 0;
	for (auto point : line_points){
		sum += pow(point.x, exp) * point.y;
	}
	return sum;
}

double x_sum(vector<Point2d> line_points, int exp){
	double sum = 0;
	for (auto point : line_points){
		sum += pow(point.x, exp);
	}
	return sum;
}


Mat get_x_mat(vector<Point2d> line_points, int poly_order){
	Mat x_mat = Mat::zeros(Size(poly_order + 1, poly_order + 1), CV_64F);
	for (int i = 0; i <= poly_order; i++){
		for (int j = 0; j <= poly_order; j++){
			x_mat.at<double>(i, j) = x_sum(line_points, i + j);
		}
	}
	return x_mat;
}

vector<double> mat_to_vec(Mat m){
	vector<double> ret_vec;
	for (int i = 0; i < m.size().height; i++){
		ret_vec.push_back(m.at<double>(i, 0));
	}
	return ret_vec;
}

Mat draw_line(vector<double> line_coefs, Mat image,Scalar color){
	for (int i = 0; i < image.size().width; i++){
		circle(image, Point2d(i, calc_line_y(line_coefs, i)), 0, color);
	}
	return image;
}
double calc_line_y(vector<double> line_coefs, double x){
	double y = 0;
	for (int i = 0; i < line_coefs.size(); i++){
		y += line_coefs.at(i) * pow(x, i);
	}
	return y;
}