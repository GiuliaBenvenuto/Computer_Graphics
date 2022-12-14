// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION 
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
// Avoid us to specify Eigen::
using namespace Eigen;


double check_pgram_intersection_t(Vector3d pgram_origin, Vector3d pgram_u, Vector3d pgram_v, Vector3d ray_origin, Vector3d ray_direction);
double check_sphere_intersection_t(double sphere_radius, Vector3d sphere_centre, Vector3d ray_origin, Vector3d ray_direction);


void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");

	MatrixXd C = MatrixXd::Zero(800, 800); // Store the color --> grey level intensity
	MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask


	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1, 1, 1);
	Vector3d x_displacement(2.0 / C.cols(), 0, 0);
	Vector3d y_displacement(0, -2.0 / C.rows(), 0);


	// Single light source
	const Vector3d light_position(-1, 1, 1);


	// Basic RayTracer: (Parallel)
	// Two for which iterate the pixels
	for (unsigned i = 0; i < C.cols(); ++i) {
		for (unsigned j = 0; j < C.rows(); ++j) {

			// Prepare the ray:
			// Compute the origin of the ray and then the ray direction

			// The ray starts from the orthographic camera (observer - origin)
			Vector3d ray_origin = origin + double(i) * x_displacement + double(j) * y_displacement;

			// The direction of all the ray is parallel to the direction of w but opposite 
			Vector3d ray_direction = RowVector3d(0, 0, -1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
			const double sphere_radius = 0.9;

			// To check if the ray intersect the sphere we only need to compare the ray norm with the radius of the sphere
			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0), ray_on_xy(1), sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				// See tablet notes
				C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i, j) = std::max(C(i, j), 0.);

				A(i, j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C, C, C, A, filename);

}



void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
	MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1, 1, 1);
	Vector3d x_displacement(2.0 / C.cols(), 0, 0);
	Vector3d y_displacement(0, -2.0 / C.rows(), 0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.75, -0.75, 0.5);			// a
	Vector3d pgram_u = RowVector3d(-1.25, 0, 0);		// a-c
	Vector3d pgram_v = RowVector3d(-0.25, -1.5, 0);		// a-b

	// Single light source
	const Vector3d light_position(-1, 1, 1);

	for (unsigned i = 0; i < C.cols(); ++i) {
		for (unsigned j = 0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i) * x_displacement + double(j) * y_displacement;  // p (in this case is the position of the pixel) 
			Vector3d ray_direction = RowVector3d(0, 0, -1);  // d

			Vector2d ray_on_xy(ray_origin(0), ray_origin(1));

			// TODO: Check if the ray intersects with the parallelogram
			// Call the function to check if there is an intersection:
			double t = check_pgram_intersection_t(pgram_origin, pgram_u, pgram_v, ray_origin, ray_direction);

			if (t != 0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0), ray_on_xy(1), pgram_origin(2));

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i, j) = std::max(C(i, j), 0.);

				// Disable the alpha mask for this pixel
				A(i, j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C, C, C, A, filename);
}



double check_pgram_intersection_t(Vector3d pgram_origin, Vector3d pgram_u, Vector3d pgram_v, Vector3d ray_origin, Vector3d ray_direction)
{
	// pgram_u --> a-c
	// pgram_v --> a-b

	// Calculate (a-e):
	Vector3d a_e = RowVector3d(pgram_origin - ray_origin);


	// Fill the matrices:
	Matrix3f A;
	Vector3f b;
	A << pgram_v(0), pgram_u(0), ray_direction(0), pgram_v(1), pgram_u(1), ray_direction(1), pgram_v(2), pgram_u(2), ray_direction(2);
	b << a_e(0), a_e(1), a_e(2);

	// Solve the system:
	Vector3f x = A.colPivHouseholderQr().solve(b);

	// x(0)=u, x(1)=v, x(2)=t

	// Check intersection conditions:
	if (x(2) > 0 && x(0) >= 0 && x(1) >= 0 && x(0) <= 1 && x(1) <= 1) {
		return x(2);
	}
	else {
		return 0;
	}
}


void raytrace_pgram_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
	MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1, 1, 1);
	Vector3d x_displacement(2.0 / C.cols(), 0, 0);
	Vector3d y_displacement(0, -2.0 / C.rows(), 0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.75, -0.75, 0.5);			// a
	Vector3d pgram_u = RowVector3d(-1.25, 0, -0.25);		// a-c
	Vector3d pgram_v = RowVector3d(-0.25, -1.5, -0.25);		// a-b

	// Single light source
	const Vector3d light_position(-1, 1, 1);

	for (unsigned i = 0; i < C.cols(); ++i) {
		for (unsigned j = 0; j < C.rows(); ++j) {

			// TODO: Prepare the ray (origin point and direction)
			// In the perspective projection the origin of the ray is "e" (origin of camera reference system).
			Vector3d ray_origin = RowVector3d(-1, 1, 3);	// e

			// The direction is different for each pixel and we have to recompute it for each of them. 
			// Direction value = p-e 
			// Compute p (position of the pixel):
			Vector3d p = origin + double(i) * x_displacement + double(j) * y_displacement;	// p

			// Compute p-e:
			Vector3d ray_direction = RowVector3d(p - ray_origin);  // d

			// Vector2d ray_on_xy(p(0), p(1));

			// TODO: Check if the ray intersects with the parallelogram
			double t = check_pgram_intersection_t(pgram_origin, pgram_u, pgram_v, ray_origin, ray_direction);

			if (t != 0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				// Intersection point: p(t) = e + td
				Vector3d ray_intersection = ray_origin + (t * ray_direction);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i, j) = std::max(C(i, j), 0.);

				// Disable the alpha mask for this pixel
				A(i, j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C, C, C, A, filename);
}


double check_sphere_intersection_t(double sphere_radius, Vector3d sphere_centre, Vector3d ray_origin, Vector3d ray_direction)
{
	// e-c:
	Vector3d e_c = RowVector3d(ray_origin - sphere_centre);

	// Coefficient a:
	double a = ray_direction.transpose() * ray_direction;

	// Coefficient b: 
	double b = 2 * ray_direction.transpose() * e_c;

	// Coefficient c:
	double c = (e_c.transpose() * e_c) - (sphere_radius * sphere_radius);

	double delta = (b * b) - (4 * a * c);

	if (delta < 0) {
		return 0;
	}
	else if (delta == 0) {
		return (-b / (2 * a));
	}
	else {

		double t1 = ((-b) + sqrt(delta)) / (2 * a);
		double t2 = ((-b) - sqrt(delta)) / (2 * a);

		if (t1 < t2) { return t1; }
		else { return t2; }
	}
}


void raytrace_sphere_perspective() {
	std::cout << "Simple ray tracer, one sphere with perspective projection" << std::endl;

	const std::string filename("sphere_perspective.png");

	MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
	MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1, 1, 1);
	Vector3d x_displacement(2.0 / C.cols(), 0, 0);
	Vector3d y_displacement(0, -2.0 / C.rows(), 0);

	// Single light source
	const Vector3d light_position(-1, 1, 1);

	// Sphere:
	const double sphere_radius = 0.9;
	const Vector3d sphere_centre(0, 0, 0);

	// RayTracer:
	for (unsigned i = 0; i < C.cols(); ++i) {
		for (unsigned j = 0; j < C.rows(); ++j) {

			// TODO: Prepare the ray (origin point and direction)
			// In the perspective projection the origin of the ray is "e" (origin of camera reference system).
			Vector3d ray_origin = RowVector3d(-1,1,3);
			//(0,0, 3);	// e

			// The direction is different for each pixel and we have to recompute it for each of them. 
			// Direction value = p-e 
			// Compute p (position of the pixel):
			Vector3d p = origin + double(i) * x_displacement + double(j) * y_displacement;	// p

			// Compute p-e:
			Vector3d ray_direction = RowVector3d(p - ray_origin);  // d

			// TODO: Check if the ray intersects with the parallelogram
			double t = check_sphere_intersection_t(sphere_radius, sphere_centre, ray_origin, ray_direction);

			if (t != 0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				// Intersection point: p(t) = e + td
				Vector3d ray_intersection = ray_origin + (t * ray_direction);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i, j) = std::max(C(i, j), 0.);

				// Disable the alpha mask for this pixel
				A(i, j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C, C, C, A, filename);
}



void raytrace_shading() {
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");

	// Some matrices to store the RGB components:
	MatrixXd R = MatrixXd::Zero(800, 800); // Store the color
	MatrixXd G = MatrixXd::Zero(800, 800); // Store the color
	MatrixXd B = MatrixXd::Zero(800, 800); // Store the color
	MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

	const Vector3d color(1, 1, 0);

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1, 1, 1);
	Vector3d x_displacement(2.0 / R.cols(), 0, 0);
	Vector3d y_displacement(0, -2.0 / R.rows(), 0);

	// Single light source
	const Vector3d light_position(-1, 1, 1);
	
	double ambient = 0.3;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	// Define coefficients:
	double Kd = 0.5;
	double Ks = 0.5;
	double p = 1000;

	for (unsigned i = 0; i < R.cols(); ++i) {
		for (unsigned j = 0; j < R.rows(); ++j) {
			// Prepare the ray:
			Vector3d ray_origin = origin + double(i) * x_displacement + double(j) * y_displacement;
			Vector3d ray_direction = RowVector3d(0, 0, -1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0), ray_on_xy(1), sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// TODO: Add shading parameter here
				diffuse(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;
				specular(i, j) = pow(((ray_origin - ray_intersection) + (light_position - ray_intersection)).normalized().transpose() * ray_normal, p);

				// Simple diffuse model
				R(i, j) = ambient + (Kd * diffuse(i, j)) + (Ks * specular(i, j));
				G(i, j) = ambient + (Kd * diffuse(i, j)) + (Ks * specular(i, j));
				B(i, j) = ambient + (Kd * diffuse(i, j)) + (Ks * specular(i, j));

				// Clamp to zero
				R(i, j) = (std::max(R(i, j), 0.)) * color(0);
				G(i, j) = (std::max(G(i, j), 0.)) * color(1);
				B(i, j) = (std::max(B(i, j), 0.)) * color(2);

				// Disable the alpha mask for this pixel
				A(i, j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(R, G, B, A, filename);
}


int main() {
	 raytrace_sphere();  
	 raytrace_parallelogram();  
	 raytrace_pgram_perspective();  
	 raytrace_sphere_perspective(); 
	 raytrace_shading();  

	return 0;
}
