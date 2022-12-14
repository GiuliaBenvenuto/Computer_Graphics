#include "raster.h"	
#include <iostream>

void rasterize_triangle(const Program& program, const UniformAttributes& uniform, const VertexAttributes& v1, const VertexAttributes& v2, const VertexAttributes& v3, FrameBuffer& frameBuffer)
{
	// Collect coordinates into a matrix and convert to canonical representation
	Eigen::Matrix<float, 3, 4> p;
	p.row(0) = v1.position.array() / v1.position[3];
	p.row(1) = v2.position.array() / v2.position[3];
	p.row(2) = v3.position.array() / v3.position[3];


	// Coordinates are in -1..1, rescale to pixel size (x,y only)
	// ViewPort transformation. 
	// Pass from the bi-unit cube to the the window in device coordinates which has the origin on the lower left corner and it 
	// spans integers from 0 to the height and from 0 to the width.
	// Shift by 1 on x and y and devide by 2 because of rescale. 
	// At this point, all the points of our scene, belong to the rectangle which corresponds to the image. 
	p.col(0) = ((p.col(0).array() + 1.0) / 2.0) * frameBuffer.rows();
	p.col(1) = ((p.col(1).array() + 1.0) / 2.0) * frameBuffer.cols();


	// Find bounding box in pixels. Finding a bounding box for the triangle in order to rasterize only the points inside the 
	// viewport. The points that fall outside are discarded. Finding it corresponds to find the min and the max
	// coordinates in both directions, of all the points that we have. 
	int lx = std::floor(p.col(0).minCoeff());
	int ly = std::floor(p.col(1).minCoeff());
	int ux = std::ceil(p.col(0).maxCoeff());
	int uy = std::ceil(p.col(1).maxCoeff());


	// Clamp to framebuffer. (Clipping) Ex: if the triangle is totally outside the viewport the rasterizer will discard it. 
	// If the triangle is totally inside nothing happens, if it is partly inside, only the inside part will be rasterized. 
	lx = std::min(std::max(lx, int(0)), int(frameBuffer.rows() - 1));
	ly = std::min(std::max(ly, int(0)), int(frameBuffer.cols() - 1));
	ux = std::min(std::max(ux, int(0)), int(frameBuffer.rows() - 1));
	uy = std::min(std::max(uy, int(0)), int(frameBuffer.cols() - 1));


	// Build the implicit triangle representation. 
	// Solve a linear system (see slide 13/30 slide 09 "Rasterization Theory")
	// The unknowns are the baricentric coordinates that we want to compute and the known therms are the coordinates of 
	// the point that we want to test. These lines are the assembly of the matrix A. 
	Eigen::Matrix3f A;
	A.col(0) = p.row(0).segment(0, 3);
	A.col(1) = p.row(1).segment(0, 3);
	A.col(2) = p.row(2).segment(0, 3);
	A.row(2) << 1.0, 1.0, 1.0; // last row

	// Computing the inverse of the matrix and use it after to solve the system.
	Eigen::Matrix3f Ai = A.inverse();

	// Rasterize the triangle.
	// Scan the clamping bounding box around the primitive (triangle). 
	// Scan using the max and the min as limits (computed before).
	for (unsigned i = lx; i <= ux; i++)
	{
		for (unsigned j = ly; j <= uy; j++)
		{
			// The pixel center is offset by 0.5, 0.5
			// Shift in order to move to the centere of the pixel. 
			Eigen::Vector3f pixel(i + 0.5, j + 0.5, 1);

			// Solve the system.
			// Now in b we have the coordinates of the pixel of the fragment that we want to test.
			Eigen::Vector3f b = Ai * pixel;

			if (b.minCoeff() >= 0)
			{
				// We might have several attributes for each vertex and these attributes are 
				// interpolated using alpha, beta and gamma as weights. 
				// For all the attributes that we get from the vertex, they are interpolated using
				// the three vertices and b[0], b[1], b[2] (weights).
				VertexAttributes va = VertexAttributes::interpolate(v1, v2, v3, b[0], b[1], b[2]);

				// Only render fragments within the bi-unit cube
				// Safety check, check that the depth of the fragment is within -1 and 1. 
				if (va.position[2] >= -1 && va.position[2] <= 1)
				{
					// Generate a fragment by calling the FragmentShader and then write this fragment to the 
					// FrameBuffer by using the BlendingShader. 
					FragmentAttributes frag = program.FragmentShader(va, uniform);
					frameBuffer(i, j) = program.BlendingShader(frag, frameBuffer(i, j));
					// These two lines are calling the two next stages of the pipeline. 
				}
			}
		}
	}
}




void rasterize_triangles(const Program& program, const UniformAttributes& uniform, const std::vector<VertexAttributes>& vertices, FrameBuffer& frameBuffer)
{
	// Call the rasterize_trinagle function for each triangle. So now the input is a vector of vertices, 
	// The conventional assumption is that each three vertices form a triangle. 
	// Call vertex shader on all vertices
	std::vector<VertexAttributes> v(vertices.size());
	for (unsigned i = 0; i < vertices.size(); i++)
		// Calling thhe VertexShader on all the vertices which generates v[i] which is a new vertex, a modified 
		// version of the input. The VertexShader does different operation depending on what we write inside the 
		// VertexShader function. 
		v[i] = program.VertexShader(vertices[i], uniform); // Buffer v of the transformed vertices.


	// Call the rasterization function on every triangle.
	// Takes the first three vertices and call the rasterizer, takes the next three and calls the rasterizer and so on. 
	// Calls the rasterize_triangle for three consecutive vertices.
	for (unsigned i = 0; i < vertices.size() / 3; i++)
		rasterize_triangle(program, uniform, v[i * 3 + 0], v[i * 3 + 1], v[i * 3 + 2], frameBuffer);
}




void rasterize_line(const Program& program, const UniformAttributes& uniform, const VertexAttributes& v1, const VertexAttributes& v2, float line_thickness, FrameBuffer& frameBuffer)
{
	// Collect coordinates into a matrix and convert to canonical representation
	Eigen::Matrix<float, 2, 4> p;
	p.row(0) = v1.position.array() / v1.position[3];
	p.row(1) = v2.position.array() / v2.position[3];

	// Coordinates are in -1..1, rescale to pixel size (x,y only). (Viewport transformation)
	p.col(0) = ((p.col(0).array() + 1.0) / 2.0) * frameBuffer.rows();
	p.col(1) = ((p.col(1).array() + 1.0) / 2.0) * frameBuffer.cols();

	// Find bounding box in pixels, adding the line thickness
	// assume that the line has a thickness. all the centers that belong to the band around the line will
	// generate a colored fragment. 
	int lx = std::floor(p.col(0).minCoeff() - line_thickness);
	int ly = std::floor(p.col(1).minCoeff() - line_thickness);
	int ux = std::ceil(p.col(0).maxCoeff() + line_thickness);
	int uy = std::ceil(p.col(1).maxCoeff() + line_thickness);

	// Clamp to framebuffer
	lx = std::min(std::max(lx, int(0)), int(frameBuffer.rows() - 1));
	ly = std::min(std::max(ly, int(0)), int(frameBuffer.cols() - 1));
	ux = std::min(std::max(ux, int(0)), int(frameBuffer.rows() - 1));
	uy = std::min(std::max(uy, int(0)), int(frameBuffer.cols() - 1));

	// We only need the 2d coordinates of the endpoints of the line
	Eigen::Vector2f l1(p(0, 0), p(0, 1));
	Eigen::Vector2f l2(p(1, 0), p(1, 1));

	// Parametrize the line as l1 + t (l2-l1)
	float t = -1;
	float ll = (l1 - l2).squaredNorm();

	// Rasterize the line
	// See 31/03/21 lesson at 5:52 of 17:51
	for (unsigned i = lx; i <= ux; i++)
	{
		for (unsigned j = ly; j <= uy; j++)
		{
			// The pixel center is offset by 0.5, 0.5
			Eigen::Vector2f pixel(i + 0.5, j + 0.5);

			if (ll == 0.0)
				// The segment has zero length
				t = 0;
			else
			{
				// Project p on the line
				t = (pixel - l1).dot(l2 - l1) / ll;
				// Clamp between 0 and 1
				t = std::fmax(0, std::fmin(1, t));
			}

			Eigen::Vector2f pixel_p = l1 + t * (l2 - l1);

			if ((pixel - pixel_p).squaredNorm() < (line_thickness * line_thickness))
			{
				VertexAttributes va = VertexAttributes::interpolate(v1, v2, v1, 1 - t, t, 0);
				FragmentAttributes frag = program.FragmentShader(va, uniform);
				frameBuffer(i, j) = program.BlendingShader(frag, frameBuffer(i, j));
			}
		}
	}
}




void rasterize_lines(const Program& program, const UniformAttributes& uniform, const std::vector<VertexAttributes>& vertices, float line_thickness, FrameBuffer& frameBuffer)
{
	// Call vertex shader on all vertices
	std::vector<VertexAttributes> v(vertices.size());
	for (unsigned i = 0; i < vertices.size(); i++)
		v[i] = program.VertexShader(vertices[i], uniform);

	// Call the rasterization function on every line
	// Take the vertices two by two and it rasterizes the line segment.
	for (unsigned i = 0; i < vertices.size() / 2; i++)
		rasterize_line(program, uniform, v[i * 2 + 0], v[i * 2 + 1], line_thickness, frameBuffer);
}




// Takes the FrameBuffer which contains our image, it scans the FrameBuffer, it converts each color component of the FrameBuffer into a 
// color component for the image, assigns the color component computed to the corresponding entry of the image. 
// Linearized matrix which will be taken from the Main at the end of the code and will be written into a .png file. 
void framebuffer_to_uint8(const FrameBuffer& frameBuffer, std::vector<uint8_t>& image)
{
	const int w = frameBuffer.rows();					// Image width
	const int h = frameBuffer.cols();					// Image height
	const int comp = 4;									// 4 Channels Red, Green, Blue, Alpha
	const int stride_in_bytes = w * comp;					// Length of one row in bytes
	image.resize(w * h * comp, 0);							// The image itself;

	for (unsigned wi = 0; wi < w; ++wi)
	{
		for (unsigned hi = 0; hi < h; ++hi)
		{
			unsigned hif = h - 1 - hi;
			image[(hi * w * 4) + (wi * 4) + 0] = frameBuffer(wi, hif).color[0];
			image[(hi * w * 4) + (wi * 4) + 1] = frameBuffer(wi, hif).color[1];
			image[(hi * w * 4) + (wi * 4) + 2] = frameBuffer(wi, hif).color[2];
			image[(hi * w * 4) + (wi * 4) + 3] = frameBuffer(wi, hif).color[3];
		}
	}
}
