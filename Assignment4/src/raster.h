#pragma once

#include <Eigen/Core>
#include <Eigen/LU> // Needed for .inverse()
#include <vector>
#include <string>
#include "attributes.h"



// Stores the final image
// Definition of the type for the FrameBuffer, it will contains the final image that will be written in a .png file.
// The final image will be wiritten on a png and the FrameBuffer will contains all the informations that we will see as output into 
// the image. The FrameBuffer is a matix typed with FrameBufferAttributes which are inside the file "attributes.h".
typedef Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> FrameBuffer;



// Contains the three shaders used by the rasterizer
// The pipeline needs us to specify at three diferent stages what it shall do. This is written three pieces of code: Vertex shader, Fragment shader 
// and Blending shader. These three shaders are called "Program". That's the program running on our GPU. 
// These are fields of our class "Program", all of them are functions. 
// We can define our own functions and assign them to these shaders.
class Program
{
public:
	// Vertex Shader
	// Takes in input some VertexAttributes and UniformAttributes. 
	// Generates as output VertexAttributes, new attributes for that vertex.
	std::function<VertexAttributes(const VertexAttributes&, const UniformAttributes&)> VertexShader;

	// Fragment Shader
	// Takes in input some VertexAttributes and UniformAttributes.
	// Generates as output FragmentAttributes.
	std::function<FragmentAttributes(const VertexAttributes&, const UniformAttributes&)> FragmentShader;

	// Blending Shader
	// Takes in input some VertexAttributes and FrameBufferAttributes.
	// Generates as output FrameBufferAttributes.
	std::function<FrameBufferAttributes(const FragmentAttributes&, const FrameBufferAttributes&)> BlendingShader;

	// We have to implement the body of these functions in our Program which will be the Main, in the Main we will
	// set the content of those functions. 
};




// Rasterizes a single triangle v1,v2,v3 using the provided program and uniforms.
// Note: v1, v2, and v3 needs to be in the canonical view volume (i.e. after being processed by the vertex shader)
// Takes a triangle in therms of VertexAttributes of its three vertices, it takes a FrameBuffer which tells how much it has to split, 
// takes the uniform if there's any, takes the Program and it performs rasterization for this triangle. 
void rasterize_triangle(const Program& program, const UniformAttributes& uniform, const VertexAttributes& v1, const VertexAttributes& v2, const VertexAttributes& v3, FrameBuffer& frameBuffer);




// Rasterizes a collection of triangles, assembling one triangle for each 3 consecutive vertices.
// Note: the vertices will be processed by the vertex shader
// Does the same things done in rasterize_triangle but for all the triangles that come in input, repeats the same operation for all the triangles. 
// It reads three vertices at time and interprets them as a triangle then calls the rasterize_triangle function.
// Stream of vertices (vector).
void rasterize_triangles(const Program& program, const UniformAttributes& uniform, const std::vector<VertexAttributes>& vertices, FrameBuffer& frameBuffer);




// Rasterizes a single line v1,v2 of thickness line_thickness using the provided program and uniforms.
// Note: v1, v2 needs to be in the canonical view volume (i.e. after being processed by the vertex shader)
void rasterize_line(const Program& program, const UniformAttributes& uniform, const VertexAttributes& v1, const VertexAttributes& v2, float line_thickness, FrameBuffer& frameBuffer);




// Rasterizes a collection of lines, assembling one line for each 2 consecutive vertices.
// Note: the vertices will be processed by the vertex shader
void rasterize_lines(const Program& program, const UniformAttributes& uniform, const std::vector<VertexAttributes>& vertices, float line_thickness, FrameBuffer& frameBuffer);




// Exports the framebuffer to a uint8 raw image
// Writes the output from the FrameBuffer to the image, it just takes the color from the FrameBuffer and writes it down to the image. 
void framebuffer_to_uint8(const FrameBuffer& frameBuffer, std::vector<uint8_t>& image);