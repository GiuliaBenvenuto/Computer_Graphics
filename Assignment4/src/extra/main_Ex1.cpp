/*
* Exercize 1: "Load and render a 3D model".
* Extend the main_base code to load the same scenes used in Assignment 3 (bunny.off), and render it using rasterization in a uniform color.
* At this stage, you should see a correct silhouette of the object rendered (but no shading).
*/


// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Geometry>


// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"

using namespace std;



// Read a triangle mesh from an off file
// Code taken from the Assignment3, it loads a .off file passed as parameter and stores all the vertices and faces 
// inside two matrices: V and F. 
void load_off(const std::string &filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
    std::ifstream in(filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    V.resize(nv, 3);
    F.resize(nf, 3);
    for (int i = 0; i < nv; ++i) {
        in >> V(i, 0) >> V(i, 1) >> V(i, 2);
    }
    for (int i = 0; i < nf; ++i) {
        int s;
        in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
        assert(s == 3);
    }
}




int main() {

    // The Framebuffer storing the image rendered by the rasterizer
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(500, 500);

    // Global Constants (empty in this example)
    UniformAttributes uniform;

    // Basic rasterization program
    Program program;


    // The vertex shader is the identity, takes care of the view transformation done to move the bunny to the origin and scale 
    // the image in order to make it bigger. (Same as main_projective.cpp)
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        VertexAttributes out;
        out.position = uniform.view * va.position;
        return out;
    };


    // The fragment shader uses a fixed color.
    // Fixed chosen color: light blue. 
    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        return FragmentAttributes(0.6, 1, 1);
    };


    // The blending shader converts colors between 0 and 1 to uint8
    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };


    // Instantiate two matrices to store the vertices and the faces of the triangles that make the bunny. 
    // To store them call the function "load_off" taken from the Assignment3 passing to it the two matrices. 
    Eigen::MatrixXi F;
    Eigen::MatrixXd V;
    load_off("../data/bunny.off", V, F);


    // Now we have all the data stored inside F and V, save them into a vector so that three vertices represent a triangle. 
    // Vector to store all the vertices of the bunny.
    vector<VertexAttributes> bunnyVertices;
    for (int j = 0; j < F.rows(); ++j) {
        bunnyVertices.push_back(VertexAttributes(V(F(j, 0), 0), V(F(j, 0), 1), V(F(j, 0), 2)));
        bunnyVertices.push_back(VertexAttributes(V(F(j, 1), 0), V(F(j, 1), 1), V(F(j, 1), 2)));
        bunnyVertices.push_back(VertexAttributes(V(F(j, 2), 0), V(F(j, 2), 1), V(F(j, 2), 2)));
    }

    
    // Compute the center of the scene in order to translate the origin there. 
    Eigen::Vector3f bunnyCenter(V.col(0).sum() / V.rows(), V.col(1).sum() / V.rows(), V.col(2).sum() / V.rows());

    
    // Compute the 3D translation matrix in order to traslate the object to the computed position. 
    Eigen::Matrix4f translationMatrix;
    translationMatrix <<    1, 0, 0, - bunnyCenter(0),
                            0, 1, 0, - bunnyCenter(1),
                            0, 0, 1, - bunnyCenter(2),
                            0, 0, 0, 1;


    // Compute the 3D scaling matrix in order to scale the object and make it bigger.
    float scaleCoeff = 5;
    Eigen::Matrix4f scaleMatrix;
    scaleMatrix <<  scaleCoeff, 0, 0, 0,
                    0, scaleCoeff, 0, 0,
                    0, 0, scaleCoeff, 0,
                    0, 0, 0, 1;


    // Compute the model matrix which is the multiplication between all the transformation matrices computed. 
    Eigen::Matrix4f modelMatrix = scaleMatrix * translationMatrix;


    // Define the bounding box of the object.
    float l = -1,   b = -1,    n = -2;       // Left - bottom corner of the bounding box. 
    float r = 1,    t = 1,     f = -8;       // Right - top corner of the bounding box. 


    // Compute the othogonal matrix as in slide 6/26 of "08 - Viewing transformations".
    Eigen::Matrix4f orthMatrix;
    orthMatrix <<   2 / (r - l), 0, 0, -(r + l) / (r - l),
                    0, 2 / (t - b), 0, -(t + b) / (t - b),
                    0, 0, 2 / (n - f), -(n + f) / (n - f),
                    0, 0, 0, 1;


    // Slide 10/28 of "08 - Viewing tranformations".
    // Define eye position, gaze direction and view up vector to compute the unique transformations that converts world 
    // coordinates into camera coordinates.
    // Construction of the camera reference system.
    Eigen::Vector3f Eye(0, 0, 5);           // Eye position
    Eigen::Vector3f Gaze(0, 0, -1);         // Gaze direction
    Eigen::Vector3f ViewUp(0, 1, 0);        // View up vector


    // Compute w, u, v in order to fill the Camera matrix.
    Eigen::Vector3f w = - Gaze.normalized();
    Eigen::Vector3f u = ViewUp.cross(w).normalized();
    Eigen::Vector3f v = w.cross(u);


    // Define and fill the camera matrix.
    Eigen::Matrix4f cameraMatrix;
    cameraMatrix <<     u(0), v(0), w(0), Eye(0),
                        u(1), v(1), w(1), Eye(1),
                        u(2), v(2), w(2), Eye(2),
                        0, 0, 0, 1;
    // Invert the matrix.
    cameraMatrix = cameraMatrix.inverse().eval();


    // Set the uniform attribute computed.
    uniform.view = orthMatrix * cameraMatrix * modelMatrix;
    

    // Rasterization calling the rasterize_triangles function passing the program, the uniform attributes, the vector inside 
    // which all the vertices of the mesh are stored and the frame buffer. 
    rasterize_triangles(program, uniform, bunnyVertices, frameBuffer);


    // Save the result into a .png file.
    vector<uint8_t> image;
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);


    return 0;
}
