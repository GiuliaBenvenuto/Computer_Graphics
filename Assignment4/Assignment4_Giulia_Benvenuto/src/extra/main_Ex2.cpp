// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Geometry>
#include <gif.h>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"

using namespace std;


// Read a triangle mesh from an off file
// Code taken from the Assignment3, it loads a .off file passed as parameter and stores all the vertices and faces 
// inside two matrices: V and F. 
void load_off(const string& filename, Eigen::MatrixXf& V, Eigen::MatrixXi& F) {
    ifstream in(filename);
    string token;
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
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(1000, 1000);

    // Global Constants (empty in this example)
    UniformAttributes uniform;

    // Basic rasterization program
    Program program;

    // The vertex shader is the identity
    program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform) {
        VertexAttributes out;

        // Transformation done in the veretx shader. 
        // Final tranformation matrix:
        Eigen::Matrix4f finalTransformation = uniform.cameraMatrix * uniform.modelMatrix;

        out.position = finalTransformation * va.position;
        out.normal = (finalTransformation * va.normal).normalized();


        // (Lighting equation implementation taken from the ray-tracing assignment.)
        // Lights contribution
        Eigen::Vector4f Li = (uniform.lightPosition - out.position).normalized();
        Eigen::Vector4f N = out.normal;

        // Diffuse contribution
        Eigen::Vector3f diffuse = uniform.diffuse_color * std::max(Li.dot(N), float(0));

        // Specular contribution
        Eigen::Vector4f H = ((uniform.cameraPosition - out.position) + (uniform.lightPosition - out.position)).normalized();
        Eigen::Vector3f specular = uniform.specular_color * pow(std::max(H.dot(N), float(0)), uniform.specular_exponent);


        // Attenuate lights according to the squared distance to the lights
        Eigen::Vector4f D = uniform.lightPosition - out.position;
        out.color = (diffuse + specular).cwiseProduct(uniform.lightIntensity) / D.squaredNorm();

        if (uniform.renderOption == "Wireframe") {
            out.color = Eigen::Vector3f(out.color(0) - float(0.2), out.color(1) - float(0.2), out.color(2) - float(0.2));
        }

        out.color = Eigen::Vector3f(max(out.color(0), float(0)), max(out.color(1), float(0)), max(out.color(2), float(0)));
        out.color = Eigen::Vector3f(min(out.color(0), float(1)), min(out.color(1), float(1)), min(out.color(2), float(1)));

        out.position = uniform.projectionMatrix * out.position;
        return out;
    };



    program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform) {
        // In order to see the wireframe over the flat shading
        float depth = (uniform.cameraPosition - va.position).norm();
        if (uniform.renderOption == "Wireframe") {
            depth -= 0.001;
        }

        return FragmentAttributes(va.color(0), va.color(1), va.color(2), 1, depth);
    };


    // The blending shader converts colors between 0 and 1 to uint8
    program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous) {
        if (fa.depth < previous.depth - 0.0001) {
            return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255, fa.depth);
        }
        else return previous;
    };


    // Choose the render option (Wireframe, Flat_Shading, Per_Vertex_Shading)
    uniform.renderOption = "Per_Vertex_Shading";
    // Chose the transformation option (true --> make a .gif, false --> make a .png)
    uniform.objectTransformation = true;

    // (Values taken from the assignment 2)
    uniform.lightPosition = Eigen::Vector4f(0, 0, 5, 1);
    uniform.lightIntensity = Eigen::Vector3f(25, 25, 25);
    uniform.diffuse_color = Eigen::Vector3f(0.5, 0.5, 0.5);
    uniform.specular_color = Eigen::Vector3f(0.2, 0.2, 0.2);
    uniform.specular_exponent = 256.0;


    // Define the bounding box of the object.
    Eigen::Matrix4f orthMatrix;
    float l = -1,   b = -1,     n = -2;      // Left - bottom corner of the bounding box. 
    float r = 1,    t = 1,      f = -8;      // Right - top corner of the bounding box.

    // Compute the othogonal matrix as in slide 6/26 of "08 - Viewing transformations".
    orthMatrix <<   2 / (r - l), 0, 0, -(r + l) / (r - l),
                    0, 2 / (t - b), 0, -(t + b) / (t - b),
                    0, 0, 2 / (n - f), -(n + f) / (n - f),
                    0, 0, 0, 1;

    uniform.projectionMatrix = orthMatrix;


    // Slide 10/28 of "08 - Viewing tranformations".
    // Define eye position, gaze direction and view up vector to compute the unique transformations that converts world 
    // coordinates into camera coordinates.
    // Construction of the camera reference system
    Eigen::Vector3f Eye(0, 0, 5);       //camera position
    Eigen::Vector3f Gaze(0, 0, -1);     //gaze direction
    Eigen::Vector3f ViewUp(0, 1, 0);    //view up vector


    // Compute w, u, v in order to fill the Camera matrix.
    Eigen::Vector3f w = - Gaze.normalized();
    Eigen::Vector3f u = ViewUp.cross(w).normalized();
    Eigen::Vector3f v = w.cross(u);


    // Fill the camera matrix.
    uniform.cameraMatrix <<  u(0), v(0), w(0), Eye(0),
                             u(1), v(1), w(1), Eye(1),
                             u(2), v(2), w(2), Eye(2),
                             0, 0, 0, 1;
    // Invert the matrix.
    uniform.cameraMatrix = uniform.cameraMatrix.inverse().eval();

    // Set the uniform attributes for the camera postion and the light position.
    uniform.cameraPosition << Eye(0), Eye(1), Eye(2), 1;
    uniform.lightPosition = uniform.cameraMatrix * uniform.lightPosition;


    // One triangle in the center of the screen
    vector<VertexAttributes> vertices;      // (V,F)
    vector<VertexAttributes> vertices2;     // (V',F')

    // Instantiate two matrices to store the vertices and the faces of the triangles that make the bunny. 
    // To store them call the function "load_off" taken from the Assignment3 passing to it the two matrices.
    Eigen::MatrixXf V;
    Eigen::MatrixXi F;
    load_off("../data/bunny.off", V, F);


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


    // Compute the model matrix which is the multiplication between all the transformation matrices computed 
    // and save it as uniform attribute.
    uniform.modelMatrix = scaleMatrix * translationMatrix;


    // Different procedure according to the chosen render option.

    // Wireframe case:
    if (uniform.renderOption == "Wireframe") {
        for (int i = 0; i < F.rows(); ++i) {
            Eigen::Vector3f vertex1 = V.row(F(i, 0));
            Eigen::Vector3f vertex2 = V.row(F(i, 1));
            Eigen::Vector3f vertex3 = V.row(F(i, 2));
            Eigen::Vector3f normal = (vertex2 - vertex1).cross(vertex3 - vertex1).normalized();
            Eigen::Vector4f N(normal(0), normal(1), normal(2), 0);

            // First edge corners
            vertices.emplace_back(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2), 1, N);
            vertices.emplace_back(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2), 1, N);
            // Second edge corners
            vertices.emplace_back(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2), 1, N);
            vertices.emplace_back(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2), 1, N);
            // Third edge corners 
            vertices.emplace_back(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2), 1, N);
            vertices.emplace_back(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2), 1, N);
        }
    }
    else if (uniform.renderOption == "Flat_Shading") {
        for (int i = 0; i < F.rows(); ++i) {
            Eigen::Vector3f vertex1 = V.row(F(i, 0));
            Eigen::Vector3f vertex2 = V.row(F(i, 1));
            Eigen::Vector3f vertex3 = V.row(F(i, 2));
            Eigen::Vector3f normal = (vertex2 - vertex1).cross(vertex3 - vertex1).normalized();
            Eigen::Vector4f N(normal(0), normal(1), normal(2), 0);

            vertices.emplace_back(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2), 1, N);
            vertices.emplace_back(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2), 1, N);
            vertices.emplace_back(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2), 1, N);
        }
        for (int i = 0; i < F.rows(); ++i) {
            Eigen::Vector3f vertex1 = V.row(F(i, 0));
            Eigen::Vector3f vertex2 = V.row(F(i, 1));
            Eigen::Vector3f vertex3 = V.row(F(i, 2));
            Eigen::Vector3f normal = (vertex2 - vertex1).cross(vertex3 - vertex1).normalized();
            Eigen::Vector4f N(normal(0), normal(1), normal(2), 0);

            vertices2.emplace_back(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2), 1, N);
            vertices2.emplace_back(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2), 1, N);
            vertices2.emplace_back(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2), 1, N);
            vertices2.emplace_back(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2), 1, N);
            vertices2.emplace_back(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2), 1, N);
            vertices2.emplace_back(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2), 1, N);
        }
    }
    else if (uniform.renderOption == "Per_Vertex_Shading") {
        Eigen::MatrixXf average_normals;
        average_normals.resize(V.rows(), 4);

        Eigen::MatrixXf vertices_norm;
        vertices_norm.resize(V.rows(), 4);
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector3f vertex1 = V.row(F(i, 0));
            Eigen::Vector3f vertex2 = V.row(F(i, 1));
            Eigen::Vector3f vertex3 = V.row(F(i, 2));
            Eigen::Vector3f facet_norm = (vertex2 - vertex1).cross(vertex3 - vertex1).normalized();
            Eigen::Vector4f norm_row(facet_norm(0), facet_norm(1), facet_norm(2), 1);
            vertices_norm.row(F(i, 0)) += norm_row;
            vertices_norm.row(F(i, 1)) += norm_row;
            vertices_norm.row(F(i, 2)) += norm_row;
        }
        // Normalize the norm of vertex
        for (int i = 0; i < vertices_norm.rows(); i++) {
            Eigen::Vector4f norm = vertices_norm.row(i).normalized();
            vertices_norm(i, 0) = norm(0);
            vertices_norm(i, 1) = norm(1);
            vertices_norm(i, 2) = norm(2);
        }

        // Render the per-vertex scene
        // Averaging the per-face normals on the neighboring vertices.
        // uniform.color << 0.8, 0.8, 0.8, 1;
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector4f N1 (vertices_norm(F(i, 0), 0) / vertices_norm(F(i, 0), 3), 
                                vertices_norm(F(i, 0), 1) / vertices_norm(F(i, 0), 3),
                                vertices_norm(F(i, 0), 2) / vertices_norm(F(i, 0), 3), 
                                0);
            vertices.emplace_back(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2), 1, N1.normalized());


            Eigen::Vector4f N2 (vertices_norm(F(i, 1), 0) / vertices_norm(F(i, 1), 3),
                                vertices_norm(F(i, 1), 1) / vertices_norm(F(i, 1), 3),
                                vertices_norm(F(i, 1), 2) / vertices_norm(F(i, 1), 3), 
                                0);
            vertices.emplace_back(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2), 1, N2.normalized());


            Eigen::Vector4f N3 (vertices_norm(F(i, 2), 0) / vertices_norm(F(i, 2), 3),
                                vertices_norm(F(i, 2), 1) / vertices_norm(F(i, 2), 3),
                                vertices_norm(F(i, 2), 2) / vertices_norm(F(i, 2), 3), 
                                0);
            vertices.emplace_back(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2), 1, N3.normalized());
        }
    }


    // Gif animation.
    if (uniform.objectTransformation) {
        vector<uint8_t> image;
        GifWriter g{};
        GifBegin(&g, "out.gif", frameBuffer.rows(), frameBuffer.cols(), 25);


        int gifSize = 20;
        for (int i = 0; i < gifSize; i++) {
            frameBuffer.setConstant(FrameBufferAttributes(255, 255, 255));

            if (uniform.renderOption == "Wireframe") {
                rasterize_lines(program, uniform, vertices, 1, frameBuffer);
            }
            else if (uniform.renderOption == "Flat_Shading") {
                rasterize_triangles(program, uniform, vertices, frameBuffer);
                
                // Change the render option in order to draw the wirefram on top of the 
                // flat shaded triangles. 
                uniform.renderOption = "Wireframe";
                rasterize_lines(program, uniform, vertices2, 1, frameBuffer);

                // Change again to flat shading render option.
                uniform.renderOption = "Flat_Shading";
            }
            else if (uniform.renderOption == "Per_Vertex_Shading") {
                rasterize_triangles(program, uniform, vertices, frameBuffer);
            }

            // Write the gif.
            framebuffer_to_uint8(frameBuffer, image);
            GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), 25);


            // Translation of the object toward the camera, while rotating around its barycenter.
            Eigen::Matrix4f rotationY;
            float angle = M_PI / 10;


            rotationY <<    float(cos(angle)), 0, float(sin(angle)), 0,
                            0, 1, 0, 0,
                            float(-sin(angle)), 0, float(cos(angle)), 0,
                            0, 0, 0, 1;



            Eigen::Matrix4f translateMatrix;
            translateMatrix <<  1, 0, 0, w(0)* float(0.1),
                                0, 1, 0, w(1)* float(0.1),
                                0, 0, 1, w(2)* float(0.1),
                                0, 0, 0, 1;

            Eigen::Matrix4f moveToOriginMatrix;
            moveToOriginMatrix <<   1, 0, 0, -w(0) * float(i * 0.1),
                                    0, 1, 0, -w(1) * float(i * 0.1),
                                    0, 0, 1, -w(2) * float(i * 0.1),
                                    0, 0, 0, 1;

            Eigen::Matrix4f moveBackMatrix;
            moveBackMatrix <<   1, 0, 0, w(0)* float(i * 0.1),
                                0, 1, 0, w(1)* float(i * 0.1),
                                0, 0, 1, w(2)* float(i * 0.1),
                                0, 0, 0, 1;

            uniform.modelMatrix = translateMatrix * moveBackMatrix * rotationY * moveToOriginMatrix * uniform.modelMatrix;

        }
        GifEnd(&g);
    }
    // .png image.
    else {
        if (uniform.renderOption == "Wireframe") {
            rasterize_lines(program, uniform, vertices, 1, frameBuffer);
        }
        else if (uniform.renderOption == "Flat_Shading") {
            rasterize_triangles(program, uniform, vertices, frameBuffer);

            uniform.renderOption = "Wireframe";
            rasterize_lines(program, uniform, vertices2, 1, frameBuffer);

            uniform.renderOption = "Flat_Shading";
        }
        else if (uniform.renderOption == "Per_Vertex_Shading") {
            rasterize_triangles(program, uniform, vertices, frameBuffer);
        }

        vector<uint8_t> image;
        framebuffer_to_uint8(frameBuffer, image);
        stbi_write_png("bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);
    }


    return 0;
}
