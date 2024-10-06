#include "blastgeolib/PolygonMesh.h"
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input OBJ file> <output STL file>" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = argv[2];

    PolygonMesh mesh;

    if (mesh.load(inputFile)) {
        std::cout << "OBJ file loaded successfully: " << inputFile << std::endl;
    }
    else {
        std::cerr << "Failed to load OBJ file: " << inputFile << std::endl;
        return 1;
    }

    float surfaceArea = mesh.getSurfaceArea();
    std::cout << "The surface area of the model is: " << surfaceArea << " units^2" << std::endl;

    float volume = mesh.getVolume();
    std::cout << "The volume of the model is: " << volume << " cubic units" << std::endl;

    std::vector<Eigen::Vector3f> testPoints = {
            {0.0f, 0.0f, 0.0f},        // Inside
            {0.25f, -0.25f, 0.25f},    // Inside
            {-0.25f, 0.25f, -0.25f},   // Inside
            {0.5f, 0.0f, 0.0f},        // On the surface
            {-0.5f, -0.5f, -0.5f},     // On the surface
            {0.0f, 0.5f, 0.5f},        // On the surface
            {1.0f, 1.0f, 1.0f},        // Outside
            {-1.0f, -1.0f, -1.0f},     // Outside
            {0.0f, -0.75f, 0.0f},      // Outside
    };

    for (const auto& point : testPoints) {
        if (mesh.isPointInside(point)) {
            std::cout << "Point (" << point.x() << ", " << point.y() << ", " << point.z() << ") is inside the model." << std::endl;
        }
        else {
            std::cout << "Point (" << point.x() << ", " << point.y() << ", " << point.z() << ") is outside the model." << std::endl;
        }
    }

    Eigen::Vector3f translation(1.0f, 2.0f, 3.0f);
    mesh.translate(translation);
    std::cout << "Mesh translated by (1, 2, 3)." << std::endl;

    float angle = 3.14f / 4;
    Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);
    mesh.rotate(angle, axis);
    std::cout << "Mesh rotated 45 degrees around the Z-axis." << std::endl;

    Eigen::Vector3f scaleFactors(2.0f, 2.0f, 2.0f);
    mesh.scale(scaleFactors);
    std::cout << "Mesh scaled by (2, 2, 2)." << std::endl;

    if (mesh.write(outputFile)) {
        std::cout << "Transformed STL file written successfully: " << outputFile << std::endl;
    }
    else {
        std::cerr << "Failed to write STL file: " << outputFile << std::endl;
    }

    return 0;
}
