#include "PolygonMesh.h"
#include "Utils.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

PolygonMesh::PolygonMesh(std::vector<Vertex>&& vertices, std::vector<Face>&& faces) :
    vertices(std::move(vertices)), faces(std::move(faces)) {}

void PolygonMesh::translate(const Eigen::Vector3f& translation) {
    Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();
    translationMatrix.block<3, 1>(0, 3) = translation;

    for (Vertex& vertex : vertices) {
        Eigen::Vector4f homoVertex(vertex.x(), vertex.y(), vertex.z(), 1.0f);
        homoVertex = translationMatrix * homoVertex;
        vertex = homoVertex.head<3>();
    }
}

void PolygonMesh::rotate(float angle, const Eigen::Vector3f& axis) {
    Eigen::Matrix3f rotation3x3 = Eigen::AngleAxisf(angle, axis.normalized()).toRotationMatrix();
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix.block<3, 3>(0, 0) = rotation3x3;

    for (Vertex& vertex : vertices) {
        Eigen::Vector4f homoVertex(vertex.x(), vertex.y(), vertex.z(), 1.0f);
        homoVertex = rotationMatrix * homoVertex;
        vertex = homoVertex.head<3>();
    }
}

void PolygonMesh::scale(const Eigen::Vector3f& scaleFactors) {
    Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
    scaleMatrix(0, 0) = scaleFactors.x();
    scaleMatrix(1, 1) = scaleFactors.y();
    scaleMatrix(2, 2) = scaleFactors.z();

    for (Vertex& vertex : vertices) {
        Eigen::Vector4f homoVertex(vertex.x(), vertex.y(), vertex.z(), 1.0f);
        homoVertex = scaleMatrix * homoVertex;
        vertex = homoVertex.head<3>();
    }
}

const std::vector<PolygonMesh::Vertex>& PolygonMesh::getVertices() const {
    return vertices;
}

const std::vector<PolygonMesh::Face>& PolygonMesh::getFaces() const {
    return faces;
}

bool PolygonMesh::loadOBJ(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open OBJ file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        ss >> token;

        if (token == "v") {
            Vertex vertex;
            ss >> vertex.x() >> vertex.y() >> vertex.z();
            vertices.push_back(vertex);
        } else if (token == "f") {
            Face face;
            std::vector<std::string> faceIndices = split(line.substr(2), ' ');
            for (const std::string& idxToken : faceIndices) {
                std::vector<std::string> indices = split(idxToken, '/');
                int vIdx = parseIndex(indices[0], vertices.size());
                face.vertexIndices.push_back(vIdx);
            }
            faces.push_back(face);
        }
    }


    file.close();
    std::cout << "Successfully loaded OBJ file: " << filename << std::endl;
    return true;
}

bool PolygonMesh::writeSTL(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open STL file: " << filename << std::endl;
        return false;
    }

    char header[80] = {};
    file.write(header, 80);

    std::vector<std::array<int, 3>> triangles = computeTriangulation();

    uint32_t numTriangles = static_cast<uint32_t>(triangles.size());
    file.write(reinterpret_cast<const char*>(&numTriangles), sizeof(numTriangles));

    for (const auto& tri : triangles) {
        writeTriangle(file, tri[0], tri[1], tri[2]);
    }

    file.close();
    std::cout << "Successfully wrote STL file: " << filename << std::endl;
    return true;
}

void PolygonMesh::writeTriangle(std::ofstream& file, int idx1, int idx2, int idx3) {
    float normal[3] = {0.0f, 0.0f, 0.0f};
    file.write(reinterpret_cast<const char*>(normal), sizeof(normal));

    const Vertex& v1 = vertices[idx1];
    const Vertex& v2 = vertices[idx2];
    const Vertex& v3 = vertices[idx3];

    float coords1[3] = {v1.x(), v1.y(), v1.z()};
    float coords2[3] = {v2.x(), v2.y(), v2.z()};
    float coords3[3] = {v3.x(), v3.y(), v3.z()};

    file.write(reinterpret_cast<const char*>(coords1), sizeof(coords1));
    file.write(reinterpret_cast<const char*>(coords2), sizeof(coords2));
    file.write(reinterpret_cast<const char*>(coords3), sizeof(coords3));

    uint16_t attributeByteCount = 0;
    file.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(attributeByteCount));
}

/*
Robust Inside-Outside Segmentation using Generalized Winding Numbers by Alec Jacobson, Ladislav Kavan and Olga Sorkine-Hornung
impl inspired by https://github.com/marmakoide/inside-3d-mesh
*/
bool PolygonMesh::isPointInside(const Eigen::Vector3f& point) {
    float windingNumber = 0.0f;
    
    std::vector<std::array<int, 3>> triangles = computeTriangulation();

    for (const auto& tri : triangles) {
        float t;
        const Eigen::Vector3f& v0 = vertices[tri[0]];
        const Eigen::Vector3f& v1 = vertices[tri[1]];
        const Eigen::Vector3f& v2 = vertices[tri[2]];

        windingNumber += generalizedWindingNumber(v0, v1, v2, point);
    }

    const float twoPi = 2.0f * 3.14;
    return std::abs(windingNumber) >= twoPi;
}

float PolygonMesh::getSurfaceArea() const {
    float totalArea = 0.0f;

    std::vector<std::array<int, 3>> triangles = computeTriangulation();

    for (const auto& tri : triangles) {
        const Eigen::Vector3f& v0 = vertices[tri[0]];
        const Eigen::Vector3f& v1 = vertices[tri[1]];
        const Eigen::Vector3f& v2 = vertices[tri[2]];

        Eigen::Vector3f edge1 = v1 - v0;
        Eigen::Vector3f edge2 = v2 - v0;
        float triangleArea = 0.5f * edge1.cross(edge2).norm();
        totalArea += triangleArea;
    }

    return totalArea;
}

float PolygonMesh::getVolume() const {
    float totalVolume = 0.0f;
    
    std::vector<std::array<int, 3>> triangles = computeTriangulation();

    for (const auto& tri : triangles) {
        const Eigen::Vector3f& v0 = vertices[tri[0]];
        const Eigen::Vector3f& v1 = vertices[tri[1]];
        const Eigen::Vector3f& v2 = vertices[tri[2]];

        float tetrahedronVolume = (v0.dot(v1.cross(v2))) / 6.0f;
        totalVolume += tetrahedronVolume;
    }

    return totalVolume;
}

std::vector<std::array<int, 3>> PolygonMesh::computeTriangulation() const {
    std::vector<std::array<int, 3>> allTriangles;

    for (const Face& face : faces) {
        std::vector<std::array<int, 3>> triangles = triangulatePolygon(face.vertexIndices);

        allTriangles.insert(allTriangles.end(), triangles.begin(), triangles.end());
    }

    return allTriangles;
}

