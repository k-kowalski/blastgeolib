#include "PolygonMesh.h"
#include "Utils.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace blastgeolib {

PolygonMesh::PolygonMesh(std::vector<Vertex>&& vertices, 
                std::vector<Face>&& faces, 
                std::vector<TexCoord>&& texCoords, 
                std::vector<Normal>&& normals)
        : vertices(std::move(vertices)), faces(std::move(faces)),
          texCoords(std::move(texCoords)), normals(std::move(normals)) {}

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

    Eigen::Matrix4f invTranspose = scaleMatrix.inverse().transpose();
    for (Normal& normal : normals) {
        Eigen::Vector4f homoNormal(normal.x(), normal.y(), normal.z(), 0.0f);
        homoNormal = invTranspose * homoNormal;
        normal = homoNormal.head<3>().normalized();
    }
}

/*
Robust Inside-Outside Segmentation using Generalized Winding Numbers by Alec Jacobson, Ladislav Kavan and Olga Sorkine-Hornung
impl inspired by https://github.com/marmakoide/inside-3d-mesh
*/
bool PolygonMesh::isPointInside(const Eigen::Vector3f& point) const {
    float windingNumber = 0.0f;

    streamFuncOnTriangulation([&](const std::array<int, 3>& tri) {
        const Eigen::Vector3f& v0 = vertices[tri[0]];
        const Eigen::Vector3f& v1 = vertices[tri[1]];
        const Eigen::Vector3f& v2 = vertices[tri[2]];

        windingNumber += generalizedWindingNumber(v0, v1, v2, point);
    });

    const float twoPi = 2.0f * EIGEN_PI;
    return std::abs(windingNumber) >= twoPi;
}

float PolygonMesh::getSurfaceArea() const {
    float totalArea = 0.0f;

    streamFuncOnTriangulation([&](const std::array<int, 3>& tri) {
        const Eigen::Vector3f& v0 = vertices[tri[0]];
        const Eigen::Vector3f& v1 = vertices[tri[1]];
        const Eigen::Vector3f& v2 = vertices[tri[2]];

        Eigen::Vector3f edge1 = v1 - v0;
        Eigen::Vector3f edge2 = v2 - v0;
        float triangleArea = 0.5f * edge1.cross(edge2).norm();
        totalArea += triangleArea;
    });

    return totalArea;
}

float PolygonMesh::getVolume() const {
    float totalVolume = 0.0f;

    streamFuncOnTriangulation([&](const std::array<int, 3>& tri) {
        const Eigen::Vector3f& v0 = vertices[tri[0]];
        const Eigen::Vector3f& v1 = vertices[tri[1]];
        const Eigen::Vector3f& v2 = vertices[tri[2]];

        float tetrahedronVolume = (v0.dot(v1.cross(v2))) / 6.0f;
        totalVolume += tetrahedronVolume;
    });

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

void PolygonMesh::streamFuncOnTriangulation(const std::function<void(const std::array<int, 3>&)>& func) const {
    std::for_each(faces.begin(), faces.end(), [func](const Face& face) {
        std::vector<std::array<int, 3>> triangles = triangulatePolygon(face.vertexIndices);
        std::for_each(triangles.begin(), triangles.end(), func);
    });
}

} // namespace blastgeolib
