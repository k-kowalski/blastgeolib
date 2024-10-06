#include "STLSerializer.h"
#include <fstream>

PolygonMesh STLSerializer::loadMeshData(const std::string&) {
    throw std::runtime_error("STL loading is not supported.");
}

void STLSerializer::writeMeshData(const std::string& filename, const PolygonMesh& mesh) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    char header[80] = {};
    file.write(header, 80);

    const std::vector<std::array<int,3>> tris = mesh.computeTriangulation();

    uint32_t numTriangles = 0;
    for (const auto& tri : tris) {
        numTriangles++;
    }
    file.write(reinterpret_cast<const char*>(&numTriangles), sizeof(numTriangles));

    const std::vector<PolygonMesh::Vertex>& verts = mesh.getVertices();

    for (const auto& tri : tris) {
        writeTriangle(file, verts, tri[0], tri[1], tri[2]);
    }

    file.close();
}

void STLSerializer::writeTriangle(std::ofstream& file, const std::vector<PolygonMesh::Vertex>& vertices, int idx1, int idx2, int idx3) {
    float normal[3] = { 0.0f, 0.0f, 0.0f };
    file.write(reinterpret_cast<const char*>(normal), sizeof(normal));

    const PolygonMesh::Vertex& v1 = vertices[idx1];
    const PolygonMesh::Vertex& v2 = vertices[idx2];
    const PolygonMesh::Vertex& v3 = vertices[idx3];

    float coords1[3] = { v1.x(), v1.y(), v1.z() };
    float coords2[3] = { v2.x(), v2.y(), v2.z() };
    float coords3[3] = { v3.x(), v3.y(), v3.z() };

    file.write(reinterpret_cast<const char*>(coords1), sizeof(coords1));
    file.write(reinterpret_cast<const char*>(coords2), sizeof(coords2));
    file.write(reinterpret_cast<const char*>(coords3), sizeof(coords3));

    // Write the attribute byte count (set to 0)
    uint16_t attributeByteCount = 0;
    file.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(attributeByteCount));
}
