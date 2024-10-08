#include "STLSerializer.h"
#include <fstream>

namespace blastgeolib {

std::unique_ptr<PolygonMesh> STLSerializer::loadMeshData(const std::string&) {
    throw std::runtime_error("STL loading is not supported.");
}

void STLSerializer::writeMeshData(const std::string& filename, const PolygonMesh& mesh) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    char header[80] = {};
    file.write(header, 80);

    uint32_t triangleCountPos = file.tellp();
    uint32_t dummyTriangleCount = 0;
    file.write(reinterpret_cast<const char*>(&dummyTriangleCount), sizeof(dummyTriangleCount));

    uint32_t triangleCount = 0;

    const std::vector<PolygonMesh::Vertex>& verts = mesh.getVertices();
    const std::vector<PolygonMesh::Normal>& normals = mesh.getNormals();
    mesh.streamFuncOnTriangulation([&](const std::array<int, 3>& tri) {

        const int idx1 = tri[0];
        const int idx2 = tri[1];
        const int idx3 = tri[2];

        const PolygonMesh::Vertex& v1 = verts[idx1];
        const PolygonMesh::Vertex& v2 = verts[idx2];
        const PolygonMesh::Vertex& v3 = verts[idx3];

        PolygonMesh::Normal faceNormal;
        if (normals.size() > 0)
        {
            faceNormal = (normals[idx1] + normals[idx2] + normals[idx3]) / 3.0f;
        }
        else
        {
            Eigen::Vector3f edge1 = v2 - v1;
            Eigen::Vector3f edge2 = v3 - v1;
            faceNormal = edge1.cross(edge2).normalized();
        }

        float normal[3] = { faceNormal.x(), faceNormal.y(), faceNormal.z() };
        file.write(reinterpret_cast<const char*>(normal), sizeof(normal));

        float coords1[3] = { v1.x(), v1.y(), v1.z() };
        float coords2[3] = { v2.x(), v2.y(), v2.z() };
        float coords3[3] = { v3.x(), v3.y(), v3.z() };

        file.write(reinterpret_cast<const char*>(coords1), sizeof(coords1));
        file.write(reinterpret_cast<const char*>(coords2), sizeof(coords2));
        file.write(reinterpret_cast<const char*>(coords3), sizeof(coords3));

        uint16_t attributeByteCount = 0;
        file.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(attributeByteCount));

        triangleCount++;
    });

    file.seekp(triangleCountPos);
    file.write(reinterpret_cast<const char*>(&triangleCount), sizeof(triangleCount));

    file.close();
}

} // namespace blastgeolib
