#ifndef STL_SERIALIZER_H
#define STL_SERIALIZER_H

#include "MeshSerializer.h"

namespace blastgeolib {

class STLSerializer : public MeshSerializer {
public:
    PolygonMesh loadMeshData(const std::string& filename) override;
    void writeMeshData(const std::string& filename, const PolygonMesh& mesh) override;
private:
    void writeTriangle(std::ofstream& file, const std::vector<PolygonMesh::Vertex>& vertices, int idx1, int idx2, int idx3);
};

} // namespace blastgeolib

#endif // STL_SERIALIZER_H
