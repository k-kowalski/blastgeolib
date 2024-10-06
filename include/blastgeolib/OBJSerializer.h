#ifndef OBJ_SERIALIZER_H
#define OBJ_SERIALIZER_H

#include "MeshSerializer.h"

class OBJSerializer : public MeshSerializer {
public:
    PolygonMesh loadMeshData(const std::string& filename) override;
    void writeMeshData(const std::string& filename, const PolygonMesh& mesh) override;
};

#endif // OBJ_SERIALIZER_H