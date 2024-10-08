#ifndef MESH_SERIALIZER_H
#define MESH_SERIALIZER_H

#include "PolygonMesh.h"
#include <memory>
#include <string>

namespace blastgeolib {

class MeshSerializer {
public:
    virtual ~MeshSerializer() = default;

    static std::unique_ptr<PolygonMesh> load(const std::string& filename);
    static void write(const std::string& filename, const PolygonMesh& mesh);

protected:
    virtual std::unique_ptr<PolygonMesh> loadMeshData(const std::string& filename) = 0;
    virtual void writeMeshData(const std::string& filename, const PolygonMesh& mesh) = 0;

private:
    static std::unique_ptr<MeshSerializer> createSerializer(const std::string& filename);
};

} // namespace blastgeolib

#endif // MESH_SERIALIZER_H
