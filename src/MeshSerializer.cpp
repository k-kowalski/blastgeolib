#include "MeshSerializer.h"
#include "OBJSerializer.h"
#include "STLSerializer.h"
#include "Utils.h"
#include <stdexcept>

namespace blastgeolib {

std::unique_ptr<MeshSerializer> MeshSerializer::createSerializer(const std::string& filename) {
    std::string extension = getFileExtension(filename);
    if (extension == ".obj") {
        return std::make_unique<OBJSerializer>();
    } else if (extension == ".stl") {
        return std::make_unique<STLSerializer>();
    } else {
        throw std::runtime_error("Unsupported file format: " + extension);
    }
}

std::unique_ptr<PolygonMesh> MeshSerializer::load(const std::string& filename) {
    auto serializer = createSerializer(filename);
    return serializer->loadMeshData(filename);
}

void MeshSerializer::write(const std::string& filename, const PolygonMesh& mesh) {
    auto serializer = createSerializer(filename);
    serializer->writeMeshData(filename, mesh);
}

} // namespace blastgeolib
