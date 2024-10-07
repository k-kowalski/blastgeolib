#ifndef POLYGONMESH_H
#define POLYGONMESH_H

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace blastgeolib {

class PolygonMesh {
public:
    using Vertex = Eigen::Vector3f;
    using TexCoord = Eigen::Vector2f;
    using Normal = Eigen::Vector3f;

    struct Face {
        std::vector<int> vertexIndices;
        std::vector<int> texCoordIndices;
        std::vector<int> normalIndices;
    };

    PolygonMesh(std::vector<Vertex>&& vertices, 
                std::vector<Face>&& faces, 
                std::vector<TexCoord>&& texCoords, 
                std::vector<Normal>&& normals);

    void translate(const Eigen::Vector3f& translation);
    void rotate(float angle, const Eigen::Vector3f& axis);
    void scale(const Eigen::Vector3f& scaleFactors);

    bool isPointInside(const Eigen::Vector3f& point);
    float getSurfaceArea() const;
    float getVolume() const;
    const std::vector<Vertex>& getVertices() const { return vertices; }
    const std::vector<TexCoord>& getTexCoords() const { return texCoords; }
    const std::vector<Normal>& getNormals() const { return normals; }
    const std::vector<Face>& getFaces() const { return faces; }
    std::vector<std::array<int, 3>> computeTriangulation() const;
    void streamFuncOnTriangulation(const std::function<void(const std::array<int, 3>&)>& func) const;
private:
    std::vector<Vertex> vertices;
    std::vector<TexCoord> texCoords;
    std::vector<Normal> normals;
    std::vector<Face> faces;

    bool loadOBJ(const std::string& filename);
    bool writeSTL(const std::string& filename);
    void writeTriangle(std::ofstream& file, int idx1, int idx2, int idx3);
};

} // namespace blastgeolib

#endif // POLYGONMESH_H
