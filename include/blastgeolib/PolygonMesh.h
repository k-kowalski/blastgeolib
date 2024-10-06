#ifndef POLYGONMESH_H
#define POLYGONMESH_H

#include <string>
#include <vector>


#include <Eigen/Dense>

class PolygonMesh {
public:
    using Vertex = Eigen::Vector3f;

    struct Face {
        std::vector<int> vertexIndices;
    };

    PolygonMesh();

    bool load(const std::string& filename);

    bool write(const std::string& filename);


    void translate(const Eigen::Vector3f& translation);
    void rotate(float angle, const Eigen::Vector3f& axis);
    void scale(const Eigen::Vector3f& scaleFactors);

    bool isPointInside(const Eigen::Vector3f& point);
    float getSurfaceArea() const;
    float getVolume() const;
private:
    std::vector<Vertex> vertices;
    std::vector<Face> faces;

    bool loadOBJ(const std::string& filename);
    bool writeSTL(const std::string& filename);
    void writeTriangle(std::ofstream& file, int idx1, int idx2, int idx3);
    std::vector<std::array<int, 3>> computeTriangulation() const;
};

#endif // POLYGONMESH_H
