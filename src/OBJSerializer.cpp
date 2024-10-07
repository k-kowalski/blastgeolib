#include "OBJSerializer.h"
#include "Utils.h"
#include <unordered_map>
#include <sstream>
#include <fstream>

namespace blastgeolib {

    struct VertexCombination {
        int vertexIndex;
        int texCoordIndex;
        int normalIndex;

        bool operator==(const VertexCombination& other) const {
            return vertexIndex == other.vertexIndex &&
                texCoordIndex == other.texCoordIndex &&
                normalIndex == other.normalIndex;
        }
    };

    // based on Cantor pairing function
    struct VertexCombinationHash {
        std::size_t operator()(const VertexCombination& vc) const {
            return hashTriple(vc.vertexIndex, vc.texCoordIndex, vc.normalIndex);
        }

        std::size_t hashPair(int a, int b) const {
            return 0.5 * (a + b) * (a + b + 1) + b;
        }

        std::size_t hashTriple(int a, int b, int c) const {
            std::size_t pairHash = hashPair(a, b);
            return 0.5 * (pairHash + c) * (pairHash + c + 1) + c;
        }
    };

    PolygonMesh OBJSerializer::loadMeshData(const std::string& filename) {
        std::vector<PolygonMesh::Vertex> vertices;
        std::vector<PolygonMesh::TexCoord> texCoords;
        std::vector<PolygonMesh::Normal> normals;
        std::vector<PolygonMesh::Face> faces;

        std::vector<PolygonMesh::Vertex> finalVertices;
        std::vector<PolygonMesh::TexCoord> finalTexCoords;
        std::vector<PolygonMesh::Normal> finalNormals;
        std::unordered_map<VertexCombination, int, VertexCombinationHash> vertexMap;

        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        std::string line;
        int nextIndex = 0;

        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string token;
            ss >> token;

            if (token == "v") {
                PolygonMesh::Vertex vertex;
                ss >> vertex.x() >> vertex.y() >> vertex.z();
                vertices.push_back(vertex);
            }
            else if (token == "vt") {
                PolygonMesh::TexCoord texCoord;
                ss >> texCoord.x() >> texCoord.y();
                texCoords.push_back(texCoord);
            }
            else if (token == "vn") {
                PolygonMesh::Normal normal;
                ss >> normal.x() >> normal.y() >> normal.z();
                normals.push_back(normal);
            }
            else if (token == "f") {
                PolygonMesh::Face face;
                std::vector<std::string> faceIndices = split(line.substr(2), ' ');

                for (const std::string& idxToken : faceIndices) {
                    std::vector<std::string> indices = split(idxToken, '/');
                    int vIdx = parseIndex(indices[0], vertices.size());
                    int vtIdx = (indices.size() > 1 && !indices[1].empty()) ? parseIndex(indices[1], texCoords.size()) : -1;
                    int vnIdx = (indices.size() > 2 && !indices[2].empty()) ? parseIndex(indices[2], normals.size()) : -1;

                    VertexCombination vc = { vIdx, vtIdx, vnIdx };

                    if (vertexMap.find(vc) == vertexMap.end()) {
                        vertexMap[vc] = nextIndex;

                        finalVertices.push_back(vertices[vIdx]);
                        if (vtIdx != -1) {
                            finalTexCoords.push_back(texCoords[vtIdx]);
                        }
                        if (vnIdx != -1) {
                            finalNormals.push_back(normals[vnIdx]);
                        }

                        nextIndex++;
                    }

                    face.vertexIndices.push_back(vertexMap[vc]);
                }

                faces.push_back(face);
            }
        }

        file.close();

        return PolygonMesh(std::move(finalVertices), std::move(faces), std::move(finalTexCoords), std::move(finalNormals));
    }

    void OBJSerializer::writeMeshData(const std::string&, const PolygonMesh&) {
        throw std::runtime_error("OBJ writing is not supported.");
    }

} // namespace blastgeolib
