#include "OBJSerializer.h"
#include "Utils.h"
#include <sstream>
#include <fstream>

PolygonMesh OBJSerializer::loadMeshData(const std::string& filename) {
    std::vector<PolygonMesh::Vertex> vertices;
    std::vector<PolygonMesh::TexCoord> texCoords;
    std::vector<PolygonMesh::Normal> normals;
    std::vector<PolygonMesh::Face> faces;

    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
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
                face.vertexIndices.push_back(vIdx);

                if (indices.size() > 1 && !indices[1].empty()) {
                    int vtIdx = parseIndex(indices[1], texCoords.size());
                    face.texCoordIndices.push_back(vtIdx);
                }

                if (indices.size() > 2 && !indices[2].empty()) {
                    int vnIdx = parseIndex(indices[2], normals.size());
                    face.normalIndices.push_back(vnIdx);
                }
            }
            faces.push_back(face);
        }
    }

    file.close();

    return PolygonMesh(std::move(vertices), std::move(faces), std::move(texCoords), std::move(normals));
}



void OBJSerializer::writeMeshData(const std::string&, const PolygonMesh&) {
    throw std::runtime_error("OBJ writing is not supported.");
}
