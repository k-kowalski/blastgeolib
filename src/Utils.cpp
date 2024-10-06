#include "Utils.h"
#include <sstream>
#include <stdexcept>

std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<int>& polygon) {
    std::vector<std::array<int, 3>> triangles;
    std::vector<int> remainingVertices = polygon;

    if (remainingVertices.size() < 3) {
        throw std::runtime_error("Polygon with less than 3 vertices detected.");
    }

    if (remainingVertices.size() == 3) {
        triangles.push_back({ remainingVertices[0], remainingVertices[1], remainingVertices[2] });
        return triangles;
    }

    while (remainingVertices.size() > 3) {
        triangles.push_back({ remainingVertices[0], remainingVertices[1], remainingVertices[2] });
        remainingVertices.erase(remainingVertices.begin() + 1);
    }

    triangles.push_back({ remainingVertices[0], remainingVertices[1], remainingVertices[2] });

    return triangles;
}

float generalizedWindingNumber(const Eigen::Vector3f& A, const Eigen::Vector3f& B, const Eigen::Vector3f& C, const Eigen::Vector3f& X) {
    Eigen::Vector3f vecA = A - X;
    Eigen::Vector3f vecB = B - X;
    Eigen::Vector3f vecC = C - X;

    float normA = vecA.norm();
    float normB = vecB.norm();
    float normC = vecC.norm();

    float det = vecA.dot(vecB.cross(vecC));

    float dotAB = vecA.dot(vecB);
    float dotBC = vecB.dot(vecC);
    float dotCA = vecC.dot(vecA);

    float windingContrib = std::atan2(det, normA * normB * normC + normC * dotAB + normA * dotBC + normB * dotCA);

    return windingContrib;
}

std::vector<std::string> split(const std::string& s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

int parseIndex(const std::string& token, int offset) {
    int idx = std::stoi(token);
    return idx > 0 ? idx - 1 : idx + offset;
}

std::string getFileExtension(const std::string& filename) {
    size_t dotPos = filename.rfind('.');
    if (dotPos != std::string::npos) {
        return filename.substr(dotPos + 1);
    }
    return "";
}
