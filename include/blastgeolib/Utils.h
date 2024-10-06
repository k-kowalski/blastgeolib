#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<int>& polygon);
float generalizedWindingNumber(const Eigen::Vector3f& A, const Eigen::Vector3f& B, const Eigen::Vector3f& C, const Eigen::Vector3f& X);
std::vector<std::string> split(const std::string& s, char delim);
int parseIndex(const std::string& token, int offset);
std::string getFileExtension(const std::string& filename);

#endif // UTILS_H