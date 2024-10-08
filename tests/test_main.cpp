#include <gtest/gtest.h>
#include "blastgeolib/PolygonMesh.h"

using namespace blastgeolib;

std::vector<PolygonMesh::Vertex> vertices = {
    { 0.5f, -0.5f, 0.5f },
    { 0.5f, 0.5f, 0.5f },
    { -0.5f, 0.5f, 0.5f },
    { -0.5f, -0.5f, 0.5f },
    { -0.5f, -0.5f, -0.5f },
    { -0.5f, 0.5f, -0.5f },
    { 0.5f, 0.5f, -0.5f },
    { 0.5f, -0.5f, -0.5f }
};

std::vector<PolygonMesh::Face> faces = {
    { {0, 1, 2, 3} },
    { {4, 5, 6, 7} },
    { {5, 2, 1, 6} },
    { {7, 0, 3, 4} },
    { {7, 6, 1, 0} },
    { {3, 2, 5, 4} }
};

PolygonMesh mesh(std::move(vertices), std::move(faces), {}, {});

TEST(PolygonMeshTest, Translation) {
    Eigen::Vector3f translation(1.0f, 2.0f, 3.0f);
    mesh.translate(translation);

    const auto& vertices = mesh.getVertices();
    EXPECT_FLOAT_EQ(vertices[0].x(), 1.5f);
    EXPECT_FLOAT_EQ(vertices[0].y(), 1.5f);
    EXPECT_FLOAT_EQ(vertices[0].z(), 3.5f);
}


TEST(PolygonMeshTest, SurfaceArea) {

    float surfaceArea = mesh.getSurfaceArea();

    float volume = mesh.getVolume();
    EXPECT_FLOAT_EQ(volume, 1.0f);
}

TEST(PolygonMeshTest, Volume) {

    float surfaceArea = mesh.getSurfaceArea();

    float volume = mesh.getVolume();
    EXPECT_FLOAT_EQ(volume, 1.0f);
}

