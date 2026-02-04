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


TEST(PolygonMeshTest, Scaling) {
    std::vector<PolygonMesh::Vertex> v = { {1.0f, 1.0f, 1.0f} };
    std::vector<PolygonMesh::Face> f;
    // Normal (1, 1, 0) normalized
    std::vector<PolygonMesh::Normal> n = { Eigen::Vector3f(1.0f, 1.0f, 0.0f).normalized() };
    
    PolygonMesh mesh(std::move(v), std::move(f), {}, std::move(n));
    
    // Test 1: Uniform scaling
    mesh.scale({2.0f, 2.0f, 2.0f});
    EXPECT_FLOAT_EQ(mesh.getVertices()[0].x(), 2.0f);
    EXPECT_FLOAT_EQ(mesh.getVertices()[0].y(), 2.0f);
    EXPECT_FLOAT_EQ(mesh.getVertices()[0].z(), 2.0f);

    // Test 2: Zero scaling (should be ignored)
    mesh.scale({0.0f, 1.0f, 1.0f});
    EXPECT_FLOAT_EQ(mesh.getVertices()[0].x(), 2.0f); // Unchanged
    
    // Test 3: Normal transformation
    // Current normal is still (1,1,0) normalized (uniform scale preserves direction)
    // Scale X by 2. Matrix M = diag(2,1,1).
    // InvTrans M^(-T) = diag(0.5, 1, 1).
    // New normal vector pre-normalization: (0.5 * x, 1.0 * y, 0).
    // Since x=y originally, new y should be 2x new x.
    mesh.scale({2.0f, 1.0f, 1.0f});
    
    const auto& normals = mesh.getNormals();
    // Use near equality or check ratio
    float nx = normals[0].x();
    float ny = normals[0].y();
    
    // We expect ny approx 2 * nx
    EXPECT_NEAR(ny, 2.0f * nx, 1e-5f);
}
