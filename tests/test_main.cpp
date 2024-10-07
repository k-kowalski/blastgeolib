#include <gtest/gtest.h>
#include "blastgeolib/PolygonMesh.h"

using namespace blastgeolib;

TEST(PolygonMeshTest, TranslationTest) {
    PolygonMesh::Vertex v1(0.0f, 0.0f, 0.0f);
    PolygonMesh::Vertex v2(1.0f, 0.0f, 0.0f);
    PolygonMesh::Vertex v3(0.0f, 1.0f, 0.0f);

    PolygonMesh mesh({v1, v2, v3}, {}, {}, {});

    Eigen::Vector3f translation(1.0f, 2.0f, 3.0f);
    mesh.translate(translation);

    const auto& vertices = mesh.getVertices();
    EXPECT_EQ(vertices[0].x(), 1.0f);
    EXPECT_EQ(vertices[0].y(), 2.0f);
    EXPECT_EQ(vertices[0].z(), 3.0f);
}


TEST(PolygonMeshTest, SurfaceAreaAndVolume) {
    PolygonMesh::Vertex v1(0.5f, -0.5f, 0.5f);
    PolygonMesh::Vertex v2(0.5f, 0.5f, 0.5f);
    PolygonMesh::Vertex v3(-0.5f, 0.5f, 0.5f);
    PolygonMesh::Vertex v4(-0.5f, -0.5f, 0.5f);
    PolygonMesh::Vertex v5(-0.5f, -0.5f, -0.5f);
    PolygonMesh::Vertex v6(-0.5f, 0.5f, -0.5f);
    PolygonMesh::Vertex v7(0.5f, 0.5f, -0.5f);
    PolygonMesh::Vertex v8(0.5f, -0.5f, -0.5f);

    std::vector<PolygonMesh::Face> faces = {
        { {0, 1, 2, 3} },
        { {4, 5, 6, 7} },
        { {5, 2, 1, 6} },
        { {7, 0, 3, 4} },
        { {7, 6, 1, 0} },
        { {3, 2, 5, 4} }
    };

    std::vector<PolygonMesh::Vertex> vertices = { v1, v2, v3, v4, v5, v6, v7, v8 };


    PolygonMesh mesh(std::move(vertices), std::move(faces), {}, {});

    float surfaceArea = mesh.getSurfaceArea();

    float volume = mesh.getVolume();
    EXPECT_FLOAT_EQ(volume, 1.0f);
}

