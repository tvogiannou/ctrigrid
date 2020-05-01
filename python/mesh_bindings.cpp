
#include "module.h"

#include <fstream>
#include <unordered_set>
#include <vector>

#include <ctrigrid/Vector3.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


pybind11::array_t<float> 
CTRIGRID_ComputeVertexNormals(
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices)
{
    pybind11::buffer_info vertexBufferInfo = vertices.request();
    pybind11::buffer_info indexBufferInfo = indices.request();

    if (vertexBufferInfo.ndim != 1u || indexBufferInfo.ndim != 1u)
        throw std::runtime_error("Number of dimensions must be one");

    // allocate normal buffer
    auto result = pybind11::array_t<float>(vertexBufferInfo.size);
    pybind11::buffer_info normalBufferInfo = result.request();

    const float* vtx = (const float*)vertexBufferInfo.ptr;
    float* normals = (float*)normalBufferInfo.ptr;
    const uint32_t* idx = (const uint32_t*)indexBufferInfo.ptr;

    const size_t numVertices = vertexBufferInfo.shape[0];
    const size_t numIndices = indexBufferInfo.shape[0];

    // reset normals
    for (size_t k = 0u; k < numVertices; ++k)
        normals[k] = 0.f;

    // iterate through tris to compute each normal and aggregate for each vertex
    size_t i = 0;
    while (i < numIndices)
    {
        const size_t idx0 = 3u * idx[i];
        const size_t idx1 = 3u * idx[i + 1];
        const size_t idx2 = 3u * idx[i + 2];

        if (idx0 >= numVertices ||
            idx1 >= numVertices ||
            idx2 >= numVertices)
            throw std::runtime_error("Value in indices out of vertices range");

        const ctrigrid::Vector3 v0(vtx[idx0], vtx[idx0 + 1], vtx[idx0 + 2]);
        const ctrigrid::Vector3 v1(vtx[idx1], vtx[idx1 + 1], vtx[idx1 + 2]);
        const ctrigrid::Vector3 v2(vtx[idx2], vtx[idx2 + 1], vtx[idx2 + 2]);

        // compute normal from cross product
        ctrigrid::Vector3 e01 = v1; e01.Sub(v0);
        ctrigrid::Vector3 e12 = v2; e12.Sub(v1);
        ctrigrid::Vector3 n = ctrigrid::Vector3::Cross(e01, e12);

        // add up to normals buffer
        normals[idx0] += n.x;
        normals[idx0 + 1] += n.y;
        normals[idx0 + 2] += n.z;

        normals[idx1] += n.x;
        normals[idx1 + 1] += n.y;
        normals[idx1 + 2] += n.z;

        normals[idx2] += n.x;
        normals[idx2 + 1] += n.y;
        normals[idx2 + 2] += n.z;

        i += 3u;
    }

    // TODO: normalize normals
    for (size_t k = 0u; k < numVertices; k += 3u)
    {
        ctrigrid::Vector3 n(normals[k], normals[k + 1], normals[k + 2]);
        n.Normalize();
        
        normals[k] += n.x;
        normals[k + 1] += n.y;
        normals[k + 2] += n.z;
    }

    return result;
}

pybind11::array_t<float> 
CTRIGRID_ComputeTriNormals(
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices)
{
    pybind11::buffer_info vertexBufferInfo = vertices.request();
    pybind11::buffer_info indexBufferInfo = indices.request();

    if (vertexBufferInfo.ndim != 1u || indexBufferInfo.ndim != 1u)
        throw std::runtime_error("Number of dimensions must be one");

    // allocate normal buffer
    auto result = pybind11::array_t<float>(indexBufferInfo.size);   // one float for each index
    pybind11::buffer_info normalBufferInfo = result.request();

    const float* vtx = (const float*)vertexBufferInfo.ptr;
    float* normals = (float*)normalBufferInfo.ptr;
    const uint32_t* idx = (const uint32_t*)indexBufferInfo.ptr;

    const size_t numVertices = vertexBufferInfo.shape[0];

    for (size_t i = 0; i < (size_t)indexBufferInfo.size; i += 3u)
    {
        const size_t idx0 = 3u * idx[i];
        const size_t idx1 = 3u * idx[i + 1];
        const size_t idx2 = 3u * idx[i + 2];

        if (idx0 >= numVertices ||
            idx1 >= numVertices ||
            idx2 >= numVertices)
            throw std::runtime_error("Value in indices out of vertices range");

        const ctrigrid::Vector3 v0(vtx[idx0], vtx[idx0 + 1], vtx[idx0 + 2]);
        const ctrigrid::Vector3 v1(vtx[idx1], vtx[idx1 + 1], vtx[idx1 + 2]);
        const ctrigrid::Vector3 v2(vtx[idx2], vtx[idx2 + 1], vtx[idx2 + 2]);

        // compute normal from cross product
        ctrigrid::Vector3 e01 = v1; e01.Sub(v0);
        ctrigrid::Vector3 e12 = v2; e12.Sub(v1);
        ctrigrid::Vector3 n = ctrigrid::Vector3::Cross(e01, e12);
        n.Normalize();

        normals[i] = n.x;
        normals[i + 1] = n.y;
        normals[i + 2] = n.z;
    }

    return result;
}

struct edge_t
{
    size_t i, j;
};

struct hash_fn
{
    size_t operator()(const edge_t& e) const
    {
        auto minmax0 = std::minmax(e.i, e.j);
        return minmax0.first << 16u | minmax0.second;
    }
};

struct equal_to_fn 
{
    bool operator()(const edge_t& e0, const edge_t& e1) const
    {
        auto minmax0 = std::minmax(e0.i, e0.j);
        auto minmax1 = std::minmax(e1.i, e1.j);

        return minmax0.first == minmax1.first && minmax0.second == minmax1.second;
    }
};

pybind11::array_t<uint32_t> 
CTRIGRID_GetUniqueEdges(pybind11::array_t<uint32_t> indices)
{
    pybind11::buffer_info indexBufferInfo = indices.request();

    if (indexBufferInfo.ndim != 1u)
        throw std::runtime_error("Number of dimensions must be one");

    const uint32_t* idx = (const uint32_t*)indexBufferInfo.ptr;

    std::unordered_set<edge_t, hash_fn, equal_to_fn> edgeSet(indexBufferInfo.size);
    for (size_t i = 0; i < (size_t)indexBufferInfo.size; i += 3u)
    {
        edgeSet.insert({ idx[i], idx[i + 1] });
        edgeSet.insert({ idx[i + 1], idx[i + 2] });
    }

    auto result = pybind11::array_t<uint32_t>(edgeSet.size() * 2u);   // two indices for each edge
    pybind11::buffer_info edgeBufferInfo = result.request();
    uint32_t* uniqueEdges = (uint32_t*)edgeBufferInfo.ptr;

    size_t edgeIndex = 0u;
    for (auto it = edgeSet.begin(); it != edgeSet.end(); ++it)
    {
        uniqueEdges[edgeIndex] = it->i;
        uniqueEdges[edgeIndex + 1] = it->j;

        edgeIndex += 2u;
    }

    return result;
}


std::tuple<pybind11::array_t<float>, pybind11::array_t<uint32_t>> 
CTRIGRID_LoadObjFile(const std::string& filename)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str());

    if (!err.empty())
        throw std::runtime_error(err);

    if (!ret)
        throw std::runtime_error("tinyobj returned false");

    if (shapes.size() > 1)
        pybind11::print("Only a single shape is supported when loading an OBJ");

    if (materials.size() > 1)
        pybind11::print("Only a single material is supported when loading an OBJ");

    std::vector<float> vertexData;
    std::vector<uint32_t> indices;

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++)
    {
        // TODO: support more shapes
        if (s > 1)
            break;

        //const bool hasNormals = !attrib.normals.empty();
        //const bool hasUVs = !attrib.texcoords.empty();
        //const bool hasMaterial = materials.size() == 1; // only one material supported

        // copy vertex data first
        {
            size_t vertexIndex = 0;
            size_t texIndex = 0;
            //const uint32_t vertexCount = (uint32_t)(attrib.vertices.size() / 3);
            while (vertexIndex < attrib.vertices.size())
            {
                // load vertex data
                const tinyobj::real_t vx = attrib.vertices[vertexIndex + 0];
                const tinyobj::real_t vy = attrib.vertices[vertexIndex + 1];
                const tinyobj::real_t vz = attrib.vertices[vertexIndex + 2];

                // write to output buffer
                vertexData.push_back((float)vx);
                vertexData.push_back((float)vy);
                vertexData.push_back((float)vz);

                // get next vertex
                vertexIndex += 3;
                texIndex += 2;
            }
        }

        // copy indices
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            const size_t fv = shapes[s].mesh.num_face_vertices[f];
            if (fv != 3)
                throw std::runtime_error("only support triangulated faces");

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++)
            {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                indices.push_back((uint32_t)idx.vertex_index);
            }
            index_offset += fv;

            // per-face material
            //shapes[s].mesh.material_ids[f];
        }
    }

    // allocate return buffers
    auto verticesBuffer = pybind11::array_t<float>(vertexData.size());
    auto indicesBuffer = pybind11::array_t<uint32_t>(indices.size());
    pybind11::buffer_info verticesBufferInfo = verticesBuffer.request();
    pybind11::buffer_info indicesBufferInfo = indicesBuffer.request();

    // copy to output buffers
    std::memcpy(verticesBufferInfo.ptr, vertexData.data(), vertexData.size() * sizeof(float));
    std::memcpy(indicesBufferInfo.ptr, indices.data(), indices.size() * sizeof(uint32_t));

    return std::make_tuple(verticesBuffer, indicesBuffer);
}

void 
CTRIGRID_SaveObjFile(const std::string& filename,
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices)
{
    pybind11::buffer_info vertexBufferInfo = vertices.request();
    pybind11::buffer_info indexBufferInfo = indices.request();

    if (vertexBufferInfo.ndim != 1u || indexBufferInfo.ndim != 1u)
        throw std::runtime_error("Number of dimensions must be one");

    const float* vtx = (const float*)vertexBufferInfo.ptr;
    const uint32_t* idx = (const uint32_t*)indexBufferInfo.ptr;

    const size_t numVertices = vertexBufferInfo.shape[0];
    const size_t numIndices = indexBufferInfo.shape[0];

    std::ofstream outFile(filename.c_str());
    if (outFile.fail())
        throw std::runtime_error("Failed to open file");

    for (size_t i = 0; i < numVertices; i += 3)
    {
        const float x = vtx[i];
        const float y = vtx[i + 1];
        const float z = vtx[i + 2];

        outFile << "v " << x << " " << y << " " << z << "\n";
    }

    for (size_t i = 0; i < numIndices; i += 3)
    {
        // obj format indices start at 1
        const uint32_t v0 = idx[i] + 1;
        const uint32_t v1 = idx[i + 1] + 1;
        const uint32_t v2 = idx[i + 2] + 1;

        outFile << "f " << v0 << " " << v1 << " " << v2 << "\n";
    }

    outFile.close();
}