#pragma once

#include "mochi/core.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/rhi/buffer.hh"
#include "mochi/module/memory.hh"
#include "Baryon/math/Vector3.hpp"
#include <vector>
#include <cmath>
#include <cstring>

namespace DebugDraw {

static sptr<mochi::asset::mesh> createWireframeBox(mochi::core& eng, Baryon::Vector3 extents) {
    std::vector<mochi::asset::vertex_t> vertices;
    Baryon::Vector3 c[8] = {
        {-extents.x, -extents.y, -extents.z}, { extents.x, -extents.y, -extents.z},
        { extents.x,  extents.y, -extents.z}, {-extents.x,  extents.y, -extents.z},
        {-extents.x, -extents.y,  extents.z}, { extents.x, -extents.y,  extents.z},
        { extents.x,  extents.y,  extents.z}, {-extents.x,  extents.y,  extents.z}
    };
    
    int indices[] = {
        0,1, 1,2, 2,3, 3,0, // bottom
        4,5, 5,6, 6,7, 7,4, // top
        0,4, 1,5, 2,6, 3,7  // pillars
    };
    
    for (int idx : indices) {
        mochi::asset::vertex_t v;
        v.position = {c[idx].x, c[idx].y, c[idx].z};
        v.normal = {0,1,0};
        v.color = {0.2f, 0.6f, 1.0f}; 
        v.uv = {0,0};
        vertices.push_back(v);
    }

    auto buffer = mochi::rhi::buffer::make(
        eng.sub<mochi::module::device>(),
        eng.sub<mochi::module::memory>(),
        mochi::asset::vertex_i, vertices.size(),
        flags(mochi::rhi::BufferUsage::VertexBuffer) | mochi::rhi::BufferUsage::TransferDst,
        flags(mochi::rhi::BufferCreate::Mapped) | mochi::rhi::BufferCreate::HostSequentialWrite,
        mochi::rhi::BufferLocation::PreferDevice,
        [&](void* ptr) {
            std::memcpy(ptr, vertices.data(), vertices.size() * sizeof(mochi::asset::vertex_t));
        }
    );


    auto material = make_sptr(new mochi::asset::material(eng));
    material->setColor({0,0,1});
    material->setPolymode(mochi::rhi::PolygonMode::Line);
    material->setPrimitiveTopology(mochi::rhi::PrimitiveTopology::eLineList);


    // Güncel mesh yapısına uygun olarak buffer'dan mesh oluşturuluyor
    return mochi::asset::mesh::make(buffer, {{0, vertices.size()}}, {material}, {0});
}

static sptr<mochi::asset::mesh> createWireframeSphere(mochi::core& eng, float radius) {
    std::vector<mochi::asset::vertex_t> vertices;
    int segments = 16;
    
    for (int p = 0; p < 3; ++p) {
        for (int i = 0; i < segments; ++i) {
            float a1 = (float)i / segments * 3.14159f * 2.0f;
            float a2 = (float)(i+1) / segments * 3.14159f * 2.0f;
            
            Baryon::Vector3 v1, v2;
            if (p == 0) { v1 = {std::cos(a1)*radius, std::sin(a1)*radius, 0}; v2 = {std::cos(a2)*radius, std::sin(a2)*radius, 0}; }
            else if (p == 1) { v1 = {std::cos(a1)*radius, 0, std::sin(a1)*radius}; v2 = {std::cos(a2)*radius, 0, std::sin(a2)*radius}; }
            else { v1 = {0, std::cos(a1)*radius, std::sin(a1)*radius}; v2 = {0, std::cos(a2)*radius, std::sin(a2)*radius}; }
            
            mochi::asset::vertex_t vert1, vert2;
            vert1.position = {v1.x, v1.y, v1.z}; vert1.normal={0,1,0}; vert1.color={0.2f, 0.6f, 1.0f}; vert1.uv={0,0};
            vert2.position = {v2.x, v2.y, v2.z}; vert2.normal={0,1,0}; vert2.color={0.2f, 0.6f, 1.0f}; vert2.uv={0,0};
            vertices.push_back(vert1);
            vertices.push_back(vert2);
        }
    }
    
    
    auto buffer = mochi::rhi::buffer::make(
        eng.sub<mochi::module::device>(),
        eng.sub<mochi::module::memory>(),
        mochi::asset::vertex_i, vertices.size(),
        flags(mochi::rhi::BufferUsage::VertexBuffer) | mochi::rhi::BufferUsage::TransferDst,
        flags(mochi::rhi::BufferCreate::Mapped) | mochi::rhi::BufferCreate::HostSequentialWrite,
        mochi::rhi::BufferLocation::PreferDevice,
        [&](void* ptr) {
            std::memcpy(ptr, vertices.data(), vertices.size() * sizeof(mochi::asset::vertex_t));
        }
    );


    auto material = make_sptr(new mochi::asset::material(eng));
    material->setColor({0,0,1});
    material->setPolymode(mochi::rhi::PolygonMode::Line);
    material->setPrimitiveTopology(mochi::rhi::PrimitiveTopology::eLineList);
    
    return mochi::asset::mesh::make(buffer, {{0, vertices.size()}}, {material}, {0});
}

static sptr<mochi::asset::mesh> createWireframeConvexHull(mochi::core& eng, const std::vector<Baryon::Vector3>& verts) {
    std::vector<mochi::asset::vertex_t> vertices;
    for (size_t i = 0; i < verts.size(); ++i) {
        size_t next = (i + 1) % verts.size();
        mochi::asset::vertex_t v1, v2;
        v1.position = {verts[i].x, verts[i].y, verts[i].z}; v1.normal={0,1,0}; v1.color={0.2f, 0.6f, 1.0f}; v1.uv={0,0};
        v2.position = {verts[next].x, verts[next].y, verts[next].z}; v2.normal={0,1,0}; v2.color={0.2f, 0.6f, 1.0f}; v2.uv={0,0};
        vertices.push_back(v1);
        vertices.push_back(v2);
        
        size_t across = (i + verts.size()/2) % verts.size();
        mochi::asset::vertex_t v3;
        v3.position = {verts[across].x, verts[across].y, verts[across].z}; v3.normal={0,1,0}; v3.color={0.2f, 0.6f, 1.0f}; v3.uv={0,0};
        vertices.push_back(v1);
        vertices.push_back(v3);
    }
    
    auto buffer = mochi::rhi::buffer::make(
        eng.sub<mochi::module::device>(),
        eng.sub<mochi::module::memory>(),
        mochi::asset::vertex_i, vertices.size(),
        flags(mochi::rhi::BufferUsage::VertexBuffer) | mochi::rhi::BufferUsage::TransferDst,
        flags(mochi::rhi::BufferCreate::Mapped) | mochi::rhi::BufferCreate::HostSequentialWrite,
        mochi::rhi::BufferLocation::PreferDevice,
        [&](void* ptr) {
            std::memcpy(ptr, vertices.data(), vertices.size() * sizeof(mochi::asset::vertex_t));
        }
    );

    auto material = make_sptr(new mochi::asset::material(eng));
    material->setColor({0,0,1});
    material->setPolymode(mochi::rhi::PolygonMode::Line);
    material->setPrimitiveTopology(mochi::rhi::PrimitiveTopology::eLineList);
    
    return mochi::asset::mesh::make(buffer, {{0, vertices.size()}}, {material}, {0});
}

} // namespace DebugDraw
