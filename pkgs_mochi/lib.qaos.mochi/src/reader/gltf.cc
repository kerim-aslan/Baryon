/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "Basis.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/reader/reader.hh"
#define CGLTF_IMPLEMENTATION
#include "cgltf.h"



namespace mochi
{

  template<>
  fun read<ft_gltf>(data src) -> gltf_obj
  {
    gltf_obj result;
    
    cgltf_options options = {};
    cgltf_data* data = NULL;
    
    
    // Read File
    if (cgltf_parse(&options, src.ptr(), src.size(), &data) != cgltf_result_success)
      throw std::runtime_error("glTF file cannot parsed.");

    
    // Binary
    if (cgltf_load_buffers(&options, data, nullptr) != cgltf_result_success) {
      cgltf_free(data);
      throw std::runtime_error("glTF buffers cannot loaded.");
    }

    
    for (cgltf_size i = 0; i < data->meshes_count; i++)
    {
      cgltf_mesh *mesh = &data->meshes[i];
      
      for (cgltf_size j = 0; j < mesh->primitives_count; j++)
      {
        cgltf_primitive *prim = &mesh->primitives[j];

        // 1. Önce bu parçadaki benzersiz (unique) noktaları geçici bir diziye al
        cgltf_size vertex_count = prim->attributes[0].data->count;
        std::vector<asset::vertex_t> temp_vertices(vertex_count);

        // Her vertex için renkleri başta beyaz yapalım
        for (auto& v : temp_vertices) v.color = {1.0f, 1.0f, 1.0f};

        // Nitelikleri (Position, Normal, UV) bul ve kopyala
        for (cgltf_size a = 0; a < prim->attributes_count; ++a) {
          cgltf_attribute* attr = &prim->attributes[a];
          
          if (attr->type == cgltf_attribute_type_position) {
            for (cgltf_size v = 0; v < vertex_count; ++v)
              cgltf_accessor_read_float(attr->data, v, &temp_vertices[v].position.X, 3);
          } 
          ef (attr->type == cgltf_attribute_type_normal) {
            for (cgltf_size v = 0; v < vertex_count; ++v)
              cgltf_accessor_read_float(attr->data, v, &temp_vertices[v].normal.X, 3);
          } 
          ef (attr->type == cgltf_attribute_type_texcoord) {
            for (cgltf_size v = 0; v < vertex_count; ++v)
              cgltf_accessor_read_float(attr->data, v, &temp_vertices[v].uv.X, 2);
          }
        }

        // Unroll
        if (prim->indices) {
          for (cgltf_size id = 0; id < prim->indices->count; id++) {
            cgltf_size index = cgltf_accessor_read_index(prim->indices, id);
            result.vertices.push_back(temp_vertices[index]);
          }
        } else {
          result.vertices.insert(result.vertices.end(), temp_vertices.begin(), temp_vertices.end());
        }
      }
    }
    
    cgltf_free(data);
    return result;
  }
  
}
