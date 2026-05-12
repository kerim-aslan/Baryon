/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "Basis.hh"
#include "mochi/reader/reader.hh"
#include <span>
#include <spanstream>
#include <sstream>
#include <string>



namespace mochi
{

  template<>
  fun read<ft_wavefront>(data src) -> wf_obj
  {
    wf_obj data;
    std::spanstream file(std::span<char>((char*)src.ptr(), src.size()));

    
    std::string line;
    while (std::getline(file, line)) 
    {
      if (line.empty() || line[0] == '#') continue;

      std::istringstream iss(line);
      std::string prefix;
      iss >> prefix;

      if (prefix == "v") {
        mochi::vec3<f32> pos;
        iss >> pos.X >> pos.Y >> pos.Z;
        data.v.push_back(pos);
      }
      else if (prefix == "vt") {
        mochi::vec2<f32> uv;
        iss >> uv.X >> uv.Y;
        data.vt.push_back(uv); 
      }
      else if (prefix == "vn") {
        mochi::vec3<f32> normal;
        iss >> normal.X >> normal.Y >> normal.Z;
        data.vn.push_back(normal);
      }
      else if (prefix == "f") {
        wf_face face;
        std::string vertex_data;
        
        while (iss >> vertex_data) {
          wf_index idx;
          size_t first_slash = vertex_data.find('/');
          size_t second_slash = vertex_data.find('/', first_slash + 1);

          idx.v_idx = std::stoi(vertex_data.substr(0, first_slash)) - 1;

          if (first_slash != std::string::npos) {
            if (first_slash + 1 != second_slash) {
              idx.vt_idx = std::stoi(vertex_data.substr(first_slash + 1, second_slash - first_slash - 1)) - 1;
            }
            
            if (second_slash != std::string::npos && second_slash + 1 < vertex_data.length()) {
              idx.vn_idx = std::stoi(vertex_data.substr(second_slash + 1)) - 1;
            }
          }
          face.vertices.push_back(idx);
        }
        data.f.push_back(face);
      }
      else if (prefix == "o") {
        std::string cac;
        iss >> cac; // data.object_name;
      }
      else if (prefix == "usemtl") {
        std::string cac;
        iss >> cac; //data.material_name;
      }
    }

    return data;
  }
  
}
