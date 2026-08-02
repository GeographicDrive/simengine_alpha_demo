// simengine/render/obj_loader.cpp — see obj_loader.hpp for design notes.

#include "simengine/render/obj_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace simengine::render::objio {

namespace {

struct RawVec3 { float x = 0, y = 0, z = 0; };

// AC3D/JSBSim structural frame (X aft, Y right, Z up) -> this engine's
// body frame (X forward, Y right, Z down).
RawVec3 convertAxes(RawVec3 v) { return RawVec3{-v.x, v.y, -v.z}; }

// Wing/tail-surface files' local frame (X chord-aft, Y up, Z span with
// positive = left) -> this engine's body frame. See obj_loader.hpp.
RawVec3 convertWingAxes(RawVec3 v) { return RawVec3{-v.x, -v.z, -v.y}; }

struct FaceRef { int posIdx; int normIdx; }; // 0-based, -1 if absent

} // namespace

std::unordered_map<std::string, MeshPart> loadObjGroups(const std::string& path, bool wingSurfaceAxes) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("obj_loader: cannot open " + path);
    auto convert = wingSurfaceAxes ? convertWingAxes : convertAxes;

    std::vector<RawVec3> positions;
    std::vector<RawVec3> normals;
    std::unordered_map<std::string, MeshPart> groups;
    std::string currentGroup = "default";

    auto currentPart = [&]() -> MeshPart& {
        auto it = groups.find(currentGroup);
        if (it != groups.end()) return it->second;
        MeshPart p;
        p.name = currentGroup;
        return groups.emplace(currentGroup, std::move(p)).first->second;
    };

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            RawVec3 p;
            ss >> p.x >> p.y >> p.z;
            positions.push_back(convert(p));
        } else if (tag == "vn") {
            RawVec3 n;
            ss >> n.x >> n.y >> n.z;
            normals.push_back(convert(n));
        } else if (tag == "g") {
            std::string name;
            ss >> name;
            currentGroup = name.empty() ? "default" : name;
        } else if (tag == "f") {
            std::vector<FaceRef> refs;
            std::string tok;
            while (ss >> tok) {
                int posIdx = -1, normIdx = -1;
                // formats: "v", "v/vt", "v/vt/vn", "v//vn"
                size_t firstSlash = tok.find('/');
                if (firstSlash == std::string::npos) {
                    posIdx = std::stoi(tok) - 1;
                } else {
                    posIdx = std::stoi(tok.substr(0, firstSlash)) - 1;
                    size_t secondSlash = tok.find('/', firstSlash + 1);
                    if (secondSlash != std::string::npos && secondSlash + 1 < tok.size()) {
                        normIdx = std::stoi(tok.substr(secondSlash + 1)) - 1;
                    }
                }
                refs.push_back(FaceRef{posIdx, normIdx});
            }
            if (refs.size() < 3) continue;
            MeshPart& part = currentPart();
            // Fan-triangulate.
            for (size_t i = 1; i + 1 < refs.size(); ++i) {
                const FaceRef* tri[3] = {&refs[0], &refs[i], &refs[i + 1]};
                const uint32_t base = static_cast<uint32_t>(part.vertices.size());
                for (int k = 0; k < 3; ++k) {
                    const FaceRef& r = *tri[k];
                    Vertex v{};
                    if (r.posIdx >= 0 && static_cast<size_t>(r.posIdx) < positions.size()) {
                        v.px = positions[r.posIdx].x;
                        v.py = positions[r.posIdx].y;
                        v.pz = positions[r.posIdx].z;
                    }
                    if (r.normIdx >= 0 && static_cast<size_t>(r.normIdx) < normals.size()) {
                        v.nx = normals[r.normIdx].x;
                        v.ny = normals[r.normIdx].y;
                        v.nz = normals[r.normIdx].z;
                    } else {
                        v.nx = 0; v.ny = 0; v.nz = 1;
                    }
                    part.vertices.push_back(v);
                }
                part.indices.insert(part.indices.end(), {base, base + 1, base + 2});
            }
        }
        // ignore vt, usemtl, mtllib, comments, o, s — not needed here.
    }
    return groups;
}

} // namespace simengine::render::objio
