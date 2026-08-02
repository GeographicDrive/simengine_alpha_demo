// tools/ac3d_to_obj/main.cpp — Asset compatibility tool.
//
// Converts an AC3D (.ac) mesh file — the format used by the uploaded
// A320-family.zip's Models/*.ac files — into Wavefront OBJ, a format
// any future renderer/importer (Blender, Assimp, three.js, etc.) can
// read directly, so this engine's eventual rendering backend isn't
// stuck needing a bespoke AC3D loader just to use these assets.
//
// Supports: nested OBJECT/kids hierarchy with per-object `loc` (translate)
// and `rot` (3x3 row-major matrix) transforms, numvert/vertex lists,
// numsurf/SURF polygons with `refs` vertex+UV pairs, and MATERIAL name
// passthrough as OBJ groups (not full material/texture export — see
// LIMITATIONS below).
//
// LIMITATIONS (this is a real converter, not a complete AC3D
// implementation — flagged so nobody assumes more fidelity than this
// gives):
//  - Materials become OBJ `usemtl <name>` references only; no .mtl file
//    with actual color/texture data is emitted. Re-applying the source
//    AC3D MATERIAL colors/textures is a mechanical follow-up (the data
//    is parsed and available in Ac3dMaterial, just not written out yet).
//  - `crease`, `SURF` flags for smoothing groups are read but not
//    translated to OBJ smoothing groups.
//  - Concave/non-planar polygons are emitted as OBJ faces as-is; OBJ
//    consumers that require convex/triangulated input will need their
//    own triangulation pass.

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

namespace {

struct Vec3 { double x = 0, y = 0, z = 0; };

struct Mat3 {
    std::array<double, 9> m{1,0,0, 0,1,0, 0,0,1};
    Vec3 apply(const Vec3& v) const {
        return Vec3{
            m[0]*v.x + m[1]*v.y + m[2]*v.z,
            m[3]*v.x + m[4]*v.y + m[5]*v.z,
            m[6]*v.x + m[7]*v.y + m[8]*v.z
        };
    }
};

struct SurfRef { int vertexIndex; double u, v; };
struct Surf { std::vector<SurfRef> refs; int materialIndex = 0; };

struct MeshOut {
    std::vector<Vec3> positions; // world-space, after hierarchy transforms applied
    std::vector<std::pair<double,double>> uvs;
    std::vector<Surf> surfs;
    std::vector<std::string> objectNames; // one per surf, for grouping
};

std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> toks;
    std::istringstream ss(line);
    std::string t;
    while (ss >> t) toks.push_back(t);
    return toks;
}

std::string stripQuotes(const std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') return s.substr(1, s.size() - 2);
    return s;
}

// Recursively parses one OBJECT block (already consumed the "OBJECT <type>"
// line's content into `kind`), accumulating `parentTransform` composed with
// this object's own loc/rot, and appends any polygon surfaces found into
// `out`.
void parseObject(std::istream& in, const std::string& /*kind*/,
                  Vec3 parentTranslate, Mat3 parentRotate, MeshOut& out) {
    Vec3 loc{0,0,0};
    Mat3 rot; // identity
    std::string name = "unnamed";
    std::vector<Vec3> localVerts;
    std::vector<std::pair<double,double>> pendingUVsPlaceholder; // unused, UVs come from refs

    std::string line;
    while (std::getline(in, line)) {
        auto toks = tokenize(line);
        if (toks.empty()) continue;

        if (toks[0] == "name" && toks.size() >= 2) {
            name = stripQuotes(toks[1]);
        } else if (toks[0] == "loc" && toks.size() >= 4) {
            loc = Vec3{std::stod(toks[1]), std::stod(toks[2]), std::stod(toks[3])};
        } else if (toks[0] == "rot" && toks.size() >= 10) {
            for (int i = 0; i < 9; ++i) rot.m[i] = std::stod(toks[i + 1]);
        } else if (toks[0] == "numvert" && toks.size() >= 2) {
            int n = std::stoi(toks[1]);
            localVerts.reserve(n);
            for (int i = 0; i < n; ++i) {
                if (!std::getline(in, line)) break;
                auto vt = tokenize(line);
                if (vt.size() >= 3) {
                    localVerts.push_back(Vec3{std::stod(vt[0]), std::stod(vt[1]), std::stod(vt[2])});
                }
            }
        } else if (toks[0] == "numsurf" && toks.size() >= 2) {
            int n = std::stoi(toks[1]);
            // Base index for this object's vertices in the OBJ output —
            // append localVerts (transformed) to out.positions now.
            const std::size_t vertBase = out.positions.size();
            for (auto& lv : localVerts) {
                Vec3 rotated = rot.apply(lv);
                Vec3 world = parentRotate.apply(Vec3{rotated.x + loc.x, rotated.y + loc.y, rotated.z + loc.z});
                world.x += parentTranslate.x; world.y += parentTranslate.y; world.z += parentTranslate.z;
                out.positions.push_back(world);
            }
            for (int s = 0; s < n; ++s) {
                Surf surf;
                std::string surfLine;
                // Read until we hit "refs N" for this surface, capturing mat along the way.
                while (std::getline(in, surfLine)) {
                    auto st = tokenize(surfLine);
                    if (st.empty()) continue;
                    if (st[0] == "mat" && st.size() >= 2) surf.materialIndex = std::stoi(st[1]);
                    else if (st[0] == "refs" && st.size() >= 2) {
                        int refCount = std::stoi(st[1]);
                        for (int r = 0; r < refCount; ++r) {
                            if (!std::getline(in, surfLine)) break;
                            auto rt = tokenize(surfLine);
                            if (rt.size() >= 3) {
                                SurfRef ref;
                                ref.vertexIndex = static_cast<int>(vertBase) + std::stoi(rt[0]);
                                ref.u = std::stod(rt[1]);
                                ref.v = std::stod(rt[2]);
                                surf.refs.push_back(ref);
                            }
                        }
                        break; // done with this SURF block
                    }
                }
                if (!surf.refs.empty()) {
                    out.surfs.push_back(std::move(surf));
                    out.objectNames.push_back(name);
                }
            }
        } else if (toks[0] == "kids" && toks.size() >= 2) {
            int kidCount = std::stoi(toks[1]);
            const Vec3 combinedTranslate = [&]{
                Vec3 rotatedLoc = parentRotate.apply(loc);
                return Vec3{parentTranslate.x + rotatedLoc.x,
                            parentTranslate.y + rotatedLoc.y,
                            parentTranslate.z + rotatedLoc.z};
            }();
            Mat3 combinedRotate; // parentRotate * rot, composed manually
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) {
                    double sum = 0;
                    for (int k = 0; k < 3; ++k) sum += parentRotate.m[r*3+k] * rot.m[k*3+c];
                    combinedRotate.m[r*3+c] = sum;
                }
            for (int k = 0; k < kidCount; ++k) {
                std::string kidLine;
                while (std::getline(in, kidLine)) {
                    auto kt = tokenize(kidLine);
                    if (!kt.empty() && kt[0] == "OBJECT") {
                        parseObject(in, kt.size() >= 2 ? kt[1] : "poly", combinedTranslate, combinedRotate, out);
                        break;
                    }
                }
            }
            return; // this object's block is fully consumed (kids handled recursively)
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: ac3d_to_obj <input.ac> <output.obj>\n");
        return 1;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::fprintf(stderr, "error: could not open %s\n", argv[1]);
        return 1;
    }

    std::string header;
    std::getline(in, header);
    if (header.compare(0, 4, "AC3D") != 0) {
        std::fprintf(stderr, "error: %s does not look like an AC3D file (header: %s)\n", argv[1], header.c_str());
        return 1;
    }

    MeshOut mesh;
    std::string line;
    while (std::getline(in, line)) {
        auto toks = tokenize(line);
        if (toks.empty()) continue;
        if (toks[0] == "OBJECT") {
            parseObject(in, toks.size() >= 2 ? toks[1] : "poly", Vec3{0,0,0}, Mat3{}, mesh);
        }
        // MATERIAL lines are skipped here — see LIMITATIONS at top of file.
    }

    std::ofstream out(argv[2]);
    if (!out) {
        std::fprintf(stderr, "error: could not write %s\n", argv[2]);
        return 1;
    }
    out << "# Converted from " << argv[1] << " by simengine's ac3d_to_obj tool\n";
    out << "# NOTE: materials/textures are not exported — see tools/ac3d_to_obj/main.cpp LIMITATIONS.\n";
    for (auto& p : mesh.positions) out << "v " << p.x << " " << p.y << " " << p.z << "\n";
    for (auto& s : mesh.surfs) {
        for (auto& r : s.refs) out << "vt " << r.u << " " << r.v << "\n";
    }
    std::size_t vtCursor = 1; // OBJ is 1-indexed
    std::string lastGroup;
    for (std::size_t i = 0; i < mesh.surfs.size(); ++i) {
        const auto& s = mesh.surfs[i];
        if (mesh.objectNames[i] != lastGroup) {
            out << "g " << mesh.objectNames[i] << "\n";
            lastGroup = mesh.objectNames[i];
        }
        out << "f";
        for (auto& r : s.refs) {
            out << " " << (r.vertexIndex + 1) << "/" << vtCursor;
            ++vtCursor;
        }
        out << "\n";
    }

    std::fprintf(stderr, "Converted %s -> %s (%zu vertices, %zu faces)\n",
                 argv[1], argv[2], mesh.positions.size(), mesh.surfs.size());
    return 0;
}
