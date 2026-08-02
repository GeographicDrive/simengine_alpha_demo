// simengine/aircraft/a320_variants.hpp — the A320-family.zip package
// (Aircraft/A320-family/*) actually contains eight distinct real
// aircraft, not just the A320-211 this project started with. Each has
// its own JSBSim FDM XML (mass/CG/engine differ) bundled under
// assets/fgfs_source/variants/. All eight were verified with
// tools/fgfs_import — zero import warnings for any of them (see
// docs/ROADMAP.md).
//
// NOTE on the mesh: all eight variants report identical wing reference
// geometry (S=122.613 m^2, b=33.912 m, c=4.194 m) in their JSBSim data
// — this package's aero model doesn't vary that per variant even though
// the real aircraft have different fuselage lengths (A318 shortest,
// A321 longest). render/mesh_a320.cpp's converted airframe mesh is the
// A320-211 shape specifically; picking another variant here changes the
// real mass/CG/thrust it flies with, but currently still draws the
// A320-211 geometry. Converting each variant's own stretched fuselage
// .ac (present in the package under Models/, not yet extracted here)
// is the natural next step if the visual length needs to match too.
#pragma once

#include <array>
#include <string_view>

namespace simengine::aircraft {

struct A320Variant {
    std::string_view id;          // e.g. "A320-211" — matches the FDM filename stem
    std::string_view displayName; // e.g. "A320-211 (CFM56-5A1)"
    std::string_view fdmFile;     // filename under assets/fgfs_source/variants/
    std::string_view engineFile;  // filename under assets/fgfs_source/variants/
};

inline constexpr std::array<A320Variant, 8> kA320Variants = {{
    {"A318-111", "A318-111 (CFM56-5B8)", "A318-111.xml", "cfm56_5b8.xml"},
    {"A319-111", "A319-111 (CFM56-5B5)", "A319-111.xml", "cfm56_5b5.xml"},
    {"A319-131", "A319-131 (V2522-A5)", "A319-131.xml", "v2522_a5.xml"},
    {"A320-111", "A320-111 (CFM56-5A1)", "A320-111.xml", "cfm56_5a1.xml"},
    {"A320-211", "A320-211 (CFM56-5A1)", "A320-211.xml", "cfm56_5a1.xml"},
    {"A320-231", "A320-231 (CFM56-5A1)", "A320-231.xml", "cfm56_5a1.xml"},
    {"A321-211", "A321-211 (CFM56-5B3)", "A321-211.xml", "cfm56_5b3.xml"},
    {"A321-231", "A321-231 (V2533-A5)", "A321-231.xml", "v2533_a5.xml"},
}};

// Index of "A320-211" in kA320Variants — the default/original variant,
// and the one the converted mesh geometry actually represents.
inline constexpr std::size_t kDefaultA320VariantIndex = 4;

} // namespace simengine::aircraft
