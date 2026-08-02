// simengine/io/xml.hpp — IO subsystem.
//
// Minimal dependency-free DOM XML parser. Not a general-purpose/spec-
// complete XML implementation (no DTD, no namespaces, no entity
// expansion beyond the five XML predefined entities) — it exists solely
// to read well-formed JSBSim aircraft data files (see io/jsbsim_import.hpp)
// without pulling in a third-party dependency for that one job. If a
// broader XML need shows up later, replace this with a real library
// (e.g. pugixml) rather than growing this file.

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <optional>

namespace simengine::io {

struct XmlElement {
    std::string tag;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::string text;                                  // direct text content (trimmed)
    std::vector<std::unique_ptr<XmlElement>> children;

    const std::string* attribute(const std::string& name) const noexcept {
        for (auto& [k, v] : attributes) if (k == name) return &v;
        return nullptr;
    }

    // First direct child with this tag, or nullptr.
    const XmlElement* child(const std::string& tagName) const noexcept {
        for (auto& c : children) if (c->tag == tagName) return c.get();
        return nullptr;
    }

    // All direct children with this tag.
    std::vector<const XmlElement*> childrenOf(const std::string& tagName) const {
        std::vector<const XmlElement*> out;
        for (auto& c : children) if (c->tag == tagName) out.push_back(c.get());
        return out;
    }

    // Recursively finds the first descendant with this tag (depth-first).
    const XmlElement* find(const std::string& tagName) const noexcept {
        for (auto& c : children) {
            if (c->tag == tagName) return c.get();
            if (auto* found = c->find(tagName)) return found;
        }
        return nullptr;
    }

    // Convenience: trimmed text of a direct child, parsed as double.
    std::optional<double> childDouble(const std::string& tagName) const {
        if (auto* c = child(tagName)) {
            try { return std::stod(c->text); } catch (...) { return std::nullopt; }
        }
        return std::nullopt;
    }
};

// Parses `xmlText` and returns the root element, or nullptr on malformed
// input (this parser is intentionally forgiving about things that don't
// matter for JSBSim files — e.g. it does not validate against a schema —
// but a structurally broken document, mismatched tags, etc., returns
// nullptr rather than guessing).
std::unique_ptr<XmlElement> parseXml(const std::string& xmlText);

} // namespace simengine::io
