#include "simengine/io/xml.hpp"

#include <cctype>

namespace simengine::io {

namespace {

void skipWhitespace(const std::string& s, std::size_t& i) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
}

std::string trim(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string decodeEntities(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '&') {
            if (s.compare(i, 5, "&amp;") == 0) { out += '&'; i += 4; continue; }
            if (s.compare(i, 4, "&lt;") == 0) { out += '<'; i += 3; continue; }
            if (s.compare(i, 4, "&gt;") == 0) { out += '>'; i += 3; continue; }
            if (s.compare(i, 6, "&quot;") == 0) { out += '"'; i += 5; continue; }
            if (s.compare(i, 6, "&apos;") == 0) { out += '\''; i += 5; continue; }
        }
        out += s[i];
    }
    return out;
}

// Skips <?...?> processing instructions and <!--...--> comments and
// <!DOCTYPE ...> declarations starting at position i (which must point
// at the leading '<'). Returns true if something was skipped.
bool skipNonElement(const std::string& s, std::size_t& i) {
    if (s.compare(i, 2, "<?") == 0) {
        auto end = s.find("?>", i);
        i = (end == std::string::npos) ? s.size() : end + 2;
        return true;
    }
    if (s.compare(i, 4, "<!--") == 0) {
        auto end = s.find("-->", i);
        i = (end == std::string::npos) ? s.size() : end + 3;
        return true;
    }
    if (s.compare(i, 2, "<!") == 0) {
        auto end = s.find('>', i);
        i = (end == std::string::npos) ? s.size() : end + 1;
        return true;
    }
    return false;
}

std::unique_ptr<XmlElement> parseElement(const std::string& s, std::size_t& i);

// Parses attributes up to (not including) the closing '>' or "/>".
void parseAttributes(const std::string& s, std::size_t& i, XmlElement& el) {
    for (;;) {
        skipWhitespace(s, i);
        if (i >= s.size() || s[i] == '>' || s[i] == '/') return;
        std::size_t nameStart = i;
        while (i < s.size() && s[i] != '=' && !std::isspace(static_cast<unsigned char>(s[i])) && s[i] != '>' && s[i] != '/') ++i;
        std::string name = s.substr(nameStart, i - nameStart);
        skipWhitespace(s, i);
        if (i < s.size() && s[i] == '=') {
            ++i;
            skipWhitespace(s, i);
            if (i < s.size() && (s[i] == '"' || s[i] == '\'')) {
                char quote = s[i++];
                std::size_t valStart = i;
                while (i < s.size() && s[i] != quote) ++i;
                std::string value = decodeEntities(s.substr(valStart, i - valStart));
                if (i < s.size()) ++i; // closing quote
                if (!name.empty()) el.attributes.emplace_back(std::move(name), std::move(value));
            }
        }
    }
}

std::unique_ptr<XmlElement> parseElement(const std::string& s, std::size_t& i) {
    // Precondition: s[i] == '<' and this is a real element open tag.
    ++i; // consume '<'
    std::size_t nameStart = i;
    while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i])) && s[i] != '>' && s[i] != '/') ++i;
    auto el = std::make_unique<XmlElement>();
    el->tag = s.substr(nameStart, i - nameStart);

    parseAttributes(s, i, *el);

    skipWhitespace(s, i);
    if (i < s.size() && s[i] == '/') {
        // Self-closing: <tag .../>
        i += 2; // "/>"
        return el;
    }
    if (i < s.size() && s[i] == '>') ++i; // consume '>'

    std::string textAccum;
    for (;;) {
        if (i >= s.size()) break;
        if (s[i] == '<') {
            if (s.compare(i, 2, "</") == 0) {
                // Closing tag for this element.
                auto end = s.find('>', i);
                i = (end == std::string::npos) ? s.size() : end + 1;
                break;
            }
            if (skipNonElement(s, i)) continue;
            el->children.push_back(parseElement(s, i));
        } else {
            std::size_t textStart = i;
            while (i < s.size() && s[i] != '<') ++i;
            textAccum += s.substr(textStart, i - textStart);
        }
    }
    el->text = trim(decodeEntities(textAccum));
    return el;
}

} // namespace

std::unique_ptr<XmlElement> parseXml(const std::string& xmlText) {
    std::size_t i = 0;
    // Skip leading processing instructions/comments/whitespace to find
    // the root element.
    for (;;) {
        skipWhitespace(xmlText, i);
        if (i >= xmlText.size()) return nullptr;
        if (xmlText[i] != '<') return nullptr;
        if (skipNonElement(xmlText, i)) continue;
        break;
    }
    if (i >= xmlText.size() || xmlText[i] != '<') return nullptr;
    return parseElement(xmlText, i);
}

} // namespace simengine::io
