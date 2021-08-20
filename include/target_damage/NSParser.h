#pragma once

#include <string>
#include <vector>

class NSParser
{
public:
    using NameSpace = std::string;
    using Segment = std::string;

public:
    NSParser(const NameSpace& nameSpace);
    NSParser(const NSParser& rhs);
    NSParser& operator=(const NSParser&) = delete;
    ~NSParser() = default;

    std::size_t size() const { return m_segments.size(); }

    Segment& operator[](std::size_t n) { return m_segments[n]; }

    const Segment& operator[](std::size_t n) const { return m_segments[n]; }

    auto begin() { return m_segments.begin(); }
    auto end() { return m_segments.end(); }

    auto begin() const { return m_segments.begin(); }
    auto end() const { return m_segments.end(); }

    NameSpace getSubNameSpace(std::size_t from, std::size_t to) const;

private:
    const NameSpace m_nameSpace;
    std::vector<Segment> m_segments;
};
