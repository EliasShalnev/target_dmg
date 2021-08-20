#include "target_damage/NSParser.h"


NSParser::NSParser(const NameSpace& nameSpace) 
    : m_nameSpace(nameSpace)
{
    std::size_t segmentBegin = m_nameSpace.find('/');
    std::size_t segmentEnd = m_nameSpace.find('/');
     
    while(segmentEnd != std::string::npos)
    {
        segmentEnd = m_nameSpace.find('/', segmentEnd+1);
        
        Segment segment = m_nameSpace.substr(segmentBegin, segmentEnd-segmentBegin);

        m_segments.emplace_back(segment);

        segmentBegin = segmentEnd;
    }
}


NSParser::NSParser(const NSParser& rhs) 
    : m_nameSpace(rhs.m_nameSpace)
    , m_segments(rhs.m_segments)
{ }


NSParser::NameSpace NSParser::getSubNameSpace(std::size_t from, std::size_t to) const
{
    if( from >= m_segments.size() ) { return nullptr; }
    if( to >= m_segments.size() ) { return nullptr; }
    if( from > to ) { return nullptr; }

    NameSpace subNS;
    for(std::size_t i = from; i<=to; ++i)
    {
        subNS.append(m_segments[i]);
    }

    return subNS;
}
