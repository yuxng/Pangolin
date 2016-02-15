/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pangolin/geometry/geometry.h>
#include <pangolin/platform.h>
#include <pangolin/gl/gl.h>

#include "Parse.h"

#include <fstream>
#include <vector>

namespace pangolin
{

#define PLY_GROUP_LIST(m)  m(PlyHeader) m(PlyFormat) m(PlyType)
#define PLY_HEADER_LIST(m) m(ply) m(format) m(comment) m(property) m(element) m(end_header)
#define PLY_FORMAT_LIST(m) m(ascii) m(binary_big_endian) m(binary_little_endian)
#define PLY_TYPE_LIST(m)   m(char) m(uchar) m(short) m(ushort) m(int) m(uint) m(float) m(double) m(list)

#define FORMAT_STRING_LIST(x) #x,

// Define Enums / strings
enum PlyHeader {
#define FORMAT_ENUM(x) PlyHeader_##x,
    PLY_HEADER_LIST(FORMAT_ENUM)
    PlyHeaderSize
#undef FORMAT_ENUM
};
const char* PlyHeaderString[] = {
   PLY_HEADER_LIST(FORMAT_STRING_LIST)
};

enum PlyFormat {
#define FORMAT_ENUM(x) PlyFormat_##x,
    PLY_FORMAT_LIST(FORMAT_ENUM)
    PlyFormatSize
#undef FORMAT_ENUM
};
const char* PlyFormatString[] = {
   PLY_FORMAT_LIST(FORMAT_STRING_LIST)
};

enum PlyType {
#define FORMAT_ENUM(x) PlyType_##x,
    PLY_TYPE_LIST(FORMAT_ENUM)
    PlyTypeSize
#undef FORMAT_ENUM
};
const char* PlyTypeString[] = {
   PLY_TYPE_LIST(FORMAT_STRING_LIST)
};
const GLenum PlyTypeGl[] = {
    GL_BYTE, GL_UNSIGNED_BYTE,
    GL_SHORT, GL_UNSIGNED_SHORT,
    GL_INT, GL_UNSIGNED_INT,
    GL_FLOAT, GL_DOUBLE,
    GL_NONE
};

#undef FORMAT_ENUM
#undef FORMAT_STRING_LIST

PLY_GROUP_LIST(PANGOLIN_DEFINE_PARSE_TOKEN)

struct PlyPropertyDetails
{
    std::string name;

    // Type of property (or type of list size, if a list)
    GLenum type;

    // Type of list elements if a list, or 0 otherwise.
    GLenum list_type;

    bool isList() const {
        return list_type > 0;
    }
};

struct PlyElementDetails
{
    std::string name;
    int num_items;
    int size_bytes;
    std::vector<PlyPropertyDetails> properties;
};

struct PlyHeaderDetails
{
    PlyFormat format;
    std::string version;
    std::vector<PlyElementDetails> elements;
};

void ParsePlyHeader(PlyHeaderDetails& ply, std::istream& is)
{
    // 'Active' element for property definitions.
    int current_element = -1;

    // Check header is correct
    PlyHeader token = ParseTokenPlyHeader(is);
    if( token != PlyHeader_ply) {
        throw std::runtime_error("Bad PLY header magic.");
    }
    ConsumeToNewline(is);

    while(is.good() && token != PlyHeader_end_header) {
        token = ParseTokenPlyHeader(is);
        switch (token) {
        case PlyHeader_format:
            // parse PLY format and version
            ConsumeWhitespace(is);
            ply.format = ParseTokenPlyFormat(is);
            ConsumeWhitespace(is);
            ply.version = ReadToken(is);
            break;
        case PlyHeader_element:
        {
            current_element = ply.elements.size();
            PlyElementDetails el;
            el.size_bytes = 0;
            ConsumeWhitespace(is);
            el.name = ReadToken(is);
            ConsumeWhitespace(is);
            el.num_items = FromString<int>(ReadToken(is));
            ply.elements.push_back(el);
            break;
        }
        case PlyHeader_property:
            if(current_element >= 0) {
                PlyElementDetails& el = ply.elements[current_element];
                PlyPropertyDetails prop;
                ConsumeWhitespace(is);
                const PlyType t = ParseTokenPlyType(is);
                if( t == PlyType_list) {
                    ConsumeWhitespace(is);
                    const PlyType idtype = ParseTokenPlyType(is);
                    ConsumeWhitespace(is);
                    const PlyType listtype = ParseTokenPlyType(is);
                    el.size_bytes = -1;
                    prop.type = PlyTypeGl[idtype];
                    prop.list_type = PlyTypeGl[listtype];
                }else{
                    prop.type = PlyTypeGl[t];
                    prop.list_type = GL_NONE;
                    const size_t size_bytes = GlDataTypeBytes(prop.type);
                    if( el.size_bytes >= 0) {
                        el.size_bytes += size_bytes;
                    }
                }
                ConsumeWhitespace(is);
                prop.name = ReadToken(is);
                el.properties.push_back(prop);
            }else{
                pango_print_warn("PLY Parser: property declaration before element. Ignoring line.");
            }
            break;
        case PlyHeader_comment:
        case PlyHeader_end_header:
            break;
        default:
            pango_print_warn("PLY Parser: Unknown token - ignoring line.");
        }
        ConsumeToNewline(is);
    }
}

struct PlyBuffer
{
    size_t index_size_bytes;
    size_t element_item_size_bytes;
    std::vector<unsigned char> data;
};

void ParsePlyAscii(pangolin::Geometry& geom, const PlyHeaderDetails& ply, std::istream& is)
{
    throw std::runtime_error("Not implemented.");
}

void ParsePlyLE(pangolin::Geometry& geom, const PlyHeaderDetails& ply, std::istream& is)
{
    for(int e=0; e < ply.elements.size(); ++e) {
        const PlyElementDetails& el = ply.elements[e];

        std::vector<unsigned char> buffer;
        if(el.size_bytes > 0) {
            // Read data in as fixed sized block
            const size_t num_bytes = el.size_bytes * el.num_items;
            buffer.resize(num_bytes);
            is.read((char*)buffer.data(), num_bytes);
        }else {
            // Read incrementally - assume all variable sized lists for element are the same size.
            buffer.reserve(1024*1024);
            unsigned char* buf = buffer.data();
            for(int i=0; i< el.num_items; ++i) {
                for(int p=0; p < el.properties.size(); ++p) {

//                    ReadIndex(p)
                }
            }
        }
    }
}

void ParsePlyBE(pangolin::Geometry& geom, const PlyHeaderDetails& ply, std::istream& is)
{
    throw std::runtime_error("Not implemented.");
}

pangolin::Geometry LoadGeometryPly(const std::string& filename)
{
    std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
    if( !bFile.is_open() ) throw std::runtime_error("Unable to open PLY file: " + filename);

    PlyHeaderDetails ply;
    ParsePlyHeader(ply, bFile);

    // Initialise geom object
    pangolin::Geometry geom;
    geom.buffers.resize(ply.elements.size());
    // fill in details...

    if(ply.format == PlyFormat_ascii) {
        ParsePlyAscii(geom, ply, bFile);
    }else if(ply.format == PlyFormat_binary_little_endian) {

    }
    return pangolin::Geometry();
}

pangolin::Geometry LoadGeometry(const std::string& filename)
{
    const ImageFileType ft = FileType(filename);
    if(ft == ImageFileTypePly) {
        return LoadGeometryPly(filename);
    }else{
        throw std::runtime_error("Unsupported geometry file type.");
    }
}


}
