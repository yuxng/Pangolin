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

#ifndef PANGOLIN_GEOMETRY_H
#define PANGOLIN_GEOMETRY_H

#include <map>
#include <vector>

#include <pangolin/geometry/geometry_buffer.h>

namespace pangolin
{

struct GeometryAttrib
{
    GLuint index;
    GLint num_items;
    GLenum type;
    GLboolean normalized;
    GLsizei stride;
    GLvoid* pointer;
};

struct Geometry
{
    Geometry()
    {
        buffers.reserve(10);
    }

    inline void PushSwap(GeometryBufferData& b)
    {
        buffers.push_back( GeometryBufferData(b.buffer_type, 0, b.gluse) );
        std::swap(buffers.back().data, b.data);
    }

    std::vector<GeometryBufferData> buffers;
    std::map<std::string, GeometryAttrib> attribs;
};

struct GlGeometry
{
    GlGeometry(const Geometry& geom)
        : attribs(geom.attribs)
    {
        buffers.resize(geom.buffers.size());
        for(int i=0; i < buffers.size(); ++i) {
            geom.buffers[i].CopyTo(buffers[i]);
        }
    }

    std::vector<GlBufferData> buffers;
    std::map<std::string, GeometryAttrib> attribs;
};


}

#endif // PANGOLIN_GEOMETRY_H
