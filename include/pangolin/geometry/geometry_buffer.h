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

#ifndef PANGOLIN_GEOMETRY_BUFFER_H
#define PANGOLIN_GEOMETRY_BUFFER_H

#include <pangolin/gl/gl.h>

namespace pangolin
{

struct GeometryBufferData
{
    GeometryBufferData()
        : buffer_type(pangolin::GlUndefined), gluse(0)
    {
    }

    GeometryBufferData(GlBufferType buffer_type, GLuint size_bytes, GLenum gluse = GL_DYNAMIC_DRAW )
        : buffer_type(buffer_type), gluse(gluse)
    {
        data.resize(size_bytes);
    }

#ifdef CALLEE_HAS_RVALREF
    //! Move Constructor
    GeometryBufferData(GeometryBufferData&& o)
        : buffer_type(o.buffer_type), gluse(o.gluse), data(std::move(o.data))
    {
    }
#endif

    unsigned char* Data()
    {
        return data.data();
    }

    void CopyTo(GlBufferData& b) const
    {
        b.Reinitialise(buffer_type, data.size(), gluse, data.data());
    }

    GlBufferType buffer_type;
    GLenum gluse;
    std::vector<unsigned char> data;
private:
    // Private copy constructor
    GeometryBufferData(const GeometryBufferData&) {}
};

}

#endif // PANGOLIN_GEOMETRY_BUFFER_H
