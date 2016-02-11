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

class GeometryBuffer
{
public:
    GeometryBuffer()
        : buffer_type(0), num_elements(0), datatype(0), count_per_element(0), gluse(0), data(0)
    {
    }

    GeometryBuffer(GlBufferType buffer_type, GLuint num_elements, GLenum datatype, GLuint count_per_element, GLenum gluse = GL_DYNAMIC_DRAW )
    {
        Reinitialise(buffer_type, num_elements, datatype, count_per_element, gluse);
    }

#ifdef CALLEE_HAS_RVALREF
    //! Move Constructor
    GeometryBuffer(GeometryBuffer&& other)
        : buffer_type(other.buffer_type), num_elements(other.num_elements),
          datatype(other.datatype), count_per_element(other.count_per_element),
          gluse(other.gluse), data(other.data)
    {
        other.data = 0;
    }
#endif

    void Clear()
    {
        if(data) {
            delete[] data;
        }
    }

    void Reinitialise(GlBufferType buffer_type, GLuint num_elements, GLenum datatype, GLuint count_per_element, GLenum gluse = GL_DYNAMIC_DRAW )
    {
        Clear();
        this->buffer_type = buffer_type;
        this->num_elements = num_elements;
        this->datatype = datatype;
        this->count_per_element = count_per_element;
        this->gluse = gluse;
    }

    void OwnData(void* data)
    {
        Clear();
        this->data = data;
    }

    ~GeometryBuffer()
    {
        Clear();
    }

    GlBuffer CreateGlBuffer() const
    {
        GlBuffer glbuf(buffer_type, num_elements, datatype, count_per_element, gluse);
        size_t size_bytes = num_elements * count_per_element * GlDataTypeBytes(datatype);
        glbuf.Upload(data, size_bytes);
        return glbuf;
    }

    GlBufferType buffer_type;
    GLuint num_elements;
    GLenum datatype;
    GLuint count_per_element;
    GLenum gluse;
    void* data;

private:
    // Private copy constructor
    GeometryBuffer(const GeometryBuffer&) {}
};

}

#endif // PANGOLIN_GEOMETRY_BUFFER_H
