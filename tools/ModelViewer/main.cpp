#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>

#include <pangolin/utils/file_utils.h>

#include "GeometryPly.h"

#define DEBUG_PRINT(x) std::cerr << (#x) << ": " << x << std::endl

int main( int argc, char** argv )
{
    if( argc < 2) {
        std::cout << "usage: ModelViewer modelname.ply" << std::endl;
        exit(-1);
    }

    const std::string model_filename = argv[1];
    const std::string filename_vertshader = "shader.vert";
    const std::string filename_fragshader = "shader.frag";

    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(0,0,10, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    pangolin::Geometry geom = pangolin::LoadGeometry(model_filename);
    geom.buffers[1].buffer_type = pangolin::GlElementArrayBuffer;

    for(auto& kv : geom.attribs) {
        std::cerr << "--------------------------------------------------------" << std::endl;
        std::cerr << kv.first << ":" << std::endl;
        pangolin::GeometryAttrib& a = kv.second;
        DEBUG_PRINT(a.index);
        DEBUG_PRINT(a.pointer);
        DEBUG_PRINT(a.num_items);
        DEBUG_PRINT(a.stride);
        DEBUG_PRINT(a.type);
    }

    pangolin::GlGeometry glgeom(geom);

    pangolin::GlSlProgram prog;
    prog.AddShaderFromFile(pangolin::GlSlVertexShader,   filename_vertshader);
    prog.AddShaderFromFile(pangolin::GlSlFragmentShader, filename_fragshader);
    prog.Link();

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        prog.Bind();
        prog.SetUniform("T_cw",   s_cam.GetModelViewMatrix() );
        prog.SetUniform("KT_cw", s_cam.GetProjectionModelViewMatrix() );
        prog.SetUniform("K",      s_cam.GetProjectionMatrix() );

        CheckGlDieOnError();
        // Enable
        int num_elements = 0;
        for(auto& kv : glgeom.attribs) {
            const std::string& attrib_name = kv.first;
            if(pangolin::StartsWith(attrib_name, "vertex")) {
                const pangolin::GeometryAttrib& a = kv.second;
                glgeom.buffers[a.index].Bind();
                GLint attrib = prog.GetAttributeHandle(attrib_name);
                glEnableVertexAttribArray(attrib);
                glVertexAttribPointer(attrib, 1, a.type, a.normalized, a.stride, a.pointer);
                CheckGlDieOnError();
                num_elements = a.num_items;
            }
        }

        // Render Mesh
        pangolin::GeometryAttrib& tristrip = glgeom.attribs["tristrips_vertex_indices"];
        glgeom.buffers[tristrip.index].Bind();
        // OR with 1 to make sure we have the unsigned type
        glDrawElements(GL_TRIANGLE_STRIP, tristrip.num_items, tristrip.type | 1, tristrip.pointer);
        glgeom.buffers[tristrip.index].Unbind();

//        // Draw points only
//        glDrawArrays(GL_POINTS, 0, num_elements);

        CheckGlDieOnError();
        // Disable
        for(auto& kv : glgeom.attribs) {
            const std::string& attrib_name = kv.first;
            if(pangolin::StartsWith(attrib_name, "vertex")) {
                GLint attrib = prog.GetAttributeHandle(attrib_name);
                glDisableVertexAttribArray(attrib);
            }
        }
        CheckGlDieOnError();

        glBindBuffer(pangolin::GlArrayBuffer, 0);
        prog.Unbind();

        pangolin::glDrawAxis(1.0);

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}
