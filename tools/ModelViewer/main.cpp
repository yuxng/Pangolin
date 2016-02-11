#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/utils/file_utils.h>

int main( int argc, char** argv )
{
    if( argc < 2) {
        std::cout << "usage: " << argv[0] << " modelname.ply";
    }

    const std::string model_filename = argv[1];
    const std::string filename_vertshader = "shader.vert";
    const std::string filename_fragshader = "shader.frag";

    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    pangolin::GlGeometry geometry = pangolin::LoadGeometry(model_filename);

    pangolin::GlSlProgram prog;
    prog.AddShaderFromFile(pangolin::GlSlVertexShader,   filename_vertshader);
    prog.AddShaderFromFile(pangolin::GlSlFragmentShader, filename_fragshader);
    prog.BindPangolinDefaultAttribLocationsAndLink();

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        prog.Bind();
        prog.SetUniform("T_wc",  s_cam.GetModelViewMatrix());
        prog.SetUniform("KT_wc", s_cam.GetProjectionModelViewMatrix();
        prog.SetUniform("K",     s_cam.GetProjectionMatrix());

        for(int i=0; i < geometry.buffers.size(); ++i) {

        }

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}
