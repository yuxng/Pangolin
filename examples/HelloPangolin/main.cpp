#include <pangolin/pangolin.h>
#include <pangolin/compat/thread.h>

void Win(const std::string& title)
{

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );
    pangolin::Handler3D handler(s_cam);

    pangolin::CreateWindowAndBind(title,640,480);
    glEnable(GL_DEPTH_TEST);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);


    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        pangolin::FinishFrame();
    }

    pangolin::DestroyWindow(title);
}

void Shared()
{
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );
    pangolin::Handler3D handler(s_cam);

    pangolin::WindowInterface& win1 = pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);
    pangolin::View& d_cam1 = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    pangolin::WindowInterface& win2 = pangolin::CreateWindowAndBind("Main2",640,480);
    glEnable(GL_DEPTH_TEST);
    pangolin::View& d_cam2 = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( true )
    {
        win1.MakeCurrent();

        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam1.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        pangolin::FinishFrame();
        if(pangolin::ShouldQuit()) break;


        win2.MakeCurrent();

        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam2.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        pangolin::FinishFrame();
        if(pangolin::ShouldQuit()) break;
    }
}

void SeperateThreads()
{
    boostd::thread th1(Win, "Main1");
    boostd::thread th2(Win, "Main2");
    th1.join();
    th2.join();
}

int main( int /*argc*/, char** /*argv*/ )
{  
//    Shared();
    SeperateThreads();

    return 0;
}
