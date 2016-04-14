#include <pangolin/pangolin.h>
#include <pangolin/compat/bind.h>
#include <pangolin/compat/thread.h>
#include <pangolin/compat/mutex.h>

#include <OVR.h>
#include <OVR_CAPI_GL.h>

#include <pthread.h>
#include <pangolin/display/device/X11Window.h>

class PangoOculus
{
public:
    static const unsigned int num_eyes = 2;
    static const unsigned int num_buffers = 3;

    struct OculusFrameBuffer
    {
        void Reinitialise(int w, int h)
        {
            eye_fb.Reinitialise();
            eye_rb.Reinitialise(w, h);
            eye_tex.Reinitialise(w, h);
            eye_fb.AttachColour(eye_tex);
            eye_fb.AttachDepth(eye_rb);
        }

        pangolin::GlFramebuffer eye_fb;
        pangolin::GlRenderBuffer eye_rb;
        pangolin::GlTexture eye_tex;
    };

    struct OculusStereoBuffer
    {
        void Reinitialise(int w, int h) {
            for(unsigned int eye=0; eye<num_eyes; ++eye) {
                buffer[eye].Reinitialise(w,h);

                ovrSizei size;
                size.w = w;
                size.h = h;
                eyeTex[eye].OGL.Header.API = ovrRenderAPI_OpenGL;
                eyeTex[eye].OGL.Header.TextureSize = size;
                eyeTex[eye].OGL.Header.RenderViewport = OVR::Recti(OVR::Vector2i(0, 0), size);
                eyeTex[eye].OGL.TexId = buffer[eye].eye_tex.tid;
            }
        }

        void SetPose(const OVR::Posef& T_hw, const ovrPosef T_we[num_eyes])
        {
            occ_cam.SetModelViewMatrix( pangolin::OpenGlMatrix(OVR::Matrix4f(T_hw)) );
            for(unsigned int eye=0; eye < num_eyes; ++eye) {
                Pose_w_eye[eye] = T_we[eye];
            }
        }

        void SetupEye(unsigned int eye) {
            if(eye < num_eyes) {
                buffer[eye].eye_fb.Bind();
                glViewport(0,0,buffer[eye].eye_tex.width, buffer[eye].eye_tex.height);
                occ_cam.ApplyNView((int)eye);
            }else{
                throw std::invalid_argument("Bad eye");
            }
        }

        void Unsetup() {
            buffer[0].eye_fb.Unbind();
        }

        unsigned int frame_index;

        pangolin::OpenGlRenderState occ_cam;
        OculusFrameBuffer buffer[num_eyes];
        ovrGLTexture eyeTex[num_eyes];
        ovrPosef Pose_w_eye[num_eyes];
    };

    PangoOculus(const char* window_title = "HMDWindow")
        : should_run(false), rendering(false), hmd(0), window_title(window_title), render_id(0)
    {
        occthread = boostd::thread(&PangoOculus::Run, boostd::ref(*this));

        sched_param sch;
        int policy;
        pthread_getschedparam(occthread.native_handle(), &policy, &sch);
        sch.sched_priority = 100;
        if(pthread_setschedparam(occthread.native_handle(), SCHED_FIFO, &sch)) {
            pango_print_warn("Failed to set thread priority\n");
        }
    }

    ~PangoOculus()
    {        
        should_run = false;
        rendering = false;
        occthread.join();
    }

//protected:

    void DetectOrSleep()
    {
        const int num_hmd = ovrHmd_Detect();
        if(num_hmd) {
            hmd = ovrHmd_Create(0);
        }else{
            boostd::this_thread::sleep_for(boostd::chrono::milliseconds(100));
        }
    }

    void Run()
    {
        ovr_Initialize();

        should_run = true;

        while(should_run) {
            if(!hmd) {
                DetectOrSleep();
            }else{
                RunWithHmd();
            }
        }

        ovrHmd_Destroy(hmd);
        ovr_Shutdown();
    }

    void RunWithHmd()
    {
        pangolin::WindowInterface& win = pangolin::CreateWindowAndBind( window_title, 10, 10 );
        pangolin::X11Window& x11win = dynamic_cast<pangolin::X11Window&>(win);

        glEnable(GL_DEPTH_TEST);

        ovrGLConfig config;
        config.OGL.Header.API			   = ovrRenderAPI_OpenGL;
        config.OGL.Header.BackBufferSize.w = hmd->Resolution.h;
        config.OGL.Header.BackBufferSize.h = hmd->Resolution.w;
        config.OGL.Header.Multisample      = 0;
        config.OGL.Disp                    = x11win.display->display;

        ovrEyeRenderDesc EyeRenderDesc[2];
        ovrHmd_ConfigureRendering( hmd, &config.Config,
            ovrDistortionCap_Vignette | ovrDistortionCap_TimeWarp |
            ovrDistortionCap_Overdrive, hmd->DefaultEyeFov, EyeRenderDesc
        );

        ovrHmd_SetEnabledCaps(hmd, ovrHmdCap_LowPersistence | ovrHmdCap_DynamicPrediction );
        ovrHmd_AttachToWindow(hmd, (void*)x11win.win, 0, 0);

        ovrHmd_ConfigureTracking(
            hmd, ovrTrackingCap_Orientation|ovrTrackingCap_MagYawCorrection| ovrTrackingCap_Position, 0
        );

        win.Resize(config.OGL.Header.BackBufferSize.w, config.OGL.Header.BackBufferSize.h);
        win.Move(hmd->WindowsPos.x, hmd->WindowsPos.y);
        win.ToggleFullscreen();

        for(size_t eye=0; eye<num_eyes; ++eye) {
            T_eh[eye].Translation = EyeRenderDesc[eye].HmdToEyeViewOffset;
        }

        const ovrSizei eye_tex_size = ovrHmd_GetFovTextureSize(hmd, (ovrEyeType)0, hmd->DefaultEyeFov[0], 1);

        for(unsigned int b=0; b < num_buffers; ++b) {
            buffers[b].Reinitialise(eye_tex_size.w, eye_tex_size.h);

            for(unsigned int eye=0; eye<num_eyes; ++eye) {
                buffers[b].occ_cam.GetViewOffset(eye) = pangolin::OpenGlMatrix( OVR::Matrix4f( T_eh[eye] ) );\
                buffers[b].occ_cam.GetProjectionMatrix(eye) = pangolin::OpenGlMatrix(ovrMatrix4f_Projection(
                    EyeRenderDesc[eye].Fov, 0.1f, 10.0f, ovrProjection_ClipRangeOpenGL | ovrProjection_RightHanded
                ));

//              buffers[b].occ_cam.GetProjectionMatrix(eye) = pangolin::ProjectionMatrix(
//                  eye_tex_size[eye].w, eye_tex_size[eye].h,
//                  focalLength, focalLength,
//                  eye_tex_size[eye].w/2.0 -(eye*2-1)*lensXOffset, eye_tex_size[eye].h/2.0,
//                  0.2, 100
//              );
            }
        }

        ovrHmd_DismissHSWDisplay(hmd);
        rendering = true;
        while(should_run) {
//            PangoOculus::OculusStereoBuffer* frame = GetBufferToRender();
//            if(frame) {
//                std::cout << frame->frame_index << std::endl;
//                for(unsigned int eye=0; eye < PangoOculus::num_eyes; ++eye) {
//                    frame->SetupEye(eye);
//                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//                    glTranslatef(0,0,-0.5);
//                    pangolin::glDrawColouredCube(-0.1f,0.1f);
//                    frame->Unsetup();
//                }
//                PutBufferToRender(frame);
//            }

            OculusStereoBuffer* render_frame = &buffers[render_id];
            ovrHmd_BeginFrame(hmd, render_frame->frame_index);
//            ovrHmd_BeginFrame(hmd, 0);
            ovrHmd_EndFrame(hmd, render_frame->Pose_w_eye, &render_frame->eyeTex[0].Texture);
            win.ProcessEvents();
        }
        rendering = false;

        pangolin::DestroyWindow(window_title);
    }

    OculusStereoBuffer* GetBufferToRender()
    {
        static unsigned int frame_index = 0;

        if(rendering) {
            OculusStereoBuffer* next_buffer = &buffers[ (render_id+1)%num_buffers];

            next_buffer->frame_index = frame_index++;
            ovrFrameTiming frameTiming = ovrHmd_GetFrameTiming(hmd, next_buffer->frame_index);
            ovrTrackingState tracking_state = ovrHmd_GetTrackingState(hmd, frameTiming.ScanoutMidpointSeconds );
//            ovrTrackingState tracking_state = ovrHmd_GetTrackingState(hmd, 0.0 );
            OVR::Posef T_hw = OVR::Posef(tracking_state.HeadPose.ThePose).Inverted();
            ovrPosef T_we[2] = { (T_eh[0]*T_hw).Inverted(), (T_eh[1] * T_hw).Inverted()};
            next_buffer->SetPose(T_hw, T_we);
            return next_buffer;
        }else{
            return 0;
        }
    }

    void PutBufferToRender(OculusStereoBuffer* buffer)
    {
        unsigned int next_render_id = (render_id+1)%num_buffers;

        if( buffer != &buffers[next_render_id] ) {
            throw std::runtime_error("Buffer returned out of order.");
        }

        render_id = next_render_id;
    }

    volatile bool should_run;
    volatile bool rendering;

    boostd::thread occthread;
    ovrHmd hmd;

    const char* window_title;
    OVR::Posef T_eh[2];

    unsigned int render_id;
    OculusStereoBuffer buffers[num_buffers];

    boostd::mutex flip_mutex;
};

void main_new()
{
    PangoOculus oculus;

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );
    pangolin::Handler3D handler(s_cam);
    pangolin::CreateWindowAndBind("Oculus",640,480);
    glEnable(GL_DEPTH_TEST);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    bool render_stuff = true;
    pangolin::RegisterKeyPressCallback(' ', [&](){render_stuff = !render_stuff;} );

    while( !pangolin::ShouldQuit() )
    {
        boostd::this_thread::sleep_for(boostd::chrono::milliseconds(60));

//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        d_cam.Activate(s_cam);

//        pangolin::glDrawColouredCube();

        // Submit frame for Oculus;
        if(render_stuff) {
            PangoOculus::OculusStereoBuffer* frame = oculus.GetBufferToRender();
            if(frame) {
//                std::cout << frame->frame_index << std::endl;
                for(unsigned int eye=0; eye < PangoOculus::num_eyes; ++eye) {
                    frame->SetupEye(eye);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    glTranslatef(0,0,-0.5);
                    pangolin::glDrawColouredCube(-0.1f,0.1f);
                    frame->Unsetup();
                }
                oculus.PutBufferToRender(frame);
            }
        }

        pangolin::FinishFrame();
    }
}

void main_old()
{
    ovr_Initialize();
    ovrHmd hmd = ovrHmd_Create(0);

    if(!hmd) {
        throw std::runtime_error("No Oculus headset available.");
    }
    pangolin::Params win_params;
    win_params.Set(pangolin::PARAM_DOUBLEBUFFER, false);
    pangolin::WindowInterface& win = pangolin::CreateWindowAndBind("Oculus", 10, 10, win_params);
    pangolin::X11Window& x11win = dynamic_cast<pangolin::X11Window&>(win);

    glEnable(GL_DEPTH_TEST);

    ovrGLConfig config;
    config.OGL.Header.API			   = ovrRenderAPI_OpenGL;
    config.OGL.Header.BackBufferSize.w = hmd->Resolution.h;
    config.OGL.Header.BackBufferSize.h = hmd->Resolution.w;
    config.OGL.Header.Multisample      = 0;
    config.OGL.Disp                    = x11win.display->display;

    ovrEyeRenderDesc eyeRenderDesc[2];
    ovrHmd_ConfigureRendering( hmd, &config.Config,
                               ovrDistortionCap_Vignette | /*ovrDistortionCap_TimeWarp |*/
                               ovrDistortionCap_Overdrive, hmd->DefaultEyeFov, eyeRenderDesc );

    ovrHmd_SetEnabledCaps(hmd, ovrHmdCap_LowPersistence /*| ovrHmdCap_DynamicPrediction*/);
    ovrHmd_AttachToWindow(hmd, (void*)x11win.win, 0, 0);

    ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation|ovrTrackingCap_MagYawCorrection|
                                  ovrTrackingCap_Position, 0);

    win.Resize(config.OGL.Header.BackBufferSize.w, config.OGL.Header.BackBufferSize.h);
    win.Move(hmd->WindowsPos.x, hmd->WindowsPos.y);
    win.ToggleFullscreen();

    pangolin::GlFramebuffer eye_fb[2];
    pangolin::GlRenderBuffer eye_rb[2];
    pangolin::GlTexture eye_tex[2];
    ovrGLTexture eyeTex[2];

    pangolin::OpenGlRenderState occ_cam;
    OVR::Posef T_eh[2];

    for(size_t i=0; i<2; ++i)
    {
        const ovrSizei eye_tex_size = ovrHmd_GetFovTextureSize(hmd, (ovrEyeType)i, hmd->DefaultEyeFov[i], 1);
        eye_rb[i].Reinitialise(eye_tex_size.w, eye_tex_size.h);
        eye_tex[i].Reinitialise(eye_tex_size.w, eye_tex_size.h);
        eye_fb[i].AttachColour(eye_tex[i]);
        eye_fb[i].AttachDepth(eye_rb[i]);
        std::cout << "Eye res: " << eye_tex_size.w << ", " << eye_tex_size.h << std::endl;

        // Setup OVR interop struct
        eyeTex[i].OGL.Header.API = ovrRenderAPI_OpenGL;
        eyeTex[i].OGL.Header.TextureSize = eye_tex_size;
        eyeTex[i].OGL.Header.RenderViewport = OVR::Recti(OVR::Vector2i(0, 0), eye_tex_size);
        eyeTex[i].OGL.TexId = eye_tex[i].tid;

        T_eh[i].Translation = eyeRenderDesc[i].HmdToEyeViewOffset;
        occ_cam.GetViewOffset(i) = pangolin::OpenGlMatrix( OVR::Matrix4f( T_eh[i] ) );

        occ_cam.GetProjectionMatrix(i) = pangolin::OpenGlMatrix(ovrMatrix4f_Projection(
            eyeRenderDesc[i].Fov, 0.1f, 10.0f, ovrProjection_ClipRangeOpenGL | ovrProjection_RightHanded
        ));

//        occ_cam.GetProjectionMatrix(i) = pangolin::ProjectionMatrix(
//            eye_tex_size[i].w, eye_tex_size[i].h,
//            focalLength, focalLength,
//            eye_tex_size[i].w/2.0 -(i*2-1)*lensXOffset, eye_tex_size[i].h/2.0,
//            0.2, 100
//            );
    }

    glViewport(0, 0, eye_tex[0].width, eye_tex[0].height);

    ovrHmd_DismissHSWDisplay(hmd);

    const int w = eye_tex[0].width;
    const int h = eye_tex[0].height;
    unsigned char buffer[w*h];

    unsigned int frame = 0;
    while(!pangolin::ShouldQuit()) {
//        boostd::this_thread::sleep_for(boostd::chrono::milliseconds(30));

        ovrFrameTiming timing = ovrHmd_BeginFrame(hmd, frame);

        ovrTrackingState tracking_state = ovrHmd_GetTrackingState(hmd, timing.ScanoutMidpointSeconds);

//        OVR::Posef T_hw = OVR::Posef(tracking_state.HeadPose.ThePose).Inverted();
//        occ_cam.SetModelViewMatrix( pangolin::OpenGlMatrix(OVR::Matrix4f(T_hw)) );

        ovrPosef    EyeRenderPose[2];
        ovrVector3f HmdToEyeViewOffset[2] = { eyeRenderDesc[0].HmdToEyeViewOffset, eyeRenderDesc[1].HmdToEyeViewOffset };
        ovrHmd_GetEyePoses(hmd, frame, HmdToEyeViewOffset, EyeRenderPose, &tracking_state);

//        for(int y=0; y<h; ++y) {
//            for(int x=0; x<w; ++x) {
//                double cx = w/2 - EyeRenderPose[0].Position.x * 1000;
//                double cy = h/2 - EyeRenderPose[0].Position.y * 1000;
//                double dx = (x - cx);
//                double dy = (y - cy);
//                double dist_sq = dx*dx + dy*dy;
//                unsigned char val = sqrt(dist_sq) < 20 ? 255 : 0;
//                buffer[w*y + x] = val;
//            }
//        }

//        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
//        glPixelStorei(GL_PACK_ALIGNMENT, 1);
//        eye_tex[0].Upload(buffer, GL_LUMINANCE, GL_UNSIGNED_BYTE);
//        eye_tex[1].Upload(buffer, GL_LUMINANCE, GL_UNSIGNED_BYTE);

        // Render each eye
        for(int eye=0; eye<2; ++eye) {
            OVR::Matrix4f proj = ovrMatrix4f_Projection(eyeRenderDesc[eye].Fov, 0.2f, 1000.0f, ovrProjection_RightHanded);

            glMatrixMode(GL_PROJECTION);
            pangolin::OpenGlMatrix(proj).Load();

            // Leave in MODEVIEW mode
            glMatrixMode(GL_MODELVIEW);
            pangolin::OpenGlMatrix(OVR::Matrix4f(OVR::Posef(EyeRenderPose[eye]).Inverted())).Load();

//            occ_cam.ApplyNView(eye);

            eye_fb[eye].Bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glTranslatef(0,0,-0.5);
            pangolin::glDrawColouredCube(-0.1f,0.1f);
            eye_fb[eye].Unbind();
        }


//        ovrPosef Pose_w_eye[2] = { (T_eh[0]*T_hw).Inverted(), (T_eh[1] * T_hw).Inverted()};
//        ovrHmd_EndFrame(HMD, Pose_w_eye, &eyeTex[0].Texture);

        ovrHmd_EndFrame(hmd, EyeRenderPose, &eyeTex[0].Texture);
        glFlush();
//        glFinish();
        ovrHmd_EndFrameTiming(hmd);

        win.ProcessEvents();
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
//    main_old();
    main_new();
    return 0;
}

