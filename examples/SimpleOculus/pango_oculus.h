#ifndef PANGOLIN_PANGO_OCULUS_H
#define PANGOLIN_PANGO_OCULUS_H

#include <pangolin/pangolin.h>
#include <pangolin/compat/bind.h>
#include <pangolin/compat/thread.h>
#include <pangolin/compat/mutex.h>

#include <OVR.h>
#include <OVR_CAPI_GL.h>

#include <pthread.h>
#include <pangolin/display/device/X11Window.h>

namespace pangolin
{

struct FrameBufferAndTargets
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

struct StereoFrameBufferAndTargets
{
    static const unsigned int num_eyes = 2;

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
    FrameBufferAndTargets buffer[num_eyes];
    ovrGLTexture eyeTex[num_eyes];
    ovrPosef Pose_w_eye[num_eyes];
};

class PangoOculus
{
public:
    static const unsigned int num_eyes = 2;
    static const unsigned int num_buffers = 3;


    PangoOculus(const char* window_title = "HMDWindow")
        : should_run(false), rendering(false), hmd(0), window_title(window_title), render_id(0)
    {
        occthread = boostd::thread(&PangoOculus::Run, boostd::ref(*this));
    }

    ~PangoOculus()
    {
        should_run = false;
        rendering = false;
        occthread.join();
    }

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

        // RdfVision.inverse() is the identity matrix
        pangolin::OpenGlMatrix rdfOpenGL; pangolin::SetZero<4,4>(rdfOpenGL.m);
        rdfOpenGL(0,0) =  1.0f; rdfOpenGL(1,1) = -1.0f; rdfOpenGL(2,2) = -1.0f; rdfOpenGL(3,3) =  1.0f;
        const pangolin::OpenGlMatrix T_gl_vis = rdfOpenGL /** RdfVision.inverse()*/;

        // 7cm back, 8.5 down
        const pangolin::OpenGlMatrix T_headset_camera = pangolin::OpenGlMatrix::Translate(0, -0.07, 0.085) * pangolin::OpenGlMatrix::RotateX(-M_PI*15.0/180.0);

        for(unsigned int b=0; b < num_buffers; ++b) {
            buffers[b].Reinitialise(eye_tex_size.w, eye_tex_size.h);

            for(unsigned int eye=0; eye<num_eyes; ++eye) {
                buffers[b].occ_cam.GetViewOffset(eye) = T_gl_vis * pangolin::OpenGlMatrix( OVR::Matrix4f( T_eh[eye] ) ) * T_headset_camera;

                buffers[b].occ_cam.GetProjectionMatrix(eye) = pangolin::OpenGlMatrix(ovrMatrix4f_Projection(
                    EyeRenderDesc[eye].Fov, 0.1f, 10.0f, ovrProjection_ClipRangeOpenGL | ovrProjection_RightHanded
                ));

//              float focalLength = 600.0;
//              float lensXOffset = 0.0; //focalLength * 0.06;
//              buffers[b].occ_cam.GetProjectionMatrix(eye) = pangolin::ProjectionMatrixRDF_TopLeft(
//                  eye_tex_size.w, eye_tex_size.h,
//                  focalLength, focalLength,
//                  eye_tex_size.w/2.0 -(eye*2-1)*lensXOffset, eye_tex_size.h/2.0,
//                  0.2, 100
//              );
            }
        }

        rendering = true;
        ovrHmd_DismissHSWDisplay(hmd);

        while(should_run) {
//            TICK("HUD_LOOP");
            StereoFrameBufferAndTargets* render_frame = &buffers[render_id];
            ovrHmd_BeginFrame(hmd, render_frame->frame_index);
            ovrHmd_EndFrame(hmd, render_frame->Pose_w_eye, &render_frame->eyeTex[0].Texture);
            win.ProcessEvents();
//            TOCK("HUD_LOOP");
        }
        rendering = false;

        ovrHmd_Destroy(hmd);
        hmd = 0;

        pangolin::DestroyWindow(window_title);
    }

    StereoFrameBufferAndTargets* GetBufferToRender()
    {
        static double last_frame_time_s = 0.0;
        static unsigned int frame_index = 0;

        if(rendering) {
            StereoFrameBufferAndTargets* next_buffer = &buffers[ (render_id+1)%num_buffers];

            next_buffer->frame_index = frame_index++;
            ovrFrameTiming frameTiming = ovrHmd_GetFrameTiming(hmd, next_buffer->frame_index);


            ovrTrackingState tracking_state = ovrHmd_GetTrackingState(hmd, last_frame_time_s );
            last_frame_time_s = frameTiming.ScanoutMidpointSeconds;
            OVR::Posef T_hw = OVR::Posef(tracking_state.HeadPose.ThePose).Inverted();
            ovrPosef T_we[2] = { (T_eh[0]*T_hw).Inverted(), (T_eh[1] * T_hw).Inverted()};
            next_buffer->SetPose(T_hw, T_we);
            return next_buffer;
        }else{
            return 0;
        }
    }

    void PutBufferToRender(StereoFrameBufferAndTargets* buffer)
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
    StereoFrameBufferAndTargets buffers[num_buffers];

    boostd::mutex flip_mutex;
};

}

#endif // PANGOLIN_PANGO_OCULUS_H
