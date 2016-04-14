#include <pangolin/pangolin.h>

#include "pango_oculus.h"

void main_new()
{
    pangolin::PangoOculus oculus;

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
//        boostd::this_thread::sleep_for(boostd::chrono::milliseconds(60));

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        pangolin::glDrawColouredCube();

        // Submit frame for Oculus;
        if(render_stuff) {
            pangolin::StereoFrameBufferAndTargets* frame = oculus.GetBufferToRender();
            if(frame) {
//                std::cout << frame->frame_index << std::endl;
                for(unsigned int eye=0; eye < pangolin::PangoOculus::num_eyes; ++eye) {
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

