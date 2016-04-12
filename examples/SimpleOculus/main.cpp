#include <pangolin/pangolin.h>
#include <pangolin/compat/bind.h>

#include <OVR.h>
#include <OVR_CAPI_GL.h>

struct PangoOculus
{
    PangoOculus()
    {
    }
};

int main(int /*argc*/, char ** /*argv*/)
{
    ovr_Initialize();
    ovrHmd HMD = ovrHmd_Create(0);

    if(!HMD) {
        throw std::runtime_error("No Oculus headset available.");
    }

    pangolin::WindowInterface& win = pangolin::CreateWindowAndBind("Oculus", HMD->Resolution.h, HMD->Resolution.w);
    glEnable(GL_DEPTH_TEST);

    ovrGLConfig config;
    config.OGL.Header.API			   = ovrRenderAPI_OpenGL;
    config.OGL.Header.BackBufferSize.w = HMD->Resolution.h;
    config.OGL.Header.BackBufferSize.h = HMD->Resolution.w;
    config.OGL.Header.Multisample      = 0;
//    config.OGL.Disp                    = display;

    ovrEyeRenderDesc EyeRenderDesc[2];
    ovrHmd_ConfigureRendering( HMD, &config.Config,
                               ovrDistortionCap_Vignette | ovrDistortionCap_TimeWarp |
                               ovrDistortionCap_Overdrive, HMD->DefaultEyeFov, EyeRenderDesc );

    ovrHmd_SetEnabledCaps(HMD, ovrHmdCap_LowPersistence|ovrHmdCap_DynamicPrediction);
//    ovrHmd_AttachToWindow(HMD, (void*)win, 0, 0);

    ovrHmd_ConfigureTracking(HMD, ovrTrackingCap_Orientation|ovrTrackingCap_MagYawCorrection|
                                  ovrTrackingCap_Position, 0);

    pangolin::GlFramebuffer eye_fb[2];
    pangolin::GlRenderBuffer eye_rb[2];
    pangolin::GlTexture eye_tex[2];
    ovrGLTexture eyeTex[2];

    pangolin::OpenGlRenderState occ_cam;
    OVR::Posef T_eh[2];

    for(size_t i=0; i<2; ++i)
    {
        const ovrSizei eye_tex_size = ovrHmd_GetFovTextureSize(HMD, (ovrEyeType)i, HMD->DefaultEyeFov[i], 1);
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

        T_eh[i].Translation = EyeRenderDesc[i].HmdToEyeViewOffset;
        occ_cam.GetViewOffset(i) = pangolin::OpenGlMatrix( OVR::Matrix4f( T_eh[i] ) );

        occ_cam.GetProjectionMatrix(i) = pangolin::OpenGlMatrix(ovrMatrix4f_Projection(
            EyeRenderDesc[i].Fov, 0.1f, 10.0f, ovrProjection_ClipRangeOpenGL | ovrProjection_RightHanded
        ));

//        occ_cam.GetProjectionMatrix(i) = pangolin::ProjectionMatrix(
//            eye_tex_size[i].w, eye_tex_size[i].h,
//            focalLength, focalLength,
//            eye_tex_size[i].w/2.0 -(i*2-1)*lensXOffset, eye_tex_size[i].h/2.0,
//            0.2, 100
//            );
    }

    glViewport(0, 0, eye_tex[0].width, eye_tex[0].height);

    ovrHmd_DismissHSWDisplay(HMD);

    OVR::Posef T_hw;

    while(!pangolin::ShouldQuit()) {
        ovrTrackingState tracking_state = ovrHmd_GetTrackingState(HMD, 0.0);
        OVR::Pose<float> p(tracking_state.HeadPose.ThePose);
        T_hw = OVR::Posef(tracking_state.HeadPose.ThePose).Inverted();
        occ_cam.SetModelViewMatrix( pangolin::OpenGlMatrix(OVR::Matrix4f(T_hw)) );

        // Render each eye
        ovrHmd_BeginFrame(HMD, 0);
        for(int i=0; i<2; ++i) {
            occ_cam.ApplyNView(i);

            eye_fb[i].Bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glTranslatef(0,0,-0.5);
            pangolin::glDrawColouredCube(-0.1f,0.1f);
            eye_fb[i].Unbind();
        }


        ovrPosef Pose_w_eye[2] = { (T_eh[0]*T_hw).Inverted(), (T_eh[1] * T_hw).Inverted()};
        ovrHmd_EndFrame(HMD, Pose_w_eye, &eyeTex[0].Texture);
        win.ProcessEvents();
    }

    return 0;
}
