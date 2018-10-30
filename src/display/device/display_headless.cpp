#include <pangolin/display/display_internal.h>
#include <pangolin/factory/factory_registry.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>

namespace pangolin {

extern __thread PangolinGl* context;

namespace headless {

class EGLDisplayHL {
public:
    EGLDisplayHL(const int width, const int height);

    ~EGLDisplayHL();

    void swap();

    void makeCurrent();

private:
    EGLSurface egl_surface;
    EGLContext egl_context;
    EGLDisplay egl_display;

    static constexpr EGLint attribs[] = {
        EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
        EGL_RED_SIZE        , 8,
        EGL_GREEN_SIZE      , 8,
        EGL_BLUE_SIZE       , 8,
        EGL_ALPHA_SIZE      , 8,
        EGL_DEPTH_SIZE      , 8,
        EGL_STENCIL_SIZE    , 8,
        EGL_NONE
    };
};

constexpr EGLint EGLDisplayHL::attribs[];

struct HeadlessWindow : public PangolinGl {
    HeadlessWindow(const int width, const int height);

    ~HeadlessWindow() override;

    void ToggleFullscreen() override;

    void Move(const int x, const int y) override;

    void Resize(const unsigned int w, const unsigned int h) override;

    void MakeCurrent() override;

    void SwapBuffers() override;

    void ProcessEvents() override;

    EGLDisplayHL display;
};

EGLDisplayHL::EGLDisplayHL(const int width, const int height) {
    egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if(egl_display == EGL_NO_DISPLAY) {
        std::cerr << "Failed to open EGL display" << std::endl;
    }

    EGLint major, minor;
    if(eglInitialize(egl_display, &major, &minor)==EGL_FALSE) {
        std::cerr << "EGL init failed" << std::endl;
    }
    std::cout << "EGL version: " << major << "." << minor << std::endl;

    char const * client_apis = eglQueryString(egl_display, EGL_CLIENT_APIS);
    if(!client_apis)
    {
        std::cerr << "Failed to eglQueryString(display, EGL_CLIENT_APIS)" << std::endl;
    }
    std::cout << "Supported client rendering APIs: " << client_apis << std::endl;

    EGLConfig config;
    EGLint num_config;
    if(eglChooseConfig(egl_display, attribs, &config, 1, &num_config) == EGL_FALSE)
    {
        std::cerr << "Failed to eglChooseConfig" << std::endl;
    }
    if(num_config < 1)
    {
        std::cerr << "No matching EGL frame buffer configuration" << std::endl;
    }

    if(eglBindAPI(EGL_OPENGL_API)==EGL_FALSE) {
        std::cerr << "EGL bind failed" << std::endl;
    }

    egl_context = eglCreateContext(egl_display, config, EGL_NO_CONTEXT, NULL);
    if(egl_context == EGL_NO_CONTEXT)
    {
        std::cerr << "Failed to eglCreateContext" << std::endl;
        EGLint error =  eglGetError();
        switch(error)
        {
        case EGL_BAD_CONFIG:
            std::cerr << "config is not an EGL frame buffer configuration, or does not support the current rendering API" << std::endl;
            break;
        case EGL_BAD_ATTRIBUTE:
            std::cerr << "attrib_list contains an invalid context attribute or if an attribute is not recognized or out of range" << std::endl;
            break;
        default:
            std::cerr << "Unknown error " << error << std::endl;
            break;
        }
    }

    if(eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, egl_context) == EGL_FALSE)
    {
        std::cerr << "Failed to eglMakeCurrent" << std::endl;
    }

    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        std::cerr << "Failed to glewInit\n";
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }

    // TODO: this should not be required
    // missing: "#define GLEW_EGL" before including glew.h
    glGenRenderbuffersEXT = (PFNGLGENFRAMEBUFFERSEXTPROC)eglGetProcAddress("glGenFramebuffersEXT");
    glBindRenderbufferEXT = (PFNGLBINDFRAMEBUFFEREXTPROC)eglGetProcAddress("glBindRenderbufferEXT");
    glRenderbufferStorageEXT = (PFNGLRENDERBUFFERSTORAGEEXTPROC)eglGetProcAddress("glRenderbufferStorageEXT");
    glGenFramebuffersEXT = (PFNGLGENFRAMEBUFFERSEXTPROC)eglGetProcAddress("glGenFramebuffersEXT");
    glBindFramebufferEXT = (PFNGLBINDFRAMEBUFFEREXTPROC)eglGetProcAddress("glBindFramebufferEXT");
    glFramebufferTexture2DEXT = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)eglGetProcAddress("glFramebufferTexture2DEXT");
    glFramebufferRenderbufferEXT = (PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC)eglGetProcAddress("glFramebufferRenderbufferEXT");
    glDrawBuffers = (PFNGLDRAWBUFFERSPROC)eglGetProcAddress("glDrawBuffers");
    glDeleteFramebuffersEXT = (PFNGLDELETEFRAMEBUFFERSPROC)eglGetProcAddress("glDeleteFramebuffersEXT");
}

EGLDisplayHL::~EGLDisplayHL() {
    if(egl_context) eglDestroyContext(egl_display, egl_context);
    if(egl_surface) eglDestroySurface(egl_display, egl_surface);
    if(egl_display) eglTerminate(egl_display);
}

void EGLDisplayHL::swap() {
    eglSwapBuffers(egl_display, egl_surface);
}

void EGLDisplayHL::makeCurrent() {
//    eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
    eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, egl_context);
}

HeadlessWindow::HeadlessWindow(const int w, const int h) : display(w, h) {
    windowed_size[0] = w;
    windowed_size[1] = h;
}

HeadlessWindow::~HeadlessWindow() { }

void HeadlessWindow::MakeCurrent() {
    display.makeCurrent();
    context = this;
}

void HeadlessWindow::ToggleFullscreen() { }

void HeadlessWindow::Move(const int /*x*/, const int /*y*/) { }

void HeadlessWindow::Resize(const unsigned int /*w*/, const unsigned int /*h*/) { }

void HeadlessWindow::ProcessEvents() { }

void HeadlessWindow::SwapBuffers() {
    display.swap();
    MakeCurrent();
}

} // namespace headless

PANGOLIN_REGISTER_FACTORY(NoneWindow) {
struct HeadlessWindowFactory : public FactoryInterface<WindowInterface> {
    std::unique_ptr<WindowInterface> Open(const Uri& uri) override {
        return std::unique_ptr<WindowInterface>(new headless::HeadlessWindow(uri.Get<int>("w", 640), uri.Get<int>("h", 480)));
    }

    virtual ~HeadlessWindowFactory() { }
};

auto factory = std::make_shared<HeadlessWindowFactory>();
FactoryRegistry<WindowInterface>::I().RegisterFactory(factory, 1, "headless");
}

} // namespace pangolin

