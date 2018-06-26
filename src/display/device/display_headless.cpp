#include <pangolin/platform.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/factory/factory_registry.h>
#include <pangolin/gl/colour.h>
#include <pangolin/gl/gldraw.h>

#include <EGL/egl.h>

namespace pangolin {

extern __thread PangolinGl* context;

namespace headless {

struct EGLDisplayHL {
    EGLDisplayHL(const int width, const int height);

    ~EGLDisplayHL();

    std::vector<EGLConfig> egl_configs;
    EGLSurface egl_surface;
    EGLContext egl_context;
    EGLDisplay egl_display;

    EGLint width, height;
    static constexpr EGLint attribs[] = {
        EGL_RENDERABLE_TYPE , EGL_OPENGL_BIT,
        EGL_RED_SIZE        , 8,
        EGL_GREEN_SIZE      , 8,
        EGL_BLUE_SIZE       , 8,
        EGL_ALPHA_SIZE      , 8,
        EGL_DEPTH_SIZE      , 24,
        EGL_STENCIL_SIZE    , 8,
        EGL_NONE
    };

    static constexpr EGLint pbufferAttribs[] = {
            EGL_WIDTH, 9,
            EGL_HEIGHT, 9,
            EGL_NONE,
      };
};

constexpr EGLint EGLDisplayHL::attribs[];
constexpr EGLint EGLDisplayHL::pbufferAttribs[];

struct HeadlessWindow : public PangolinGl
{
    HeadlessWindow(const int width, const int height, std::unique_ptr<EGLDisplayHL> display);

    ~HeadlessWindow() override;

    void ToggleFullscreen() override;

    void Move(const int x, const int y) override;

    void Resize(const unsigned int w, const unsigned int h) override;

    void MakeCurrent() override;

    void SwapBuffers() override;

    void ProcessEvents() override;

    std::unique_ptr<EGLDisplayHL> display;
};

EGLDisplayHL::EGLDisplayHL(const int width, const int height) : width(width), height(height) {
//    wdisplay = wl_display_connect(nullptr);
//    if (wdisplay == nullptr) { return; }

//    wregistry = wl_display_get_registry(wdisplay);
//    wl_registry_add_listener(wregistry, &wregistry_listener, this);

//    wl_display_roundtrip(wdisplay);

//    egl_display = eglGetDisplay(wdisplay);
    egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if(!egl_display) {
        std::cerr << "Failed to open EGL display" << std::endl;
    }

    EGLint major, minor;
    if(eglInitialize(egl_display, &major, &minor)==EGL_FALSE) {
        std::cerr << "EGL init failed" << std::endl;
    }

    if(eglBindAPI(EGL_OPENGL_API)==EGL_FALSE) {
        std::cerr << "EGL bind failed" << std::endl;
    }

    EGLint count;
    eglGetConfigs(egl_display, nullptr, 0, &count);

    egl_configs.resize(count);

    EGLint numConfigs;
    eglChooseConfig(egl_display, attribs, egl_configs.data(), count, &numConfigs);

    egl_context = eglCreateContext(egl_display, egl_configs[0], EGL_NO_CONTEXT, nullptr);

//    egl_surface = eglCreateWindowSurface(egl_display, egl_configs[0], (EGLNativeWindowType)egl_window, nullptr);
    egl_surface = eglCreatePbufferSurface(egl_display, egl_configs[0],  pbufferAttribs);
    if (egl_surface == EGL_NO_SURFACE) {
        std::cerr << "Cannot create EGL surface" << std::endl;
    }
    std::cout << "EGL error: " << std::hex << eglGetError() << std::endl;

//    pangolin::Colour colour = pangolin::Colour::Red();
//    eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
//    glClearColor(colour.r, colour.g, colour.b, colour.a);
//    glClear(GL_COLOR_BUFFER_BIT);
//    eglSwapBuffers(egl_display, egl_surface);
}

EGLDisplayHL::~EGLDisplayHL() {
    // cleanup EGL
    if(egl_context) eglDestroyContext(egl_display, egl_context);
    if(egl_surface) eglDestroySurface(egl_display, egl_surface);
//    if(egl_window)  wl_egl_window_destroy(egl_window);
    if(egl_display) eglTerminate(egl_display);
}

HeadlessWindow::HeadlessWindow(const int w, const int h, std::unique_ptr<EGLDisplayHL> display) : display(std::move(display)){
    windowed_size[0] = w;
    windowed_size[1] = h;
}

HeadlessWindow::~HeadlessWindow() { }

void HeadlessWindow::MakeCurrent() {
    eglMakeCurrent(display->egl_display, display->egl_surface, display->egl_surface, display->egl_context);
    context = this;
}

void HeadlessWindow::ToggleFullscreen() { }

void HeadlessWindow::Move(const int /*x*/, const int /*y*/) { }

void HeadlessWindow::Resize(const unsigned int /*w*/, const unsigned int /*h*/) { }

void HeadlessWindow::ProcessEvents() { }

void HeadlessWindow::SwapBuffers() {
    eglSwapBuffers(display->egl_display, display->egl_surface);
    MakeCurrent();
}


std::unique_ptr<WindowInterface> CreateHeadlessWindowAndBind(const int w, const int h) {

    std::unique_ptr<EGLDisplayHL> newdisplay = std::unique_ptr<EGLDisplayHL>(new EGLDisplayHL(w, h));

    HeadlessWindow* win = new HeadlessWindow(w, h, std::move(newdisplay));

    return std::unique_ptr<WindowInterface>(win);
}

} // namespace headless

PANGOLIN_REGISTER_FACTORY(NoneWindow)
{
  struct HeadlessWindowFactory : public FactoryInterface<WindowInterface> {
    std::unique_ptr<WindowInterface> Open(const Uri& uri) override {
      const int w = uri.Get<int>("w", 640);
      const int h = uri.Get<int>("h", 480);
      std::cout << "create HEADLESS window" << std::endl;
      return std::unique_ptr<WindowInterface>(headless::CreateHeadlessWindowAndBind(w, h));
    }

    virtual ~HeadlessWindowFactory() { }
  };

    auto factory = std::make_shared<HeadlessWindowFactory>();
    FactoryRegistry<WindowInterface>::I().RegisterFactory(factory, 1, "headless");
}

} // namespace pangolin

