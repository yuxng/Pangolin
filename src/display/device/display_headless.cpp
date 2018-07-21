#include <pangolin/display/display_internal.h>
#include <pangolin/factory/factory_registry.h>
#include <EGL/egl.h>

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
        EGL_RENDERABLE_TYPE , EGL_OPENGL_BIT,
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

    std::unique_ptr<EGLDisplayHL> display;
};

EGLDisplayHL::EGLDisplayHL(const int width, const int height) {
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

    std::vector<EGLConfig> egl_configs(count);

    EGLint numConfigs;
    eglChooseConfig(egl_display, attribs, egl_configs.data(), count, &numConfigs);

    egl_context = eglCreateContext(egl_display, egl_configs[0], EGL_NO_CONTEXT, nullptr);

    const EGLint pbufferAttribs[] = {
        EGL_WIDTH, width,
        EGL_HEIGHT, height,
        EGL_NONE,
    };
    egl_surface = eglCreatePbufferSurface(egl_display, egl_configs[0],  pbufferAttribs);
    if (egl_surface == EGL_NO_SURFACE) {
        std::cerr << "Cannot create EGL surface" << std::endl;
    }
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
    eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
//    eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, egl_context);
}

HeadlessWindow::HeadlessWindow(const int w, const int h) {
    windowed_size[0] = w;
    windowed_size[1] = h;
    display = std::unique_ptr<EGLDisplayHL>(new EGLDisplayHL(w, h));
}

HeadlessWindow::~HeadlessWindow() { }

void HeadlessWindow::MakeCurrent() {
    display->makeCurrent();
    context = this;
}

void HeadlessWindow::ToggleFullscreen() { }

void HeadlessWindow::Move(const int /*x*/, const int /*y*/) { }

void HeadlessWindow::Resize(const unsigned int /*w*/, const unsigned int /*h*/) { }

void HeadlessWindow::ProcessEvents() { }

void HeadlessWindow::SwapBuffers() {
    display->swap();
    MakeCurrent();
}

std::unique_ptr<WindowInterface> CreateHeadlessWindowAndBind(const int w, const int h) {
    return std::unique_ptr<WindowInterface>(new HeadlessWindow(w, h));
}

} // namespace headless

PANGOLIN_REGISTER_FACTORY(NoneWindow) {
struct HeadlessWindowFactory : public FactoryInterface<WindowInterface> {
    std::unique_ptr<WindowInterface> Open(const Uri& uri) override {
        const int w = uri.Get<int>("w", 640);
        const int h = uri.Get<int>("h", 480);
        return std::unique_ptr<WindowInterface>(headless::CreateHeadlessWindowAndBind(w, h));
    }

    virtual ~HeadlessWindowFactory() { }
};

auto factory = std::make_shared<HeadlessWindowFactory>();
FactoryRegistry<WindowInterface>::I().RegisterFactory(factory, 1, "headless");
}

} // namespace pangolin

