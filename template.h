/*
 * Copyright (c) 2026 the ThorVG project. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _TVG_DEMO_TEMPLATE_H_
#define _TVG_DEMO_TEMPLATE_H_

#include <memory>
#include <iostream>
#include <cstring>
#include <thorvg-1/thorvg.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>

#ifdef THORVG_WG_RASTER_SUPPORT
    #include <webgpu/webgpu.h>
    #if defined(SDL_VIDEO_DRIVER_COCOA)
        #include <Cocoa/Cocoa.h>
        #include <QuartzCore/CAMetalLayer.h>
    #endif
#endif

using namespace std;
using namespace tvg;

/************************************************************************/
/* Common Template Code                                                 */
/************************************************************************/

namespace tvgdemo
{

bool verify(tvg::Result result, string failMsg = "");

struct Demo
{
    uint32_t elapsed = 0;

    virtual bool content(tvg::Canvas* canvas, uint32_t w, uint32_t h) = 0;
    virtual bool update(tvg::Canvas* canvas, uint32_t elapsed) { return false; }
    virtual ~Demo() {}
};


struct Window
{
    SDL_Window* window = nullptr;

    tvg::Canvas* canvas = nullptr;
    uint32_t width;
    uint32_t height;

    Demo* demo = nullptr;

    bool needResize = false;
    bool needDraw = false;
    bool initialized = false;
    bool clearBuffer = false;

    Window(Demo* demo, uint32_t width, uint32_t height, uint32_t threadsCnt)
    {
        if (!verify(tvg::Initializer::init(threadsCnt), "Failed to init ThorVG engine!")) return;

        //Initialize the SDL
        SDL_Init(SDL_INIT_VIDEO);

        //Init member variables
        this->width = width;
        this->height = height;
        this->demo = demo;
        this->initialized = true;
    }

    virtual ~Window()
    {
        delete(demo);
        delete(canvas);

        //Terminate the SDL
        SDL_DestroyWindow(window);
        SDL_Quit();

        //Terminate ThorVG Engine
        tvg::Initializer::term();
    }

    bool draw()
    {
        //Draw the contents to the Canvas
        if (verify(canvas->draw(clearBuffer))) {
            verify(canvas->sync());
            return true;
        }

        return false;
    }

    bool ready()
    {
        if (!canvas) return false;

        if (!demo->content(canvas, width, height)) return false;

        //initiate the first rendering before window pop-up.
        if (!verify(canvas->draw())) return false;
        if (!verify(canvas->sync())) return false;

        return true;
    }

    void show()
    {
        SDL_ShowWindow(window);
        refresh();

        //Mainloop
        SDL_Event event;
        auto running = true;

        auto ptime = SDL_GetTicks();
        demo->elapsed = 0;
        uint32_t tickCnt = 0;

        while (running) {

            //SDL Event handling
            while (SDL_PollEvent(&event)) {
                switch (event.type) {
                    case SDL_QUIT: {
                        running = false;
                        break;
                    }
                    case SDL_KEYDOWN: {
                        if (event.key.keysym.sym == SDLK_ESCAPE) running = false;
                        break;
                    }
                    case SDL_WINDOWEVENT: {
                        if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                            width = event.window.data1;
                            height = event.window.data2;
                            needResize = true;
                            needDraw = true;
                        }
                    }
                }
            }

            if (needResize) {
                resize();
                needResize = false;
            }

            if (tickCnt > 0) {
                needDraw |= demo->update(canvas, demo->elapsed);
            }

            if (needDraw) {
                if (draw()) refresh();
                needDraw = false;
            }

            auto ctime = SDL_GetTicks();
            demo->elapsed += (ctime - ptime);
            tickCnt++;
            ptime = ctime;
        }
    }

    virtual void resize() {}
    virtual void refresh() {}
};


/************************************************************************/
/* SwCanvas Window Code                                                 */
/************************************************************************/

struct SwWindow : Window
{
    SwWindow(Demo* demo, uint32_t width, uint32_t height, uint32_t threadsCnt) : Window(demo, width, height, threadsCnt)
    {
        if (!initialized) return;

        window = SDL_CreateWindow("ThorVG Merge Path (Software)", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);

        //Create a Canvas
        canvas = tvg::SwCanvas::gen();
        if (!canvas) {
            cout << "SwCanvas is not supported. Did you enable the SwEngine?" << endl;
            return;
        }

        resize();
    }

    void resize() override
    {
        auto surface = SDL_GetWindowSurface(window);
        if (!surface) return;

        //Set the canvas target and draw on it.
        verify(static_cast<tvg::SwCanvas*>(canvas)->target((uint32_t*)surface->pixels, surface->pitch / 4, surface->w, surface->h, tvg::ColorSpace::ARGB8888));
    }

    void refresh() override
    {
        SDL_UpdateWindowSurface(window);
    }
};

/************************************************************************/
/* GlCanvas Window Code                                                 */
/************************************************************************/

struct GlWindow : Window
{
    SDL_GLContext context;

    GlWindow(Demo* demo, uint32_t width, uint32_t height, uint32_t threadsCnt) : Window(demo, width, height, threadsCnt)
    {
        if (!initialized) return;

#ifdef THORVG_GL_TARGET_GLES
        SDL_SetHint(SDL_HINT_OPENGL_ES_DRIVER, "1");
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
        window = SDL_CreateWindow("ThorVG Merge Path (OpenGL/ES)", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
        context = SDL_GL_CreateContext(window);

        //Create a Canvas
        canvas = tvg::GlCanvas::gen();
        if (!canvas) {
            cout << "GlCanvas is not supported. Did you enable the GlEngine?" << endl;
            return;
        }

        resize();
    }

    virtual ~GlWindow()
    {
        //Free in the reverse order of their creation.
        delete(canvas);
        canvas = nullptr;

        SDL_GL_DeleteContext(context);
    }

    void resize() override
    {
        //Set the canvas target and draw on it.
        verify(static_cast<tvg::GlCanvas*>(canvas)->target(nullptr, nullptr, context, 0, width, height, tvg::ColorSpace::ABGR8888S));
    }

    void refresh() override
    {
        SDL_GL_SwapWindow(window);
    }
};

/************************************************************************/
/* WgCanvas Window Code                                                 */
/************************************************************************/

#ifdef THORVG_WG_RASTER_SUPPORT

struct WgWindow : Window
{
    WGPUInstance instance;
    WGPUSurface surface;
    WGPUAdapter adapter;
    WGPUDevice device;

    WgWindow(Demo* demo, uint32_t width, uint32_t height, uint32_t threadsCnt) : Window(demo, width, height, threadsCnt)
    {
        if (!initialized) return;

        window = SDL_CreateWindow("ThorVG Merge Path (WebGPU)", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_HIDDEN);

        //Create WebGPU surface from the window
        SDL_SysWMinfo windowWMInfo;
        SDL_VERSION(&windowWMInfo.version);
        SDL_GetWindowWMInfo(window, &windowWMInfo);

        //Init WebGPU
        WGPUInstanceDescriptor desc = {.nextInChain = nullptr};
        instance = wgpuCreateInstance(&desc);

        #if defined(SDL_VIDEO_DRIVER_COCOA)
            [windowWMInfo.info.cocoa.window.contentView setWantsLayer:YES];
            auto layer = [CAMetalLayer layer];
            [windowWMInfo.info.cocoa.window.contentView setLayer:layer];

            WGPUSurfaceDescriptorFromMetalLayer surfaceNativeDesc = {
                .chain = {nullptr, WGPUSType_SurfaceDescriptorFromMetalLayer},
                .layer = layer
            };
        #elif defined(SDL_VIDEO_DRIVER_X11)
            WGPUSurfaceDescriptorFromXlibWindow surfaceNativeDesc = {
                .chain = {nullptr, WGPUSType_SurfaceDescriptorFromXlibWindow},
                .display = windowWMInfo.info.x11.display,
                .window = windowWMInfo.info.x11.window
            };
        #elif defined(SDL_VIDEO_DRIVER_WAYLAND)
            WGPUSurfaceDescriptorFromWaylandSurface surfaceNativeDesc = {
                .chain = {nullptr, WGPUSType_SurfaceDescriptorFromWaylandSurface},
                .display = windowWMInfo.info.wl.display,
                .surface = windowWMInfo.info.wl.surface
            };
        #elif defined(SDL_VIDEO_DRIVER_WINDOWS)
            WGPUSurfaceDescriptorFromWindowsHWND surfaceNativeDesc = {
                .chain = {nullptr, WGPUSType_SurfaceDescriptorFromWindowsHWND},
                .hinstance = GetModuleHandle(nullptr),
                .hwnd = windowWMInfo.info.win.window
            };
        #endif

        // create surface
        WGPUSurfaceDescriptor surfaceDesc{};
        surfaceDesc.nextInChain = (const WGPUChainedStruct*)&surfaceNativeDesc;
        surfaceDesc.label = "The surface";
        surface = wgpuInstanceCreateSurface(instance, &surfaceDesc);

        // request adapter
        const WGPURequestAdapterOptions requestAdapterOptions { .compatibleSurface = surface, .powerPreference = WGPUPowerPreference_HighPerformance };
        auto onAdapterRequestEnded = [](WGPURequestAdapterStatus status, WGPUAdapter adapter, char const * message, void * pUserData) { *((WGPUAdapter*)pUserData) = adapter; };
        wgpuInstanceRequestAdapter(instance, &requestAdapterOptions, onAdapterRequestEnded, &adapter);

        // get adapter and surface properties
        WGPUFeatureName featureNames[32]{};
        size_t featuresCount = wgpuAdapterEnumerateFeatures(this->adapter, featureNames);

        // request device
        const WGPUDeviceDescriptor deviceDesc { .label = "The owned device", .requiredFeatureCount = featuresCount, .requiredFeatures = featureNames };
        auto onDeviceRequestEnded = [](WGPURequestDeviceStatus status, WGPUDevice device, char const * message, void * pUserData) { *((WGPUDevice*)pUserData) = device; };
        wgpuAdapterRequestDevice(this->adapter, &deviceDesc, onDeviceRequestEnded, &device);

        //Create a Canvas
        canvas = tvg::WgCanvas::gen();
        if (!canvas) {
            cout << "WgCanvas is not supported. Did you enable the WgEngine?" << endl;
            return;
        }

        resize();
    }

    virtual ~WgWindow()
    {
        //Free in the reverse order of their creation.
        delete(canvas);
        canvas = nullptr;

        wgpuDeviceRelease(device);
        wgpuAdapterRelease(adapter);
        wgpuSurfaceRelease(surface);
        wgpuInstanceRelease(instance);
    }

    void resize() override
    {
        //Set the canvas target and draw on it.
        verify(static_cast<tvg::WgCanvas*>(canvas)->target(device, instance, surface, width, height, tvg::ColorSpace::ABGR8888S));
    }

    void refresh() override 
    {
        wgpuSurfacePresent(surface);
    }
};

#else
struct WgWindow : Window
{
    WgWindow(Demo* demo, uint32_t width, uint32_t height, uint32_t threadsCnt) : Window(demo, width, height, threadsCnt)
    {
        cout << "webgpu driver is not detected!" << endl;
    }
};

#endif


float progress(uint32_t elapsed, float durationInSec, bool rewind = false)
{
    auto duration = uint32_t(durationInSec * 1000.0f); //sec -> millisec.
    if (elapsed == 0 || duration == 0) return 0.0f;
    auto forward = ((elapsed / duration) % 2 == 0) ? true : false;
    if (elapsed % duration == 0) return forward ? 0.0f : 1.0f;
    auto progress = (float(elapsed % duration) / (float)duration);
    if (rewind) return forward ? progress : (1 - progress);
    return progress;
}


bool verify(tvg::Result result, string failMsg)
{
    switch (result) {
        case tvg::Result::FailedAllocation: {
            cout << "FailedAllocation! " << failMsg << endl;
            return false;
        }
        case tvg::Result::InsufficientCondition: {
            cout << "InsufficientCondition! " << failMsg << endl;
            return false;
        }
        case tvg::Result::InvalidArguments: {
            cout << "InvalidArguments! " << failMsg << endl;
            return false;
        }
        case tvg::Result::MemoryCorruption: {
            cout << "MemoryCorruption! " << failMsg << endl;
            return false;
        }
        case tvg::Result::NonSupport: {
            cout << "NonSupport! " << failMsg << endl;
            return false;
        }
        case tvg::Result::Unknown: {
            cout << "Unknown! " << failMsg << endl;
            return false;
        }
        default: break;
    };
    return true;
}


int main(Demo* demo, int argc, char **argv, bool clearBuffer = false, uint32_t width = 800, uint32_t height = 800, uint32_t threadsCnt = 4)
{
    auto engine = 0; //0: sw, 1: gl, 2: wg

    if (argc > 1) {
        if (!strcmp(argv[1], "gl")) engine = 1;
        if (!strcmp(argv[1], "wg")) engine = 2;
    }

    unique_ptr<Window> window;

    if (engine == 0) {
        window = unique_ptr<Window>(new SwWindow(demo, width, height, threadsCnt));
    } else if (engine == 1) {
        window = unique_ptr<Window>(new GlWindow(demo, width, height, threadsCnt));
    } else if (engine == 2) {
        window = unique_ptr<Window>(new WgWindow(demo, width, height, threadsCnt));
    }

    window->clearBuffer = clearBuffer;

    if (window->ready()) {
        window->show();
    }

    return 0;
}

};

#endif //_TVG_DEMO_TEMPLATE_H_
