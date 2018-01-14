#pragma once

#include <cinder/app/AppNative.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/params/Params.h>

#include <CinderOpenCV.h>

namespace ttrk {

  /**
  * @class SubWindow
  * @brief Sub window regions to draw to in the main viewer.
  * Allows the main window to be split up into different viewports.
  */
  class SubWindow {


  public:

    /**
    * Empty constructor.
    */
    SubWindow() : can_save_(false) { }

    /**
    * Create a window with dimensions.
    * @param[in] name The name of this sub window.
    * @param[in] start_x The x coordinate of the top left of the box in the main window reference frame.
    * @param[in] start_y The y coordinate of the top left of the box in the main window reference frame.
    * @param[in] eye_width The width of the content in the sub window in pixels.
    * @param[in] eye_height The height of the content in the sub window in pixels.
    * @param[in] draw_width The width that the content in the sub window should be drawn in pixels.
    * @param[in] draw_height The height that the content in the sub window should be drawn in pixels.
    * @param[in] can_save If the sub window should be able to dump its contents to disk.
    */
    void Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, int draw_width, int draw_height, bool can_save);
    
    /**
    * Create a window with dimensions. Provides a cleaner interface when the draw and eye size is the same.
    * @param[in] name The name of this sub window.
    * @param[in] start_x The x coordinate of the top left of the box in the main window reference frame.
    * @param[in] start_y The y coordinate of the top left of the box in the main window reference frame.
    * @param[in] eye_width The width of the content in the sub window in pixels.
    * @param[in] eye_height The height of the content in the sub window in pixels.
    * @param[in] can_save If the sub window should be able to dump its contents to disk.
    */
    void Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, bool can_save);

    /**
    * Bind the framebuffer that belongs to this window.
    */
    void Bind();

    /**
    * Bind and clear to black the framebuffer that belongs to this window.
    */
    void BindAndClear();

    /**
    * Unbind the framebuffer that belongs to this window.
    */
    void UnBind();

    /**
    * Query if the window is setup for saving its contents.
    * @return true if this window supports dumping content to file. false otherwise.
    */
    bool CanSave() const;

    /**
    * Query if the window currently saving its contents.
    * @return true if this window is currently saving its content to file. false otherwise.
    */
    bool IsSaving() const;

    /**
    * Write the current contents to file. Assumes the capture has been initialized with InitSavingWindow().
    */
    void WriteFrameToFile();

    /**
    * Draw a GUI window onto the internal texture.
    * @param[in] params A cinder/AntTweakBar UI element. This is drawn to fill up the entire subwindow. If this is not desired behaviour use the specialisation Draw(ci::params::InterfaceGlRef, cv::Vec2i, ci::Vec2i).
    */
    void Draw(ci::params::InterfaceGlRef params);

    /**
    * Draw a GUI window onto the internal texture at a specific location.
    * @param[in] params A cinder/AntTweakBar UI element.
    * @param[in] tl The top left of the UI element from the top left of the main window (not the subwindow).
    */
    void Draw(ci::params::InterfaceGlRef params, ci::Vec2i tl, ci::Vec2i size);

    /**
    * Draw the contents of the framebuffer in the subwindow with a small margin/border.
    */
    void Draw();

    /**
    * Initialise the window for saving. This process opens a file handle to an avi file using the OpenCV VideoWriter interface.
    */
    void InitSavingWindow();

    /**
    * Close the currently open video file (if applicable).
    */
    void CloseStream();

    /**
    * Accessor for the subwindow coordinates in the reference frame of the main window.
    * @return The window coordinates.
    */
    ci::Rectf GetRect() const { return window_coords_; }
    
    /**
    * Accessor for the subwindow coordinates in the reference frame of the main window with a small buffer of 0.95 x window size
    * @return The window coordinates with margin.
    */
    ci::Rectf GetRectWithBuffer() const { ci::Rectf r = window_coords_; r.scaleCentered(0.95); return r; }
    
    size_t BufferWidth() const { return framebuffer_->getWidth(); }
    size_t BufferHeight() const { return framebuffer_->getHeight(); }

    /**
    * Accessor for the subwindow width.
    * @return The window width.
    */
    size_t Width() const { return window_coords_.getWidth(); }
    
    /**
    * Accessor for the subwindow height.
    * @return The window height.
    */
    size_t Height() const { return window_coords_.getHeight(); }

    ci::gl::Texture GetContents() { return framebuffer_->getTexture(); }
    ci::gl::Texture GetDepth() { return framebuffer_->getDepthTexture(); }

    static std::string output_directory; /**< The output directory where the subwindows all dump their content. */

    protected:

    ci::Rectf window_coords_; /**< The window coordinates within the main window reference frame. */
    boost::shared_ptr<ci::gl::Fbo> framebuffer_; /**< The framebuffer of size width, height which is rendered to when we draw to this subwindow. */

    std::string name_; /**< The window name, must be unique. */
    cv::VideoWriter writer_; /**< The video writer. */

    bool can_save_; /**< If the window is capable of saving its contents. */
    ci::params::InterfaceGlRef save_params_; /**< Small UI element to switch on an off saving. */

  };




}