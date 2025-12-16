/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "mujoco_sim_interface/MujocoRenderer.h"

#include <GLFW/glfw3.h>  // for creating the OpenGL context
#include <mujoco/mujoco.h>

#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <regex>
#include <string>
#include <thread>

#include "mujoco_sim_interface/MujocoSimInterface.h"

namespace robot::mujoco_sim_interface {

/// GLFW callbacks

// keyboard callback
void MujocoRenderer::keyboard(GLFWwindow* window, int key, int, int act, int mods) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));

  // 'c' key: toggle contact point visualization
  if (act == GLFW_PRESS && key == GLFW_KEY_C) {
    renderer->mujocoOptions_.flags[mjVIS_CONTACTPOINT] = !renderer->mujocoOptions_.flags[mjVIS_CONTACTPOINT];
  }

  // 'f' key: toggle contact force visualization
  if (act == GLFW_PRESS && key == GLFW_KEY_F) {
    renderer->mujocoOptions_.flags[mjVIS_CONTACTFORCE] = !renderer->mujocoOptions_.flags[mjVIS_CONTACTFORCE];
  }

  // 'm' key: toggle centre of mass visualization
  if (act == GLFW_PRESS && key == GLFW_KEY_M) {
    renderer->mujocoOptions_.flags[mjVIS_COM] = !renderer->mujocoOptions_.flags[mjVIS_COM];
  }

  // 't' key: toggle model transparency
  if (act == GLFW_PRESS && key == GLFW_KEY_T) {
    renderer->model_transparent = !renderer->model_transparent;
    if (renderer->model_transparent) {
      renderer->setTransparency(0.3f);
    } else {
      renderer->setTransparency(1.0f);
    }
  }

  // 'i' key: toggle inertia visualization
  if (act == GLFW_PRESS && key == GLFW_KEY_I) {
    renderer->mujocoOptions_.flags[mjVIS_INERTIA] = !renderer->mujocoOptions_.flags[mjVIS_INERTIA];
  }

  // 'h' key: toggle hull visualization
  if (act == GLFW_PRESS && key == GLFW_KEY_H) {
    renderer->mujocoOptions_.flags[mjVIS_CONVEXHULL] = !renderer->mujocoOptions_.flags[mjVIS_CONVEXHULL];
  }

  // 'p' key: print hotkeys
  if (act == GLFW_PRESS && key == GLFW_KEY_P) {
    std::cerr << "\n\n==========================================================="
              << "\nHotkeys\n===========================================================\n"
              << "c => toggle contact point visualization\n"
              << "f => toggle contact force visualization\n"
              << "m => toggle center of mass visualization\n"
              << "t => toggle model transparency\n"
              << "i => toggle interia visualization\n"
              << "h => toggle hull visualization\n";
  }
}

// mouse button callback
void MujocoRenderer::mouse_button(GLFWwindow* window, int, int, int) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  // update button state
  renderer->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  renderer->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  renderer->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &(renderer->lastx), &(renderer->lasty));
}

// mouse move callback
void MujocoRenderer::mouse_move(GLFWwindow* window, double xpos, double ypos) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  // no buttons down: nothing to do
  if (!renderer->button_left && !renderer->button_middle && !renderer->button_right) return;

  // compute mouse displacement, save
  double dx = xpos - renderer->lastx;
  double dy = ypos - renderer->lasty;
  renderer->lastx = xpos;
  renderer->lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (renderer->button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (renderer->button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  // move camera
  mjv_moveCamera(renderer->simInterface_->getModel(), action, dx / width, dy / height, &renderer->mujocoScene_, &renderer->mujocoCam_);
}

// scroll callback
void MujocoRenderer::scroll(GLFWwindow* window, double, double yoffset) {
  auto* renderer = static_cast<MujocoRenderer*>(glfwGetWindowUserPointer(window));
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(renderer->simInterface_->getModel(), mjMOUSE_ZOOM, 0, -0.05 * yoffset, &renderer->mujocoScene_, &renderer->mujocoCam_);
}

//// Public

MujocoRenderer::MujocoRenderer(const MujocoSimInterface* simInterface)
    : simInterface_(simInterface),
      simState_(simInterface_->getModel()),
      timeStepMicro_(1e6 / simInterface_->getConfig().renderFrequencyHz) {
  mujocoScene_.flags[mjRND_SHADOW] = 1;
  mujocoScene_.flags[mjRND_REFLECTION] = 1;
}

MujocoRenderer::~MujocoRenderer() {
  std::cerr << "Cleaning up renderer ..." << std::endl;
  ;
  if (render_thread_.joinable()) {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
    render_thread_.join();
  }
}

bool MujocoRenderer::ok() const {
  return !window_closed_.load(std::memory_order_acquire);
}

void MujocoRenderer::launchRenderThread() {
  render_thread_ = std::thread(&MujocoRenderer::renderLoop, this);
}

void MujocoRenderer::waitForInit() const {
  while (!init_complete_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MujocoRenderer::setTransparency(float transparency) const {
  for (int i = 0; i < simInterface_->getModel()->ngeom; i++) {
    simInterface_->getModel()->geom_rgba[4 * i + 3] = transparency;
  }
}

namespace {
void renderMetrics(const mjrContext* con, const mjrRect& viewport, const MjState& state, double fpsRender, double elapsed_time) {
  std::ostringstream metrics;

  // FPS (Simulation & Renderer)
  metrics << "Render FPS: " << static_cast<int>(fpsRender) << "\n";
  metrics << "Sim FPS: " << static_cast<int>(state.metrics.fpsSim) << "\n";

  // The actual amount of time elapsed in simulation.
  metrics << "Real Time[s]: " << std::fixed << std::setprecision(3) << elapsed_time << "\n";
  metrics << "Sim  Time[s]: " << std::fixed << std::setprecision(3) << state.data->time << "\n\n";

  // Real-time tracking
  metrics << "RTF: " << std::fixed << std::setprecision(3) << state.metrics.rtfTick << "\n";
  metrics << "Drift[ms]: " << std::fixed << std::setprecision(3) << state.metrics.driftTick * 1e3 << "\n";
  metrics << "Cummulative Drift[ms]: " << std::fixed << std::setprecision(3) << state.metrics.driftCumulative * 1e3;

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, metrics.str().c_str(), nullptr, con);
}
}  // namespace

void MujocoRenderer::renderExternalForces() {
  auto* model = simInterface_->getModel();
  auto* data = simState_.data;

  // Safety check
  if (!model || !data) {
    return;
  }

  for (int body_id = 0; body_id < model->nbody; ++body_id) {
    const double* force = &data->xfrc_applied[6 * body_id];
    double fx = force[0];
    double fy = force[1];
    double fz = force[2];

    double magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (magnitude < 1e-6) {
      continue;
    }

    const double* xpos = &data->xpos[3 * body_id];

    // Create arrow geom
    mjvGeom* arrow = nullptr;
    if (mujocoScene_.ngeom < mujocoScene_.maxgeom) {
      arrow = &mujocoScene_.geoms[mujocoScene_.ngeom];
      mujocoScene_.ngeom++;
    } else {
      continue;  // Skip if we're out of geom space
    }

    // Clear the geom
    std::memset(arrow, 0, sizeof(mjvGeom));

    // Scale factor that grows with force magnitude
    // Adjust these constants to tune the visualization
    const double base_scale = 0.1;    // Minimum arrow length
    const double force_scale = 0.05;  // How much to scale with force
    const double scale = base_scale + force_scale * magnitude;

    // Calculate arrow end point using normalized force direction and scale
    double end_x = xpos[0] + scale * (fx / magnitude);
    double end_y = xpos[1] + scale * (fy / magnitude);
    double end_z = xpos[2] + scale * (fz / magnitude);

    mjtNum from[3] = {xpos[0], xpos[1], xpos[2]};
    mjtNum to[3] = {end_x, end_y, end_z};

    mjv_connector(arrow,         // geom to write to
                  mjGEOM_ARROW,  // type (arrow)
                  0.005,         // width (thin arrows)
                  from,          // from position
                  to             // to position
    );

    // Set color
    arrow->rgba[0] = 0.5f;
    arrow->rgba[1] = 1.0f;
    arrow->rgba[2] = 0.0f;
    arrow->rgba[3] = 1.0f;

    // Set additional properties
    arrow->category = mjCAT_DECOR;
    arrow->emission = 1.0f;  // Makes it glow/bright
  }
}

void MujocoRenderer::renderLoop() {
  initialize();
  init_complete_.store(true);

  const auto start_time = std::chrono::steady_clock::now();

  while (!glfwWindowShouldClose(window_)) {
    auto start = std::chrono::steady_clock::now();

    glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

    // Copy physics data to render data
    simInterface_->copyMjState(simState_);
    mj_forward(simInterface_->getModel(), simState_.data);

    mjv_updateScene(simInterface_->getModel(), simState_.data, &mujocoOptions_, nullptr, nullptr, mjCAT_ALL, &mujocoScene_);

    renderExternalForces();

    // render to glfw window
    mjv_updateCamera(simInterface_->getModel(), simState_.data, &mujocoCam_, &mujocoScene_);
    mjr_render(viewport_, &mujocoScene_, &mujocoContext_);

    // render text overlay
    const auto current_time = std::chrono::steady_clock::now();
    const auto elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
    renderMetrics(&mujocoContext_, viewport_, simState_, rendererFps_.fps(), elapsed_time);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    // Sleep in case render loop is faster than specified sim rate.
    std::this_thread::sleep_until(start + std::chrono::microseconds(timeStepMicro_));

    rendererFps_.tick();
  }

  std::cerr << "Exited Mujoco renderLoop." << std::endl;

  window_closed_.store(true);
  cleanup();
}

void MujocoRenderer::initialize() {
  // init GLFW
  if (!glfwInit()) mju_error("Could not initialize GLFW");

  // create window, make OpenGL context current, request v-sync
  window_ = glfwCreateWindow(viewportWidth, viewportHeight, "Mujoco Robot Sim", nullptr, nullptr);
  glfwMakeContextCurrent(window_);

  // init glew
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return;
  }

  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&mujocoCam_);
  mjv_defaultOption(&mujocoOptions_);
  mjv_defaultScene(&mujocoScene_);
  mjr_defaultContext(&mujocoContext_);
  mjv_makeScene(simInterface_->getModel(), &mujocoScene_, 2000);                 // space for 2000 objects
  mjr_makeContext(simInterface_->getModel(), &mujocoContext_, mjFONTSCALE_150);  // model-specific context

  // Set mujoco option
  mujocoOptions_.flags[mjVIS_CONTACTPOINT] = 0;
  mujocoOptions_.flags[mjVIS_CONTACTFORCE] = 0;
  mujocoOptions_.flags[mjVIS_COM] = 0;
  mujocoOptions_.flags[mjVIS_INERTIA] = 0;

  glfwSetWindowUserPointer(window_, this);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, keyboard);
  glfwSetCursorPosCallback(window_, mouse_move);
  glfwSetMouseButtonCallback(window_, mouse_button);
  glfwSetScrollCallback(window_, scroll);

  // Setup Camera
  double arr_view[] = {89.608063, -5.588379, 3, 0.000000, 0.000000, 0.500000};  // view the left side (for ll, lh, left_side)
  mujocoCam_.azimuth = arr_view[0];
  mujocoCam_.elevation = arr_view[1];
  mujocoCam_.distance = arr_view[2];
  mujocoCam_.lookat[0] = arr_view[3];
  mujocoCam_.lookat[1] = arr_view[4];
  mujocoCam_.lookat[2] = arr_view[5];

  simInterface_->copyMjState(simState_);

  // get framebuffer viewport
  // We don't want to step the actual simulation here, as it screws up the initialization of the IMU's
  mj_step(simInterface_->getModel(), simState_.data);  // populate state info
  glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

  mjv_updateScene(simInterface_->getModel(), simState_.data, &mujocoOptions_, nullptr, &mujocoCam_, mjCAT_ALL, &mujocoScene_);

  mjr_render(viewport_, &mujocoScene_, &mujocoContext_);
  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window_);
  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoRenderer::cleanup() {
  // Cleanup mujoco stuff.
  mjv_freeScene(&mujocoScene_);
  mjr_freeContext(&mujocoContext_);

  glfwDestroyWindow(window_);
  glfwTerminate();
}

}  // namespace robot::mujoco_sim_interface
