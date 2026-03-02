#pragma once
#include <GL/glew.h>
#include <vector>

// ── GlobeRenderer ─────────────────────────────────────────────────────────
// Renders an interactive 3D globe with vector map lines.
// Uses OpenGL 3.3 Core Profile, hand-coded 4×4 column-major math (no GLM).
//
// Usage:
//   GlobeRenderer globe;
//   globe.init();
//   // per frame:
//   glEnable(GL_DEPTH_TEST);
//   globe.set_viewport(fw, fh);
//   globe.render();
//   glDisable(GL_DEPTH_TEST);
//   // mouse:
//   globe.on_drag_begin(mx, my);
//   globe.on_drag(mx, my);
//   globe.on_scroll(delta);
//   // hit test:
//   float lat, lon;
//   if (globe.pick(mx, my, lat, lon)) { ... }
//   // project to screen:
//   float sx, sy;
//   if (globe.project(lat, lon, sx, sy)) { ... }

class GlobeRenderer {
public:
    bool init();
    void destroy();
    void set_viewport(int w, int h);
    void render();

    // Arcball mouse interaction
    void on_drag_begin(float mx, float my);
    void on_drag(float mx, float my);
    void on_scroll(float delta);

    // Ray-sphere intersection → geographic coordinates; false if miss
    bool pick(float mx, float my, float& lat_deg, float& lon_deg) const;

    // Project lat/lon → screen pixels; false if behind camera
    bool project(float lat_deg, float lon_deg, float& sx, float& sy) const;

private:
    // OpenGL objects
    GLuint prog_sphere_ = 0;
    GLuint prog_lines_  = 0;
    GLuint prog_land_   = 0;
    GLuint vao_sphere_  = 0;
    GLuint vbo_sphere_  = 0;
    GLuint ebo_sphere_  = 0;
    GLuint vao_lines_   = 0;
    GLuint vbo_lines_   = 0;
    GLuint vao_land_    = 0;
    GLuint vbo_land_    = 0;
    GLuint tex_earth_   = 0;   // Blue Marble texture
    GLint  idx_count_   = 0;
    GLint  land_vtx_count_ = 0;

    // Map line segment boundaries for glMultiDrawArrays
    std::vector<GLint>   seg_starts_;
    std::vector<GLsizei> seg_counts_;

    // Camera state
    float qw_ = 1.f, qx_ = 0.f, qy_ = 0.f, qz_ = 0.f;
    float zoom_ = 3.5f;
    float yaw_rad_   = 0.f;   // accumulated Y-axis rotation
    float pitch_deg_ = 0.f;   // accumulated pitch, clamped ±30°
    int   vp_w_ = 1920, vp_h_ = 1080;

    // Drag
    float drag_ax_ = 0.f, drag_ay_ = 0.f, drag_az_ = 0.f;

    // ── Math helpers (column-major, matching OpenGL convention) ──────────
    void mat4_identity(float* M) const;
    void mat4_mul(float* C, const float* A, const float* B) const;
    void mat4_perspective(float* M, float fovy_rad, float aspect,
                          float near_z, float far_z) const;
    void mat4_translate(float* M, float tx, float ty, float tz) const;
    void mat4_from_quat(float* M, float w, float x, float y, float z) const;
    void quat_mul(float& rw, float& rx, float& ry, float& rz,
                  float aw, float ax, float ay, float az,
                  float bw, float bx, float by, float bz) const;
    void quat_normalize(float& w, float& x, float& y, float& z) const;
    void get_mvp(float* mvp) const;
    void get_view_inv(float* inv) const;
    bool screen_to_arcball(float mx, float my,
                           float& ax, float& ay, float& az) const;

    // ── GL setup ─────────────────────────────────────────────────────────
    void   build_sphere(int stacks, int slices);
    void   build_map_lines();
    void   build_land();
    bool   load_earth_texture();
    GLuint compile_shader(const char* vsrc, const char* fsrc);
};
