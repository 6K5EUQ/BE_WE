#include "globe.hpp"
#include "world_map_data.hpp"
#include "bewe_paths.hpp"
#include <GL/glew.h>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <vector>
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

// ── Shader sources ────────────────────────────────────────────────────────

static const char* GLOBE_VERT = R"GLSL(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aUV;
uniform mat4 uMVP;
out vec2 vUV;
out vec3 vNorm;
void main(){
    vNorm = normalize(aPos);
    vUV   = aUV;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)GLSL";

static const char* GLOBE_FRAG = R"GLSL(
#version 330 core
in vec2 vUV;
in vec3 vNorm;
out vec4 FragColor;
uniform sampler2D uEarthTex;
uniform bool uHasTex;
void main(){
    vec4 base;
    if(uHasTex){
        vec4 tex = texture(uEarthTex, vUV);
        float rim = 1.0 - max(dot(normalize(vNorm), vec3(0.0,0.0,1.0)), 0.0);
        rim = pow(rim, 3.0) * 0.5;
        base = vec4(tex.rgb * (1.0 - rim * 0.6), 1.0);
    } else {
        base = vec4(0.04, 0.10, 0.28, 1.0);
    }

    FragColor = base;
}
)GLSL";

static const char* LAND_VERT = R"GLSL(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){
    gl_Position = uMVP * vec4(aPos * 1.001, 1.0);
}
)GLSL";

static const char* LAND_FRAG = R"GLSL(
#version 330 core
out vec4 FragColor;
uniform vec3 uLightDir;
uniform vec3 aPos_world;
void main(){
    // Flat green land
    FragColor = vec4(0.13, 0.30, 0.13, 1.0);
}
)GLSL";

static const char* LINES_VERT = R"GLSL(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){
    // Slight outward offset to avoid z-fighting with sphere surface
    gl_Position = uMVP * vec4(aPos * 1.002, 1.0);
}
)GLSL";

static const char* LINES_FRAG = R"GLSL(
#version 330 core
out vec4 FragColor;
void main(){
    FragColor = vec4(0.35, 0.48, 0.65, 1.0);
}
)GLSL";

// ── Static helpers ────────────────────────────────────────────────────────

static void latlon_to_xyz(float lat_deg, float lon_deg,
                           float& x, float& y, float& z) {
    float lat = lat_deg * (float)M_PI / 180.f;
    float lon = -lon_deg * (float)M_PI / 180.f; // negate to match sphere UV winding
    x = cosf(lat) * cosf(lon);
    y = sinf(lat);
    z = cosf(lat) * sinf(lon);
}

// ── GlobeRenderer ─────────────────────────────────────────────────────────

bool GlobeRenderer::init() {
    prog_sphere_ = compile_shader(GLOBE_VERT, GLOBE_FRAG);
    prog_lines_  = compile_shader(LINES_VERT, LINES_FRAG);
    prog_land_   = compile_shader(LAND_VERT,  LAND_FRAG);
    if (!prog_sphere_ || !prog_lines_ || !prog_land_) return false;
    build_sphere(30, 60);
    build_land();
    build_map_lines();
    load_earth_texture();
    // Default orientation: face lon=127 (Korea), equator at screen center
    yaw_rad_   = -127.f * (float)M_PI / 180.f;
    pitch_deg_ = 0.f;
    float qyw = cosf(yaw_rad_ * 0.5f), qyy = sinf(yaw_rad_ * 0.5f);
    qw_ = qyw; qx_ = 0.f; qy_ = qyy; qz_ = 0.f;
    return true;
}

void GlobeRenderer::destroy() {
    if (vao_sphere_) { glDeleteVertexArrays(1, &vao_sphere_); vao_sphere_=0; }
    if (vbo_sphere_) { glDeleteBuffers(1, &vbo_sphere_); vbo_sphere_=0; }
    if (ebo_sphere_) { glDeleteBuffers(1, &ebo_sphere_); ebo_sphere_=0; }
    if (vao_lines_)  { glDeleteVertexArrays(1, &vao_lines_); vao_lines_=0; }
    if (vbo_lines_)  { glDeleteBuffers(1, &vbo_lines_); vbo_lines_=0; }
    if (vao_land_)   { glDeleteVertexArrays(1, &vao_land_); vao_land_=0; }
    if (vbo_land_)   { glDeleteBuffers(1, &vbo_land_); vbo_land_=0; }
    if (tex_earth_)  { glDeleteTextures(1, &tex_earth_); tex_earth_=0; }
    if (prog_sphere_){ glDeleteProgram(prog_sphere_); prog_sphere_=0; }
    if (prog_lines_) { glDeleteProgram(prog_lines_); prog_lines_=0; }
    if (prog_land_)  { glDeleteProgram(prog_land_); prog_land_=0; }
    seg_starts_.clear(); seg_counts_.clear();
}

void GlobeRenderer::set_viewport(int w, int h) {
    vp_w_ = w; vp_h_ = h;
}

void GlobeRenderer::render() {
    float mvp[16];
    get_mvp(mvp);

    // 1. Draw globe sphere (with texture if loaded, else flat color)
    glUseProgram(prog_sphere_);
    GLint u = glGetUniformLocation(prog_sphere_, "uMVP");
    glUniformMatrix4fv(u, 1, GL_FALSE, mvp);
    if (tex_earth_) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex_earth_);
        glUniform1i(glGetUniformLocation(prog_sphere_, "uEarthTex"), 0);
        glUniform1i(glGetUniformLocation(prog_sphere_, "uHasTex"), 1);
    } else {
        glUniform1i(glGetUniformLocation(prog_sphere_, "uHasTex"), 0);
    }

    glBindVertexArray(vao_sphere_);
    glDrawElements(GL_TRIANGLES, idx_count_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    if (tex_earth_) glBindTexture(GL_TEXTURE_2D, 0);

    // 2. Land polygons (only drawn when no texture)
    if (!tex_earth_) {
        glUseProgram(prog_land_);
        u = glGetUniformLocation(prog_land_, "uMVP");
        glUniformMatrix4fv(u, 1, GL_FALSE, mvp);
        glBindVertexArray(vao_land_);
        glDrawArrays(GL_TRIANGLES, 0, land_vtx_count_);
        glBindVertexArray(0);
    }

    // 3. Draw map lines only when no texture (texture already shows coastlines)
    if (!tex_earth_) {
        glUseProgram(prog_lines_);
        u = glGetUniformLocation(prog_lines_, "uMVP");
        glUniformMatrix4fv(u, 1, GL_FALSE, mvp);

        glBindVertexArray(vao_lines_);
        if (!seg_starts_.empty())
            glMultiDrawArrays(GL_LINE_STRIP,
                              seg_starts_.data(),
                              seg_counts_.data(),
                              (GLsizei)seg_starts_.size());
        glBindVertexArray(0);
    }

    glUseProgram(0);
}

// ── Mouse interaction ─────────────────────────────────────────────────────

bool GlobeRenderer::screen_to_arcball(float mx, float my,
                                       float& ax, float& ay, float& az) const {
    // unused — kept for signature compatibility
    ax = mx; ay = my; az = 0.f;
    return true;
}

void GlobeRenderer::on_drag_begin(float mx, float my) {
    drag_ax_ = mx;
    drag_ay_ = my;
    drag_az_ = 0.f; // 0=undecided, 1=horizontal, 2=vertical
}

void GlobeRenderer::on_drag(float mx, float my) {
    float ddx = mx - drag_ax_;
    float ddy = my - drag_ay_;
    drag_ax_ = mx;
    drag_ay_ = my;

    if (fabsf(ddx) < 0.5f && fabsf(ddy) < 0.5f) return;

    // Lock axis on first significant movement
    if (drag_az_ == 0.f) {
        drag_az_ = (fabsf(ddx) >= fabsf(ddy)) ? 1.f : 2.f;
    }

    // Only apply the locked axis
    if (drag_az_ == 1.f) ddy = 0.f;
    else                  ddx = 0.f;

    float nw, nx, ny, nz;

    if (fabsf(ddx) > 0.f) {
        // Horizontal → rotate around world Y axis
        float angle = -ddx * 0.002f;
        float dw = cosf(angle * 0.5f), dy = sinf(angle * 0.5f);
        // pre-multiply: delta * current  (world-space axis)
        quat_mul(nw,nx,ny,nz, dw,0,dy,0, qw_,qx_,qy_,qz_);
        quat_normalize(nw,nx,ny,nz);
        qw_=nw; qx_=nx; qy_=ny; qz_=nz;
        yaw_rad_ += angle;
    }

    if (fabsf(ddy) > 0.f) {
        // Vertical → rotate around camera's right axis in world space.
        // Camera right = col0 of the rotation matrix = what the globe's local X
        // maps to in world space under the current quaternion.
        // view_rot = transpose(rot), so camera right in world = rot col0 = (M[0],M[1],M[2])
        // from mat4_from_quat: col0 = (1-2(y²+z²), 2(xy+wz), 2(xz-wy))
        float ax =  1.f - 2.f*(qy_*qy_ + qz_*qz_);
        float ay =  2.f*(qx_*qy_ + qw_*qz_);
        float az =  2.f*(qx_*qz_ - qw_*qy_);

        float angle = -ddy * 0.002f;
        pitch_deg_ += angle * (180.f / (float)M_PI);
        if (pitch_deg_ >  90.f) { angle -= (pitch_deg_ - 90.f) * (float)M_PI / 180.f; pitch_deg_ =  90.f; }
        if (pitch_deg_ < -90.f) { angle -= (pitch_deg_ + 90.f) * (float)M_PI / 180.f; pitch_deg_ = -90.f; }
        if (fabsf(angle) < 1e-6f) return;

        float s = sinf(angle * 0.5f), dw = cosf(angle * 0.5f);
        // pre-multiply: world-space axis rotates the globe body from outside
        quat_mul(nw,nx,ny,nz, dw, s*ax, s*ay, s*az, qw_,qx_,qy_,qz_);
        quat_normalize(nw,nx,ny,nz);
        qw_=nw; qx_=nx; qy_=ny; qz_=nz;
    }
}

void GlobeRenderer::on_scroll(float delta) {
    zoom_ -= delta * 0.3f;
    if (zoom_ < 1.5f) zoom_ = 1.5f;
    if (zoom_ > 8.f)  zoom_ = 8.f;
}

// ── Picking ───────────────────────────────────────────────────────────────

bool GlobeRenderer::pick(float mx, float my,
                          float& lat_deg, float& lon_deg) const {
    float inv[16];
    get_view_inv(inv);

    float fovy = 45.f * (float)M_PI / 180.f;
    float tan_half = tanf(fovy * 0.5f);
    float aspect = (float)vp_w_ / (float)vp_h_;

    // NDC coords
    float nx = (2.f * mx / vp_w_ - 1.f);
    float ny = (1.f - 2.f * my / vp_h_);

    // Ray direction in view space (pointing into -Z)
    float vx = nx * tan_half * aspect;
    float vy = ny * tan_half;
    float vz = -1.f;

    // Transform to world space using view inverse (3×3 rotation part)
    // view_inv columns 0-2 are the world-space basis vectors
    float wx = inv[0]*vx + inv[4]*vy + inv[8]*vz;
    float wy = inv[1]*vx + inv[5]*vy + inv[9]*vz;
    float wz = inv[2]*vx + inv[6]*vy + inv[10]*vz;

    // Normalize ray direction
    float wlen = sqrtf(wx*wx + wy*wy + wz*wz);
    if (wlen < 1e-8f) return false;
    wx/=wlen; wy/=wlen; wz/=wlen;

    // Ray origin = camera world position (column 3 of view_inv)
    float ox = inv[12], oy = inv[13], oz = inv[14];

    // Ray-sphere intersection with unit sphere at origin
    // |o + t*d|² = 1  →  t² + 2(o·d)t + (o·o - 1) = 0
    float b = 2.f * (ox*wx + oy*wy + oz*wz);
    float c = ox*ox + oy*oy + oz*oz - 1.f;
    float disc = b*b - 4.f*c;
    if (disc < 0.f) return false;

    float t = (-b - sqrtf(disc)) * 0.5f;
    if (t < 0.f) t = (-b + sqrtf(disc)) * 0.5f;
    if (t < 0.f) return false;

    float hx = ox + t*wx;
    float hy = oy + t*wy;
    float hz = oz + t*wz;

    lat_deg = asinf(hy < -1.f ? -1.f : (hy > 1.f ? 1.f : hy)) * 180.f / (float)M_PI;
    lon_deg = -atan2f(hz, hx) * 180.f / (float)M_PI; // negate to match latlon_to_xyz convention
    return true;
}

bool GlobeRenderer::project(float lat_deg, float lon_deg,
                             float& sx, float& sy) const {
    float x, y, z;
    latlon_to_xyz(lat_deg, lon_deg, x, y, z);

    float mvp[16];
    get_mvp(mvp);

    // Transform to clip space
    float cx = mvp[0]*x + mvp[4]*y + mvp[8]*z  + mvp[12];
    float cy = mvp[1]*x + mvp[5]*y + mvp[9]*z  + mvp[13];
    // float cz = mvp[2]*x + mvp[6]*y + mvp[10]*z + mvp[14];
    float cw = mvp[3]*x + mvp[7]*y + mvp[11]*z + mvp[15];

    if (cw <= 0.f) return false; // behind camera

    float ndcx = cx / cw;
    float ndcy = cy / cw;

    sx = (ndcx + 1.f) * 0.5f * vp_w_;
    sy = (1.f - ndcy) * 0.5f * vp_h_;
    return true;
}

// ── Math implementation ───────────────────────────────────────────────────

void GlobeRenderer::mat4_identity(float* M) const {
    memset(M, 0, 16*sizeof(float));
    M[0]=M[5]=M[10]=M[15]=1.f;
}

void GlobeRenderer::mat4_mul(float* C, const float* A, const float* B) const {
    float T[16] = {};
    for (int col=0; col<4; col++)
        for (int row=0; row<4; row++)
            for (int k=0; k<4; k++)
                T[col*4+row] += A[k*4+row] * B[col*4+k];
    memcpy(C, T, 64);
}

void GlobeRenderer::mat4_perspective(float* M, float fovy, float aspect,
                                      float near_z, float far_z) const {
    memset(M, 0, 64);
    float t = 1.f / tanf(fovy * 0.5f);
    M[0]  = t / aspect;
    M[5]  = t;
    M[10] = -(far_z + near_z) / (far_z - near_z);
    M[11] = -1.f;          // col=2, row=3 (column-major: index = col*4+row)
    M[14] = -(2.f * far_z * near_z) / (far_z - near_z);
}

void GlobeRenderer::mat4_translate(float* M, float tx, float ty, float tz) const {
    mat4_identity(M);
    M[12] = tx; M[13] = ty; M[14] = tz;
}

void GlobeRenderer::mat4_from_quat(float* M,
                                    float w, float x, float y, float z) const {
    mat4_identity(M);
    float x2=x*x, y2=y*y, z2=z*z;
    float xy=x*y, xz=x*z, yz=y*z, wx=w*x, wy=w*y, wz=w*z;
    // Column-major: M[col*4+row]
    M[0]=1-2*(y2+z2); M[1]=2*(xy+wz);    M[2]=2*(xz-wy);
    M[4]=2*(xy-wz);   M[5]=1-2*(x2+z2); M[6]=2*(yz+wx);
    M[8]=2*(xz+wy);   M[9]=2*(yz-wx);   M[10]=1-2*(x2+y2);
}

void GlobeRenderer::quat_mul(float& rw, float& rx, float& ry, float& rz,
                              float aw, float ax, float ay, float az,
                              float bw, float bx, float by, float bz) const {
    rw = aw*bw - ax*bx - ay*by - az*bz;
    rx = aw*bx + ax*bw + ay*bz - az*by;
    ry = aw*by - ax*bz + ay*bw + az*bx;
    rz = aw*bz + ax*by - ay*bx + az*bw;
}

void GlobeRenderer::quat_normalize(float& w, float& x, float& y, float& z) const {
    float len = sqrtf(w*w + x*x + y*y + z*z);
    if (len < 1e-8f) { w=1.f; x=y=z=0.f; return; }
    w/=len; x/=len; y/=len; z/=len;
}

void GlobeRenderer::get_mvp(float* mvp) const {
    float proj[16], view_rot[16], rot[16], trans[16], view[16];

    float fovy = 45.f * (float)M_PI / 180.f;
    float aspect = (float)vp_w_ / (float)vp_h_;
    mat4_perspective(proj, fovy, aspect, 0.1f, 100.f);

    // Model rotation from quaternion
    mat4_from_quat(rot, qw_, qx_, qy_, qz_);

    // View rotation = transpose of model rotation
    // (camera orbits around globe at origin)
    mat4_identity(view_rot);
    for (int r=0; r<3; r++)
        for (int c=0; c<3; c++)
            view_rot[c*4+r] = rot[r*4+c];

    // Camera at (0,0,zoom_) → translate view by (0,0,-zoom_)
    mat4_translate(trans, 0.f, 0.f, -zoom_);

    // view = trans * view_rot
    mat4_mul(view, trans, view_rot);

    // mvp = proj * view
    mat4_mul(mvp, proj, view);
}

void GlobeRenderer::get_view_inv(float* inv) const {
    // View = trans(-zoom_) * view_rot_transpose
    // view_inv = view_rot * trans(zoom_) = rot * trans(zoom_)
    // Camera world position = rot * (0, 0, zoom_)^T = col2 of rot * zoom_
    float rot[16];
    mat4_from_quat(rot, qw_, qx_, qy_, qz_);
    memcpy(inv, rot, 64);
    inv[12] = rot[8]  * zoom_;  // col=3, row=0
    inv[13] = rot[9]  * zoom_;  // col=3, row=1
    inv[14] = rot[10] * zoom_;  // col=3, row=2
    inv[15] = 1.f;
}

// ── GL setup ──────────────────────────────────────────────────────────────

GLuint GlobeRenderer::compile_shader(const char* vsrc, const char* fsrc) {
    auto compile = [](GLenum type, const char* src) -> GLuint {
        GLuint sh = glCreateShader(type);
        glShaderSource(sh, 1, &src, nullptr);
        glCompileShader(sh);
        GLint ok; glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512]; glGetShaderInfoLog(sh, sizeof(log), nullptr, log);
            printf("[Globe] shader compile error: %s\n", log);
        }
        return sh;
    };
    GLuint vs = compile(GL_VERTEX_SHADER,   vsrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fsrc);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    GLint ok; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
        printf("[Globe] shader link error: %s\n", log);
        prog = 0;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

void GlobeRenderer::build_sphere(int stacks, int slices) {
    // Vertex: vec3 pos + vec2 uv = 5 floats per vertex
    std::vector<float> verts;
    std::vector<GLuint> indices;

    for (int st = 0; st <= stacks; st++) {
        float phi = (float)M_PI * ((float)st / stacks - 0.5f); // -pi/2 to pi/2
        float y   = sinf(phi);
        float r   = cosf(phi);
        float v   = (float)st / stacks;
        for (int sl = 0; sl <= slices; sl++) {
            float theta = 2.f * (float)M_PI * (float)sl / slices;
            float x = r * cosf(theta);
            float z = r * sinf(theta);
            float u = 1.f - (float)sl / slices; // mirror U to match lon-negated geometry
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
            verts.push_back(u); verts.push_back(v);
        }
    }

    for (int st = 0; st < stacks; st++) {
        for (int sl = 0; sl < slices; sl++) {
            GLuint a = (GLuint)(st * (slices+1) + sl);
            GLuint b = a + (slices+1);
            indices.push_back(a);   indices.push_back(b);   indices.push_back(a+1);
            indices.push_back(a+1); indices.push_back(b);   indices.push_back(b+1);
        }
    }

    idx_count_ = (GLint)indices.size();

    glGenVertexArrays(1, &vao_sphere_);
    glGenBuffers(1, &vbo_sphere_);
    glGenBuffers(1, &ebo_sphere_);

    glBindVertexArray(vao_sphere_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sphere_);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(verts.size() * sizeof(float)),
                 verts.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_sphere_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)(indices.size() * sizeof(GLuint)),
                 indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          5*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                          5*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

void GlobeRenderer::build_map_lines() {
    std::vector<float> verts;

    GLint current_start = 0;
    int   current_count = 0;
    bool  in_segment    = false;

    for (int i = 0; i + 1 < WORLD_MAP_DATA_COUNT; i += 2) {
        float lat = WORLD_MAP_DATA[i];
        float lon = WORLD_MAP_DATA[i+1];

        if (std::isnan(lat) || std::isnan(lon)) {
            // End of segment
            if (in_segment && current_count > 0) {
                seg_starts_.push_back(current_start);
                seg_counts_.push_back((GLsizei)current_count);
            }
            current_start = (GLint)(verts.size() / 3);
            current_count = 0;
            in_segment = false;
        } else {
            float x, y, z;
            latlon_to_xyz(lat, lon, x, y, z);
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
            current_count++;
            in_segment = true;
        }
    }
    // Flush final segment
    if (in_segment && current_count > 0) {
        seg_starts_.push_back(current_start);
        seg_counts_.push_back((GLsizei)current_count);
    }

    glGenVertexArrays(1, &vao_lines_);
    glGenBuffers(1, &vbo_lines_);

    glBindVertexArray(vao_lines_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_lines_);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(verts.size() * sizeof(float)),
                 verts.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          3*sizeof(float), (void*)0);
    glBindVertexArray(0);

    printf("[Globe] map lines: %zu segments, %zu vertices\n",
           seg_starts_.size(), verts.size()/3);
}

void GlobeRenderer::build_land() {
    // LAND_TRI_DATA: flat array of triangles, 9 floats each (v0.xyz, v1.xyz, v2.xyz)
    // Already on unit sphere surface from the Python generator.
    glGenVertexArrays(1, &vao_land_);
    glGenBuffers(1, &vbo_land_);

    glBindVertexArray(vao_land_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_land_);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(LAND_TRI_COUNT * sizeof(float)),
                 LAND_TRI_DATA, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          3*sizeof(float), (void*)0);
    glBindVertexArray(0);

    land_vtx_count_ = LAND_TRI_COUNT / 3; // 3 floats per vertex
    printf("[Globe] land triangles: %d vertices\n", land_vtx_count_);
}

bool GlobeRenderer::load_earth_texture() {
    std::string path = BEWEPaths::assets_dir() + "/earth.jpg";
    stbi_set_flip_vertically_on_load(1); // OpenGL UV: v=0 at bottom
    int w, h, ch;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &ch, 3);
    if (!data) {
        printf("[Globe] earth texture not found: %s\n", path.c_str());
        return false;
    }
    glGenTextures(1, &tex_earth_);
    glBindTexture(GL_TEXTURE_2D, tex_earth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // Anisotropic filtering if available
    float maxAniso = 1.f;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY, &maxAniso);
    if (maxAniso > 1.f)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, maxAniso);
    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);
    printf("[Globe] earth texture loaded: %dx%d\n", w, h);
    return true;
}
