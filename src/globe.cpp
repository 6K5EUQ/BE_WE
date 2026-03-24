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
    vec3 N = normalize(vNorm);
    vec3 V = vec3(0.0, 0.0, 1.0);

    if(uHasTex){
        vec3 tex = texture(uEarthTex, vUV).rgb;

        // Atmosphere rim glow (works on both front and back faces)
        float rim = 1.0 - abs(dot(N, V));
        float atmo = pow(rim, 4.0) * 0.4;
        vec3 col = tex + vec3(0.3, 0.5, 1.0) * atmo;

        FragColor = vec4(col, 1.0);
    } else {
        FragColor = vec4(0.04, 0.10, 0.28, 1.0);
    }
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

static const char* STARS_VERT = R"GLSL(
#version 330 core
layout(location=0) in vec2 aPos;   // NDC x,y directly
layout(location=1) in float aBrightness;
out float vBright;
void main(){
    vBright = aBrightness;
    gl_Position = vec4(aPos, 0.999, 1.0);
    gl_PointSize = 1.5 + aBrightness * 2.5;
}
)GLSL";

static const char* STARS_FRAG = R"GLSL(
#version 330 core
in float vBright;
out vec4 FragColor;
void main(){
    vec2 c = gl_PointCoord - 0.5;
    float d = dot(c, c) * 4.0;
    if(d > 1.0) discard;
    float alpha = (1.0 - d * 0.8) * vBright;
    vec3 col = mix(vec3(1.0, 0.88, 0.75), vec3(0.88, 0.95, 1.0), vBright);
    FragColor = vec4(col, alpha);
}
)GLSL";

// ── Static helpers ────────────────────────────────────────────────────────

static void latlon_to_xyz(float lat_deg, float lon_deg,
                           float& x, float& y, float& z) {
    // Standard spherical: lon→theta, lat→phi
    // x=cos(lat)*cos(lon), y=sin(lat), z=cos(lat)*sin(lon)
    // Inverse: lat=asin(y), lon=atan2(z,x)
    float lat   = lat_deg * (float)M_PI / 180.f;
    float theta = lon_deg * (float)M_PI / 180.f;
    x = cosf(lat) * cosf(theta);
    y = sinf(lat);
    z = cosf(lat) * sinf(theta);
}

// ── GlobeRenderer ─────────────────────────────────────────────────────────

bool GlobeRenderer::init() {
    prog_sphere_ = compile_shader(GLOBE_VERT, GLOBE_FRAG);
    prog_lines_  = compile_shader(LINES_VERT, LINES_FRAG);
    prog_land_   = compile_shader(LAND_VERT,  LAND_FRAG);
    if (!prog_sphere_ || !prog_lines_ || !prog_land_) return false;
    build_sphere(64, 128);
    build_land();
    build_map_lines();
    build_stars();
    load_earth_texture();
    // Default orientation: screen center = 38N 127E, north pole straight up
    // col2=pick-fwd(38N,-127lon), col1=Gram-Schmidt(north,fwd), col0=cross(col1,col2)
    // Quaternion via Shepperd from that rotation matrix
    qw_ = -0.300017f;
    qx_ = -0.103304f;
    qy_ =  0.896658f;
    qz_ = -0.308744f;
    yaw_rad_   = 0.f;  // arcball 드래그 추적용 (초기값 무관)
    pitch_deg_ = 38.f; // 위도 추적용
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
    if (vao_stars_)  { glDeleteVertexArrays(1, &vao_stars_); vao_stars_=0; }
    if (vbo_stars_)  { glDeleteBuffers(1, &vbo_stars_); vbo_stars_=0; }
    if (prog_stars_) { glDeleteProgram(prog_stars_); prog_stars_=0; }
    seg_starts_.clear(); seg_counts_.clear();
}

void GlobeRenderer::set_viewport(int w, int h) {
    vp_w_ = w; vp_h_ = h;
}

void GlobeRenderer::render() {
    float mvp[16];
    get_mvp(mvp);

    // 0. Draw star background in NDC — no depth test, always behind globe
    if (prog_stars_ && vao_stars_) {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glEnable(GL_PROGRAM_POINT_SIZE);

        glUseProgram(prog_stars_);
        glBindVertexArray(vao_stars_);
        glDrawArrays(GL_POINTS, 0, star_count_);
        glBindVertexArray(0);

        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_PROGRAM_POINT_SIZE);
    }

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

    // 줌 5단계별 드래그 속도 계산
    // zoom_ 범위: 1.5(최대 확대) ~ 8.0(최대 축소)
    // 단계: 1=가장 멀리(wide), 5=가장 가까이(close)
    // 멀리 볼수록 빠르게, 가까이 볼수록 느리게
    float drag_scale;
    if      (zoom_ >= 6.5f) drag_scale = 2.2f;  // 단계1: 가장 멀리 — 빠르게
    else if (zoom_ >= 5.0f) drag_scale = 1.6f;  // 단계2
    else if (zoom_ >= 3.5f) drag_scale = 1.0f;  // 단계3: 기본 (zoom_=3.5)
    else if (zoom_ >= 2.5f) drag_scale = 0.55f; // 단계4
    else                    drag_scale = 0.28f;  // 단계5: 가장 가까이 — 느리게

    if (fabsf(ddx) > 0.f) {
        // Horizontal → rotate around world Y axis
        float angle = -ddx * 0.002f * drag_scale;
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

        float angle = ddy * 0.002f * drag_scale;
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
// Strategy: unproject two NDC points (near/far) through MVP_inverse to get
// a ray in globe LOCAL space, then intersect with unit sphere.
// This is independent of camera orientation — the hit point is always in
// the globe's own coordinate frame where latlon_to_xyz is defined.

static bool mat4_inverse(const float* M, float* I) {
    // General 4x4 inverse via cofactors (column-major input/output)
    float s0  = M[0]*M[5]  - M[4]*M[1];
    float s1  = M[0]*M[9]  - M[8]*M[1];
    float s2  = M[0]*M[13] - M[12]*M[1];
    float s3  = M[4]*M[9]  - M[8]*M[5];
    float s4  = M[4]*M[13] - M[12]*M[5];
    float s5  = M[8]*M[13] - M[12]*M[9];
    float c5  = M[10]*M[15] - M[14]*M[11];
    float c4  = M[6]*M[15]  - M[14]*M[7];
    float c3  = M[6]*M[11]  - M[10]*M[7];
    float c2  = M[2]*M[15]  - M[14]*M[3];
    float c1  = M[2]*M[11]  - M[10]*M[3];
    float c0  = M[2]*M[7]   - M[6]*M[3];
    float det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;
    if (fabsf(det) < 1e-10f) return false;
    float idet = 1.f / det;
    I[0]  = ( M[5]*c5  - M[9]*c4  + M[13]*c3) * idet;
    I[1]  = (-M[1]*c5  + M[9]*c2  - M[13]*c1) * idet;
    I[2]  = ( M[1]*c4  - M[5]*c2  + M[13]*c0) * idet;
    I[3]  = (-M[1]*c3  + M[5]*c1  - M[9]*c0)  * idet;
    I[4]  = (-M[4]*c5  + M[8]*c4  - M[12]*c3) * idet;
    I[5]  = ( M[0]*c5  - M[8]*c2  + M[12]*c1) * idet;
    I[6]  = (-M[0]*c4  + M[4]*c2  - M[12]*c0) * idet;
    I[7]  = ( M[0]*c3  - M[4]*c1  + M[8]*c0)  * idet;
    I[8]  = ( M[7]*s5  - M[11]*s4 + M[15]*s3) * idet;
    I[9]  = (-M[3]*s5  + M[11]*s2 - M[15]*s1) * idet;
    I[10] = ( M[3]*s4  - M[7]*s2  + M[15]*s0) * idet;
    I[11] = (-M[3]*s3  + M[7]*s1  - M[11]*s0) * idet;
    I[12] = (-M[6]*s5  + M[10]*s4 - M[14]*s3) * idet;
    I[13] = ( M[2]*s5  - M[10]*s2 + M[14]*s1) * idet;
    I[14] = (-M[2]*s4  + M[6]*s2  - M[14]*s0) * idet;
    I[15] = ( M[2]*s3  - M[6]*s1  + M[10]*s0) * idet;
    return true;
}

static void mat4_mul_vec4(const float* M, float x, float y, float z, float w,
                           float& rx, float& ry, float& rz, float& rw) {
    rx = M[0]*x + M[4]*y + M[8]*z  + M[12]*w;
    ry = M[1]*x + M[5]*y + M[9]*z  + M[13]*w;
    rz = M[2]*x + M[6]*y + M[10]*z + M[14]*w;
    rw = M[3]*x + M[7]*y + M[11]*z + M[15]*w;
}

bool GlobeRenderer::pick(float mx, float my,
                          float& lat_deg, float& lon_deg) const {
    // NDC of the mouse pixel
    float ndcx =  2.f * mx / (float)vp_w_ - 1.f;
    float ndcy =  1.f - 2.f * my / (float)vp_h_;

    // Unproject two NDC depths through MVP_inverse → globe local space
    float mvp[16], mvp_inv[16];
    get_mvp(mvp);
    if (!mat4_inverse(mvp, mvp_inv)) return false;

    float nx0, ny0, nz0, nw0;   // near (ndcz = -1)
    float nx1, ny1, nz1, nw1;   // far  (ndcz = +1)
    mat4_mul_vec4(mvp_inv, ndcx, ndcy, -1.f, 1.f, nx0,ny0,nz0,nw0);
    mat4_mul_vec4(mvp_inv, ndcx, ndcy, +1.f, 1.f, nx1,ny1,nz1,nw1);

    // Perspective divide → Cartesian globe-local coords
    if (fabsf(nw0) < 1e-8f || fabsf(nw1) < 1e-8f) return false;
    float px0 = nx0/nw0, py0 = ny0/nw0, pz0 = nz0/nw0; // ray origin
    float px1 = nx1/nw1, py1 = ny1/nw1, pz1 = nz1/nw1; // ray far

    // Ray in globe local space
    float dx = px1-px0, dy = py1-py0, dz = pz1-pz0;
    float dlen = sqrtf(dx*dx + dy*dy + dz*dz);
    if (dlen < 1e-8f) return false;
    dx/=dlen; dy/=dlen; dz/=dlen;

    // Ray-sphere intersection with unit sphere at local origin
    float b = 2.f*(px0*dx + py0*dy + pz0*dz);
    float c = px0*px0 + py0*py0 + pz0*pz0 - 1.f;
    float disc = b*b - 4.f*c;
    if (disc < 0.f) return false;

    float t = (-b - sqrtf(disc)) * 0.5f;
    if (t < 0.f) t = (-b + sqrtf(disc)) * 0.5f;
    if (t < 0.f) return false;

    float hx = px0 + t*dx;
    float hy = py0 + t*dy;
    float hz = pz0 + t*dz;

    // hit point is already in globe local space — extract lat/lon directly
    // latlon_to_xyz: x=cos(lat)*cos(lon), y=sin(lat), z=cos(lat)*sin(lon)
    lat_deg = -asinf(hy < -1.f ? -1.f : (hy > 1.f ? 1.f : hy)) * 180.f / (float)M_PI;
    lon_deg = atan2f(hz, hx) * 180.f / (float)M_PI;
    return true;
}

bool GlobeRenderer::project(float lat_deg, float lon_deg,
                             float& sx, float& sy) const {
    float x, y, z;
    // lat_deg uses the user-facing convention (pick() negates y internally),
    // so negate back to match globe geometry before converting to xyz.
    latlon_to_xyz(-lat_deg, lon_deg, x, y, z);

    // Cull markers on the far side of the globe.
    // rot column 2 (indices 8,9,10) is the local-space direction toward the camera.
    float qx=qx_, qy=qy_, qz=qz_, qw=qw_;
    float cam_dx = 2*(qx*qz + qw*qy);
    float cam_dy = 2*(qy*qz - qw*qx);
    float cam_dz = 1 - 2*(qx*qx + qy*qy);
    if (x*cam_dx + y*cam_dy + z*cam_dz < 0.f) return false; // behind globe

    float mvp[16];
    get_mvp(mvp);

    // Transform to clip space
    float cx = mvp[0]*x + mvp[4]*y + mvp[8]*z  + mvp[12];
    float cy = mvp[1]*x + mvp[5]*y + mvp[9]*z  + mvp[13];
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
    M[5]  = -t;
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
    // View = trans(-zoom_) * view_rot_T   where view_rot_T = transpose(rot)
    // view_inv = rot * trans(zoom_)
    // Camera world pos col = rot * (0,0,zoom,1)^T
    //   col3,row0 = rot[8]*zoom, col3,row1 = rot[9]*zoom, col3,row2 = rot[10]*zoom
    float rot[16];
    mat4_from_quat(rot, qw_, qx_, qy_, qz_);
    memcpy(inv, rot, 64);
    inv[12] = rot[8]  * zoom_;
    inv[13] = rot[9]  * zoom_;
    inv[14] = rot[10] * zoom_;
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
        float v   = 1.f - (float)st / stacks; // st=0=south(y=-1)→v=1, stacks=north(y=+1)→v=0
        for (int sl = 0; sl <= slices; sl++) {
            float theta = 2.f * (float)M_PI * (float)sl / slices;
            float x = r * cosf(theta);
            float z = r * sinf(theta);
            // u=0 at theta=0 (lon=0°E), offset +0.5 so Greenwich→texture center
            float u = (float)sl / slices + 0.5f;
            if (u > 1.f) u -= 1.f;
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
    float prev_lon      = 0.f;

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
            prev_lon = 0.f;
        } else {
            // 180° 안티메리디안 교차 감지: 인접 두 점 경도 차 > 180° → 세그먼트 분할
            if (in_segment) {
                float dlon = lon - prev_lon;
                if (dlon >  180.f) dlon -= 360.f;
                if (dlon < -180.f) dlon += 360.f;
                if (fabsf(dlon) > 170.f) {
                    // 현재 세그먼트 저장 후 새 세그먼트 시작
                    if (current_count > 0) {
                        seg_starts_.push_back(current_start);
                        seg_counts_.push_back((GLsizei)current_count);
                    }
                    current_start = (GLint)(verts.size() / 3);
                    current_count = 0;
                }
            }
            float x, y, z;
            latlon_to_xyz(lat, lon, x, y, z);
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
            current_count++;
            in_segment = true;
            prev_lon = lon;
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
    // Flip image horizontally (mirror each row) to match sphere u-direction
    for (int row = 0; row < h; row++) {
        unsigned char* r = data + row * w * 3;
        for (int col = 0; col < w / 2; col++) {
            int opp = w - 1 - col;
            unsigned char tmp[3] = {r[col*3], r[col*3+1], r[col*3+2]};
            r[col*3]=r[opp*3]; r[col*3+1]=r[opp*3+1]; r[col*3+2]=r[opp*3+2];
            r[opp*3]=tmp[0];   r[opp*3+1]=tmp[1];     r[opp*3+2]=tmp[2];
        }
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

void GlobeRenderer::build_stars() {
    // Stars stored as NDC x,y + brightness (3 floats each)
    static const int N = 2500;
    float verts[N * 3];

    uint32_t seed = 0xDEADBEEFu;
    auto lcg = [&]() -> float {
        seed = seed * 1664525u + 1013904223u;
        return (float)(seed >> 8) / (float)(1 << 24); // [0,1)
    };

    for (int i = 0; i < N; i++) {
        verts[i*3+0] = lcg() * 2.f - 1.f; // NDC x
        verts[i*3+1] = lcg() * 2.f - 1.f; // NDC y
        float b = lcg(); b = b * b;        // bias toward dim
        verts[i*3+2] = 0.2f + b * 0.8f;
    }
    star_count_ = N;

    prog_stars_ = compile_shader(STARS_VERT, STARS_FRAG);

    glGenVertexArrays(1, &vao_stars_);
    glGenBuffers(1, &vbo_stars_);
    glBindVertexArray(vao_stars_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_stars_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)(2*sizeof(float)));
    glBindVertexArray(0);
}
