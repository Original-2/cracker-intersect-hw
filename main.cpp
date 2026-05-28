#include "Vtop.h"
#include "verilated.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <random>
#include <limits>
#include <memory>

// Q16.16 fixed‑point conversion
#define Q16_16(x) ((int32_t)((x) * 65536.0))
#define TO_DOUBLE(x) ((x) / 65536.0)

// BRAM layout for one triangle (16 words)
const int TRI_BRAM_WORDS = 16;
const int V0_X_OFF = 0;
const int V0_Y_OFF = 1;
const int V0_Z_OFF = 2;
const int V1_X_OFF = 3;
const int V1_Y_OFF = 4;
const int V1_Z_OFF = 5;
const int V2_X_OFF = 6;
const int V2_Y_OFF = 7;
const int V2_Z_OFF = 8;

// 3D vector
struct Vector3 {
    double x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3 &v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vector3 operator-(const Vector3 &v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vector3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vector3 operator/(double s) const { return {x/s, y/s, z/s}; }
    Vector3 operator*(const Vector3 &v) const { return {x*v.x, y*v.y, z*v.z}; }

    Vector3 cross(const Vector3 &v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double dot(const Vector3 &v) const { return x*v.x + y*v.y + z*v.z; }
    double lengthSquared() const { return x*x + y*y + z*z; }
    double length() const { return std::sqrt(lengthSquared()); }
    Vector3 normalized() const {
        double len = length();
        return (len < 1e-12) ? *this : *this / len;
    }
};

struct Material {
    Vector3 albedo;
    Vector3 emission;
};

struct Triangle {
    Vector3 v0, v1, v2;
    int materialIdx;
};


// Compute triangle normal
Vector3 triangleNormal(const Triangle &tri) {
    Vector3 e1 = tri.v1 - tri.v0;
    Vector3 e2 = tri.v2 - tri.v0;
    return e1.cross(e2).normalized();
}

void addRectangle(std::vector<Triangle> &triangles, int matIdx,
                  const Vector3 &a, const Vector3 &b,
                  const Vector3 &c, const Vector3 &d,
                  const Vector3 &desiredNormal) {
    Triangle t1{a, b, c, matIdx};
    Triangle t2{a, c, d, matIdx};

    // Flip t1 if needed
    Vector3 n1 = triangleNormal(t1);
    if (n1.dot(desiredNormal) < 0) std::swap(t1.v1, t1.v2);

    // Flip t2 if needed
    Vector3 n2 = triangleNormal(t2);
    if (n2.dot(desiredNormal) < 0) std::swap(t2.v1, t2.v2);

    triangles.push_back(t1);
    triangles.push_back(t2);
}

void addCube(std::vector<Triangle> &triangles, int matIdx,
             const Vector3 &center, double sx, double sy, double sz) {
    double x = center.x, y = center.y, z = center.z;
    double hx = sx*0.5, hy = sy*0.5, hz = sz*0.5;
    // front
    addRectangle(triangles, matIdx,
                 {x-hx, y-hy, z+hz}, {x+hx, y-hy, z+hz},
                 {x+hx, y+hy, z+hz}, {x-hx, y+hy, z+hz}, {0,0,1});
    // back
    addRectangle(triangles, matIdx,
                 {x-hx, y-hy, z-hz}, {x+hx, y-hy, z-hz},
                 {x+hx, y+hy, z-hz}, {x-hx, y+hy, z-hz}, {0,0,-1});
    // left
    addRectangle(triangles, matIdx,
                 {x-hx, y-hy, z-hz}, {x-hx, y+hy, z-hz},
                 {x-hx, y+hy, z+hz}, {x-hx, y-hy, z+hz}, {-1,0,0});
    // right
    addRectangle(triangles, matIdx,
                 {x+hx, y-hy, z-hz}, {x+hx, y+hy, z-hz},
                 {x+hx, y+hy, z+hz}, {x+hx, y-hy, z+hz}, {1,0,0});
    // bottom
    addRectangle(triangles, matIdx,
                 {x-hx, y-hy, z-hz}, {x+hx, y-hy, z-hz},
                 {x+hx, y-hy, z+hz}, {x-hx, y-hy, z+hz}, {0,-1,0});
    // top
    addRectangle(triangles, matIdx,
                 {x-hx, y+hy, z-hz}, {x+hx, y+hy, z-hz},
                 {x+hx, y+hy, z+hz}, {x-hx, y+hy, z+hz}, {0,1,0});
}

// Hardware intersection wrapper
class HWIntersector {
public:
    HWIntersector() : dut(new Vtop) {
        dut->clk = 0;
        dut->rst_n = 0;
        dut->scene_valid = 0;
        dut->scene_ignore_idx = -1;
        dut->io_next = 0;
        dut->eval();

        dut->clk = 1; dut->eval();
        dut->clk = 0; dut->eval();
        dut->rst_n = 1;
        dut->eval();
        dut->clk = 1; dut->eval();
        dut->clk = 0; dut->eval();
    }


    void bramWrite(uint32_t addr, uint32_t data) {
        dut->bram_addr = addr;
        dut->bram_wdata = data;
        dut->bram_we = 1;
        dut->clk = 1; dut->eval();
        dut->clk = 0; dut->eval();
        dut->bram_we = 0;
    }

    // configure camera
    void configureCamera(const Vector3& pos,
                         const Vector3& uvec, const Vector3& vvec, const Vector3& wvec,
                         double halfW, double halfH) {
        dut->cam_pos_x    = Q16_16(pos.x);
        dut->cam_pos_y    = Q16_16(pos.y);
        dut->cam_pos_z    = Q16_16(pos.z);
        dut->cam_u_vec_x  = Q16_16(uvec.x);
        dut->cam_u_vec_y  = Q16_16(uvec.y);
        dut->cam_u_vec_z  = Q16_16(uvec.z);
        dut->cam_v_vec_x  = Q16_16(vvec.x);
        dut->cam_v_vec_y  = Q16_16(vvec.y);
        dut->cam_v_vec_z  = Q16_16(vvec.z);
        dut->cam_w_vec_x  = Q16_16(wvec.x);
        dut->cam_w_vec_y  = Q16_16(wvec.y);
        dut->cam_w_vec_z  = Q16_16(wvec.z);
        dut->cam_halfWidth  = Q16_16(halfW);
        dut->cam_halfHeight = Q16_16(halfH);
        // propagate comb logic
        dut->eval();
    }


    // hardware to generate a ray for (u,v), intersect, and return the color
    Vector3 get_pixel_colour(double u, double v, int ignoreIdx = -1) {
        // Set camera UVs and ignore index
        dut->cam_u = Q16_16(u);
        dut->cam_v = Q16_16(v);
        dut->scene_ignore_idx = ignoreIdx;

        // Trigger the big FSM
        dut->pixel_req = 1;
        dut->clk = 1; dut->eval();
        dut->clk = 0; dut->eval();
        dut->pixel_req = 0;

        // Wait for the pipeline to finish
        int timeout = 1000000;
        while (!dut->pixel_ready && timeout > 0) {
            dut->clk = 1; dut->eval();
            dut->clk = 0; dut->eval();
            timeout--;
        }

        // Extract 32-bit fixed point color components
        return Vector3(TO_DOUBLE((int32_t)dut->pixel_r),
                       TO_DOUBLE((int32_t)dut->pixel_g),
                       TO_DOUBLE((int32_t)dut->pixel_b));
    }
private:
    Vtop* dut;
};


int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    HWIntersector hw;

    std::vector<Material> materials;
    int matGray  = (int)materials.size(); materials.push_back({{0.7,0.7,0.7}, {0,0,0}});
    int matRed   = (int)materials.size(); materials.push_back({{0.8,0.2,0.2}, {0,0,0}});
    int matGreen = (int)materials.size(); materials.push_back({{0.2,0.8,0.2}, {0,0,0}});
    int matBlue = (int)materials.size(); materials.push_back({{0.2,0.2,0.8}, {0,0,0}});
    int matPink = (int)materials.size(); materials.push_back({{0.8,0.2,0.8}, {0,0,0}});
    int matWhite = (int)materials.size(); materials.push_back({{0.9,0.9,0.9}, {0,0,0}});
    int matLight = (int)materials.size(); materials.push_back({{0,0,0}, {10.0,10.0,10.0}});

    std::vector<Triangle> triangles;
    // floor
    addRectangle(triangles, matPink, {-2.0,0.0,-2.0}, { 2.0,0.0,-2.0}, { 2.0,0.0, 2.0}, {-2.0,0.0, 2.0}, {0,1,0});
    // ceiling
    addRectangle(triangles, matGray, {-2.0,2.0,-2.0}, { 2.0,2.0,-2.0}, { 2.0,2.0, 2.0}, {-2.0,2.0, 2.0}, {0,-1,0});
    // back wall
    addRectangle(triangles, matBlue, {-2.0,0.0,-2.0}, { 2.0,0.0,-2.0}, { 2.0,2.0,-2.0}, {-2.0,2.0,-2.0}, {0,0,1});
    // left wall (red)
    addRectangle(triangles, matRed,  {-2.0,0.0,-2.0}, {-2.0,2.0,-2.0}, {-2.0,2.0, 2.0}, {-2.0,0.0, 2.0}, {1,0,0});
    // right wall (green)
    addRectangle(triangles, matGreen,{ 2.0,0.0,-2.0}, { 2.0,2.0,-2.0}, { 2.0,2.0, 2.0}, { 2.0,0.0, 2.0}, {-1,0,0});
    // area light (on ceiling)
    addRectangle(triangles, matLight,{-1.5,1.98,-1.5}, { 1.5,1.98,-1.5}, { 1.5,1.98, 1.5}, {-1.5,1.98, 1.5}, {0,-1,0});
    // white diffuse cube
    addCube(triangles, matWhite, {0.0,0.5,0.0}, 1.0, 1.0, 1.0);

    // Load all triangle data into BRAM
    for (size_t i = 0; i < triangles.size(); ++i) {
        const Triangle &t  = triangles[i];
        const Material &m  = materials[t.materialIdx];
        int base = i * TRI_BRAM_WORDS;

        hw.bramWrite(base + V0_X_OFF, Q16_16(t.v0.x));
        hw.bramWrite(base + V0_Y_OFF, Q16_16(t.v0.y));
        hw.bramWrite(base + V0_Z_OFF, Q16_16(t.v0.z));
        hw.bramWrite(base + V1_X_OFF, Q16_16(t.v1.x));
        hw.bramWrite(base + V1_Y_OFF, Q16_16(t.v1.y));
        hw.bramWrite(base + V1_Z_OFF, Q16_16(t.v1.z));
        hw.bramWrite(base + V2_X_OFF, Q16_16(t.v2.x));
        hw.bramWrite(base + V2_Y_OFF, Q16_16(t.v2.y));
        hw.bramWrite(base + V2_Z_OFF, Q16_16(t.v2.z));

        // albedo
        hw.bramWrite(base + 9,  Q16_16(m.albedo.x));
        hw.bramWrite(base + 10, Q16_16(m.albedo.y));
        hw.bramWrite(base + 11, Q16_16(m.albedo.z));

        // emission
        hw.bramWrite(base + 12, Q16_16(m.emission.x));
        hw.bramWrite(base + 13, Q16_16(m.emission.y));
        hw.bramWrite(base + 14, Q16_16(m.emission.z));

        // flags: light flag = 1 if emission > 0
        bool isLight = (m.emission.x > 0 || m.emission.y > 0 || m.emission.z > 0);
        hw.bramWrite(base + 15, isLight ? 1 : 0);
    }
    hw.bramWrite(triangles.size() * TRI_BRAM_WORDS + V0_X_OFF, 0xFFFFFFFF);

    Vector3 camPos(0, 1.2, 3.5);
    Vector3 lookAt(0, 0.8, 0);
    Vector3 up(0, 1, 0);
    double fovDeg = 60.0;
    int width  = 400;
    int height = 400;

    // Pre‑compute the basis vectors
    double aspect = (double)width / height;
    double theta = fovDeg * 3.14159 / 180.0;
    double halfH = std::tan(theta * 0.5);
    double halfW = aspect * halfH;

    Vector3 w = (camPos - lookAt).normalized();   // w points from lookAt to camera
    Vector3 uvec = up.cross(w).normalized();      // right
    Vector3 vvec = w.cross(uvec);                 // up (camera space)

    // Send the basis vectors and image‑plane size to the hardware camera
    hw.configureCamera(camPos, uvec, vvec, w, halfW, halfH);
    const int spp = 2;


    // Render
    std::vector<uint8_t> image(width * height * 3, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Vector3 pixelColor(0,0,0);

            for (int s = 0; s < spp; ++s) {
                double u = ((double)x) / width;
                double v = ((double)y) / height;

                // all in hardware
                pixelColor = pixelColor + hw.get_pixel_colour(u, v);
            }

            pixelColor = pixelColor / (double)spp;

            int idx = (y * width + x) * 3;
            image[idx+0] = (uint8_t)(std::min(1.0, pixelColor.x) * 255);
            image[idx+1] = (uint8_t)(std::min(1.0, pixelColor.y) * 255);
            image[idx+2] = (uint8_t)(std::min(1.0, pixelColor.z) * 255);
        }
        if (y % 10 == 0) std::cout << "Row " << y << " done." << std::endl;
    }

    // Save PPM image
    std::ofstream out("intersectoin_all_hw.ppm", std::ios::binary);

    out << "P6\n" << width << " " << height << "\n255\n";
    out.write(reinterpret_cast<const char*>(image.data()), image.size());
    out.close();
    std::cout << "Image saved as intersection_all_hw.ppm\n";

    return 0;
}
