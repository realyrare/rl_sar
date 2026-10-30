// 临时工具：离屏渲染 ancnW1 的两种站姿，输出 PPM 供对比（用完即删）
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

static const int TW = 560, TH = 420;

static void writePPM(const char *path, const unsigned char *rgb, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(rgb, 1, (size_t)w * h * 3, f);
    fclose(f);
}

int main(int argc, char **argv)
{
    if (argc < 3) { printf("usage: %s scene.xml out_prefix\n", argv[0]); return 1; }
    char err[2000] = {0};
    mjModel *m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
    if (!m) { printf("load failed: %s\n", err); return 1; }
    mjData *d = mj_makeData(m);

    if (!glfwInit()) { printf("glfwInit failed\n"); return 1; }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow *win = glfwCreateWindow(TW, TH, "offscreen", nullptr, nullptr);
    if (!win) { printf("createWindow failed\n"); return 1; }
    glfwMakeContextCurrent(win);

    mjvScene scn; mjv_defaultScene(&scn);
    mjvOption opt; mjv_defaultOption(&opt);
    mjvCamera cam; mjv_defaultCamera(&cam);
    mjrContext con; mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 4000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 两种姿态（policy 顺序：fl, fr, hl, hr 每条腿 hip/thigh/calf/wheel）
    std::vector<double> poseA(16, 0.0);   // 训练侧 init_state：全 0
    std::vector<double> poseB = {          // 旧的 m20 符号体系
        0.0,  0.6, -1.0, 0.0,
        0.0,  0.6, -1.0, 0.0,
        0.0, -0.6,  1.0, 0.0,
        0.0, -0.6,  1.0, 0.0};
    // MJCF 里关节顺序是 fl(fr,hl,hr) 每条腿 hip/thigh/calf/wheel，与上面一致
    double zA = 0.537, zB = 0.659;

    int gridW = TW * 2, gridH = TH * 2;
    std::vector<unsigned char> big((size_t)gridW * gridH * 3, 0);
    std::vector<unsigned char> buf((size_t)TW * TH * 3, 0);

    struct View { bool training; double az, el; const char *label; };
    View views[4] = {
        {true,  90.0, -2.0, "A: joint=0 (training init), side"},
        {true, 140.0, -18.0, "A: joint=0, oblique"},
        {false, 90.0, -2.0, "B: old [0.6,-1.0], side"},
        {false, 140.0, -18.0, "B: old [0.6,-1.0], oblique"},
    };

    for (int v = 0; v < 4; ++v)
    {
        // 设置姿态
        const std::vector<double> &q = views[v].training ? poseA : poseB;
        double z = views[v].training ? zA : zB;
        d->qpos[0] = 0; d->qpos[1] = 0; d->qpos[2] = z;
        d->qpos[3] = 1; d->qpos[4] = 0; d->qpos[5] = 0; d->qpos[6] = 0;
        for (int i = 0; i < 16 && 7 + i < m->nq; ++i) d->qpos[7 + i] = q[i];
        mj_forward(m, d);

        cam.type = mjCAMERA_FREE;
        cam.lookat[0] = 0.05; cam.lookat[1] = 0.0; cam.lookat[2] = 0.25;
        cam.distance = 1.35;
        cam.azimuth = views[v].az;
        cam.elevation = views[v].el;
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

        mjrRect vp = {0, 0, TW, TH};
        mjr_render(vp, &scn, &con);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, views[v].label, nullptr, &con);
        glFinish();
        glReadPixels(0, 0, TW, TH, GL_RGB, GL_UNSIGNED_BYTE, buf.data());

        // 上下翻转并贴到大图（行优先）
        int col = v % 2, row = v / 2;
        for (int y = 0; y < TH; ++y)
        {
            const unsigned char *src = buf.data() + (size_t)(TH - 1 - y) * TW * 3;
            unsigned char *dst = big.data() + ((size_t)(row * TH + y) * gridW + col * TW) * 3;
            memcpy(dst, src, (size_t)TW * 3);
        }
    }

    std::string out = std::string(argv[2]) + ".ppm";
    writePPM(out.c_str(), big.data(), gridW, gridH);
    printf("wrote %s (%dx%d)\n", out.c_str(), gridW, gridH);

    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(win);
    glfwTerminate();
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
