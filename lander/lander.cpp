// aruco_single_camB_downward_bridge.cpp
// Single-camera (cam_B) ArUco detection with downward-facing camera mount.
// Output: send "angle_x angle_y distance\n" via UDP to 127.0.0.1:6000 for Python bridge.
// Optional: still publishes [ts, X, Y, Z, qw, qx, qy, qz] to /tmp/beta_tag (unchanged).

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <array>
#include <unordered_map>
#include <string>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <arpa/inet.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/un.h>
#include <signal.h>
#include <sys/select.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include "depthai/depthai.hpp"

// ------------- Config -------------
#define FPS 30
#define RESOLUTION_ENUM dai::MonoCameraProperties::SensorResolution::THE_480_P
#define TAG_DET_STRIDE 1
#define REPROJ_THRESH_PX 3.0

// ---- Tag config (global) ----
// World frame is NED: X North, Y East, Z Down.
struct Tag3D {
    int id = -1;
    std::array<cv::Point3f,4> world_corners{}; // in NED
};
static std::vector<Tag3D> g_tags;
static std::unordered_map<int, Tag3D> g_tag_by_id;

static bool load_tag_config(const std::string& path){
    g_tags.clear(); g_tag_by_id.clear();
    std::ifstream ifs(path);
    if(!ifs.is_open()){
        std::cerr << "Tag config open failed: " << path << "\n";
        return false;
    }
    auto trim = [](std::string s){
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        if(a==std::string::npos) return std::string();
        return s.substr(a, b-a+1);
    };
    Tag3D cur; bool have_id=false, have_p[4]{false,false,false,false};
    auto flush = [&](){
        if(have_id && have_p[0] && have_p[1] && have_p[2] && have_p[3]){
            g_tags.push_back(cur);
        }
        cur = Tag3D{}; have_id=false; std::fill(std::begin(have_p), std::end(have_p), false);
    };
    std::string line;
    while(std::getline(ifs, line)){
        line = trim(line);
        if(line.empty() || line[0]=='#' || line[0]=='['){
            if(!line.empty() && line[0]=='['){ flush(); }
            continue;
        }
        auto eq = line.find('=');
        if(eq==std::string::npos) continue;
        std::string key = trim(line.substr(0,eq));
        std::string val = trim(line.substr(eq+1));
        if(key=="tag"){
            if(have_id){ flush(); }
            cur.id = std::stoi(val);
            have_id = true;
        } else if(key=="p0" || key=="p1" || key=="p2" || key=="p3"){
            std::istringstream ss(val);
            float x,y,z;
            if(ss>>x>>y>>z){
                int idx = key[1]-'0';
                cur.world_corners[idx] = cv::Point3f(x,y,z); // NED
                have_p[idx]=true;
            }
        }
    }
    flush();
    for(const auto& t : g_tags) g_tag_by_id[t.id] = t;
    std::cerr << "Loaded " << g_tags.size() << " tags from " << path << "\n";
    return !g_tags.empty();
}

// ---------- Pose/Math helpers ----------
static inline cv::Mat quat_from_ZYX(double yaw, double pitch, double roll) {
    const double hy = 0.5 * yaw;
    const double hp = 0.5 * pitch;
    const double hr = 0.5 * roll;

    const double cy = std::cos(hy), sy = std::sin(hy);
    const double cp = std::cos(hp), sp = std::sin(hp);
    const double cr = std::cos(hr), sr = std::sin(hr);

    double w = cr*cp*cy + sr*sp*sy;
    double x = sr*cp*cy - cr*sp*sy;
    double y = cr*sp*cy + sr*cp*sy;
    double z = cr*cp*sy - sr*sp*cy;

    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n > 0.0) { w /= n; x /= n; y /= n; z /= n; }
    return (cv::Mat_<double>(1,4) << w, x, y, z);
}

static inline double nowSec() {
    using clock = std::chrono::steady_clock;
    static auto t0 = clock::now();
    auto dt = std::chrono::duration<double>(clock::now() - t0).count();
    return dt;
}

// ---------- Global state ----------
struct TagPoseVis {
    int id = -1;
    cv::Point3d pos_world; // Vehicle position in NED
    cv::Point3d pos_body;  // Body (vehicle->target)
};

static double g_fps = 0.0;

// ---------- Intrinsics helpers ----------
struct Intrinsics {
    double fx=0, fy=0, cx=0, cy=0;
    bool valid() const { return fx>0 && fy>0; }
};
static bool load_intrinsics_yaml(const std::string& path, Intrinsics& K){
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if(!fs.isOpened()) return false;
    cv::Mat Km;
    if(fs["K"].isNone()){
        fs["fx"] >> K.fx;
        fs["fy"] >> K.fy;
        fs["cx"] >> K.cx;
        fs["cy"] >> K.cy;
    } else {
        fs["K"] >> Km;
        if(Km.rows==3 && Km.cols==3){
            K.fx = Km.at<double>(0,0);
            K.fy = Km.at<double>(1,1);
            K.cx = Km.at<double>(0,2);
            K.cy = Km.at<double>(1,2);
        }
    }
    fs.release();
    return K.valid();
}
static Intrinsics estimate_from_fov(int w, int h, double hfov_deg=71.9, double vfov_deg=56.0){
    Intrinsics K;
    double hfov = hfov_deg * M_PI/180.0;
    double vfov = vfov_deg * M_PI/180.0;
    K.fx = (w/2.0) / std::tan(hfov/2.0);
    K.fy = (h/2.0) / std::tan(vfov/2.0);
    K.cx = w/2.0;
    K.cy = h/2.0;
    return K;
}

// ---------- ArUco detection ----------
cv::Ptr<cv::aruco::Dictionary> g_arucoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Ptr<cv::aruco::DetectorParameters> g_arucoParams = cv::aruco::DetectorParameters::create();

// Fixed camera-to-vehicle rotation for downward mount.
// OpenCV camera: +X right, +Y down, +Z forward.
// Vehicle body:  +X forward, +Y right, +Z down.
static cv::Mat get_R_vc_downward(){
    // body_x = cam_y; body_y = cam_x; body_z = cam_z
    cv::Mat R = (cv::Mat_<double>(3,3) <<
         0, -1, 0,
         1,  0, 0,
         0,  0, 1
    );
    return R; // camera->vehicle
}

// ---------- Signals ----------
static volatile bool g_run = true;
static void sig_handler(int){ g_run = false; }

// ---------- Detection and UDP bridge TX ----------
static void detect_and_process(const cv::Mat& gray, const Intrinsics& K, double ts,
                               int ipc_sock, const sockaddr_un& tag_addr,
                               cv::Mat* vis, bool draw_viz,
                               std::vector<TagPoseVis>& visTags,
                               // UDP bridge socket (to Python) and destination
                               int bridge_sock, const sockaddr_in& bridge_addr)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(gray, g_arucoDict, corners, ids, g_arucoParams);
    if(ids.empty()) return;

    for(auto& c : corners){
        if(c.size() == 4){
            cv::cornerSubPix(gray, c, cv::Size(5,5), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.01));
        }
    }

    cv::Mat KK = (cv::Mat_<double>(3,3) << K.fx, 0, K.cx, 0, K.fy, K.cy, 0,0,1);
    cv::Mat dist = cv::Mat::zeros(5,1, CV_64F);

    const cv::Mat R_vc = get_R_vc_downward(); // camera -> vehicle
    const cv::Mat R_cv = R_vc.t();            // vehicle -> camera
    cv::Mat t_cv_body = (cv::Mat_<double>(3,1) << 0.0, 0.0, 0.0); // camera offset in body (m) if any

    for(size_t i=0;i<ids.size();++i){
        auto itT = g_tag_by_id.find(ids[i]);
        if(itT == g_tag_by_id.end()) continue;
        if(corners[i].size() != 4) continue;

        const Tag3D& T = itT->second;
        std::vector<cv::Point3f> world_pts(T.world_corners.begin(), T.world_corners.end()); // NED

        cv::Mat rvec_cw, tvec_cw;
        bool ok = cv::solvePnP(world_pts, corners[i], KK, dist, rvec_cw, tvec_cw, false, cv::SOLVEPNP_SQPNP);
        if(!ok) continue;

        // Reprojection RMS check
        std::vector<cv::Point2f> proj;
        cv::projectPoints(world_pts, rvec_cw, tvec_cw, KK, dist, proj);
        double sumSq=0.0; for(int k=0;k<4;++k){ double dx=proj[k].x-corners[i][k].x; double dy=proj[k].y-corners[i][k].y; sumSq+=dx*dx+dy*dy; }
        double rmsErr = std::sqrt(sumSq/4.0);
        if(!std::isfinite(rmsErr) || rmsErr > REPROJ_THRESH_PX) continue;

        // Camera pose in world (NED)
        cv::Mat R_cw; cv::Rodrigues(rvec_cw, R_cw);
        cv::Mat R_wc = R_cw.t();
        cv::Mat t_wc = -R_wc * tvec_cw;

        // Vehicle pose in world (NED)
        cv::Mat R_wv = R_wc * R_cv;
        cv::Mat t_wv = t_wc - R_wv * t_cv_body;

        double Xw = t_wv.at<double>(0,0);
        double Yw = t_wv.at<double>(1,0);
        double Zw = t_wv.at<double>(2,0);

        // Convert to quaternion (world<-vehicle)
        double r00=R_wv.at<double>(0,0), r10=R_wv.at<double>(1,0), r20=R_wv.at<double>(2,0);
        double r21=R_wv.at<double>(2,1), r22=R_wv.at<double>(2,2);
        double sy = std::sqrt(r00*r00 + r10*r10);
        double roll = std::atan2(r21, r22);
        double pitch = std::atan2(-r20, sy);
        double yaw = std::atan2(r10, r00);
        cv::Mat q_wv = quat_from_ZYX(yaw, pitch, roll);

        // Tag center in world (NED)
        cv::Point3d tag_center_w(0,0,0);
        for (int k=0;k<4;++k){
            tag_center_w.x += world_pts[k].x;
            tag_center_w.y += world_pts[k].y;
            tag_center_w.z += world_pts[k].z;
        }
        tag_center_w.x *= 0.25; tag_center_w.y *= 0.25; tag_center_w.z *= 0.25;

        cv::Mat d_w = (cv::Mat_<double>(3,1) << tag_center_w.x - Xw,
                                                tag_center_w.y - Yw,
                                                tag_center_w.z - Zw);
        cv::Mat R_vw = R_wv.t();
        cv::Mat d_v = R_vw * d_w;

        double Xb = d_v.at<double>(0,0);
        double Yb = d_v.at<double>(1,0);
        double Zb = d_v.at<double>(2,0);

        // Publish pose to UNIX socket (unchanged; remove if not needed)
        double qw = q_wv.at<double>(0,0);
        double qx = q_wv.at<double>(0,1);
        double qy = q_wv.at<double>(0,2);
        double qz = q_wv.at<double>(0,3);
        double buf8[8] = { ts, Xw, Yw, Zw, qw, qx, qy, qz };
        sendto(ipc_sock, buf8, 8*sizeof(double), 0, (const struct sockaddr*)&tag_addr, sizeof(sockaddr_un));

        // Compute angles and distance for Python bridge
        float angle_x = 0.0f, angle_y = 0.0f, distance = 0.0f;
        const double eps = 1e-6;
        if (std::fabs(Zb) > eps) {
            angle_x = (float)std::atan2(Xb, Zb);
            angle_y = (float)std::atan2(Yb, Zb);
        } else {
            angle_x = 0.0f;
            angle_y = 0.0f;
        }
        distance = (float)std::sqrt(Xb*Xb + Yb*Yb + Zb*Zb);

        // Send to Python via UDP 127.0.0.1:6000 in text: "ax ay d\n"
        char line[128];
        int n = std::snprintf(line, sizeof(line), "%.6f %.6f %.6f\n", angle_x, angle_y, distance);
        if (n > 0) {
            sendto(bridge_sock, line, n, 0, (const struct sockaddr*)&bridge_addr, sizeof(bridge_addr));
        }

        // Visualization
        if (draw_viz && vis){
            cv::aruco::drawDetectedMarkers(*vis, std::vector<std::vector<cv::Point2f>>{corners[i]}, std::vector<int>{});
            const double fontScale = 0.5;
            const int thickness = 2;
            cv::Point textOrg = corners[i][0] + cv::Point2f(6, -6);
            cv::putText(*vis, std::to_string(ids[i]), textOrg, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(50,50,50), thickness, cv::LINE_AA);
        }

        visTags.push_back({ids[i], cv::Point3d(Xw,Yw,Zw), cv::Point3d(Xb,Yb,Zb)});
    }
}

int main(int argc, char** argv){
    // CLI
    std::string tag_cfg_path;
    bool show_ui = false; // --viz flag controls visualization
    Intrinsics K{};
    std::string intr_yml;
    double fx_cli=0, fy_cli=0, cx_cli=0, cy_cli=0;

    for(int i=1;i<argc;++i){
        std::string a(argv[i]);
        if(a == "--viz"){ show_ui = true; continue; }
        const std::string k_cfg="--tag-config=";
        const std::string k_yaml="--k=";
        const std::string kfx="--fx=", kfy="--fy=", kcx="--cx=", kcy="--cy=";
        if(a.rfind(k_cfg,0)==0){ tag_cfg_path = a.substr(k_cfg.size()); continue; }
        if(a.rfind(k_yaml,0)==0){ intr_yml = a.substr(k_yaml.size()); continue; }
        if(a.rfind(kfx,0)==0){ fx_cli = std::stod(a.substr(kfx.size())); continue; }
        if(a.rfind(kfy,0)==0){ fy_cli = std::stod(a.substr(kfy.size())); continue; }
        if(a.rfind(kcx,0)==0){ cx_cli = std::stod(a.substr(kcx.size())); continue; }
        if(a.rfind(kcy,0)==0){ cy_cli = std::stod(a.substr(kcy.size())); continue; }
    }
    if(tag_cfg_path.empty()){
        if(const char* p = std::getenv("TAG_CONFIG")) tag_cfg_path = p;
    }

    if(!tag_cfg_path.empty()){
        if(!load_tag_config(tag_cfg_path)){
            std::cerr << "Falling back to default tag 42.\n";
        }
    }
    if(g_tags.empty()){
        // Default: example ground tag in NED
        Tag3D t; t.id=42;
        t.world_corners = {
            cv::Point3f( 0.0f,  0.0f, 0.0f),  // p0
            cv::Point3f( 0.0f,  0.1f, 0.0f),  // p1
            cv::Point3f(-0.1f,  0.1f, 0.0f),  // p2
            cv::Point3f(-0.1f,  0.0f, 0.0f)   // p3
        };
        g_tags = {t};
        g_tag_by_id[t.id]=t;
        std::cerr << "Using default ground tag world corners (NED) for ID=42.\n";
    }

    // Signal handler
    struct sigaction act{}; act.sa_handler = [](int){ g_run=false; }; sigaction(SIGINT, &act, nullptr);

    // IPC UNIX socket (optional pose out)
    int ipc_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if(ipc_sock < 0){ perror("socket"); return 1; }
    int buf = 1<<20;
    setsockopt(ipc_sock, SOL_SOCKET, SO_SNDBUF, &buf, sizeof(buf));
    setsockopt(ipc_sock, SOL_SOCKET, SO_RCVBUF, &buf, sizeof(buf));
    struct sockaddr_un local_addr{}, tag_addr{};
    local_addr.sun_family = AF_UNIX;
    strcpy(local_addr.sun_path, "/tmp/beta_aruco_pose_single");
    unlink("/tmp/beta_aruco_pose_single");
    if(bind(ipc_sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0){ perror("bind"); return 1; }
    tag_addr.sun_family = AF_UNIX; strcpy(tag_addr.sun_path, "/tmp/beta_tag");

    // UDP bridge socket to Python (127.0.0.1:6000)
    int bridge_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (bridge_sock < 0) { perror("udp bridge socket"); return 1; }
    int yes = 1;
    setsockopt(bridge_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in bridge_addr{};
    bridge_addr.sin_family = AF_INET;
    bridge_addr.sin_port = htons(6000);
    inet_pton(AF_INET, "127.0.0.1", &bridge_addr.sin_addr);
    std::cout << "Bridge target: UDP 127.0.0.1:6000 (ax ay d)\n";

    // DepthAI pipeline: single mono camera B
    dai::Pipeline pipeline;
    auto monoB   = pipeline.create<dai::node::MonoCamera>();
    auto xoutB   = pipeline.create<dai::node::XLinkOut>();
    xoutB->setStreamName("camB");

    monoB->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoB->setResolution(RESOLUTION_ENUM);
    monoB->setFps(FPS);
    monoB->out.link(xoutB->input);

    dai::Device device(pipeline);
    const int cam_w = monoB->getResolutionWidth();
    const int cam_h = monoB->getResolutionHeight();
    std::cout << "cam_B resolution: " << cam_w << "x" << cam_h << "\n";

    // Intrinsics
    bool haveK = false;
    if(fx_cli>0 && fy_cli>0){
        K.fx = fx_cli; K.fy = fy_cli;
        K.cx = (cx_cli>0?cx_cli:cam_w/2.0);
        K.cy = (cy_cli>0?cy_cli:cam_h/2.0);
        haveK = true;
    } else if(!intr_yml.empty()){
        haveK = load_intrinsics_yaml(intr_yml, K);
    }
    if(!haveK){
        K = estimate_from_fov(cam_w, cam_h);
        std::cerr << "WARN: Using FOV-estimated intrinsics for cam_B: fx=" << K.fx
                  << " fy=" << K.fy << " cx=" << K.cx << " cy=" << K.cy
                  << " (provide --fx/--fy/--cx/--cy or --k=yaml for accuracy)\n";
    } else {
        std::cout << "Using provided intrinsics: fx=" << K.fx
                  << " fy=" << K.fy << " cx=" << K.cx << " cy=" << K.cy << "\n";
    }

    auto qB = device.getOutputQueue("camB", 8, false);

    if (show_ui){
        cv::namedWindow("cam_B", cv::WINDOW_NORMAL);
    }

    uint64_t frameCounter = 0;

    while(g_run){
        auto fB = qB->get<dai::ImgFrame>();
        if(!fB) continue;

        double ts_dev = std::chrono::duration<double>(fB->getTimestampDevice().time_since_epoch()).count();
        double exp_s  = std::chrono::duration<double>(fB->getExposureTime()).count();
        double ts_mid = ts_dev - 0.5 * exp_s;

        const auto& vec = fB->getData();
        const size_t needGRAY = (size_t)cam_w * cam_h;
        cv::Mat gray;

        if(fB->getType() == dai::RawImgFrame::Type::GRAY8 && vec.size() >= needGRAY){
            cv::Mat g(cam_h, cam_w, CV_8UC1, (void*)vec.data());
            g.copyTo(gray);
        } else {
            if(vec.size() >= needGRAY){
                cv::Mat g(cam_h, cam_w, CV_8UC1, (void*)vec.data());
                g.copyTo(gray);
            } else {
                continue;
            }
        }

        frameCounter++;
        // FPS (EMA)
        double tnow = nowSec();
        static double tprev = tnow;
        double dt = tnow - tprev;
        tprev = tnow;
        if(dt > 0.0) g_fps = 0.9*g_fps + 0.1*(1.0/dt);

        cv::Mat vis; // only used if show_ui
        if(show_ui){
            cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
        }

        std::vector<TagPoseVis> visTags;

        if(frameCounter % std::max(1, (int)TAG_DET_STRIDE) == 0){
            detect_and_process(gray, K, ts_mid,
                               ipc_sock, tag_addr,
                               show_ui ? &vis : nullptr, show_ui,
                               visTags,
                               // UDP bridge to Python
                               bridge_sock, bridge_addr);
        }

        if(show_ui){
            // HUD: show Vehicle pos and Landing target deltas
            const double fontScale = 0.7;
            const int thickness = 3;
            const int lineH = 22;
            cv::Scalar textColor(255, 255, 255);
            cv::Scalar textColor_border(0, 0, 0);

            char hud1[160] = "Vehicle pos: X=nan Y=nan Z=nan m";
            char hud2[160] = "Landing target: dX=nan dY=nan dZ=nan m";
            if(!visTags.empty()){
                const auto& t = visTags.front();
                std::snprintf(hud1, sizeof(hud1), "Vehicle pos: X=%.2f Y=%.2f Z=%.2f m", t.pos_world.x, t.pos_world.y, t.pos_world.z);
                std::snprintf(hud2, sizeof(hud2), "Landing target: dX=%.2f dY=%.2f dZ=%.2f m", t.pos_body.x, t.pos_body.y, t.pos_body.z);
            }

            int yBase = vis.rows - 20 - lineH;
            cv::putText(vis, hud1, {10, yBase}, cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor_border, thickness, cv::LINE_AA);
            cv::putText(vis, hud2, {10, yBase + lineH}, cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor_border, thickness, cv::LINE_AA);
            cv::putText(vis, hud1, {10, yBase}, cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, 1, cv::LINE_AA);
            cv::putText(vis, hud2, {10, yBase + lineH}, cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, 1, cv::LINE_AA);

            char fpsTxt[64];
            std::snprintf(fpsTxt, sizeof(fpsTxt), "FPS: %.1f", g_fps);
            cv::putText(vis, fpsTxt, {10, 24}, cv::FONT_HERSHEY_PLAIN, 1.8, cv::Scalar(180,180,180), 2, cv::LINE_AA);

            cv::imshow("cam_B", vis);
            (void)cv::waitKey(1);
        }
    }

    close(ipc_sock);
    close(bridge_sock);
    std::cout << "bye\n";
    return 0;
}
