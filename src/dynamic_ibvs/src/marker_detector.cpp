#include "dynamic_ibvs/marker_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <cstring>  // memset / memcpy

// ─────────────────────────────────────────────────────────────────────────────
//  Kalman filter tuning constants
// ─────────────────────────────────────────────────────────────────────────────

// Process noise — how much we trust the constant-velocity motion model.
// Increase if the marker accelerates quickly; decrease for smooth motion.
static const double Q_POS   = 0.01;   // position process noise variance
static const double Q_VEL   = 0.5;    // velocity process noise variance
static const double Q_AREA  = 0.02;
static const double Q_VAREA = 0.5;
static const double Q_ANG   = 0.01;
static const double Q_VANG  = 0.3;

// Measurement noise — how much we trust raw ArUco detections.
static const double R_POS  = 0.002;   // centroid measurement noise variance
static const double R_AREA = 0.005;
static const double R_ANG  = 0.01;

// ─────────────────────────────────────────────────────────────────────────────
MarkerDetector::MarkerDetector(ros::NodeHandle *nh)
    : it_(*nh),
      camera_f_x_(0), camera_f_y_(0),
      camera_u_0_(0), camera_v_0_(0),
      camera_info_received_(false)
{
    fiducial_vertices_sub_ = nh->subscribe(
        "/fiducial_vertices", 10,
        &MarkerDetector::fiducialVerticesCallback, this);

    fiducial_transforms_sub_ = nh->subscribe(
        "/fiducial_transforms", 10,
        &MarkerDetector::fiducialTransformsCallback, this);

    camera_info_sub_ = nh->subscribe(
        "/iris_downward_depth_camera/camera/rgb/camera_info", 1,
        &MarkerDetector::cameraInfoCallback, this);

    image_sub_ = it_.subscribe(
        "/iris_downward_depth_camera/camera/rgb/image_raw", 2,
        &MarkerDetector::imageCallback, this);

    image_pub_ = it_.advertise("platform_ibvs/output_image", 2);

    ROS_INFO("MarkerDetector: initialised with Kalman tracking.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public accessors
// ─────────────────────────────────────────────────────────────────────────────

std::shared_ptr<MarkerDetector::MarkerData>
MarkerDetector::getMarkerById(int id) const
{
    for (const auto &m : detected_markers_)
        if (m && m->id == id) return m;
    return nullptr;
}

std::vector<std::shared_ptr<MarkerDetector::MarkerData>>
MarkerDetector::getAllMarkers() const { return detected_markers_; }

bool MarkerDetector::isMarkerDetected(int id) const
{ return getMarkerById(id) != nullptr; }

IBVSController::MarkerVelocity
MarkerDetector::getMarkerVelocity(int id) const
{
    auto it = kalman_filters_.find(id);
    if (it == kalman_filters_.end() || !it->second.initialised)
        return IBVSController::MarkerVelocity{};

    const MarkerKF &kf = it->second;
    IBVSController::MarkerVelocity mv;
    mv.vx_centroid = static_cast<float>(kf.vx());
    mv.vy_centroid = static_cast<float>(kf.vy());
    mv.v_area      = static_cast<float>(kf.varea());
    mv.v_alpha     = static_cast<float>(kf.valpha());
    return mv;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ROS callbacks
// ─────────────────────────────────────────────────────────────────────────────

void MarkerDetector::fiducialVerticesCallback(
    const fiducial_msgs::FiducialArray::ConstPtr &msg)
{
    detected_markers_.clear();

    for (const auto &fid : msg->fiducials)
    {
        auto m = std::make_shared<MarkerData>();
        m->id = fid.fiducial_id;
        m->x0 = fid.x0; m->y0 = fid.y0;
        m->x1 = fid.x1; m->y1 = fid.y1;
        m->x2 = fid.x2; m->y2 = fid.y2;
        m->x3 = fid.x3; m->y3 = fid.y3;
        m->stamp = msg->header.stamp;
        detected_markers_.push_back(m);

        // Feed Kalman filter only when we have camera intrinsics
        if (camera_info_received_)
        {
            float cx    = computeCentroid_x(*m);
            float cy    = computeCentroid_y(*m);
            float area  = computeNormalisedArea(*m, camera_f_x_, camera_f_y_);
            float alpha = computeAlpha(*m);
            updateKalman(m->id, cx, cy, area, alpha, m->stamp);
        }
    }
}

void MarkerDetector::fiducialTransformsCallback(
    const fiducial_msgs::FiducialTransformArray::ConstPtr &) {}

void MarkerDetector::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    if (!camera_info_received_)
    {
        camera_f_x_ = msg->K[0];
        camera_f_y_ = msg->K[4];
        camera_u_0_ = msg->K[2];
        camera_v_0_ = msg->K[5];
        camera_info_received_ = true;
        ROS_INFO("MarkerDetector: camera info — fx:%.2f fy:%.2f u0:%.2f v0:%.2f",
                 camera_f_x_, camera_f_y_, camera_u_0_, camera_v_0_);
    }
}

void MarkerDetector::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    try
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        for (const auto &m : detected_markers_)
        {
            if (!m) continue;
            cv::Point p0(m->x0, m->y0), p1(m->x1, m->y1),
                      p2(m->x2, m->y2), p3(m->x3, m->y3);

            cv::circle(cv_ptr->image, p0, 5, {0,255,0}, -1);
            cv::circle(cv_ptr->image, p1, 5, {0,255,0}, -1);
            cv::circle(cv_ptr->image, p2, 5, {0,255,0}, -1);
            cv::circle(cv_ptr->image, p3, 5, {0,255,0}, -1);
            cv::line(cv_ptr->image, p0, p1, {255,0,0}, 2);
            cv::line(cv_ptr->image, p1, p2, {255,0,0}, 2);
            cv::line(cv_ptr->image, p2, p3, {255,0,0}, 2);
            cv::line(cv_ptr->image, p3, p0, {255,0,0}, 2);

            // Draw Kalman-predicted centroid (yellow cross)
            auto it = kalman_filters_.find(m->id);
            if (it != kalman_filters_.end() && it->second.initialised)
            {
                const MarkerKF &kf = it->second;
                int px = static_cast<int>(kf.cx() * camera_f_x_ + camera_u_0_);
                int py = static_cast<int>(kf.cy() * camera_f_y_ + camera_v_0_);
                cv::drawMarker(cv_ptr->image, {px, py},
                               {0, 255, 255}, cv::MARKER_CROSS, 15, 2);
            }

            cv::putText(cv_ptr->image, "ID:" + std::to_string(m->id),
                        p0, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255}, 2);
        }

        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception &e)
    { ROS_ERROR("MarkerDetector: cv_bridge: %s", e.what()); }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Geometry helpers
// ─────────────────────────────────────────────────────────────────────────────

float MarkerDetector::computeCentroid_x(const MarkerData &m)
{ return (m.x0 + m.x1 + m.x2 + m.x3) * 0.25f; }

float MarkerDetector::computeCentroid_y(const MarkerData &m)
{ return (m.y0 + m.y1 + m.y2 + m.y3) * 0.25f; }

float MarkerDetector::computeNormalisedArea(const MarkerData &m,
                                            float fx, float fy)
{
    // Shoelace formula on the 4 corners → pixel area → normalise by f²
    float ax = m.x0*(m.y1-m.y3) + m.x1*(m.y2-m.y0)
             + m.x2*(m.y3-m.y1) + m.x3*(m.y0-m.y2);
    float pixel_area = 0.5f * std::abs(ax);
    return pixel_area / (fx * fy);
}

float MarkerDetector::computeAlpha(const MarkerData &m)
{
    // Angle of the diagonal corner0→corner2 (consistent with ibvs_controller)
    return std::atan2(m.y2 - m.y0, m.x2 - m.x0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Kalman filter — constant-velocity model, 8 states, 4 measurements
//
//  State:        x = [cx  vx  cy  vy  area  varea  alpha  valpha]ᵀ
//  Measurement:  z = [cx      cy      area         alpha        ]ᵀ  (4×1)
//
//  All arithmetic is done in plain C arrays (8×8 row-major) to avoid pulling
//  Armadillo into a header-only context.  The math is straightforward:
//    Predict:  x = F·x,  P = F·P·Fᵀ + Q
//    Update:   K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
//              x = x + K·(z - H·x)
//              P = (I - K·H)·P
// ─────────────────────────────────────────────────────────────────────────────

static void mat8x8_mul(const double A[64], const double B[64], double C[64])
{
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
        {
            double s = 0;
            for (int k = 0; k < 8; ++k) s += A[i*8+k] * B[k*8+j];
            C[i*8+j] = s;
        }
}

static void mat8x8_transpose(const double A[64], double At[64])
{
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            At[j*8+i] = A[i*8+j];
}

// Multiply 8×8 by 8×4 → 8×4
static void mat8x4_mul(const double A[64], const double B[32], double C[32])
{
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 4; ++j)
        {
            double s = 0;
            for (int k = 0; k < 8; ++k) s += A[i*8+k] * B[k*4+j];
            C[i*4+j] = s;
        }
}

// Multiply 4×8 by 8×4 → 4×4
static void mat4x4_from_4x8_8x4(const double A[32], const double B[32], double C[16])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            double s = 0;
            for (int k = 0; k < 8; ++k) s += A[i*8+k] * B[k*4+j];
            C[i*4+j] = s;
        }
}

// Invert 4×4 via Gauss-Jordan
static bool invert4x4(const double M[16], double inv[16])
{
    double aug[4][8];
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j) aug[i][j]   = M[i*4+j];
        for (int j = 0; j < 4; ++j) aug[i][j+4] = (i==j) ? 1.0 : 0.0;
    }
    for (int col = 0; col < 4; ++col)
    {
        int pivot = col;
        for (int r = col+1; r < 4; ++r)
            if (std::abs(aug[r][col]) > std::abs(aug[pivot][col])) pivot = r;
        if (std::abs(aug[pivot][col]) < 1e-12) return false;
        std::swap(aug[col], aug[pivot]);  // swap rows (std::array would work but raw ok)
        // Actually swap by value since it's a raw 2D array
        if (pivot != col)
            for (int j = 0; j < 8; ++j) std::swap(aug[col][j], aug[pivot][j]);
        double scale = aug[col][col];
        for (int j = 0; j < 8; ++j) aug[col][j] /= scale;
        for (int r = 0; r < 4; ++r)
            if (r != col)
            {
                double f = aug[r][col];
                for (int j = 0; j < 8; ++j) aug[r][j] -= f * aug[col][j];
            }
    }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            inv[i*4+j] = aug[i][j+4];
    return true;
}

// ── Predict step ─────────────────────────────────────────────────────────────
void MarkerDetector::kfPredict(MarkerKF &kf, double dt)
{
    // F — constant velocity transition matrix (8×8)
    // [1 dt  0  0  0  0  0  0]
    // [0  1  0  0  0  0  0  0]
    // [0  0  1 dt  0  0  0  0]
    // ...etc. (paired blocks)
    double F[64] = {};
    for (int i = 0; i < 8; ++i) F[i*8+i] = 1.0;  // identity
    F[0*8+1] = dt;  // cx  += vx  * dt
    F[2*8+3] = dt;  // cy  += vy  * dt
    F[4*8+5] = dt;  // area+= v_area * dt
    F[6*8+7] = dt;  // alpha += v_alpha * dt

    // x = F * x
    double xn[8] = {};
    for (int i = 0; i < 8; ++i)
        for (int k = 0; k < 8; ++k) xn[i] += F[i*8+k] * kf.x[k];
    std::memcpy(kf.x, xn, sizeof(xn));

    // P = F * P * Fᵀ + Q
    double FP[64], FPFt[64], Ft[64];
    mat8x8_transpose(F, Ft);
    mat8x8_mul(F,  kf.P, FP);
    mat8x8_mul(FP, Ft,  FPFt);

    // Add process noise Q (diagonal)
    double dt2 = dt * dt;
    FPFt[0*8+0] += Q_POS   * dt2;
    FPFt[1*8+1] += Q_VEL   * dt2;
    FPFt[2*8+2] += Q_POS   * dt2;
    FPFt[3*8+3] += Q_VEL   * dt2;
    FPFt[4*8+4] += Q_AREA  * dt2;
    FPFt[5*8+5] += Q_VAREA * dt2;
    FPFt[6*8+6] += Q_ANG   * dt2;
    FPFt[7*8+7] += Q_VANG  * dt2;

    std::memcpy(kf.P, FPFt, sizeof(FPFt));
}

// ── Update step ──────────────────────────────────────────────────────────────
void MarkerDetector::kfUpdate(MarkerKF &kf,
                               double cx_m, double cy_m,
                               double area_m, double alpha_m)
{
    // H — measurement matrix (4×8): picks [cx, cy, area, alpha] from state
    // row0: state[0]=cx    → H[0][0]=1
    // row1: state[2]=cy    → H[1][2]=1
    // row2: state[4]=area  → H[2][4]=1
    // row3: state[6]=alpha → H[3][6]=1
    double H[32] = {};
    H[0*8+0] = 1.0;
    H[1*8+2] = 1.0;
    H[2*8+4] = 1.0;
    H[3*8+6] = 1.0;

    // S = H * P * Hᵀ + R  (4×4)
    // PHt = P * Hᵀ  (8×4)
    double Ht[32] = {};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 8; ++j) Ht[j*4+i] = H[i*8+j];

    double PHt[32];
    mat8x4_mul(kf.P, Ht, PHt);

    double S[16];
    mat4x4_from_4x8_8x4(H, PHt, S);

    // Add measurement noise R (diagonal)
    S[0*4+0] += R_POS;
    S[1*4+1] += R_POS;
    S[2*4+2] += R_AREA;
    S[3*4+3] += R_ANG;

    // K = PHt * S⁻¹  (8×4)
    double Sinv[16];
    if (!invert4x4(S, Sinv))
    {
        ROS_WARN_THROTTLE(1.0, "MarkerDetector: KF S matrix singular, skipping update.");
        return;
    }

    // K (8×4): K = PHt (8×4) * Sinv (4×4)
    double K[32] = {};
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                K[i*4+j] += PHt[i*4+k] * Sinv[k*4+j];

    // Innovation y = z - H*x  (4×1)
    double z[4] = { cx_m, cy_m, area_m, alpha_m };
    double Hx[4] = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 8; ++j) Hx[i] += H[i*8+j] * kf.x[j];
    double y[4] = { z[0]-Hx[0], z[1]-Hx[1], z[2]-Hx[2], z[3]-Hx[3] };

    // x = x + K * y
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 4; ++j) kf.x[i] += K[i*4+j] * y[j];

    // P = (I - K*H) * P
    double KH[64] = {};
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            for (int k = 0; k < 4; ++k) KH[i*8+j] += K[i*4+k] * H[k*8+j];

    double IKH[64] = {};
    for (int i = 0; i < 8; ++i) IKH[i*8+i] = 1.0;
    for (int i = 0; i < 64; ++i) IKH[i] -= KH[i];

    double newP[64];
    mat8x8_mul(IKH, kf.P, newP);
    std::memcpy(kf.P, newP, sizeof(newP));
}

// ── Top-level Kalman update called from the detection callback ────────────────
void MarkerDetector::updateKalman(int id,
                                   float cx_meas, float cy_meas,
                                   float area_meas, float alpha_meas,
                                   ros::Time stamp)
{
    MarkerKF &kf = kalman_filters_[id];

    if (!kf.initialised)
    {
        // First detection: initialise state from measurement, zero velocities
        std::memset(kf.P, 0, sizeof(kf.P));
        kf.x[0] = cx_meas;   kf.x[1] = 0.0;       // cx, vx
        kf.x[2] = cy_meas;   kf.x[3] = 0.0;       // cy, vy
        kf.x[4] = area_meas; kf.x[5] = 0.0;       // area, v_area
        kf.x[6] = alpha_meas;kf.x[7] = 0.0;       // alpha, v_alpha

        // Initial covariance — high uncertainty on velocities
        kf.P[0*8+0]  = R_POS;   kf.P[1*8+1]  = 1.0;
        kf.P[2*8+2]  = R_POS;   kf.P[3*8+3]  = 1.0;
        kf.P[4*8+4]  = R_AREA;  kf.P[5*8+5]  = 1.0;
        kf.P[6*8+6]  = R_ANG;   kf.P[7*8+7]  = 1.0;

        kf.last_update  = stamp;
        kf.initialised  = true;
        return;
    }

    double dt = (stamp - kf.last_update).toSec();
    if (dt <= 0.0 || dt > 1.0)
    {
        // Stale or backwards timestamp — just reinitialise
        kf.initialised = false;
        return;
    }

    kfPredict(kf, dt);
    kfUpdate (kf, cx_meas, cy_meas, area_meas, alpha_meas);
    kf.last_update = stamp;

    ROS_DEBUG("MarkerDetector KF[%d]: cx=%.4f vx=%.4f  cy=%.4f vy=%.4f  "
              "area=%.4f varea=%.4f  alpha=%.4f valpha=%.4f",
              id,
              kf.cx(), kf.vx(), kf.cy(), kf.vy(),
              kf.area(), kf.varea(), kf.alpha(), kf.valpha());
}
