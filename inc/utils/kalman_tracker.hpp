#pragma once

#include <Eigen/Dense>
#include <vector>

// ── 6 状态卡尔曼滤波器 ─────────────────────────────
class KalmanFilter3D
{
public:
    KalmanFilter3D();

    void init(const Eigen::Vector3f &pos,
              const Eigen::Vector3f &vel = Eigen::Vector3f::Zero());
    void predict(float dt, const Eigen::Vector3f *gyro = nullptr);
    void update(const Eigen::Vector3f &measurement);

    Eigen::Vector3f position() const { return x_.head<3>(); }
    bool initialized() const { return initialized_; }
    float mahalanobis(const Eigen::Vector3f &z) const;

    int time_since_update = 0;
    int age = 0;
    int class_label = 0;
    float confidence = 0.0f;

private:
    bool initialized_ = false;
    Eigen::Matrix<float, 6, 1> x_;          // [cx, cy, cz, vx, vy, vz]
    Eigen::Matrix<float, 6, 6> P_, F_, Q_;
    Eigen::Matrix<float, 3, 6> H_;
    Eigen::Matrix<float, 3, 3> R_;
    void reset_covariance();
};

// ── 航迹管理器 ──────────────────────────────────────
class TrackManager
{
public:
    TrackManager();

    Eigen::Vector3f process(
        const Eigen::Vector3f &measurement,
        float dt, int cls, float conf,
        const Eigen::Vector3f *gyro = nullptr);

    int best_class_label() const { return best_label_; }
    float best_confidence() const { return best_conf_; }

    void set_max_age(int f) { max_age_ = f; }
    void set_gating_threshold(float t) { gate_thresh_ = t; }
    int track_count() const { return static_cast<int>(tracks_.size()); }

private:
    struct Track { KalmanFilter3D kf; int id; };
    std::vector<Track> tracks_;
    int next_id_ = 1;

    int max_age_ = 3;
    float gate_thresh_ = 3.0f;

    int best_label_ = 0;
    float best_conf_ = 0.0f;

    int associate(const Eigen::Vector3f &z, float &min_dist) const;
    void create_track(const Eigen::Vector3f &z, int cls, float conf);
    void maintain_tracks();
};
