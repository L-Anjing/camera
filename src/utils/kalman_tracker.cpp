#include "utils/kalman_tracker.hpp"
#include <cmath>
#include <limits>

// ═══════════════════════════════════════════════════════
//  KalmanFilter3D
// ═══════════════════════════════════════════════════════

KalmanFilter3D::KalmanFilter3D()
{
    H_.setZero();
    H_(0,0)=1; H_(1,1)=1; H_(2,2)=1;

    R_ = Eigen::Matrix3f::Identity();
    R_(0,0)=0.01f; R_(1,1)=0.01f; R_(2,2)=0.04f;

    reset_covariance();
    time_since_update=0; age=0;
}

void KalmanFilter3D::reset_covariance()
{
    F_.setIdentity();
    Q_.setZero();
    Q_(0,0)=0.01f; Q_(1,1)=0.01f; Q_(2,2)=0.01f;
    Q_(3,3)=0.05f; Q_(4,4)=0.05f; Q_(5,5)=0.05f;

    P_.setIdentity();
    P_(0,0)=P_(1,1)=P_(2,2)=0.1f;
    P_(3,3)=P_(4,4)=P_(5,5)=1.0f;
}

void KalmanFilter3D::init(const Eigen::Vector3f &pos,
                           const Eigen::Vector3f &vel)
{
    x_.head<3>()=pos; x_.tail<3>()=vel;
    reset_covariance();
    initialized_=true;
    time_since_update=0; age=0;
}

void KalmanFilter3D::predict(float dt, const Eigen::Vector3f *gyro)
{
    if(!initialized_) return;
    F_.setIdentity();
    F_(0,3)=dt; F_(1,4)=dt; F_(2,5)=dt;

    if(gyro)
    {
        float a=gyro->norm()*dt;
        if(a>1e-6f)
            x_.tail<3>() = Eigen::AngleAxisf(a, gyro->normalized()) * x_.tail<3>();
    }

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
    ++time_since_update; ++age;
}

void KalmanFilter3D::update(const Eigen::Vector3f &z)
{
    if(!initialized_) return;
    Eigen::Matrix<float,3,1> y = z - (H_ * x_);
    Eigen::Matrix<float,3,3> S = H_*P_*H_.transpose() + R_;
    Eigen::Matrix<float,6,3> K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    Eigen::Matrix<float,6,6> I = Eigen::Matrix<float,6,6>::Identity() - K*H_;
    P_ = I*P_*I.transpose() + K*R_*K.transpose();
    time_since_update = 0;
}

float KalmanFilter3D::mahalanobis(const Eigen::Vector3f &z) const
{
    if(!initialized_) return std::numeric_limits<float>::max();
    Eigen::Matrix<float,3,1> y = z - (H_*x_);
    Eigen::Matrix<float,3,3> S = H_*P_*H_.transpose() + R_;
    return std::sqrt((y.transpose() * S.inverse() * y)(0));
}

// ═══════════════════════════════════════════════════════
//  TrackManager
// ═══════════════════════════════════════════════════════

TrackManager::TrackManager() {}

int TrackManager::associate(const Eigen::Vector3f &z, float &min_dist) const
{
    int best=-1; min_dist=gate_thresh_;
    for(size_t i=0;i<tracks_.size();++i)
    {
        if(!tracks_[i].kf.initialized()) continue;
        float d=tracks_[i].kf.mahalanobis(z);
        if(d<min_dist){min_dist=d;best=static_cast<int>(i);}
    }
    return best;
}

void TrackManager::create_track(const Eigen::Vector3f &z, int cls, float conf)
{
    Track t; t.id=next_id_++;
    t.kf.init(z); t.kf.class_label=cls; t.kf.confidence=conf;
    tracks_.push_back(t);
}

void TrackManager::maintain_tracks()
{
    auto it=tracks_.begin();
    while(it!=tracks_.end())
    {
        bool del = it->kf.time_since_update > max_age_;
        Eigen::Vector3f p=it->kf.position();
        if(std::abs(p.x())>5||std::abs(p.y())>5||p.z()<-1||p.z()>10) del=true;
        if(del) it=tracks_.erase(it); else ++it;
    }
}

Eigen::Vector3f TrackManager::process(
    const Eigen::Vector3f &z, float dt, int cls, float conf,
    const Eigen::Vector3f *gyro)
{
    for(auto &t:tracks_) t.kf.predict(dt,gyro);

    bool ok=(z.x()!=0||z.y()!=0||z.z()!=0);
    if(ok)
    {
        float d; int idx=associate(z,d);
        if(idx>=0){tracks_[idx].kf.update(z); tracks_[idx].kf.class_label=cls; tracks_[idx].kf.confidence=conf;}
        else create_track(z,cls,conf);
    }

    maintain_tracks();

    Eigen::Vector3f r(0,0,0);
    best_label_=0; best_conf_=0;
    float best_dist=std::numeric_limits<float>::max();
    for(auto &t:tracks_)
    {
        if(t.kf.time_since_update>0) continue;
        float d=t.kf.position().norm();
        if(d<best_dist){best_dist=d; r=t.kf.position(); best_label_=t.kf.class_label; best_conf_=t.kf.confidence;}
    }
    return r;
}
