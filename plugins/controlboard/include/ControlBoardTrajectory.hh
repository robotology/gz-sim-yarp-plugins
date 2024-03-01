#pragma once

#include <gz/sim/Model.hh>

#include <mutex>

namespace yarp
{
namespace dev
{
namespace gzyarp
{

enum TrajectoryType
{
    TRAJECTORY_TYPE_CONST_SPEED = 0,
    TRAJECTORY_TYPE_MIN_JERK = 1,
    TRAJECTORY_TYPE_TRAP_SPEED = 2
};

class Watchdog
{
    double m_duration;
    double m_lastUpdate;

public:
    void reset();
    bool isExpired();
    void modifyDuration(double expireTime);
    double getDuration();

    Watchdog(double expireTime);
};

class RampFilter
{
private:
    std::mutex m_mutex;
    double m_final_reference;
    double m_current_value;
    double m_step;

public:
    RampFilter();

    void setReference(double ref, double step);
    void update();
    double getCurrentValue();
    void stop();
};

class TrajectoryGenerator
{
protected:
    std::mutex m_mutex;
    gz::sim::Model* m_robot;
    bool m_trajectory_complete;
    double m_x0;
    double m_xf;
    double m_speed;
    double m_acceleration;
    double m_computed_reference;
    double m_controllerPeriod;
    double m_joint_min;
    double m_joint_max;
    TrajectoryGenerator(gz::sim::Model* model);

public:
    virtual ~TrajectoryGenerator();
    virtual bool initTrajectory(double current_pos,
                                double final_pos,
                                double speed,
                                double acceleration,
                                double controller_period)
        = 0;
    virtual bool abortTrajectory(double limit) = 0;
    virtual double computeTrajectory() = 0;
    virtual double computeTrajectoryStep() = 0;
    virtual TrajectoryType getTrajectoryType() = 0;
    bool setLimits(double min, double max);
    bool isMotionDone();
};

class ConstSpeedTrajectoryGenerator : public TrajectoryGenerator
{
public:
    ConstSpeedTrajectoryGenerator(gz::sim::Model* model);
    virtual ~ConstSpeedTrajectoryGenerator();

private:
    double p_computeTrajectory();
    double p_computeTrajectoryStep();
    bool p_abortTrajectory(double limit);

public:
    bool initTrajectory(double current_pos,
                        double final_pos,
                        double speed,
                        double acceleration,
                        double controller_period);
    bool abortTrajectory(double limit);
    double computeTrajectory();
    double computeTrajectoryStep();
    TrajectoryType getTrajectoryType();
};

class TrapezoidalSpeedTrajectoryGenerator : public TrajectoryGenerator
{
public:
    TrapezoidalSpeedTrajectoryGenerator(gz::sim::Model* model);
    virtual ~TrapezoidalSpeedTrajectoryGenerator();

private:
    double m_ta;
    double m_tb;
    double m_tf;
    double m_tick;
    double m_v0;
    double m_computed_reference_velocity;

    double p_computeTrajectoryStep();

public:
    bool initTrajectory(double current_pos,
                        double final_pos,
                        double speed,
                        double acceleration,
                        double controller_period);
    bool abortTrajectory(double limit);
    double computeTrajectory();
    double computeTrajectoryStep();
    TrajectoryType getTrajectoryType();
};

class MinJerkTrajectoryGenerator : public TrajectoryGenerator
{
public:
    MinJerkTrajectoryGenerator(gz::sim::Model* model);
    virtual ~MinJerkTrajectoryGenerator();

private:
    double m_trajectory_coeff_c1;
    double m_trajectory_coeff_c2;
    double m_trajectory_coeff_c3;
    double m_dx0;
    double m_tf;
    double m_prev_a;
    double m_cur_t;
    double m_cur_step;
    double m_step;

    double p_computeTrajectory();
    double p_computeTrajectoryStep();
    bool p_abortTrajectory(double limit);

    double p_compute_p5f(double t);
    double p_compute_p5f_vel(double t);
    double p_compute_current_vel();

public:
    bool initTrajectory(double current_pos,
                        double final_pos,
                        double speed,
                        double acceleration,
                        double controller_period);
    bool abortTrajectory(double limit);
    double computeTrajectory();
    double computeTrajectoryStep();
    TrajectoryType getTrajectoryType();
};

} // namespace gzyarp
} // namespace dev
} // namespace yarp
