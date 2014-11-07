#include <iostream>

#include "Simbody.h"
#include "Atlas.h"
#include "TaskSpace.h"

using namespace SimTK;
using namespace std;

//==============================================================================
//                             TASKS MEASURE
//==============================================================================
// This Measure is added to the modelRobot and is used to manage the tasks
// it is supposed to achieve and to return as its value the control torques
// that should be applied to the realRobot (that is, the simulated one).
// This should only be instantiated with T=Vector.
template <class T>
class TasksMeasure : public Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(TasksMeasure, Measure_<T>);

    TasksMeasure(Atlas& modelRobot) 
    :   Measure_<T>(modelRobot.updForceSubsystem(), 
                    new Implementation(modelRobot),
                    AbstractMeasure::SetHandle()) {}


    const Vec3& getTarget() const { return getImpl().m_desiredTaskPosInGround; }
    Vec3& updTarget() { return updImpl().m_desiredTaskPosInGround; }
    void setTarget(Vec3 pos) { updImpl().m_desiredTaskPosInGround = pos; }

    void toggleGravityComp() {
        updImpl().m_compensateForGravity = !isGravityCompensationOn();}
    void togglePoseControl() {
        updImpl().m_controlPose = !isPoseControlOn();}
    void toggleTask() {updImpl().m_controlTask = !getImpl().m_controlTask;}
    void toggleEndEffectorSensing() 
    {   updImpl().m_endEffectorSensing = !getImpl().m_endEffectorSensing;}

    bool isGravityCompensationOn() const 
    {   return getImpl().m_compensateForGravity; }
    bool isPoseControlOn() const 
    {   return getImpl().m_controlPose; }
    bool isEndEffectorSensingOn() const 
    {   return getImpl().m_endEffectorSensing; }
    bool isTaskPointFollowingOn() const
    {   return getImpl().m_controlTask; }
    const Vec3& getTaskPointInEndEffector() const 
    {   return getImpl().m_taskPointInEndEffector; }

    SimTK_MEASURE_HANDLE_POSTSCRIPT(TasksMeasure, Measure_<T>);
};

template <class T>
class TasksMeasure<T>::Implementation : public Measure_<T>::Implementation {
public:
    Implementation(const Atlas& modelRobot,
                   Real proportionalGain=225, double derivativeGain=30) 
                   //Real proportionalGain=100, double derivativeGain=20) 
    :   Measure_<T>::Implementation(T(), 1),
        m_modelRobot(modelRobot),
        m_tspace1(m_modelRobot.getMatterSubsystem(), m_modelRobot.getGravity()),
        m_proportionalGain(proportionalGain),
        m_derivativeGain(derivativeGain),
        m_jointPositionGain(100),
        m_jointDampingGain(20),
        //m_jointPositionGain(225),
        //m_jointDampingGain(30),
        m_compensateForGravity(true),
        m_controlPose(true),
        m_controlTask(false),
        m_endEffectorSensing(false),
        m_desiredTaskPosInGround(Vec3(0.4, -0.1, 1)) // Z is up
    {       
        //TODO: should have end effector body
        m_tspace1.addStationTask(m_modelRobot.getEndEffectorBody(),
                                 m_modelRobot.getEndEffectorStation());
    }


    // Default copy constructor, destructor, copy assignment are fine.

    // Implementations of virtual methods.
    Implementation* cloneVirtual() const OVERRIDE_11
    {   return new Implementation(*this); }
    int getNumTimeDerivativesVirtual() const OVERRIDE_11 {return 0;}
    Stage getDependsOnStageVirtual(int order) const OVERRIDE_11
    {   return Stage::Velocity; }

    // This is the task space controller. It returns joint torques tau as the
    // value of the enclosing Measure.
    void calcCachedValueVirtual(const State& s, int derivOrder, T& tau) const
        OVERRIDE_11;

    // TaskSpace objects require some State resources; this call is the time
    // for doing that so forward on to the TaskSpace.
    void realizeMeasureTopologyVirtual(State& modelState) const OVERRIDE_11 {
        m_tspace1.realizeTopology(modelState);
    }
private:
friend class TasksMeasure<T>;

    const Atlas&    m_modelRobot;
    TaskSpace       m_tspace1;

    const Real      m_proportionalGain;     // task space
    const Real      m_derivativeGain;
    const Real      m_jointPositionGain;    // joint space
    const Real      m_jointDampingGain;

    bool            m_compensateForGravity;
    bool            m_controlPose;
    bool            m_controlTask;
    bool            m_endEffectorSensing;
    Vec3            m_desiredTaskPosInGround;
};

//------------------------------------------------------------------------------
//                TASKS MEASURE :: CALC CACHED VALUE VIRTUAL
//------------------------------------------------------------------------------
// Given a modelState that has been updated from the real robot's sensors, 
// generate control torques as the TasksMeasure's value. This is the only part
// of the code that is actually doing task space operations.

template <class T>
void TasksMeasure<T>::Implementation::calcCachedValueVirtual
   (const State& modelState, int derivOrder, T& tau) const
{
    SimTK_ASSERT1_ALWAYS(derivOrder==0,
        "TasksMeasure::Implementation::calcCachedValueVirtual():"
        " derivOrder %d seen but only 0 allowed.", derivOrder);

    // Shorthands.
    // -----------
    const State& ms = modelState;
    const TaskSpace& p1 = m_tspace1;

    const int mnq = ms.getNQ();
    const int mnu = ms.getNU();
    tau.resize(mnu);

    const Real& kd = m_derivativeGain;
    const Real& kp = m_proportionalGain;

    // The desired task position is in Ground. We need instead to measure it
    // from the real robot's pelvis origin so that we can translate it into the 
    // model's pelvis-centric viewpoint.
    const Transform& X_GP   = m_modelRobot.getSampledPelvisPose(ms);
    const Vec3 x1_des = ~X_GP*m_desiredTaskPosInGround; // measure in P


    // Compute control law in task space (F*).
    // ---------------------------------------
    Vec3 xd_des(0);
    Vec3 xdd_des(0);

    // Get the model's estimate of the end effector location in Ground, which
    // is also the pelvis origin.
    Vec3 x1, x1d;
    p1.findStationLocationAndVelocityInGround(ms,
            TaskSpace::StationTaskIndex(0),
            m_modelRobot.getEndEffectorStation(), x1, x1d);

    if (m_endEffectorSensing) {
        // Since the controller model has the pelvis origin fixed at (0,0,0),
        // we need to know the real robot's pelvis location so we can measure
        // the real robot's end effector from its pelvis location. We don't
        // have to modify x1d because we want the end effector stationary
        // in Ground, not in the pelvis.
        const Vec3& x1_G = m_modelRobot.getSampledEndEffectorPos(ms);
        x1 = ~X_GP*x1_G; // measure end effector in pelvis frame
    }

    // Units of acceleration.
    Vec3 Fstar1 = xdd_des + kd * (xd_des - x1d) + kp * (x1_des - x1);

    // Compute task-space force that achieves the task-space control.
    // F = Lambda Fstar + p
    Vector F1 = p1.Lambda(ms) * Fstar1 + p1.mu(ms) + p1.p(ms);
    //Vector F2 = p2.calcInverseDynamics(ms, Fstar2);

    // Combine the reaching task with the gravity compensation and pose 
    // control to a neutral q=0 pose with u=0 also.
    const Vector& q = ms.getQ();
    const Vector& u = ms.getU();
    const Real k = m_jointPositionGain;
    const Real c = m_jointDampingGain;
    Vector Mu(mnu), Mq(mnu);
    m_modelRobot.getMatterSubsystem().multiplyByM(ms, u, Mu);
    m_modelRobot.getMatterSubsystem().multiplyByM(ms, q, Mq);

    tau.setToZero();
    const Real gFac = m_compensateForGravity?1.:0.;
    const Real pFac = m_controlPose?1.:0.;
    if (m_controlTask) {
        tau += p1.JT(ms) * F1;
        tau += p1.NT(ms) * (gFac*p1.g(ms) - pFac*k*Mq - c*Mu); // damping always
    } else 
        tau += gFac*p1.g(ms) - (pFac*k*Mq + c*Mu);

    // Cut tau back to within effort limits.
    // TODO: can't use these limits with one-foot support!
    const Vector& effortLimits = m_modelRobot.getEffortLimits();
    for (int i=0; i < mnu; ++i) {
        const Real oldtau = tau[i], effort = 10*effortLimits[i]; // cheating
        if (std::abs(oldtau) <= effort) continue;
        const Real newtau = clamp(-effort, oldtau, effort);
        //printf("Limit tau[%d]: %g -> %g\n", i, oldtau, newtau);
        tau[i] = newtau;
    }
}
