#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tf/transform_listener.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

class SimpleEffortControl: public ControlEffort
{
    // Member variables
private:

    bool m_startFlag;

    Vector6d m_qStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    ros::NodeHandle n;
    ros::Publisher pubCtrlData;

    Matrix6d m_Kp;
    Matrix6d m_Kd;
    Matrix6d m_Ki;
    Vector6d m_goal;
    double m_totalTime;

    Vector6d m_DeltaQ;
    Vector6d m_DeltaQ_1;

    Vector6d m_DeltaQp;
    Vector6d m_DeltaQp_1;

    Vector6d m_iDeltaQ;
    Vector6d m_iDeltaQ_1;


    double m_controlPeriod;     //[s]
    double m_controlPeriod_2;   //[s]

    //--- Final2 -----
    Vector6d d_m_goal;  // one step: d_m_goal, m_goal should interpolate again?!
    int m_goal_cnt;  //number of new goal
    tf::TransformListener listener;
       //--- UR10 Para.   
       const float d1=0.118;
       const float a2=-0.6121;
       const float a3=-0.5716;
       const float d4=0.1639;
       const float d5=0.1157;
       const float d6=0.0922;  

    // Member Methods
public:
    SimpleEffortControl(double weight=1.0,
                        const QString& name="SimpleEffortCtrl");
    ~SimpleEffortControl();

    void setQInit(const JointState& qinit);
    void setQHome(const JointState& qhome);
    void setQPark(const JointState& qpark);
    

    // Memeber Methods
private:
    bool init();
    bool start();
    Vector6d update(const RobotTime& time, const JointState &current);
    bool stop();

    // ----- Final2 -----
    Vector6d UpdateDMGoal(Vector6d &m_goal, Vector6d &d_m_goal);
    bool UpdateMGoal(Vector6d &m_goal, double &m_totalTime);
    Matrix4d dh_fcn(double a, double alpha, double d, double theta);

};

}
}



#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
