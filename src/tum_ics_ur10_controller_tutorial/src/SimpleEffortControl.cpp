#include<tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SimpleEffortControl::SimpleEffortControl(double weight, const QString &name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE,weight),
    m_startFlag(false),
    m_Kp(Matrix6d::Zero()),
    m_Kd(Matrix6d::Zero()),
    m_Ki(Matrix6d::Zero()),
    m_goal(Vector6d::Zero()),
    m_totalTime(100.0),
    m_DeltaQ(Vector6d::Zero()),
    m_DeltaQ_1(Vector6d::Zero()),
    m_DeltaQp(Vector6d::Zero()),
    m_DeltaQp_1(Vector6d::Zero()),
    m_iDeltaQ(Vector6d::Zero()),
    m_iDeltaQ_1(Vector6d::Zero()),
    d_m_goal(Vector6d::Zero()),
    m_goal_cnt(0)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("SimpleEffortCtrlData",100);

    m_controlPeriod=0.002; //set the control period to the standard 2 ms

    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("SimpleEffortCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

}

SimpleEffortControl::~SimpleEffortControl()
{

}

void SimpleEffortControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SimpleEffortControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SimpleEffortControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool SimpleEffortControl::init()
{
    std::string ns="~simple_effort_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SimpleEffortControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }


    VDouble p;



    /////D GAINS

    s<<ns<<"/gains_d";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of d_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kd: \n"<<m_Kd);

    /////P GAINS
    s.str("");
    s<<ns<<"/gains_p";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kp(i,i)=p[i]/m_Kd(i,i);
    }

    ROS_WARN_STREAM("Kp: \n"<<m_Kp);


    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of joint goals --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_goal(i)=p[i];
    }
    m_totalTime=p[STD_DOF];

    if(!(m_totalTime>0))
    {
        m_totalTime=100.0;
    }

    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());
    ROS_WARN_STREAM("Total Time [s]: "<<m_totalTime);


    m_goal=DEG2RAD(m_goal);

    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());


}
bool SimpleEffortControl::start()
{

}
Vector6d SimpleEffortControl::update(const RobotTime &time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;
        m_startFlag=true;
    }


    Vector6d tau;

    tau.setZero();
    bool newTar;
    // --- Update to new goal ---
    ///if( (m_goal-current.q).squaredNorm()<0.2 )  // Reach the m_goal, update to new m_goal
    if(UpdateMGoal(m_goal, m_totalTime))
    {
        m_qStart=current.q;
        m_goal_cnt += 1;
        ROS_WARN_STREAM("newTar: " << newTar);
    }

    UpdateDMGoal(m_goal, d_m_goal);


    ROS_INFO_STREAM("m_goal_cnt: " << m_goal_cnt);
    ROS_INFO_STREAM("m_Goal [RAD]:\n"<<m_goal.transpose());
    ROS_INFO_STREAM("d_m_Goal [RAD]:\n"<<d_m_goal.transpose());    
    ROS_INFO_STREAM("current.q:\n"<<current.q.transpose()); 

    VVector6d vQd;
    vQd=getJointPVT5(m_qStart,d_m_goal,time.tD(),m_totalTime);
    

    m_DeltaQ=current.q-vQd[0];
    m_DeltaQp=current.qp-vQd[1];

    //ROS_WARN_STREAM("current.q:\n"<<current.q.transpose()); 
    //ROS_INFO_STREAM("(d_m_goal-current.q).squaredNorm:\n"<<(d_m_goal-current.q).squaredNorm());
    //ROS_INFO_STREAM("(d_m_goal-current.q):\n"<<(d_m_goal-current.q) ); 

    JointState js_r;

    js_r=current;
    js_r.qp=vQd[1]-m_Kp*m_DeltaQ;
    js_r.qpp=vQd[2]-m_Kp*m_DeltaQp;

    Vector6d Sq=current.qp-js_r.qp;


    tau=-m_Kd*Sq;

    /*for(int i=0; i<6; i++){
      if(tau(i)>50)
        tau(i)=50;  
      else if(tau(i)<-100)
        tau(i)=-100; 
    }*/

    //ROS_INFO_STREAM("tau:\n"<<tau.transpose());

    tum_ics_ur_robot_msgs::ControlData msg;

    msg.header.stamp=ros::Time::now();

    msg.time=time.tD();

    for(int i=0;i<STD_DOF;i++)
    {
        msg.q[i]=current.q(i);
        msg.qp[i]=current.qp(i);
        msg.qpp[i]=current.qpp(i);

        msg.qd[i]=vQd[0](i);
        msg.qpd[i]=vQd[1](i);

        msg.Dq[i]=m_DeltaQ(i);
        msg.Dqp[i]=m_DeltaQp(i);

        msg.torques[i]=current.tau(i);
    }

    pubCtrlData.publish(msg);




    //***** Testing dh_fcn ***** FK!
    Matrix4d t0_1;
    Matrix4d t1_2;
    Matrix4d t2_3;
    Matrix4d t3_4;
    Matrix4d t4_5;
    Matrix4d t5_6;
    Matrix4d t0_6;
    t0_1 = dh_fcn(0, DEG2RAD(90), d1, current.q(0));
    t1_2 = dh_fcn(a2,0          ,  0, current.q(1));
    t2_3 = dh_fcn(a3,0          ,  0, current.q(2));
    t3_4 = dh_fcn(0, DEG2RAD(90), d4, current.q(3));
    t4_5 = dh_fcn(0,-DEG2RAD(90), d5, current.q(4));
    t5_6 = dh_fcn(0, 0          , d6, current.q(5));
    t0_6 = t0_1*t1_2*t2_3*t3_4*t4_5*t5_6;

    /*Matrix4d RotXY;
    RotXY << -1, 0, 0, 0,
              0,-1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    t0_6 = RotXY * t0_6 ;*/
     
    
    ROS_INFO_STREAM("EE - t0_6: \n" << t0_6 );
    //ROS_INFO_STREAM("EE - t_6's Quaternion: \n" << (QuaternionBase(t0_6.block(0, 0, 3, 3))) );

    //    tau.setZero();

    return tau;

}
bool SimpleEffortControl::stop()
{

}

Vector6d SimpleEffortControl::UpdateDMGoal(Vector6d &m_goal, Vector6d &d_m_goal)
{
     //===== [Rad] =====
     //----- my Interpolation -----
     Vector6d step;  //step_size 
     step << 0.001, 0.0001, 0.0008, 0.0001 ,0.0008 ,0.001;
     Vector6d thres;  // 
     thres << 0.0001, 0.15, 0.07, 0.007, 0.02, 0.0001;     
     
     ///d_m_goal << d_m_goal.array()+x.array();//0, -90, 0, -90, 0, 0;*/
    
     double diff;  //tmp
     
     for(int i=0; i<6; i++)
     {
        diff = m_goal(i) - d_m_goal(i);       
        if( fabs(diff) > step(i) ){
          if(diff>0)
            d_m_goal(i) = d_m_goal(i) + step(i);
          else
            d_m_goal(i) = d_m_goal(i) - step(i);
        }
        else if( fabs(diff) > thres(i) ){  // prevent vibration
          d_m_goal(i) = d_m_goal(i) + diff;
        }

     }
     //ROS_INFO_STREAM("@ m_goal\n" << m_goal.transpose());
     //m_goal << 0, -90, 0, -90, 0, 0;     
     //m_goal=DEG2RAD(m_goal);
    
     return m_goal;
}

bool SimpleEffortControl::UpdateMGoal(Vector6d &m_goal, double &m_totalTime)
{
     m_totalTime = 10;

     bool isSuccess = true;
     //***** todo:
     //  1. subscibe targetPoint from kinect [ROS/image_converter] 
     //   -> position only?  + auto determin Orientaiton
     //  2. IK
     //m_goal << 0, -90, 0, -90, 0, 0;
     Vector6d q;  
     q << 0, 0, 0, 0, 0, 0;  //Joint1~6: q(0~5)

     //--- 1. Sub to ROS, get tarPos
        //tarX, tarY, tarZ
      double tarX = 0.57389;
      double tarY = 0.60968;
      double tarZ = -0.27996;

      ///Quaterniond tarQ(0.41748, 0.24095, 0.82717, 0.28888);
      ///Matrix3d tarR = tarQ.toRotationMatrix();
      ///ROS_WARN_STREAM(" tarR \n" <<  tarR );

      tf::StampedTransform tarTF;

      try{
        listener.lookupTransform("/base_link", "/target_point", ros::Time(0), tarTF);
        //listener.waitForTransform("target_point", "/base_link", ros::Time(0), ros::Duration(3.0));

        tarX = tarTF.getOrigin().x();
        tarY = tarTF.getOrigin().y();
        tarZ = tarTF.getOrigin().z();


      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
 	
     //--- 2. IK      
       //--- Determin Pos & Ori.
       Matrix4d T0_6;
       T0_6 <<  0, -1, 0, tarX,
                1,  0, 0, tarY,
                0,  0, 1, tarZ,
                0,  0, 0,  1  ;  
 

       Matrix4d RotXY;
       RotXY << -1, 0, 0, 0,
                 0,-1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
        
       ///T0_6.block(0, 0, 3, 3) = tarR;  

       //ROS_WARN_STREAM(" T0_6 _0 \n" <<  T0_6 );  

       T0_6 = RotXY * T0_6 ;  //?? front or back 

       ROS_WARN_STREAM(" T0_6 \n" <<  T0_6 );

       Vector4d P0_5;
       Vector4d tmp_v4_1;
       Vector4d tmp_v4_2;
       tmp_v4_1 << 0, 0, -d6, 1;
       tmp_v4_2 << 0, 0, 0, 1;
       P0_5 = T0_6*tmp_v4_1-tmp_v4_2;                  
       double psi, phi;
       psi = atan2(P0_5(1), P0_5(0));  //atan(x/y)
       if( sqrt(P0_5(0)*P0_5(0)+P0_5(1)*P0_5(1)) < d4 )  //d4>P0_5: x,y is unreachable
          return false;    
       phi = acos(d4/(sqrt(P0_5(0)*P0_5(0)+P0_5(1)*P0_5(1))));  //*** 2 solution!! *** OR -phi
       q(0) = phi+psi+DEG2RAD(90);   //J1.*** 2 solution!! *** see above
       
       Matrix4d T0_1;
       T0_1 = dh_fcn(0, DEG2RAD(90), d1, q(0));
       Matrix4d T1_6;
       if(T0_1.determinant()==0)
          return false; 
       T1_6 = T0_1.inverse()*T0_6;
       q(4) = acos((T1_6(2,3)-d4)/d6);  //P1_6_z //J5. *** 2 solution!! *** OR -J5
       q(5) = atan2( (-T1_6(1,2)/sin(q((4)))) , (T1_6(0,2)/sin(q((4)))) );  //J6.
       

       Matrix4d T5_6; 
       T5_6 = dh_fcn(0, 0, d6, q(5));
       Matrix4d T4_5; 
       T4_5 = dh_fcn(0, DEG2RAD(-90), d5, q(4));
       Matrix4d T4_6; 
       T4_6 = T4_5*T5_6;
       if(T4_6.determinant()==0)
          return false; 
       Matrix4d T1_4; 
       T1_4 = T1_6*T4_6.inverse();
       Vector4d P1_3;
       tmp_v4_1 << 0, -d4, 0, 1;
       P1_3 = T1_4*tmp_v4_1 - tmp_v4_2;       

       q(2)=acos((P1_3.squaredNorm()-a2*a2-a3*a3)/(2*a2*a3));  //J3. *** 2 solution!! *** OR -J3
       q(1)=-atan2(P1_3(1), -P1_3(0)) + asin(a3*sin(q(2))/P1_3.norm());  //J2.
       
       Matrix4d T1_2;
       Matrix4d T2_3;
       T1_2 = dh_fcn(a2, 0, 0, q(1));
       T2_3 = dh_fcn(a3, 0, 0, q(2));
       Matrix4d T3_4;
       if((T1_2*T2_3).determinant()==0)
          return false; 
       T3_4 = (T1_2*T2_3).inverse()*T1_4;
       q(3)=atan2(T3_4(1,0), T3_4(0,0));  //J4.


       for(int i=0; i<6; i++)
       {
          if (q(i) != q(i)) {  // (x != x): x is NaN
              return false;
          }
          else {
              q(i) = q(i) - DEG2RAD(360)*(int)(q(i)/DEG2RAD(360));
              
          }
       } 
       m_goal << q;
       ROS_INFO_STREAM("@ m_goal\n" << m_goal.transpose());
       

       return isSuccess;
}

Matrix4d SimpleEffortControl::dh_fcn(double a, double alpha, double d, double theta)
{
    Matrix4d dh_matrix;
    dh_matrix << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
                 sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                      0    ,        sin(alpha)    ,      cos(alpha)       ,       d     ,
                      0    ,             0        ,           0           ,       1     ;


    return dh_matrix;
}

}
}
