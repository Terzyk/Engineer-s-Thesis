#include "motionController.hpp"


MotionController::MotionController(bool _isReady){

    isReady=_isReady;

}

void MotionController::setControllerParameters(int _angle_wl, int _angle_wr, int _angle_wl_prev, int _angle_wr_prev, int _current_wl, int _current_wr, 
                                               double _var_i, double _imp_left_all, double _imp_right_all, double _vel_phi_l, double _vel_phi_r, int _trajectoryType, double _vr, double _vr_prim,
                                               double _v_read, double _vr_prev, double _omega_d, double _omega_d_prim, double _omega_read, double _omega_d_prev){


    angle_wl =_angle_wl;
    angle_wr=angle_wr;
    angle_wl_prev=_angle_wl_prev;
    angle_wr_prev=_angle_wr_prev;
    current_wl=_current_wl;
    current_wr=_current_wr;
    var_i=_var_i;
    imp_left_all=_imp_left_all;
    imp_right_all=_imp_right_all;
    vel_phi_l=_vel_phi_l;
    vel_phi_r=_vel_phi_r;
    vr=_vr;
    vr_prim=_vr_prim;
    v_read=_v_read;
    vr_prev=_vr_prev;
    omega_d=_omega_d;
    omega_d_prim=_omega_d_prim;
    omega_read=_omega_read;
    omega_d_prev=_omega_d_prev;
    trajectoryType=_trajectoryType;

}

/*
void MotionController::encodersL(unsigned int data, int &angle_wl)
{
    std::stringstream ss;
    ss << data;
    ss >>std::hex>> angle_wl;
}



void MotionController::encodersR(unsigned int data, int &angle_wr)
{
    std::stringstream ss;
    ss << data;
    ss >>std::hex>> angle_wr;
}


void MotionController::readRightEncoder(HardwareManager robotHardware, int &angle_wr, CanInterface canIN, Can &can)
{
    unsigned int data;
    int node=robotHardware.setNodeID(1);
    canIN.requestEncoderPosition(node,can, data);
    encodersR(data, angle_wr);
    //std::cout  << angle_wr << std::endl;
}


void MotionController::readLeftEncoder(HardwareManager robotHardware, int &angle_wl, CanInterface canIN, Can &can)
{
    unsigned int data;
    int node=robotHardware.setNodeID(2);
    canIN.requestEncoderPosition(node,can, data);
    encodersL(data, angle_wl);
    //std::cout << angle_wl << std::endl;
}*/


void MotionController::saveParametersForNextIteration (double &t_sec, 
                                                       controller_matrix::Vector &q,
                                                       controller_matrix::Vector &q_nominal,
                                                       int &angle_wl,
                                                       int &angle_wr,
                                                       double &t_secPrev,
                                                       controller_matrix::Vector &q_prev,
                                                       controller_matrix::Vector &q_nominalPrev,
                                                       int &angle_wl_prev,
                                                       int &angle_wr_prev)
{
    q_prev = q;
    q_nominalPrev = q_nominal;
    q_prim_prev=q_prim;
    t_secPrev = t_sec;
    angle_wl_prev = angle_wl;
    angle_wr_prev = angle_wl;
    vr_prim=vr;
    omega_d_prev=omega_d;

}



void MotionController::setControllerVariables (int trajectoryType,
                                               controller_matrix::Vector &q_nominalPrev,
                                               controller_matrix::Vector &q,
                                               controller_matrix::Vector &q_prev,
                                               controller_matrix::Vector &q_prim,
                                               controller_matrix::Vector &u,
                                               controller_matrix::Vector &q_nominal,
                                               controller_matrix::Vector &u_nominal)
{

    //  Trajectory

    q(0) = 0;
    q(1) = 0;
    q(2) = 0;
    q_nominalPrev(0) = 0;
    q_nominalPrev(1) = 0;
    switch(trajectoryType)
    {
    case 1: // straight horizontal line
        q_nominalPrev(2) = 0;
        break;
    case 2: // pi/4 straight line
        q_nominalPrev(2) = M_PI/4;
        break;
    case 3: // sinus
        q_nominalPrev(2) = M_PI/4;
        break;
    case 4: // circle
        q_nominalPrev(2) = M_PI/2;
        break;
    case 5: // eight figure
        q_nominalPrev(2) = M_PI/2;
        break;
    case 6: // point
        q_nominalPrev(2) = 0;
        break;
    }

    // Old variables

    q <<   0,
            0,
            0;

    q_prev << 0,
            0,
            0;

    q_prim<<  0,
            0,
            0;

    q_bis<<   0,
            0,
            0;

    q_prim_prev<<    0,
            0,
            0;

    u<<     0,
            0;

    u_sat<< 0,
            0;

    q_nominal<< 0,
            0,
            0;

    u_nominal<< 0,
            0;


    //New varialbles

    T<<     0,0,0,
            0,0,0,
            0,0,0;

    e <<    0,
            0,
            0;

    e_T <<  0,
            0,
            0;

    k_vect<< 2,
            2,
            5,
            20;

}


void MotionController::T_e(controller_matrix::Vector &q, controller_matrix::Vector &q_nominal)
{
    e=q_nominal-q;

    T << cos(q(2)),  sin(q(2)),     0,
            -sin(q(2)),  cos(q(2)),     0,
            0,          0,          1;

    e_T=T*e;

}

void MotionController::f_c(controller_matrix::Vector &q,
                           controller_matrix::Vector &q_prim,
                           controller_matrix::Vector &v_vect,controller_matrix::Vector &v_prev,
                           controller_matrix::Vector &v_c,
                           controller_matrix::Vector &e_T,
                           controller_matrix::Matrix &v_c_calc,
                           controller_matrix::Vector &v_c_prim,
                           controller_matrix::Vector &u_c,
                           controller_matrix::Vector &e_c,
                           controller_matrix::Vector &e_prim,
                           controller_matrix::Vector &k_vect)
{


    v_c(0)= vr*cos(e_T(2))+k_vect(0)*e_T(0);
    v_c(1)= omega_d+k_vect(1)*vr*e_T(1)+k_vect(2)*vr*sin(e_T(2));

    e_prim(0)=v_vect(1)*e_T(1)-v_vect(0)+vr*cos(e_T(2));
    e_prim(1)=-v_vect(1)*e_T(0)+vr*sin(e_T(2));
    e_prim(2)=omega_d-v_vect(1);

    e_c=k_vect(3)*(v_c-v_vect);

    v_c_calc<< k_vect(0),  0,       -vr*sin(e_T(2)),
            0  ,  k_vect(1)*vr, k_vect(2)*vr*cos(e_T(2));



    v_c_prim_acl<< vr_prim*cos(e_T(2)),
            omega_d_prim*k_vect(1)*vr_prim*e_T(1);



    v_c_prim=v_c_prim_acl+v_c_calc*e_prim;

    u_c=v_c_prim+e_c;

}

void MotionController::kinematyka(controller_matrix::Vector &u_c,
                                  controller_matrix::Vector &q,
                                  controller_matrix::Vector &q_prim,
                                  int &angle_wl,
                                  int &angle_wr,
                                  int &angle_wl_prev,
                                  int &angle_wr_prev,
                                  double &delta_t)
{
  int r = 25; 
  int b = 145;
  q_prim(0) = u_c(0)*cos(q(2));
  q_prim(1) = u_c(0)*sin(q(2));
  q_prim(2) = u_c(1);
  
  q(0)=q(0)+delta_t*q_prim(0);
  q(1)=q(1)+delta_t*q_prim(1);
  q(2)=q(2)+delta_t*q_prim(2);
  
  angle_wl_prev=angle_wl;
  angle_wr_prev=angle_wr;
  
  angle_wr = (1/r)*(u_c(0)+(b/2)*u_c(1));
  angle_wl = (1/r)*(u_c(0)-(b/2)*u_c(1));

}

void MotionController::inverse_dyn(controller_matrix::Matrix &S,
                                   controller_matrix::Matrix &S_prim,
                                   controller_matrix::Matrix &M,
                                   controller_matrix::Matrix &V,
                                   controller_matrix::Matrix &B,
                                   controller_matrix::Vector &q,
                                   controller_matrix::Vector &q_prim,
                                   controller_matrix::Vector &v_s,
                                   controller_matrix::Vector &u_s,
                                   controller_matrix::Vector &u,
                                   controller_matrix::Matrix &M_s,
                                   controller_matrix::Matrix &V_s,
                                   controller_matrix::Matrix &B_s,
                                   HardwareManager &dim)
{


    //Matrix
    S<<      (dim.r/2)*cos(q(2)), (dim.r/2)*cos(q(2)),
            (dim.r/2)*sin(q(2)), (dim.r/2)*sin(q(2)),
            (dim.r/(2*dim.l))      ,   -(dim.r/(2*dim.l))   ,
            1				      ,      0         ,
            0              ,      1         ;

    S_prim<<-q_prim(2)*(dim.r/2)*sin(q(2)), -q_prim(2)*(dim.r/2)*sin(q(2)),
            q_prim(2)*(dim.r/2)*cos(q(2)),  q_prim(2)*(dim.r/2)*cos(q(2)),
            0    		  ,             0             ,
            0    		  ,             0             ,
            0    		  ,             0             ;

    M<<				      dim.m,                 0,            -dim.m*dim.d*sin(q(2)),    0,     0,
            0,                 dim.m,             dim.m*dim.d*cos(q(2)),    0,     0,
            -dim.m*dim.d*sin(q(2)),     dim.m*dim.d*cos(q(2)),             dim.I,          0,     0,
            0,                 0,                   0,          dim.I_w,   0,
            0,                 0,                   0,          0,     dim.I_w;




    V<< 		    0 , -dim.m*dim.d*q_prim(2)*cos(q(2)), 0 ,0,0,
            0 , -dim.m*dim.d*q_prim(2)*sin(q(2)), 0 ,0,0,
            0	,             0           , 0 ,0,0,
            0	,             0           , 0 ,0,0,
            0	,             0           , 0 ,0,0;


    B<< 0,0,
            0,0,
            0,0,
            1,0,
            0,1;

    //Modified Matrix
    M_s=S.transpose()*M*S;
    V_s=S.transpose()*M*S_prim+S.transpose()*V*S;
    B_s=S.transpose()*B;

    //Velocity
    v_s<<   (v_vect(0)+v_vect(1)*dim.l)/dim.r,
            ((2*v_vect(0)/dim.r)-(v_vect(0)+v_vect(1)*dim.l)/dim.r);

    //Acceleration
    u_s<<	(u_c(0)+u_c(1)*dim.l)/dim.r,
            ((2*u_c(0)/dim.r)-(u_c(0)+u_c(1)*dim.l)/dim.r);

    //Set acceleration
    u=B_s.inverse()*(M_s*u_s+V_s*v_s);

    // Counting moment to [A]
    u(0)=u(0)/dim.k_engine_r;
    u(1)=u(1)/dim.k_engine_l;

}

void MotionController::currentLimitation(double &t_sec, 
                                         controller_matrix::Vector &u,
                                         controller_matrix::Vector &u_sat,
                                         int &current_wl, int &current_wr)
{
    double CurrentK=5000;
    double k = 1;
    u=u*1000;

    double kss = std::max((fabs(u(0))/CurrentK),(fabs(u(1))/CurrentK));
    double ks = std::max(k,kss);
    u_sat = u/ks;
    current_wr = round(u_sat(0));
    current_wl = round(u_sat(1));

}

void MotionController::generateTrajectory(int trajectoryType, double t,controller_matrix::Vector &q_nominalPrev,controller_matrix::Vector &q_nominal, controller_matrix::Vector &u_nominal,double &delta_t )
{
    double x; double y; double xp; double yp; double xpp; double ypp;

    //Paramiters of trajectory
    double A = 0.3;
    double K = 0.5;

    //Variables used to Counting theta
    double d_theta=0;
    double theta_prev_scaled;
    double delta_theta;
    double theta_np;

    switch(trajectoryType)

    {

    case 1: // straight horizontal lomegaRine
        x = A*t;
        y = 0;
        xp = A;
        yp = 0;
        xpp = 0;
        ypp = 0;
        break;

    case 2: // pi/4 straight line
        x = A*t;
        y = A*t;
        xp = A;
        yp = A;
        xpp = 0;
        ypp = 0;
        break;

    case 3: // sinus
        x = A*t;
        y = K*sin(A*t);
        xp = A;
        yp = K*A*cos(A*t);
        xpp = 0;
        ypp = -A*A*K*sin(A*t);
        break;

    case 4: // circle
        x = cos(A*t);
        y = sin(A*t);
        xp = -A*sin(A*t);
        yp = A*cos(A*t);
        xpp = -A*A*cos(A*t);
        ypp = -A*A*sin(A*t);
        break;

    case 5: // 8 figure
        x = cos(A*t);
        y = (sin(2*A*t))/2;
        xp = -A*sin(A*t);
        yp = A*cos(2*A*t);
        xpp = -A*A*cos(A*t);
        ypp = -2*A*A*sin(2*A*t);
        break;

    case 6: // point
        x = 0;
        y = 0;
        xp = 0;
        yp =  0;
        xpp =  0;
        ypp =  0;
        break;
    }

    vr = sqrt((xp*xp) + (yp*yp));

    double theta_n = atan2(vr*yp,vr*xp);
    double theta_prev_modulo = fmod(q_nominalPrev(2),(2*M_PI));


    if (theta_prev_modulo > M_PI)

        theta_prev_scaled = theta_prev_modulo - (2*M_PI);
    else if (theta_prev_modulo <= M_PI)
        theta_prev_scaled = theta_prev_modulo;

    delta_theta = theta_n - theta_prev_scaled;

    if (delta_theta > M_PI)
        d_theta = delta_theta - (2*M_PI);
    else if (delta_theta < (-1*M_PI))
        d_theta = delta_theta + (2*M_PI);
    else
        d_theta = delta_theta;

    theta_n = q_nominalPrev(2) + d_theta;
    theta_np = (ypp*xp-xpp*yp)/(vr*vr);


    // Set velocity [Forward] and [Angle]

    omega_d=theta_np;

    // Set acceleration [Forward] and [Angle]
    vr_prim=vr-vr_prev;
    omega_d_prim=omega_d-omega_d_prev;


    // Set q
    q_nominal <<      x,
            y,
            theta_n;
}

void MotionController::calculateOdometry(controller_matrix::Vector &q_prev,
                                         int &angle_wl,
                                         int &angle_wr,
                                         int &angle_wl_prev,
                                         int &angle_wr_prev,
                                         controller_matrix::Vector &q,
                                         controller_matrix::Vector &q_prim,
                                         double &delta_t,
                                         HardwareManager &dim)
{
    // Counting impulses
   // double imp_left = angle_wl - angle_wl_prev;
   // double imp_right = -angle_wr + angle_wr_prev;

    double imp_left = 10;
        double imp_right = -10;




    //Impulses to angle
    double phi_left = (imp_left*2*M_PI)/178000;
    double phi_right = (imp_right*2*M_PI)/178000;

    imp_right_all=imp_right_all+phi_left;
    imp_left_all=imp_left_all+phi_left;

    vel_phi_l=phi_left/delta_t;
    vel_phi_r=phi_right/delta_t;



    //Velocity vel_phi_l
    q_prim(0) = (dim.r/2)*cos(q_prev(2))*(vel_phi_r+vel_phi_l);
    q_prim(1) = (dim.r/2)*sin(q_prev(2))*(vel_phi_r+vel_phi_l);
    q_prim(2) = (dim.r/(2*dim.l))*(vel_phi_r-vel_phi_l);


    //Posistion q
    q(0)=q_prev(0)+q_prim(0)*delta_t;
    q(1)=q_prev(1)+q_prim(1)*delta_t;
    q(2)=q_prev(2)+q_prim(2)*delta_t;

    //Velocity [forward and angle] read
    v_read=(dim.r/2)*(vel_phi_r+vel_phi_l);
    omega_read=q_prim(2);
    v_vect<<v_read,
            omega_read;

}
