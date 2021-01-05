#pragma once
#include "definitions.hpp"
#include "controller_matrix.hpp"

class MotionController
{

public: 


    int trajectoryType;
    bool isReady;
    double vr;              // Forward velocity set
    double vr_prim;         // Forward accelertation set
    double v_read;          // Forward velocity read
    double vr_prev;         // Previous velocity set

    double omega_d;         // Angle velocity set
    double omega_d_prim;    // Angle accleceration set
    double omega_read;      // Angle velocity read
    double omega_d_prev;    // Previous angle velocity set

    int angle_wl;
    int angle_wr;
    int angle_wl_prev;
    int angle_wr_prev;
    int current_wl;
    int current_wr;



    double var_i;
    double imp_left_all;
    double imp_right_all;
    double vel_phi_l;
    double vel_phi_r;


    MotionController(bool _isReady);

    void setControllerParameters(int _angle_wl, int _angle_wr, int _angle_wl_prev, int _angle_wr_prev, int _current_wl, int _current_wr,
                                 double _var_i, double _imp_left_all, double _imp_right_all, double _vel_phi_l, double _vel_phi_r, int _trajectoryType, double _vr, double _vr_prim,
                                 double _v_read, double _vr_prev, double _omega_d, double _omega_d_prim, double _omega_read, double _omega_d_prev);

   // void encodersL(unsigned int data, int &angle_wl);

   // void encodersR(unsigned int data, int &angle_wr);

    void calculateOdometry(controller_matrix::Vector &q_prev,

                           int &angle_wl,

                           int &angle_wr,

                           int &angle_wl_prev,

                           int &angle_wr_prev,

                           controller_matrix::Vector &q,

                           controller_matrix::Vector &q_prim,

                           double &delta_t,

                           HardwareManager &dim);


    //void readRightEncoder(HardwareManager robotHardware, int &angle_wr, CanInterface canIN, Can &can);

    //void readLeftEncoder(HardwareManager robotHardware, int &angle_wl, CanInterface canIN, Can &can);

    void saveParametersForNextIteration (double &t_sec,

                                         controller_matrix::Vector &q,

                                         controller_matrix::Vector &q_nominal,

                                         int &angle_wl,

                                         int &angle_wr,

                                         double &t_secPrev,

                                         controller_matrix::Vector &q_prev,

                                         controller_matrix::Vector &q_nominalPrev,

                                         int &angle_wl_prev,

                                         int &angle_wr_prev);


    void setControllerVariables (int trajectoryType,

                                 controller_matrix::Vector &q_nominalPrev,

                                 controller_matrix::Vector &q,

                                 controller_matrix::Vector &q_prev,

                                 controller_matrix::Vector &q_prim,

                                 controller_matrix::Vector &u,

                                 controller_matrix::Vector &q_nominal,

                                 controller_matrix::Vector &u_nominal);


    void generateTrajectory(int trajectoryType,
                            double t,
                            controller_matrix::Vector &q_nominalPrev,
                            controller_matrix::Vector &q_nominal,
                            controller_matrix::Vector &u_nominal,
                            double &delta_t);

    void T_e(controller_matrix::Vector &q, controller_matrix::Vector &q_nominal);



    void f_c(controller_matrix::Vector &q,
             controller_matrix::Vector &q_prim,
             controller_matrix::Vector &v_vect,controller_matrix::Vector &v_prev,
             controller_matrix::Vector &v_c,
             controller_matrix::Vector &e_T,
             controller_matrix::Matrix &v_c_calc,
             controller_matrix::Vector &v_c_prim,
             controller_matrix::Vector &u_c,
             controller_matrix::Vector &e_c,
             controller_matrix::Vector &e_prim,
             controller_matrix::Vector &k_vect);
  // kinematyka
  void kinematyka(controller_matrix::Vector &u_c,
                  controller_matrix::Vector &q,
                  controller_matrix::Vector &q_prim,
                  int &angle_wl,
                  int &angle_wr,
                  int &angle_wl_prev,
                  int &angle_wr_prev,
                  double &delta_t);


    //Inverse dynamics

    void inverse_dyn(controller_matrix::Matrix &S,
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
                     HardwareManager &dim);


    // Limiting current and maintain angle of turn BSP

    void currentLimitation(double &t_sec,
                           controller_matrix::Vector &u,
                           controller_matrix::Vector &u_sat,
                           int &current_wl, int &current_wr);




};
