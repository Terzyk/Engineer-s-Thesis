#include "controller_matrix.hpp"
#include "ros/ros.h"


controller_matrix::Vector q_nominalPrev(3);
controller_matrix::Vector q(3);
controller_matrix::Vector q_prev(3);
controller_matrix::Vector q_prim(3);
controller_matrix::Vector q_prim_prev(3);
controller_matrix::Vector q_nominal(3);
controller_matrix::Vector q_bis(3);

controller_matrix::Vector u(2);
controller_matrix::Vector u_sat(2);
controller_matrix::Vector u_nominal(2);

controller_matrix::Vector phi_prev(2);

controller_matrix::Matrix P_out(3,3);

// New variables

controller_matrix::Vector k_vect(4);

//        T_e
controller_matrix::Vector e(3);
controller_matrix::Vector e_T(3);
controller_matrix::Matrix T(3,3);

//        f_c
controller_matrix::Vector v_c(2);
controller_matrix::Vector v_c_prim(2);
controller_matrix::Vector v_c_prim_acl(2);
controller_matrix::Matrix v_c_calc(2,3);
controller_matrix::Vector v_prev(2);
controller_matrix::Vector v_vect(2);

controller_matrix::Vector e_c(3);
controller_matrix::Vector e_prim(3);
controller_matrix::Vector u_c(2);

//  inv_kin

controller_matrix::Matrix S(5,2);
controller_matrix::Matrix S_prim(5,2);
controller_matrix::Matrix M(5,5);
controller_matrix::Matrix V(5,5);
controller_matrix::Matrix B(5,2);

controller_matrix::Matrix M_s(2,2);
controller_matrix::Matrix V_s(2,2);
controller_matrix::Matrix B_s(2,2);

controller_matrix::Vector u_s(2);
controller_matrix::Vector v_s(2);
