//#pragma once
#ifndef controller_matrix_hpp
#define controller_matrix_hpp

#include "definitions.hpp"

#include "hardwareManager.hpp"



  // Old variables
extern controller_matrix::Vector q_nominalPrev;
extern controller_matrix::Vector q;
extern controller_matrix::Vector q_prev;
extern controller_matrix::Vector q_prim;
extern controller_matrix::Vector q_prim_prev;
extern controller_matrix::Vector q_nominal;
extern controller_matrix::Vector q_bis;

extern controller_matrix::Vector u;
extern controller_matrix::Vector u_sat;
extern controller_matrix::Vector u_nominal;

extern controller_matrix::Vector phi_prev;

extern controller_matrix::Matrix P_out;

// New variables

extern controller_matrix::Vector k_vect;

//        T_e
extern controller_matrix::Vector e;
extern controller_matrix::Vector e_T;
extern controller_matrix::Matrix T;

//        f_c
extern controller_matrix::Vector v_c;
extern controller_matrix::Vector v_c_prim;
extern controller_matrix::Vector v_c_prim_acl;
extern controller_matrix::Matrix v_c_calc;
extern controller_matrix::Vector v_prev;
extern controller_matrix::Vector v_vect;

extern controller_matrix::Vector e_c;
extern controller_matrix::Vector e_prim;
extern controller_matrix::Vector u_c;

//  inv_kin

extern controller_matrix::Matrix S;
extern controller_matrix::Matrix S_prim;
extern controller_matrix::Matrix M;
extern controller_matrix::Matrix V;
extern controller_matrix::Matrix B;

extern controller_matrix::Matrix M_s;
extern controller_matrix::Matrix V_s;
extern controller_matrix::Matrix B_s;

extern controller_matrix::Vector u_s;
extern controller_matrix::Vector v_s;



#endif

