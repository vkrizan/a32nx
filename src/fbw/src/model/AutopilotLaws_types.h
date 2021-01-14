#ifndef RTW_HEADER_AutopilotLaws_types_h_
#define RTW_HEADER_AutopilotLaws_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_ap_raw_time_
#define DEFINED_TYPEDEF_FOR_ap_raw_time_

typedef struct {
  real_T dt;
  real_T simulation_time;
} ap_raw_time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_data_
#define DEFINED_TYPEDEF_FOR_ap_raw_data_

typedef struct {
  real_T Theta_deg;
  real_T Phi_deg;
  real_T q_rad_s;
  real_T r_rad_s;
  real_T p_rad_s;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T V_gnd_kn;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_ft_min;
  real_T Psi_magnetic_deg;
  real_T Psi_magnetic_track_deg;
  real_T Psi_true_deg;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  boolean_T ap_fd_active;
  real_T ap_V_c_kn;
  real_T ap_H_c_ft;
  real_T ap_Psi_c_deg;
  real_T ap_H_dot_c_ft_min;
  real_T ap_FPA_c_deg;
  boolean_T nav_valid;
  real_T nav_loc_deg;
  real_T nav_radial_error_deg;
  real_T nav_dme_nmi;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T flight_phase;
  real_T V2_kn;
  boolean_T is_flight_plan_available;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
  real_T gear_strut_compression_1;
  real_T gear_strut_compression_2;
  real_T zeta_pos;
  real_T throttle_lever_1_pos;
  real_T throttle_lever_2_pos;
} ap_raw_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_laws_input_
#define DEFINED_TYPEDEF_FOR_ap_raw_laws_input_

typedef struct {
  real_T enabled;
  real_T lateral_law;
  real_T lateral_mode;
  real_T lateral_mode_armed;
  real_T vertical_law;
  real_T vertical_mode;
  real_T vertical_mode_armed;
  real_T Psi_c_deg;
  real_T H_c_ft;
  real_T H_dot_c_fpm;
  real_T FPA_c_deg;
} ap_raw_laws_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_laws_input_
#define DEFINED_TYPEDEF_FOR_ap_laws_input_

typedef struct {
  ap_raw_time time;
  ap_raw_data data;
  ap_raw_laws_input input;
} ap_laws_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_data_
#define DEFINED_TYPEDEF_FOR_ap_data_

typedef struct {
  real_T Theta_deg;
  real_T Phi_deg;
  real_T qk_deg_s;
  real_T rk_deg_s;
  real_T pk_deg_s;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T V_gnd_kn;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_ft_min;
  real_T Psi_magnetic_deg;
  real_T Psi_magnetic_track_deg;
  real_T Psi_true_deg;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  boolean_T ap_fd_active;
  real_T ap_V_c_kn;
  real_T ap_H_c_ft;
  real_T ap_Psi_c_deg;
  real_T ap_H_dot_c_ft_min;
  real_T ap_FPA_c_deg;
  boolean_T nav_valid;
  real_T nav_loc_deg;
  real_T nav_radial_error_deg;
  real_T nav_dme_nmi;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T flight_phase;
  real_T V2_kn;
  boolean_T is_flight_plan_available;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
  real_T on_ground;
  real_T zeta_deg;
  real_T throttle_lever_1_pos;
  real_T throttle_lever_2_pos;
} ap_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_output_command_
#define DEFINED_TYPEDEF_FOR_ap_raw_output_command_

typedef struct {
  real_T Theta_c_deg;
  real_T Phi_c_deg;
  real_T Beta_c_deg;
} ap_raw_output_command;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_output_
#define DEFINED_TYPEDEF_FOR_ap_raw_output_

typedef struct {
  real_T ap_on;
  ap_raw_output_command flight_director;
  ap_raw_output_command autopilot;
} ap_raw_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_laws_output_
#define DEFINED_TYPEDEF_FOR_ap_laws_output_

typedef struct {
  ap_raw_time time;
  ap_data data;
  ap_raw_laws_input input;
  ap_raw_output output;
} ap_laws_output;

#endif

#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH_AutopilotLaws_T
#define typedef_skA4KFEZ4HPkJJBOYCrevdH_AutopilotLaws_T

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH_AutopilotLaws_T;

#endif

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_AutopilotLaws_T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_AutopilotLaws_T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E_AutopilotLaws_T;

#endif

typedef struct Parameters_AutopilotLaws_T_ Parameters_AutopilotLaws_T;

#endif

