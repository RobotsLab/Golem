
#ifdef __WIN32__
#pragma pack(push,1)
#endif

struct telemetry_packet {
	float q_cmd[19];        // commanded motor position: 3 torso joints, 7 right arm, 7 left arm, 2 head joints
	float tau_cmd[19];      // commanded torque: -"-
	float q_act[19];        // measured motor position
	float q_act_poti[19];   // measured link-side position
	float tau_act[19];      // measured torque
	float hand_q_cmd[24];   // commanded finger-motor position: 12 right hand: 3 thumb joints, 3 tip-finger, 3 middle, 3 ring-finger, 12 left hand
	float hand_q_act[24];   // measured finger-motor position: -"-
	float hand_tau_act[24]; // measured finger-torque: -"-
	float q_act_link[20];   // estimated link-side position: 4 torso, 7 right arm, 7 left arm, 2 head joints
	float physColDet;       // physical collision was detected!
	float guard_stop;       // guard stopped motion!
	float tau_ext_all[44];  /* all torques caused by external contacts/forces:
							   4 torso, 7 right arm, 7 left arm, 2 head, 12 right hand, 12 left hand
							   (without torques caused by gravitation)
							*/
}
#ifdef __LINUX__
 __attribute__((__packed__))
#endif
 ;

#ifdef __WIN32__
#pragma pack(pop)
#endif
