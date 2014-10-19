//
//  ARNavdataOptions.h
//  libARDrone
//
//  Created by Sidney Just
//  Copyright (c) 2014 by Sidney Just
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
//  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#ifndef __libARDrone__ARNavdataOptions__
#define __libARDrone__ARNavdataOptions__

#include "ARVector.h"
#include "ARLocation.h"

namespace AR
{
	typedef enum
	{
		ARDRONE_FLY_MASK            = 1 << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
		ARDRONE_VIDEO_MASK          = 1 << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
		ARDRONE_VISION_MASK         = 1 << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
		ARDRONE_CONTROL_MASK        = 1 << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
		ARDRONE_ALTITUDE_MASK       = 1 << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
		ARDRONE_USER_FEEDBACK_START = 1 << 5,  /*!< USER feedback : Start button state */
		ARDRONE_COMMAND_MASK        = 1 << 6,  /*!< Control command ACK : (0) None, (1) one received */
		ARDRONE_FW_FILE_MASK        = 1 << 7,  /* Firmware file is good (1) */
		ARDRONE_FW_VER_MASK         = 1 << 8,  /* Firmware update is newer (1) */
		//  ARDRONE_FW_UPD_MASK         = 1 << 9,  /* Firmware update is ongoing (1) */
		ARDRONE_NAVDATA_DEMO_MASK   = 1 << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
		ARDRONE_NAVDATA_BOOTSTRAP   = 1 << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
		ARDRONE_MOTORS_MASK  	      = 1 << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
		ARDRONE_COM_LOST_MASK       = 1 << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
		ARDRONE_VBAT_LOW            = 1 << 15, /*!< VBat low : (1) too low, (0) Ok */
		ARDRONE_USER_EL             = 1 << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
		ARDRONE_TIMER_ELAPSED       = 1 << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
		ARDRONE_ANGLES_OUT_OF_RANGE = 1 << 19, /*!< Angles : (0) Ok, (1) out of range */
		ARDRONE_ULTRASOUND_MASK     = 1 << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
		ARDRONE_CUTOUT_MASK         = 1 << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
		ARDRONE_PIC_VERSION_MASK    = 1 << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
		ARDRONE_ATCODEC_THREAD_ON   = 1 << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_NAVDATA_THREAD_ON   = 1 << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_VIDEO_THREAD_ON     = 1 << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_ACQ_THREAD_ON       = 1 << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
		ARDRONE_CTRL_WATCHDOG_MASK  = 1 << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
		ARDRONE_ADC_WATCHDOG_MASK   = 1 << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
		ARDRONE_COM_WATCHDOG_MASK   = 1 << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
		ARDRONE_EMERGENCY_MASK      = 1 << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
	} DroneMask;
	
	typedef enum
	{
		ControlStateDefault,
		ControlStateInit,
		ControlStateLanded,
		ControlStateFlying,
		ControlStateHovering,
		ControlStateTest,
		ControlStateTakeOff,
		ControlStateGotoFix,
		ControlStateLanding,
		ControlStateLooping
	} ControlState;
	
	enum class NavdataTag : uint16_t
	{
		Demo = 0,
		Time = 1,
		RawMeasures = 2,
		PhysMeasures = 3,
		GyrosOffsets = 4,
		EulerAngles = 5,
		References = 6,
		Trims = 7,
		RCReferences = 8,
		PWM = 9,
		Altitude = 10,
		VisionRaw = 11,
		VisionOF = 12,
		Vision = 13,
		VisionPerf = 14,
		TrackersSend = 15,
		VisionDetect = 16,
		Watchdog = 17,
		ADCDataFrame = 18,
		VideoStream = 19,
		Game = 20,
		PressureRaw = 21,
		Magneto = 22,
		Wind = 23,
		KalmanPressure = 24,
		HDVideoStream = 25,
		Wifi = 26,
		GPS = 27,
		
		Checksum = UINT16_MAX
	};
	
	
	struct NavdataOption
	{
		NavdataTag tag;
		uint16_t size;
	} __attribute__((packed));
	
	struct NavdataOptionDemo : public NavdataOption
	{
		uint32_t ctrl_state;
		uint32_t vbat_flying_percentage;
		
		float theta;
		float phi;
		float psi;
		
		int32_t altitude;
		
		float vx;
		float vy;
	} __attribute__((packed));
	
	struct NavdataOptionTime : public NavdataOption
	{
		uint32_t time;
	} __attribute__((packed));
	
	struct NavdataOptionRawMeasures : public NavdataOption
	{
		uint16_t raw_accs[3]; // filtered accelerometers
		int16_t raw_gyros[3]; // filtered gyrometers
		int16_t raw_gyros_110[2]; // gyrometers  x/y 110 deg/s
		uint32_t vbat_raw; // battery voltage raw (mV)
		uint16_t us_debut_echo;
		uint16_t us_fin_echo;
		uint16_t us_association_echo;
		uint16_t us_distance_echo;
		uint16_t us_courbe_temps;
		uint16_t us_courbe_valeur;
		uint16_t us_courbe_ref;
		uint16_t flag_echo_ini;
		uint16_t nb_echo;
		uint32_t sum_echo;
		int32_t alt_temp_raw;
		int16_t gradient;
	} __attribute__((packed));
	
	struct NavdataOptionWind : public NavdataOption
	{
		float wind_speed; // estimated wind speed [m/s]
		float wind_angle; // estimated wind direction in North-East frame [rad] e.g. if wind_angle is pi/4, wind is from South-West to North-East
		float wind_compensation_theta;
		float wind_compensation_phi;
		float state_x1;
		float state_x2;
		float state_x3;
		float state_x4;
		float state_x5;
		float state_x6;
		float magneto_debug1;
		float magneto_debug2;
		float magneto_debug3;
	} __attribute__((packed));
	
	struct NavdataOptionMagneto : public NavdataOption
	{
		int16_t mx;
		int16_t my;
		int16_t mz;
		Vector3 magneto_raw; // magneto in the body frame, in mG
		Vector3 magneto_rectified;
		Vector3 magneto_offset;
		float heading_unwrapped;
		float heading_gyro_unwrapped;
		float heading_fusion_unwrapped;
		char magneto_calibration_ok;
		uint32_t magneto_state;
		float magneto_radius;
		float error_mean;
		float error_var;
	} __attribute__((packed));
	
	struct NavdataOptionKalmanPressure : public NavdataOption
	{
		float offset_pressure;
		float est_z;
		float est_zdot;
		float est_bias_PWM;
		float est_biais_pression;
		float offset_US;
		float prediction_US;
		float cov_alt;
		float cov_PWM;
		float cov_vitesse;
		int bool_effet_sol;
		float somme_inno;
		int flag_rejet_US;
		float u_multisinus;
		float gaz_altitude;
		int Flag_multisinus;
		int Flag_multisinus_debut;
	} __attribute__((packed));
	
	struct NavdataOptionPhysMeasures : public NavdataOption
	{
		float accs_temp;
		uint16_t gyro_temp;
		float phys_accs[3];
		float phys_gyros[3];
		uint32_t alim3V3; // 3.3volt alim [LSB]
		uint32_t vrefEpson; // ref volt Epson gyro [LSB]
		uint32_t vrefIDG; // ref volt IDG gyro [LSB]
	} __attribute__((packed));
	
	struct NavdataOptionGPS : public NavdataOption
	{
		Location location;
		double elevation;
		double hdop;
		uint32_t data_available;
		int32_t zero_validated;
		int32_t wpt_validated;
		double lat0;
		double long0;
		double lat_fused;
		double long_fused;
		
		uint32_t gps_state;
		
		float X_traj;
		float X_ref;
		float Y_traj;
		float Y_ref;
		
		float theta_p;
		float phi_p;
		float theta_i;
		float phi_i;
		float theta_d;
		float phi_d;
		
		double vdop;
		double pdop;
		
		float speed;
		uint32_t lastFrameTimestamp;
		float degree;
		float degree_magnetic;
		float ehpe;
		float ehve;
		
		float c_n0;  /* Signal to noise ratio (average of the four best satellites) */
		uint32_t nbsat; /* Number of acquired satellites */
		struct
		{
			uint8_t sat; /* Satellite ID */
			uint8_t c_n0; /* Satellite C/N0 */
		}channels[12];
		
		int32_t is_gps_plugged;
		uint32_t ephemerisStatus;
		
		float vx_traj;
		float vy_traj;
		
		uint32_t firmwareStatus;
	} __attribute__ ((packed));
	
	struct NavdataOptionGyrosOffsets : public NavdataOption
	{
		float offset_g[3];
	} __attribute__((packed));
	
	struct NavdataOptionEulerAngles : public NavdataOption
	{
		float theta_a;
		float phi_a;
	} __attribute__((packed));
	
	struct NavdataOptionReferences : public NavdataOption
	{
		int32_t ref_theta;
		int32_t ref_phi;
		int32_t ref_theta_I;
		int32_t ref_phi_I;
		int32_t ref_pitch;
		int32_t ref_roll;
		int32_t ref_yaw;
		int32_t ref_psi;
		
		float vx_ref;
		float vy_ref;
		float theta_mod;
		float phi_mod;
		
		float k_v_x;
		float k_v_y;
		uint32_t  k_mode;
		
		float ui_time;
		float ui_theta;
		float ui_phi;
		float ui_psi;
		float ui_psi_accuracy;
		int32_t ui_seq;
	} __attribute__((packed));
	
	struct NavdataOptionTrims : public NavdataOption
	{
		float angular_rates_trim_r;
		float euler_angles_trim_theta;
		float euler_angles_trim_phi;
	} __attribute__((packed));
	
	struct NavdataOptionRCReferences : public NavdataOption
	{
		int32_t rc_ref_pitch;
		int32_t rc_ref_roll;
		int32_t rc_ref_yaw;
		int32_t rc_ref_gaz;
		int32_t rc_ref_ag;
	} __attribute__((packed));
	
	struct NavdataOptionPWM : public NavdataOption
	{
		uint8_t motor1;
		uint8_t motor2;
		uint8_t motor3;
		uint8_t motor4;
		uint8_t sat_motor1;
		uint8_t sat_motor2;
		uint8_t sat_motor3;
		uint8_t sat_motor4;
		float gaz_feed_forward;
		float gaz_altitude;
		float altitude_integral;
		float vz_ref;
		int32_t u_pitch;
		int32_t u_roll;
		int32_t u_yaw;
		float yaw_u_I;
		int32_t u_pitch_planif;
		int32_t u_roll_planif;
		int32_t u_yaw_planif;
		float u_gaz_planif;
		uint16_t current_motor1;
		uint16_t current_motor2;
		uint16_t current_motor3;
		uint16_t current_motor4;
		float altitude_prop;
		float altitude_der;
	} __attribute__((packed));
	
	struct NavdataOptionAltitude : public NavdataOption
	{
		int32_t altitude_vision;
		float altitude_vz;
		int32_t altitude_ref;
		int32_t altitude_raw;
		
		float obs_accZ;
		float obs_alt;
		Vector3 obs_x;
		uint32_t obs_state;
		float est_vb[2];
		uint32_t est_state;
	} __attribute__((packed));
	
	struct NavdataOptionVisionRaw : public NavdataOption
	{
		float vision_tx_raw;
		float vision_ty_raw;
		float vision_tz_raw;
	} __attribute__((packed));
	
	struct NavdataOptionVision : public NavdataOption
	{
		uint32_t vision_state;
		int32_t vision_misc;
		float  vision_phi_trim;
		float vision_phi_ref_prop;
		float vision_theta_trim;
		float vision_theta_ref_prop;
		
		int32_t new_raw_picture;
		float theta_capture;
		float phi_capture;
		float psi_capture;
		int32_t altitude_capture;
		uint32_t time_capture; // time in TSECDEC format
		Vector3 body_v;
		
		float delta_phi;
		float delta_theta;
		float delta_psi;
		
		uint32_t gold_defined;
		uint32_t gold_reset;
		float gold_x;
		float gold_y;
	} __attribute__((packed));
	
	struct NavdataOptionVisionPerf : public NavdataOption
	{
		float time_szo;
		float time_corners;
		float time_compute;
		float time_tracking;
		float time_trans;
		float time_update;
		float time_custom[20];
	} __attribute__((packed));
	
	struct NavdataOptionADCDataFrame : public NavdataOption
	{
		uint32_t version;
		uint8_t data_frame[20];
	} __attribute__((packed));
	
	struct NavdataOptionWifi : public NavdataOption
	{
		uint32_t link_quality;
	} __attribute__((packed));
	
	struct NavdataOptionPressureRaw : public NavdataOption
	{
		int32_t up;
		int16_t ut;
		int32_t Temperature_meas;
		int32_t Pression_meas;
	} __attribute__((packed));
	
	struct NavdataOptionChecksum : public NavdataOption
	{
		uint32_t checksum;
	} __attribute__((packed));
}

#endif /* defined(__libARDrone__ARNavdataOptions__) */
