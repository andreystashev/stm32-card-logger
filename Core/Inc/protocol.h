#include "sd.h"

typedef enum {false, true} bool;
//Пакет «БВМ – БСН»
typedef struct BVM_BSN {

  uint16_t id;// = 0x0711
  volatile uint16_t counter;
  volatile uint8_t performance_PGU;
  volatile uint8_t result_VSK;
  volatile uint8_t result_VSK_LS_PGU;
  volatile uint8_t management_word_in_BSN;
  volatile uint8_t ephemeris_information_code;

} BVM__BSN;

//Пакет «Управляющая информация для БВМ»
typedef struct BVM_INFO {

  uint16_t id;// = 0x0721
  volatile uint16_t counter;
  volatile uint8_t working_mode;
  volatile uint8_t management_word_in_BSN;

} BVM__INFO;

//Пакет «Ответная квитанция БСН-БВМ (неподвижное основание)»
typedef struct BSM_BVM_Answer_fixed{

	uint16_t id;//  = 0x0722
	volatile int32_t parking_latitude;
	volatile int32_t parking_longitude;
	volatile int32_t parking_height;
	volatile int16_t true_course;
	volatile int16_t roll_adjustment_angle;
	volatile int16_t pitch_adjustment_angle;

}BSM__BVM_Answer_fixed;

//Пакет «Ответная квитанция БСН-БВМ (движущееся основание)»
typedef struct BSM_BVM_Answer_move{

	uint16_t id;// = 0x0723;
	volatile int16_t distance_BINS_SNS_lateral_axis;
	volatile int16_t distance_BINS_SNS_normal_axis;
	volatile int16_t roll;
	volatile int16_t pitch;
	volatile int16_t course;
	volatile int32_t sputnik_latitude;
	volatile int32_t sputnik_longitude;
	volatile int32_t sputnik_height;
	volatile int32_t north_speed;
	volatile int32_t east_speed;
	volatile int32_t vertical_speed;
	volatile uint8_t working_mode_SNS;
	volatile uint8_t working_mode_BINS;
	volatile int32_t complex_time;
	volatile int32_t complex_latitude;
	volatile int32_t complex_longitude;
	volatile int32_t complex_height;
	volatile int32_t baro_height;
	volatile int32_t QFE;
	volatile int32_t QNH;
	volatile uint8_t status_word;
	volatile int32_t info_expiration_time_complex;
	volatile int32_t info_expiration_time_SNS;

}BSM__BVM_Answer_move;

//Пакет «Ответная квитанция БСН-БВМ (с эфемеридами)»
typedef struct BSM_BVM_Answer_ephemeris{

	uint16_t id;// = 0x0724;
	volatile uint8_t ephemer_info;

}BSM__BVM_Answer_ephemeris;

//Пакет «БВМ-ПЭ»
typedef struct BVM_PE{

	uint16_t id;// = 0x0511;
	volatile uint16_t counter;
	volatile uint8_t working_regime;

}BVM__PE;

//Пакет «Ответная квитанция ПЭ»
typedef struct answer_PE{

	uint16_t id;// = 0x0521;
	volatile uint16_t counter;
	volatile uint8_t status_word;
	volatile uint8_t battery_level;
	volatile uint8_t remaining_capacity;
	volatile uint16_t remaining_running_time;
	volatile uint8_t time_to_readiness_PE;
	volatile uint16_t output_current;
	volatile uint8_t supply_voltage;

}answer__PE;

//Пакет «БВМ-ПУСУ»
typedef struct BVM_PUSU {

  uint16_t id;// = 0x0411;
  volatile uint16_t counter;
  volatile uint8_t working_mode;

} BVM__PUSU;

//Пакет «БВМ-ПУСУ (режим маршрут)»
typedef struct BVM_PUSU_ROUTE {

  uint16_t id;// = 0x0412;
  volatile int16_t roll;
  volatile int16_t pitch;
  volatile int16_t course;
  volatile int32_t speed_Vx;
  volatile int32_t speed_Vy;
  volatile int16_t angle_speed_x;
  volatile int16_t angle_speed_y;
  volatile int16_t angle_speed_z;
  volatile int16_t acceleration_x;
  volatile int16_t acceleration_y;
  volatile int16_t acceleration_z;
  volatile float coord_Xc;
  volatile float coord_Yc;
  volatile float coord_Zc;
  volatile int32_t airspeed;

} BVM__PUSU_ROUTE;

//Пакет «БВМ-ПУСУ (режим наведения)»
typedef struct BVM_PUSU_steering{
	uint16_t id;// = 0x0413;
	volatile float bearing_angle_XOY;
	volatile float bearing_angle_XOZ;
	volatile float angle_speed_XOY;
	volatile float angle_speed_XOZ;

}BVM__PUSU_steering;

//Пакет «Ответная квитанции ПУСУ-БВМ»
typedef struct BVM_PUSU_answer{
	uint16_t id;// = 0x0421;
	volatile uint16_t counter;
	volatile uint8_t status_word;
	volatile int8_t roll_signal;
	volatile int8_t pitch_signal;
	volatile int8_t angle_rudder_1;
	volatile int8_t angle_rudder_2;
	volatile int8_t angle_rudder_3;
	volatile int8_t angle_rudder_4;
	volatile int16_t SY_rotation_frequency;
	volatile int8_t amperage_channel_1;
	volatile int8_t amperage_channel_2;
	volatile int8_t amperage_channel_3;
	volatile int8_t amperage_channel_4;
	volatile int8_t amperage_SY;

}BVM__PUSU_answer;

//Ответная квитанция ОЭС-БВМ (режим маршрут)
typedef struct OES_BVM_ROUTE{
	uint16_t id;// = 0x0221;
	volatile uint32_t counter;
	volatile float angle_coord_AZ;
	volatile float angle_coord_UM;
	volatile float angle_speed_AZ;
	volatile float angle_speed_UM;

	struct{
		volatile uint16_t IOES;
		volatile uint16_t grab;
		volatile uint16_t regime_OES;
		volatile uint16_t state_OES;
		volatile uint16_t VSK_TK;
		volatile uint16_t sign_RKS;
		volatile uint16_t reserve;
	}OES;
}OES__BVM_ROUTE;

//Пакет «Данные БОН (СНС)»
typedef struct BON_dates_SNS{
	uint16_t id;// = 0x0323;
	volatile uint32_t counter;
	volatile int16_t info_word;
	volatile uint32_t power_time;
	volatile uint32_t navigation_time;
	volatile uint16_t data_validity;
	volatile uint16_t UTC;
	volatile int32_t latitude;
	volatile int32_t longitude;
	volatile int32_t height;
	volatile uint16_t route_angle;
	volatile uint16_t spitnik_amount_SNS;
	volatile int32_t expiration_time_SNS;

}BON__dates_SNS;


typedef struct BON_dates_INS{
	uint16_t id;// = 0x0322;
	volatile uint32_t counter;
	volatile int16_t info_word;
	volatile uint32_t power_time;
	volatile uint32_t navigation_time;
	volatile uint16_t data_validity;
	volatile int16_t roll;
	volatile int16_t pitch;
	volatile int16_t course;
	volatile int16_t east_speed;
	volatile int16_t north_speed;
	volatile int16_t vertical_speed;
	volatile int16_t module_speed;
	volatile int32_t latitude;
	volatile int32_t longitude;
	volatile int32_t height;
	volatile int32_t baro_height;
	volatile int16_t angle_speedX;
	volatile int16_t angle_speedY;
	volatile int16_t angle_speedZ;
	volatile int16_t boostX;
	volatile int16_t boostY;
	volatile int16_t boostZ;
	volatile uint16_t error_code_BINS;
	volatile int32_t expiration_time_complex;

}BON__dates_INS;

//Пакет «Данные телеметрии»
typedef struct Telemetric_date{
	uint16_t id;// = 0x0611;
	volatile uint16_t counter;
	volatile uint8_t status_BPPD;
	volatile uint8_t ejection_parachute;
	volatile uint8_t signal_band;
	volatile uint8_t summ_subscribers;
	volatile uint8_t signal_level_subscriber1;
	volatile uint16_t distance_subscriber1;
	volatile uint8_t signal_level_subscriber2;
	volatile uint16_t distance_subscriber2;
	volatile uint8_t signal_level_subscriber3;
	volatile uint16_t distance_subscriber3;

}Telemetric__date;

//Пакет «МЦП2 - МЦП1»
typedef struct MCP2_MCP1{
	uint16_t id;// = 0x0612;
	volatile uint16_t counter;
	volatile unsigned char mass_id;
	volatile unsigned int sec;
	volatile unsigned int microsec;
	volatile unsigned char state_LS_OES;
	volatile unsigned char state_MCP2;
	volatile unsigned char result_TVK_MCP2;
	volatile unsigned char video_corection;
	volatile double geo_latitude_video_nav;
	volatile double geo_longitude_video_nav;
	volatile float height_over_sea;
	volatile unsigned char aim_sign;
	volatile bool recognition_sign;
	volatile unsigned char aim_type;
	volatile unsigned short screen_coord_X_object1;
	volatile unsigned short screen_coord_Y_object1;
	volatile int horizontal_aim_size1;
	volatile int vertical_aim_size1;
	volatile float distance_to_aim1;
	volatile bool recognition_aim_12;
	volatile unsigned char aim_type_12;
	volatile unsigned short screen_coord_X_object12;
	volatile unsigned short screen_coord_Y_object12;
	volatile int horizontal_aim_size12;
	volatile int vertical_aim_size12;
	volatile float distance_to_aim12;

}MCP2__MCP1;

//Пакет «МЦП1 - МЦП2»
typedef struct MCP1_MCP2{
	uint16_t id;// = 0x0613;
	volatile uint16_t counter;
	volatile bool working_ARO;
	volatile bool video_correction;
	volatile unsigned char reliability_geo_latitude;
	volatile unsigned char reliability_geo_longitude;
	volatile unsigned char reliability_course;
	volatile unsigned char reliability_geo_height;
	volatile unsigned char reliability_absolut_height;
	volatile unsigned char reliability_pitch;
	volatile unsigned char reliability_roll;
	volatile unsigned char reliability_speed;
	volatile double geo_latitude;
	volatile double geo_longitude;
	volatile float course_LA;
	volatile float geo_height;
	volatile float baro_height;
	volatile float pitch_LA;
	volatile float roll_LA;
	volatile float speed_LA;
	volatile unsigned short summ_EO_in_BD;
	volatile unsigned short type_EO_1;
	volatile unsigned short number_aim_for_CU;
	volatile unsigned short screen_coord_X;
	volatile unsigned short screen_coord_Y;
	volatile unsigned int info_EO;
	volatile double geo_latitude_EO1;
	volatile double geo_longitude_EO1;
	volatile float height_EO1;

}MCP1__MCP2;


//Пакет «Данные управления по каналу телеметрии»
typedef struct Control_Data_Telemetry{
	uint32_t id;// = 0x0621;
	volatile uint16_t counter;
	volatile uint8_t status_PR;
	volatile uint8_t command_time_NPU;
	volatile uint8_t command_code;
	volatile uint8_t command_parametr;
}Control__Data_Telemetry;

typedef struct Date{
	BVM__BSN bvm_bsn;
	BVM__INFO bvm_info;
	BSM__BVM_Answer_fixed bsm_bvm_answer_fixed;
	BSM__BVM_Answer_move bsm_bvm_answer_move;
	BSM__BVM_Answer_ephemeris bsm_bvm_answer_ephemeris;
	BVM__PE bvm_pe;
	answer__PE answer_pe;
	BVM__PUSU bvm_pusu;
	BVM__PUSU_ROUTE bvm_pusu_route;
	BVM__PUSU_steering bvm_pusu_steering;
	BVM__PUSU_answer bvm_pusu_answer;
	OES__BVM_ROUTE oes_bvm_route;
	BON__dates_SNS bon_dates_sns;
	BON__dates_INS bin_dates_ins;
	Telemetric__date telemetric_date;
	MCP2__MCP1 mcp2_msp1;
	MCP1__MCP2 mcp1_msp2;
	Control__Data_Telemetry control_data_telemetry;

}date;

