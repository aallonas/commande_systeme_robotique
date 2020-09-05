#ifndef __EC_CONTROLLER_H_
#define __EC_CONTROLLER_H_
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/io.h>
#include "ecrt.h"
#include <math.h>


/*** Ethercat Master State ***/
#define EC_MASTER_INIT      0x00
#define EC_MASTER_PREOP     0x02
#define EC_MASTER_SAFEOP    0x04
#define EC_MASTER_OP        0x08 // A changer si AL_STATE 0x08


/***   PARAMETRES DU SYSTEME ***/
#define NB_POSITION_SENSORS     1
#define NB_MOTORS               1
#define NB_JOINTS               1
#define NB_DOF                  1



static int joint_direction[NB_POSITION_SENSORS] = {1};
//
static double sensor_voltage_to_SI[2] = { 2*M_PI/(83*4*500),// rad/top
                                        30.17}; //mm/Vx



/*Valeur initiale a reneigner avant de lancer une commande du systee*/
static double InitAngularPosition = M_PI/2; //en rad

/*** ERREURS ***/
#define __NO_ERROR          0
#define NO_ERROR            0
/*MODE DE FONCTIONNEMENT*/
#define WAIT_MODE               0 /*Attente*/
#define CAL_MODE                1 /*Etalonnage des compteurs*/
#define JPOS_STATIC_MODE        2 /*Robot étalonné en position statique*/
#define POSITION_TRACKING_MODE  3 /*Suivi de mouvement*/
/*SOUS-MODE DE FONCTIONNEMENT*/
#define WAIT_SUBMODE                0


#define NB_ANALOG_INPUT     1
#define NB_ANALOG_OUTPUT    1
#define NB_INCR_ENCODER     1



#define TRUE 0
#define FALSE 1

typedef struct{
    ec_slave_config_t *sc;
    uint8_t overflow;
    uint8_t underflow;
    int turn;
    int prevoverfl;
    int prevunderfl;
    int offset_input_status;
    int offset_value_in;
    int offset_ctrl;
    int offset_latch;
    int offset_value_out;
    uint32_t value;
	int32_t init_value;/*Valeur initale du compteur en relation avec InitAngularPosition*/

}COUNTER_STRUCT;

typedef struct{
    ec_slave_config_t *sc;
    int offset_value_in;
    double value;
}ANALOG_INPUT_STRUCT;

typedef struct{
    ec_slave_config_t *sc;
    int offset_output;
    double value;
}ANALOG_OUTPUT_STRUCT;





/*Structure de données associées au controleur*/
typedef unsigned int cmd_type;
typedef unsigned int mode_type;
typedef unsigned int error_type;
typedef struct{
    mode_type               mode;
    mode_type               submode;
    /// Flags de fonctionnements
    uint8_t                 bIsRunning;
    uint8_t                 bIsCal;
    uint8_t                 bIsTrackingEnd;
    double                  measure[2]; /*0 mesure angulaire q1 en rad
                                          1 mesure linéaire x en mm*/
    double                  control[1]; /*tension de commande V*/
    double                  position_ref[2];/*0 consigne angulaire q1 en rad
                                                1 consigne linéaire x en mm*/

    int32_t                 CounterValue[1]; /*Valeur numerique du compteur*/

    double                  analog_input_value;
    double                  current_time;
    double                  start_time;
}CONTROLLER_STRUCT;


void rtController_stop(int sig);
#endif
