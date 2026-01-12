//Includes
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>

/*Structure de données du controleur*/
CONTROLLER_STRUCT rtController;

/* Varaibles spécifiques à la communication avec le bus EtherCAT */
static ec_master_t          *master        = NULL;
static ec_master_state_t    master_state   = {};
static ec_domain_t          *domain        = NULL;
static ec_domain_state_t    domain_state   = {};
static uint8_t              *domain_pd     = NULL;


/* Varaibles spécifiques à la tâche temps-réel périodique */
/*Tâche periodique de fréqeucen 100hz*/
long long main_task_period_ns = 10*1000*1000LL;
RT_TASK rtController_Main_Task;
#define RTCONTROLLER_MAIN_TASK_STKSZ 4096               /* taille de la pile execution */
#define RTCONTROLLER_MAIN_TASK_PRIO 99                  /*priorité */
#define RTCONTROLLER_MAIN_TASK_MODE 0                   /* utilisation FPU,  CPU #0 */

/* Beckhoff modules */
const uint8_t EL5101_Enable_C_Reset[]          = {0};


/***************************************************************/
/********** A FAIRE 1 (DEBUT)*************************************/

/**** VERIFIER LES MODULES ETHERCAT : Vendor ID, Product Code **/
#define BECKHOFF_EK1100 0x00000002, 0x044C2C52
#define BECKHOFF_EL5101 0x00000002, 0x13ed3052
#define BECKHOFF_EL4132 0x00000002, 0x10243052
#define BECKHOFF_EL3102 0x00000002, 0x0c1e3052

/**** VERIFIER LA TOPOLOGIE DE VOTRE BUS ID MASTER, SLAVE POSITION **/
#define BusCouplerPos           0, 0    //EK1100
#define AnaOutSlavePos          0, 2    //EL4132
#define CounterSlavePos         0, 1    //EL5101
#define AnaInSlavePos           0, 3    //EL3102

/***** INTEGRER LES STRUCTURES DE DONNEES PDO POUR CHAQUE ESCLAVE*/

/* Master 0, Slave 1, "EL5101"
 * Vendor ID:       0x00000002
 * Product code:    0x13ed3052
 * Revision number: 0x03ff0000
 */
ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x7010, 0x01, 1}, /* Enable latch C */
    {0x7010, 0x02, 1}, /* Enable latch extern on pos */
    {0x7010, 0x03, 1}, /* Set counter */
    {0x7010, 0x04, 1}, /* Enable latch extern on neg */
    {0x0000, 0x00, 4}, /* Gap */
    {0x0000, 0x00, 8}, /* Gap */
    {0x7010, 0x11, 32}, /* Set counter value */
    {0x6010, 0x01, 1}, /* Latch C valid */
    {0x6010, 0x02, 1}, /* Latch extern valid */
    {0x6010, 0x03, 1}, /* Set counter done */
    {0x6010, 0x04, 1}, /* Counter underflow */
    {0x6010, 0x05, 1}, /* Counter overflow */
    {0x6010, 0x06, 1}, /* Status of input status */
    {0x6010, 0x07, 1}, /* Open circuit */
    {0x6010, 0x08, 1}, /* Extrapolation stall */
    {0x6010, 0x09, 1}, /* Status of input A */
    {0x6010, 0x0a, 1}, /* Status of input B */
    {0x6010, 0x0b, 1}, /* Status of input C */
    {0x6010, 0x0c, 1}, /* Status of input gate */
    {0x6010, 0x0d, 1}, /* Status of extern latch */
    {0x6010, 0x0e, 1}, /* Sync error */
    {0x6010, 0x0f, 1}, /* TxPDO State */
    {0x6010, 0x10, 1}, /* TxPDO Toggle */
    {0x6010, 0x11, 32}, /* Counter value */
    {0x6010, 0x12, 32}, /* Latch value */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1603, 7, slave_1_pdo_entries + 0}, /* ENC RxPDO-Map Control */
    {0x1a04, 18, slave_1_pdo_entries + 7}, /* ENC TxPDO-Map Status */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 2, "EL4132"
 * Vendor ID:       0x00000002
 * Product code:    0x10243052
 * Revision number: 0x03fb0000
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x3001, 0x01, 16}, /* Output */
    {0x3002, 0x01, 16}, /* Output */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 1, slave_2_pdo_entries + 0}, /* RxPDO 01 mapping */
    {0x1601, 1, slave_2_pdo_entries + 1}, /* RxPDO 02 mapping */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_2_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 3, "EL3102"
 * Vendor ID:       0x00000002
 * Product code:    0x0c1e3052
 * Revision number: 0x00140000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x3101, 0x01, 8}, /* Status */
    {0x3101, 0x02, 16}, /* Value */
    {0x3102, 0x01, 8}, /* Status */
    {0x3102, 0x02, 16}, /* Value */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1a00, 2, slave_3_pdo_entries + 0}, /* TxPDO-Map Channel 1 */
    {0x1a01, 2, slave_3_pdo_entries + 2}, /* TxPDO-Map Channel 2 */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_3_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/********** A FAIRE 1 (FIN)*************************************/
/***************************************************************/

/*Initialisation des Structures de données des modules*/
ANALOG_INPUT_STRUCT     AnalogIn[NB_ANALOG_INPUT]={NULL,0,0};
ANALOG_OUTPUT_STRUCT    AnalogOut[NB_ANALOG_OUTPUT]={NULL,0,0};
COUNTER_STRUCT          Counter[NB_INCR_ENCODER]={NULL,0,0,0,0,0,0,0,0,0,0,0};

/**********************************************************/
/******* FONCTIONS BUS ETHERCAT ***************************/
/**********************************************************/
/**
  * @fn     void rt_check_domain_state(void
  * @brief
  * @arg    aucun
  * @return aucun
 */
void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain_state = ds;
}


/**
  * @fn     void rt_check_master_state()
  * @brief
  * @arg    aucun
  * @return aucun
 */
void rt_check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        fprintf(stdout,"AL states: 0x%02X.\n", ms.al_states);
        fflush(stdout);
    }

    if (ms.al_states != master_state.al_states) {
        fprintf(stdout,"AL states: 0x%02X.\n", ms.al_states);
        fflush(stdout);
    }

    if (ms.link_up != master_state.link_up)
    {
        fprintf(stdout,"Link is %s.\n", ms.link_up ? "up" : "down");
        fflush(stdout);// rt_printf("AL states: 0x%02X.\n", ms.al_states);

    }

    master_state = ms;

}

/**********************************************************/
/***                                                    ***/
/***     FONCTIONS ACQUISITION/CONTROLE                 ***/
/***                                                    ***/
/**********************************************************/

///
/// \fn rtController_read_analog_input
/// \brief lecture de l'ensemble des entrées analogiques du bus et restitution de la tension
/// \return code Erreur
///
int rtController_read_analog_input(double *value)
{

    int16_t adc_value;

    adc_value = EC_READ_S16(domain_pd+AnalogIn[0].offset_value_in);


    *value = (double)adc_value*20.0/65536.0;
    return NO_ERROR;
}

///
/// \fn rtController_write_analog_output
/// \brief envoi les sorties analogiques
/// \return
int rtController_write_analog_output(double value)
{
    int16_t dac_value;

    AnalogOut[0].value = value;
    /* Vérification Nan*/
    if ( isnan(value))
       AnalogOut[0].value = 0;
    /* Saturation*/
    if (value > 10)
        AnalogOut[0].value= 10;
    if (value < -10)
        AnalogOut[0].value = 10;

    //Conversion DAC de la tension de sortie puis envoie
    dac_value = (int16_t)(AnalogOut[0].value*65536.0/20.0);
    EC_WRITE_S16(domain_pd + AnalogOut[0].offset_output, dac_value);

    return NO_ERROR;
}

int rtController_reset_counter(void)
{
	Counter[0].init_value =  Counter[0].value;/*(int32_t)(position*1/sensor_voltage_to_SI[0]);*/		
	return NO_ERROR;
}

/**
 * \fn rtController_read_incr_encoder
 *  \brief lecture des codeurs incrémentaux
 *  \return code erreur
**/
int rtController_read_incr_encoder(int id, int32_t *encoder_value)
{
    uint8_t control;
    if (( Counter[id].prevoverfl == 0 ) && (Counter[id].overflow ==1))
         Counter[id].turn++;

     //Underflow de l'encodeur? (Bit 3 de status: underflow)
     else if ((Counter[id].prevunderfl == 0) && (Counter[id].underflow== 1))
         Counter[id].turn--;

     Counter[id].prevoverfl = Counter[id].overflow;
     Counter[id].prevunderfl = Counter[id].underflow;

    Counter[id].value = EC_READ_U32(domain_pd+Counter[id].offset_value_in);
    Counter[id].value += Counter[id].turn*2^32;

    *encoder_value =   (int32_t)Counter[id].value-Counter[id].init_value;

    return NO_ERROR;
}



///
/// \brief rtController_stop
/// \param sig
///
void rtController_stop(int sig)
{


    rtController.bIsRunning = 0;


}


/**
 * @fn rtController_Init_ControllerStruct
 * @brief Initialisation de la structure de données du controleur
 */
void rtController_Init_ControllerStruct(void)
{
    int i;
    rtController.mode        = WAIT_MODE;
    rtController.submode     = WAIT_SUBMODE;
    rtController.bIsCal      = 0;
    rtController.bIsRunning  = 0;
    rtController.bIsTrackingEnd = 0;
    rtController.CounterValue[0] = 0;
    rtController.control[0] = 0;
    for (i= 0; i < 2;i++)
    {
        rtController.measure[i] = 0;
        rtController.position_ref[i] = 0;
    }
    rtController.measure[0] = InitAngularPosition;
    rtController.current_time = 0;
    rtController.start_time = 0;
}


/**
  * Fonctions associées à la structure du système robotique
  */

/**
* \fn rtController_UpdateRobot
* \brief Mise à jour des données de la structure de données associée à la structure robotique (mapping matériel ethercat / robot)
* \return Code erreur
**/
double Calculer_MGD(double theta1)
{

    double l1 = 30.0;
    double l2 = 90.0;
    double l3 = 90.0;
    double l4 = 56.0;
    double l5 = 69.0;
    double l7 = 90.0;
    double l8 = 120.0;
    double l9 = 45.0;
    double l10 = 35.0;
    double alpha1 = 60.0 * (M_PI / 180.0); // 1.04719... rad


    double xB = l1 * cos(theta1);
    double yB = l1 * sin(theta1);


    double xD = l5;
    double yD = -l4;


    double BD2 = pow(xD - xB, 2) + pow(yD - yB, 2);
    double BD = sqrt(BD2);
    double angle_BD = atan2(yD - yB, xD - xB);


    // Attention à borner le cosinus entre -1 et 1 pour éviter les NaN
    double cos_gamma = (pow(l2, 2) + BD2 - pow(l3, 2)) / (2.0 * l2 * BD);
    
    if (cos_gamma > 1.0) cos_gamma = 1.0;
    if (cos_gamma < -1.0) cos_gamma = -1.0;
    
    double local_gamma = acos(cos_gamma); 


    double theta2 = angle_BD + local_gamma;
    double theta7 = theta2 + alpha1;


    double xE = xB + l7 * cos(theta7);
    double yE = yB + l7 * sin(theta7);


    double sin_theta4 = (l9 - yE) / l8;
    
    if (sin_theta4 > 1.0) sin_theta4 = 1.0;
    if (sin_theta4 < -1.0) sin_theta4 = -1.0;

    double cos_theta4 = sqrt(1.0 - pow(sin_theta4, 2)); 

    double xF = xE + l8 * cos_theta4 - l10;

    return xF;
}

int rtController_UpdateRobot(void)
{
    /* mesure de la position angulaire q1 en rad */
    rtController.measure[0] = joint_direction[0]*(double)(rtController.CounterValue[0])*sensor_voltage_to_SI[0]; 
    rtController.measure[0] = rtController.measure[0];
    
    /* mesure de la position x en mm (Capteur Réel) */
    rtController.measure[1] = (double)(rtController.analog_input_value)*sensor_voltage_to_SI[1];

    // --- MODIFICATION ICI ---
    // On calcule la position théorique via le MGD en utilisant l'angle mesuré (measure[0])
    double x_calcule = Calculer_MGD(rtController.measure[0]);
    
    // On stocke le résultat dans la référence [1] (ou ailleurs selon tes besoins d'affichage)
    // Cela te permettra de tracer : Mesure Réelle vs Mesure Calculée
    rtController.position_ref[1] = x_calcule; 

    return NO_ERROR;
}




/**
 * @brief rtController_DeviceUpdate
 */
void rtController_DeviceUpdate(void)
{
    /*Reception des données provenant du bus*/
    ecrt_master_receive(master);
    ecrt_domain_process(domain);
    rt_check_master_state();

    /* Mise à jour des données sur le bus ethercat*/
    rtController_read_incr_encoder(0, &rtController.CounterValue[0]);
    rtController_write_analog_output(rtController.control[0]);
    rtController_read_analog_input(&rtController.analog_input_value);
    /*Envoi des données vers le bus*/
    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    /* Mapping et mise à jour des structures de données associées à chaque éléments du système*/
    rtController_UpdateRobot();
}



/******************************************************************************************************/

int rtController_Wait_Proc(void)
{
    rtController.mode = CAL_MODE;

    return NO_ERROR;
}

int rtController_doCal_Proc(void)
{
    int i;
    rtController.control[0] = 0.0;
    rtController_reset_counter();
    rtController.bIsCal = 1;
    return NO_ERROR;
}
int rtController_doJPosStatic_Proc(void)
{
    int i;
    rtController.control[0] = 0.0;

    return NO_ERROR;
}

/***************************************************************/
/********** A FAIRE 3 (DEBUT)*************************************/
/****   Lorsque le controleur bascule en mode POSITION_TRACKING_MODE
 * la fonction suivante est appelée à chaque pas d'échantillonnage => déja implémenté 

 * Dans cette fonction nous souhaitons mettre en oeuvre un asservissement de
 * position dans l'espace articulaire. Faire une loi de commande qui calcule
 * rtControler.control à chaque pas pour que la position angulaire q1 suive
 * trajectoire sinusoidale de fréquence 0,5 Hz d'amplitude 45 degrés centrée
 * autour de la position zéro; => ça c'est bon

pendant une durée de 5 secondes.

 * La loi de commande n'a pas besoin d'etre complexe, un simple correcteur
 * propotionnel devrait suffir (par précaution rendre une valeur de gain faible
 * pour ensuite l'augmenter doucement - c'est un réglage empirique sans modèle
 * a priori) assurez-vous que la commande reste bornée et stable => go faire ca

 * Assurez vous également que vous avez bien renseigné la position initiale
 * de l'angle q1 dans le fichier main.h  => voir avec à alex 

 * Une fois cet asservissement validé (en accord avec l'enseignant) tester le. => objectif de l'aprem

 * Ensuite implenter le modèle géométrique direct dans une nouvelle fonction et comparer la
 * position mesurée et la position calcul (ce modèle aura été validé en simulation avant
 *  ... que pouvez-vous en conclure ?
  **/

int rtController_doPositionTracking_Proc(void){

    /* 1. TEMPS */
    double temps = (rtController.current_time - rtController.start_time)*(1e-9); 
    
    /* 2. PARAMETRES */
    double Freq = 0.5; 
    double Amp = M_PI/4; 
    double Consigne = 0.0;
    
    /* 3. CALCUL DE LA CONSIGNE */
    if (temps < 5.0) 
    {
        Consigne = 0.0 ; // Maintien à 0
    }
    else 
    {
        double temps_sinus = temps - 5.0;
        Consigne = Amp * sin(2 * Freq * M_PI * temps_sinus);
    }

    /* --- CORRECTION ICI : On envoie la consigne à l'affichage --- */
    rtController.position_ref[0] = Consigne; 
    /* ------------------------------------------------------------ */

    /* 4. MESURES */
    double q1_mes = rtController.measure[0];
    double X_mes = rtController.measure[1]; 
    
    /* 5. SOFT START (Kp progressif) */
    double Kp_Final = 15.0;
    double Kp_Start = 5.0; 
    double Kp_Actuel = 0.0;

    if (temps < 2.0) {
        Kp_Actuel = Kp_Start + (Kp_Final - Kp_Start) * (temps / 2.0); 
    } else {
        Kp_Actuel = Kp_Final;
    }

    /* 6. CALCUL COMMANDE */
    double erreur = Consigne - q1_mes;
    double Commande_angulaire_moteur = Kp_Actuel * erreur;  
    
    if (Commande_angulaire_moteur > 10.0) Commande_angulaire_moteur = 10.0;
    if (Commande_angulaire_moteur < -10.0) Commande_angulaire_moteur = -10.0;

    rtController.control[0] = Commande_angulaire_moteur;

    /* 7. LOGGING */
    double X_calc_MGD = rtController.position_ref[1]; 
    
    FILE *fptr;
    fptr = fopen("mesures.txt","a");
    if (fptr != NULL) {
        fprintf(fptr, "%f %f %f %f %f %f\n", temps, Kp_Actuel, Consigne, q1_mes, X_mes, X_calc_MGD);
        fclose(fptr);
    }
    
    if (temps > 20.0) {
        rtController.bIsTrackingEnd = 1;
        rtController.control[0] = 0.0;
    }

    return NO_ERROR;
}
/********** A FAIRE 3 (FIN)*************************************/
/***************************************************************/

/**
  * \fn rtController_main_sequence
  *  \brief gestion des modes de fonctionnement du robot
  *
  **/
void rtController_main_sequencer(void)
{
    switch( rtController.mode)
    {

    /* Mode Attente */
    case WAIT_MODE:
        rtController_Wait_Proc();
        break;
  
    /* Calibration */
    case CAL_MODE:
        printf("Calibration\n");
        if ( !rtController.bIsCal)
        {
            if (rtController_doCal_Proc() < 0)
                break;
        }
        else
        {
            rtController.current_time = 0;
            rtController.bIsTrackingEnd = 0;
            rtController.mode = POSITION_TRACKING_MODE;
        }

        break;

    case JPOS_STATIC_MODE:
	rtController_doJPosStatic_Proc();
        rtController.current_time = 0;
        break;

    case POSITION_TRACKING_MODE:
        if ( !rtController.bIsCal)
            rtController.mode = CAL_MODE;
        else
        {

            if (!rtController.bIsTrackingEnd)
                rtController_doPositionTracking_Proc();
            else
                rtController.mode = JPOS_STATIC_MODE;

        }
        break;
    default:
        break;
    }
}

///
/// \fn rtController_config_ethercat
/// \brief configuration du bus en fonction des modules esclaves
/// \return code d'erreur si erreur lors de la configuration d'un module
///
int  rtController_config_ethercat(void)
{
    uint32_t abort_code;

  /* Configuration du module Coupleur EK1100*/
  if (!ecrt_master_slave_config(master, BusCouplerPos, BECKHOFF_EK1100))
    return -1;
  
  /* convertisserur du module Comptage  EL5101*/
  Counter[0].sc = ecrt_master_slave_config(master,
                                                        CounterSlavePos,
                                                        BECKHOFF_EL5101);
  if (!Counter[0].sc)
    return -1;
  if (ecrt_slave_config_pdos(Counter[0].sc,
                               EC_END,
                               slave_1_syncs))
    return -1;
  ecrt_master_sdo_download(master,1, 0x8010, 0x01, EL5101_Enable_C_Reset, sizeof(EL5101_Enable_C_Reset), &abort_code);
  //Value in
  Counter[0].offset_value_in = ecrt_slave_config_reg_pdo_entry(Counter[0].sc,
                                                                            0x6010,
                                                                            0x11,
                                                                            domain,
                                                                            NULL);
  if ( Counter[0].offset_value_in < 0)
  {
      printf("Failed to register PDO Entry\n");
      return -1;
  }
  //Latch
  Counter[0].offset_latch = ecrt_slave_config_reg_pdo_entry(Counter[0].sc,
                                                               0x6010,
                                                               0x12,
                                                               domain,
                                                               NULL);
  if (Counter[0].offset_latch < 0)
  {
      printf("Failed to register PDO Entry\n");
      return -1;
  }
  //Ctrl
  Counter[0].offset_ctrl = ecrt_slave_config_reg_pdo_entry(Counter[0].sc,
                                                              0x7010,
                                                              0x01,
                                                              domain,
                                                              NULL);
  if (Counter[0].offset_ctrl < 0)
  {
      printf("Failed to register PDO Entry\n");
      return -1;
  }

  //Value out
  Counter[0].offset_value_out = ecrt_slave_config_reg_pdo_entry(Counter[0].sc,
                                                                   0x7010,
                                                                   0x11,
                                                                   domain,
                                                                   NULL);
  if (Counter[0].offset_value_out < 0)
  {
      printf( "Failed to register PDO Entry\n");
      return 1;
  }

  /* convertisseur du module CNA EL4132*/
  AnalogOut[0].sc = ecrt_master_slave_config(master,
                                                          AnaOutSlavePos,
                                                          BECKHOFF_EL4132);
  if (!AnalogOut[0].sc)
    return -1;
  if (ecrt_slave_config_pdos(AnalogOut[0].sc, EC_END, slave_2_syncs))
    return -1;

  AnalogOut[0].offset_output= ecrt_slave_config_reg_pdo_entry(AnalogOut[0].sc,
                                                               0x3001,
                                                               0x01,
                                                               domain,
                                                               NULL);

  /* convertisserur du module CAN EL3102*/
  AnalogIn[0].sc = ecrt_master_slave_config(master,
                                                         AnaInSlavePos,
                                                         BECKHOFF_EL3102);
  if (!AnalogIn[0].sc)
    return -1;
  if (ecrt_slave_config_pdos(AnalogIn[0].sc, EC_END, slave_3_syncs))
    return -1;
  
  AnalogIn[0].offset_value_in= ecrt_slave_config_reg_pdo_entry(AnalogIn[0].sc,
                                                               0x3101,
                                                               0x02,
                                                               domain,
                                                               NULL);

    return 0;
}


/**
 * \brief procédure de connection au bus ethercat
 *  \param arg
**/
void rtController_Main_Proc(void *arg)
{
    int ret, i;
    unsigned long mask_ret;
    RTIME start_ns;
    RTIME start_time;
    RTIME current_time;
    /* Initialisation de la structure associée au controleur */
    rtController_Init_ControllerStruct();
    /*****************************************************************************************/
    /* Configuration des modules EtherCAT */


    /*****************************************************************************************/
    /* Initialisation et paramétragre de la communication via le bus ethercat                */ 
    /*****************************************************************************************/
    master = ecrt_request_master(0);
    if (!master)
    {
        printf("Echec requete master\n");
        return ;
    }
    /*****************************************************************************************/
    domain = ecrt_master_create_domain(master);
    if ( !domain)
    {
        printf("Creation of domain failed\n");
	ecrt_release_master(master);
	return;
    }


    if ( rtController_config_ethercat() < 0)
      {
	printf("Echec configuration modules escslaves.\n");
	ecrt_release_master(master);
	return;
      }

    printf("Activation du master\n");
    if (ecrt_master_activate(master))
    {
      	printf("Echec activation master.\n");
      	ecrt_release_master(master);
	return;
    }
    printf("Initialisation du domaine de données ...\n");
    domain_pd = ecrt_domain_data(domain) ;


    rtController.bIsRunning = 1;
    sleep(1);
    printf("Démarrage tâche pếriodique avec une période de 10 ms ...\n");

    /***************************************************************/
    /********** A FAIRE 2 bis (DEBUT)*************************************/
    /****  Périodiser la tâche avec un décalage d'exécution de 10 ms par exemple
	la boucle infini sur le flag bIsRunning du la strucutre rtController
	Fonctions de la tâche 
		- mettre à jour les information du Device (rtController_DeviceUpdate)
		- mesurer le temps courant
		- si master_state.al_states == EC_MASTER_OP
			alors utiliser le graphe d'état du main_sequencer (rtController_main_sequencer())
	utilisez au maximum les variables déjà définies

 */

    /*On est dans la fonction rtController_Main_Proc */
    printf("\n start_time is : \n"); 
    printf ("%d",start_ns,"\n");

    rtController.start_time = rt_timer_read();
    start_ns = rtController.start_time + 10000000;
    printf("\n start_time is : \n");
    printf ("%d",start_ns,"\n");

    /* rtController.current_time = rtController.start_time; */
    rt_task_set_periodic (NULL, start_ns, main_task_period_ns); /*bascule la tache courante en tache periodique de peiode spécifiée*/
    i=0;

    int cycle_counter = 0; // Compteur pour gérer l'affichage

    while(rtController.bIsRunning) 
    {
        /* 1. Attente du top horloge (Cadencement 10ms) */
        rt_task_wait_period(NULL);
        
        /* 2. Lecture / Ecriture sur le Bus EtherCAT */
        rtController_DeviceUpdate();

        /* 3. Mise à jour du temps */
        rtController.current_time = rt_timer_read();
        
        /* 4. Gestion de la machine d'état (Si le bus est OK) */
        rt_check_master_state(); // Met à jour master_state
        
        if (master_state.al_states == 0x08) { // 0x08 = OP (Opérationnel)
            rtController_main_sequencer();
        }

        /* 5. TABLEAU DE BORD (Affichage Terminal propre) */
        /* On affiche seulement tous les 50 cycles (50 * 10ms = 0.5 sec) */
        cycle_counter++;
        if (cycle_counter % 50 == 0) 
        {
            // On efface l'écran ou on saute des lignes pour la lisibilité
            // Affichage de l'état
            double t_s = (double)rtController.current_time / 1e9;
            printf("--------------------------------------------------\n");
            printf("Temps: %.2f s | Mode: %d \n", t_s, rtController.mode);
            
            if (rtController.mode == POSITION_TRACKING_MODE) {
                // Affichage des données clés en temps réel
                printf("TRACKING EN COURS :\n");
                printf(" > Consigne Angle : %.3f rad\n", rtController.position_ref[0]);
                printf(" > Mesure  Angle : %.3f rad\n", rtController.measure[0]);
                printf(" > Erreur        : %.3f rad\n", rtController.position_ref[0] - rtController.measure[0]);
            } else {
                printf("Etat Master: 0x%02X (Attente tracking...)\n", master_state.al_states);
            }
            printf("--------------------------------------------------\n");
        }
    }
    /********** A FAIRE 2 bis (FIN)*************************************/
    /***************************************************************/ 






   
   
    printf("Fin de la tâche principale.\n");



}





/**
  * @fn int main(void)
 * @brief Fonction principale
 * @return
 */
int main(void)
{

    int ret,i;
    sigset_t set;
    int sig;


    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGKILL);
    signal(SIGTERM, rtController_stop);
    signal(SIGINT,  rtController_stop);
    signal(SIGHUP,  rtController_stop);//Interruption interactive (Ctrl C ou clavier)
    signal(SIGKILL, rtController_stop);
    /*** Initialisation de la structure*/
    /* Interdit le swapping de mémoire pour ce programme*/
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        exit(-2);
    }
    iopl(3);



/***************************************************************/
    /********** A FAIRE 2 (DEBUT)*************************************/
    /****  Créer la tache temps reel ayant pour identifiant rtController_Main_Task;
            avec pour nom "RTCONTROLLER_MAIN_TASK" avec les paramètres
            définis au début du programme pour la stack; priorité et mode
            Lancer l'exécution de la tâche en lui associant la fonction
            rtController_Main_Proc*/

    ret = rt_task_create(
	&rtController_Main_Task, /*Adresse du descripteur de la tache ?*/
	"RTCONTROLLER_MAIN_TASK", /*nom de la tache*/
	RTCONTROLLER_MAIN_TASK_STKSZ, /*Taille allouee à la pile de la tache*/
	RTCONTROLLER_MAIN_TASK_PRIO, /*priorite de base de de la tache*/
	RTCONTROLLER_MAIN_TASK_MODE /*mode de base associé à la tache*/
	);

    if(ret)
	perror("Impossible de démarrer la tâche ");

    ret = rt_task_start( /*demarrage de la tache*/
	&rtController_Main_Task, /*Adresse du descripteur de la tache ?*/
	&rtController_Main_Proc, /*pointeur sur la fonction*/
	NULL /*arguments de la fonction*/
	); 

    if(ret){
	perror("Démarrage de la tâche "); 
	rt_task_delete(&rtController_Main_Task); /*destruction de la tache*/
	exit(1);
    }

    /********** A FAIRE 2 (FIN)*************************************/
    /***************************************************************/


    pause();
    rt_task_delete(&rtController_Main_Task);
    ecrt_release_master(master);
    return 0;
}


