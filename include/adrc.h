/*
 * File     :adrc.h
 *
 * Chang Logs:
 * Date           Author         Notes
 * 2020-02-25     xianglunkai     the first version
 */

#ifndef __adrc_h
#define __adrc_h


#define SCU_USE_ADRC       /**< If use adrc as controller alogrithm to control robot */


#if defined(SCU_USE_ADRC)

/* TD controller */
typedef struct{

   float h;
   float v1;
   float v2;
   float r2;
   float h2;
}adrc_td_control_t;


/* TD filter */
typedef struct{

   float h;
   float v1;
   float v2;
   float r0;
   float h0;
}adrc_td_filter_t;


/* 1 order ESO */
typedef struct{

    float h;
    float beta1;
    float beta2;
    float u;
    float b0;

    float z1;
    float z2;
}adrc_eso1rd_t;

/* 2 order ESO */
typedef struct{

    float h;
    float beta1;
    float beta2;
    float beta3;
    float u;
    float b0;

    float z1;
    float z2;
    float z3;
}adrc_eso2rd_t;


/* nlsef */
typedef struct{

    float h;
    float h1;
    float r1;
    float c;
}adrc_nlsef_t;


/**
 * Public functions
 *
 */
float adrc_sign(float val);
float adrc_fhan(float v1, float v2, float r0, float h0);
float adrc_fal(float e, float alpha, float delta);

/* nlsef */
float adrc_fal(float e, float alpha, float delta);

void adrc_nlsef_init(adrc_nlsef_t* nlsef_t, float h, float r1, float h1, float c);
float adrc_nlsef_process(adrc_nlsef_t* nlsef_t, float e1, float e2);

/**
 * @brief: td filter parameters initialization and process
 */
void adrc_td_filter_init(adrc_td_filter_t *td_filter,float h,float r0,float h0);
void adrc_td_filter_process(adrc_td_filter_t *td_filter,float v);
void adrc_td_filter_reset(adrc_td_filter_t * td_filter,float v1,float v2);

/*
 * @brief: td controller parameters initialization and process
 **/
void adrc_td_control_init(adrc_td_control_t *td_control,float h,float r2,float h2);
float adrc_td_control_process(adrc_td_control_t *td_control,float err);
void adrc_td_control_reset(adrc_td_control_t *td_control,float v1,float v2);


/**
 * @brief: 1 order nonlinear ESO parameter initialization and process
 */
void adrc_neso1rd_init(adrc_eso1rd_t *eso,float h,float beta1,float beta2,float b0);
void adrc_neso1rd_process(adrc_eso1rd_t *eso,float y,float u);
void adrc_neso1rd_reset(adrc_eso1rd_t *eso,float z1,float z2);


/**
 * @brief: 1 order linear ESO parameter initialization and process
 */
void adrc_leso1rd_init(adrc_eso1rd_t *eso,float h,float w,float b0);
void adrc_leso1rd_process(adrc_eso1rd_t *eso,float y,float u);
void adrc_leso1rd_reset(adrc_eso1rd_t *eso,float z1,float z2);

/**
 * @brief: 2 order nonlinear ESO parameter initialization and process
 */
void adrc_neso2rd_init(adrc_eso2rd_t* eso_t, float h, float beta1, float beta2, float beta3,float b0);
void adrc_neso2rd_process(adrc_eso2rd_t* eso_t, float y,float u);
void adrc_neso2rd_reset(adrc_eso2rd_t *eso,float z1,float z2,float z3);

/**
 * @brief:  2 order linear ESO parameter initialization and process
 */
void adrc_leso2rd_init(adrc_eso2rd_t* eso_t, float h, float w,float b0);
void adrc_leso2rd(adrc_eso2rd_t* eso_t, float y,float u);
void adrc_leso2rd_reset(adrc_eso2rd_t *eso,float z1,float z2,float z3);


#endif/* if defined(SCU_USE_ADRC)*/

#endif
