/*
 * File     :adrc.c
 *
 * Chang Logs:
 * Date           Author         Notes
 * 2020-02-25     xianglunkai     the first version
 */

#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_auto.h"
#ifndef WINNT
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#else
#include <sys/timeb.h>
#endif

#include <string.h>



#if defined(SCU_USE_ADRC)

float adrc_sign(float val)
{
    if(val > 0.0f)
        return 1.0f;
    else if(val < 0.0f)
        return -1.0f;
    else
        return 0.0f;
}

float adrc_fhan(float v1, float v2, float r0, float h0)
{
    float d = h0 * h0 * r0;
    float a0 = h0 * v2;
    float y = v1 + a0;
    float a1 = sqrtf(d*(d + 8.0f*fabsf(y)));
    float a2 = a0 + adrc_sign(y)*(a1-d)*0.5f;
    float sy = (adrc_sign(y+d) - adrc_sign(y-d))*0.5f;
    float a = (a0 + y - a2)*sy + a2;
    float sa = (adrc_sign(a+d) - adrc_sign(a-d))*0.5f;

    return -r0*(a/d - adrc_sign(a))*sa - r0*adrc_sign(a);
}


float adrc_fal(float e, float alpha, float delta)
{
    if(fabsf(e) <= delta)
    {
        return e / (powf(delta, 1.0f-alpha));
    }
    else
    {
        return powf(fabsf(e), alpha) * adrc_sign(e);
    }
}



/**
 * @brief: td filter parameters initialization and process
 */
void adrc_td_filter_init(adrc_td_filter_t *td_filter,float h,float r0,float h0)
{
    td_filter->h = h;

    td_filter->r0 =  r0;
    td_filter->h0  = h0;

    td_filter->v1 = td_filter->v2 = 0.0f;
}

void adrc_td_filter_reset(adrc_td_filter_t * td_filter,float v1,float v2)
{
    td_filter->v1 = v1;
    td_filter->v2 = v2;
}


void adrc_td_filter_process(adrc_td_filter_t *td_filter,float v)
{
    float fv = adrc_fhan(td_filter->v1 - v, td_filter->v2, (td_filter->r0), (td_filter->h0));
    td_filter->v1 += td_filter->h * td_filter->v2;
    td_filter->v2 += td_filter->h * fv;
}

/*
 * @brief: td controller parameters initialization and process
 **/
void adrc_td_control_init(adrc_td_control_t *td_control,float h,float r2,float h2)
{
    td_control->h = h;
    td_control->h2 = h2;
    td_control->r2 = r2;
    td_control->v1 = td_control->v2 = 0.0f;
}

float adrc_td_control_process(adrc_td_control_t *td_control,float err)
{
    float fv = adrc_fhan(-err, td_control->v2, (td_control->r2), (td_control->h2));
    td_control->v1 += td_control->h * td_control->v2;
    td_control->v2 += td_control->h * fv;
    return td_control->v2;
}

void adrc_td_control_reset(adrc_td_control_t *td_control,float v1,float v2)
{
    td_control->v1 = v1;
    td_control->v2 = v2;
}


/**
 * @brief: 1 order nonlinear ESO parameter initialization and process
 */
void adrc_neso1rd_init(adrc_eso1rd_t *eso,float h,float beta1,float beta2,float b0)
{
    eso->h = h;
    eso->beta1 = beta1;
    eso->beta2 = beta2;
    eso->u = 0.0f;
    eso->b0 = b0;

    eso->z1 = eso->z2 = 0.0f;
}

void adrc_neso1rd_reset(adrc_eso1rd_t *eso,float z1,float z2)
{
    eso->z1 = z1;
    eso->z2 = z2;
}

void adrc_neso1rd_process(adrc_eso1rd_t *eso,float y,float u)
{
    float e = eso->z1 - y;
    eso->u = u;

    float alpha = 0.5;
    float delta = eso->h;
    float fe = adrc_fal(e, alpha, delta);

    float beta1 = (eso->beta1);
    float beta2 = (eso->beta2);
    float b0    = (eso->b0);

    eso->z1 += eso->h*(eso->z2 + b0*eso->u - beta1*e);
    eso->z2 -= eso->h*beta2*fe;
}

/**
 * @brief: 1 order linear ESO parameter initialization and process
 */
void adrc_leso1rd_init(adrc_eso1rd_t *eso,float h,float w,float b0)
{
    eso->h = h;
    // (s + w)^2 = s^2 + beta_1 * s + beta_2
    eso->beta1 = 2*w;
    eso->beta2 = w*w;
    eso->u = 0.0f;
    eso->b0 = b0;

    eso->z1 = eso->z2 = 0.0f;
}

void adrc_leso1rd_reset(adrc_eso1rd_t *eso,float z1,float z2)
{
    eso->z1 = z1;
    eso->z2 = z2;
}

void adrc_leso1rd_process(adrc_eso1rd_t *eso,float y,float u)
{
    float e = eso->z1 - y;
    float b0 = (eso->b0);

    eso->u = u;
    eso->z1 = eso->h*(eso->z2 + b0*eso->u - eso->beta1*e) + eso->z1;
    eso->z2 = eso->z2 - eso->h*eso->beta2*e;
}

/**
 * @brief: 2 order nonlinear ESO parameter initialization and process
 */
void adrc_neso2rd_init(adrc_eso2rd_t* eso_t, float h, float beta1, float beta2, float beta3,float b0)
{
    eso_t->h = h;
    eso_t->beta1 = beta1;
    eso_t->beta2 = beta2;
    eso_t->beta3 = beta3;

    eso_t->u = 0.0f;
    eso_t->b0 = b0;

    eso_t->z1 = eso_t->z2 = eso_t->z3 = 0.0f;
}

void adrc_neso2rd_reset(adrc_eso2rd_t *eso,float z1,float z2,float z3)
{
    eso->z1 = z1;
    eso->z2 = z2;
    eso->z3 = z3;
}

void adrc_neso2rd_process(adrc_eso2rd_t* eso_t, float y,float u)
{
    float e = eso_t->z1-y;
    eso_t->u = u;

   float fe = adrc_fal(e, 0.5, eso_t->h);
   float fe1 = adrc_fal(e,0.25,eso_t->h);

   eso_t->z1 += eso_t->h*(eso_t->z2 - (eso_t->beta1)*e);
   eso_t->z2 += eso_t->h*(eso_t->z3 - (eso_t->beta2)*fe + (eso_t->b0)*eso_t->u);
   eso_t->z3 += eso_t->h*(-(eso_t->beta3)*fe1);

}


/**
 * @brief:  2 order linear ESO parameter initialization and process
 */
void adrc_leso2rd_init(adrc_eso2rd_t* eso_t, float h, float w,float b0)
{
    eso_t->h = h;
    eso_t->beta1 = 3*w;
    eso_t->beta2 = 3*w*w;
    eso_t->beta3 = w*w*w;

    eso_t->u = 0.0f;
    eso_t->b0 = b0;

    eso_t->z1 = eso_t->z2 = eso_t->z3 = 0.0f;
}

void adrc_leso2rd_reset(adrc_eso2rd_t *eso,float z1,float z2,float z3)
{
    eso->z1 = z1;
    eso->z2 = z2;
    eso->z3 = z3;
}

void adrc_leso2rd(adrc_eso2rd_t* eso_t, float y,float u)
{
    float e = eso_t->z1-y;
    eso_t->u = u;

    float beta1 = eso_t->beta1;
    float beta2 = eso_t->beta2;
    float beta3 = eso_t->beta3;
    float b0 = (eso_t->b0);

   eso_t->z1 += eso_t->h*(eso_t->z2 - beta1*e);
   eso_t->z2 += eso_t->h*(eso_t->z3 - beta2*e + b0*eso_t->u);
   eso_t->z3 += eso_t->h*(-beta3*e);
}

/**
 * brief: nonlinear feedback controller NLSEF
 */
void adrc_nlsef_init(adrc_nlsef_t* nlsef_t, float h, float r1, float h1, float c)
{
    nlsef_t->h = h;
    nlsef_t->h1 = h1;
    nlsef_t->r1 = r1;
    nlsef_t->c = c;
}

float adrc_nlsef_process(adrc_nlsef_t* nlsef_t, float e1, float e2)
{
    float u0 = -adrc_fhan(e1, (nlsef_t->c)*e2, (nlsef_t->r1), (nlsef_t->h1));

    return u0;
}

#endif/* if defined(SCU_USE_ADRC)*/
