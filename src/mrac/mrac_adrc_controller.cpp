#include "mrac_adrc_controller.h"
#include <math/math_utils.h>
#include <iostream>


void MracAdrcController::Init(const MracAdrcConf &conf)
{
    // Load parameters
     model_order_     = conf.model_order;
     wc_              = conf.wc;
     wa_              = conf.wa;  
     gama_            = conf.gama;
     param_tolerance_ = conf.param_tolerance;
     param_limit_     = conf.param_limit;
     e0_              = conf.e0;
     lv_              = conf.lv;
     
     uint8_t i = 0;
     matrix_Q_    = Matrix::Zero(model_order_+1,model_order_+1);
     for( i=0; i<model_order_;i++){
         matrix_Q_(i,i) = conf.Q[i];
     }
     
    // Initalize proj operation
    _proj_oper.Init(param_tolerance_,param_limit_);

    // Initialize the system states
    _input_desired   = Matrix::Zero(1, 1);
    _state_action    = Matrix::Zero(model_order_+1, 1);
    _state_reference = Matrix::Zero(model_order_+1, 1);

    // Initialize the adaptive control gains
    _gain_state_adaption = Matrix::Zero(model_order_+1, 1);
    _gain_input_adaption = Matrix::Zero(1, 1);

    _gain_state_adaption_init = Matrix::Zero(model_order_+1, 1);
    _gain_input_adaption_init = Matrix::Zero(1, 1);
    InitGainStateAdaption();
    InitGainInputAdaption();

    // Initialize the reference model parameters
    _matrix_a_reference = Matrix::Zero(model_order_+1, model_order_+1);
    _matrix_b_reference = Matrix::Zero(model_order_+1, 1);
    SetReferenceModel();

    // Initialize the adaption model parameters
    _matrix_p_adaption = Matrix::Zero(model_order_+1, model_order_+1);
    _matrix_b_adaption = Matrix::Zero(model_order_+1, 1);
    SetAdaptionModel();

    _gain_state_adaption = _gain_state_adaption_init;
    _gain_input_adaption = _gain_input_adaption_init;

    initialized_ = false;
}


/**
 * @brief compute control value based on the original command
 * @param command original command as the input of the actuation system
 * @param state actual output state of the actuation system
 * @param dt sampling time interval
 * @param b0 physical or designed accelearation
 * @return control value based on mrac controller architecture
 */
double MracAdrcController::Control(const double command, const Eigen::MatrixXd state,
                           const double b0,const double ts)
{

    if(!initialized_){
        initialized_ = true;
        SetInitialReferenceState(state);
        SetInitialActionState(state);
    }
    
 // update the state in the real actuation system
    _state_action.col(0) = state;

    // update the desired command in the real actuation system
    _input_desired(0, 0) = command;

    // update the state in the reference system
    UpdateReference(ts);

    // update the adaption laws including state adaption, command adaption and
    // nonlinear components adaption
    UpdateAdaption(ts);

    // update the generated control based on the adaptive law
    double control_unbounded =
    (_gain_state_adaption.col(0).transpose() * _state_action.col(0) +
    _gain_input_adaption(0, 0) * _input_desired(0, 0))/b0;

    return control_unbounded;

}


/**
 * @brief reset all the variables (including all the states, gains and
 * externally-setting control parameters) for mrac controller
 */
void MracAdrcController::Reset()
{
    // reset the overall states
    ResetStates();
    // reset the adaptive gains
    ResetGains();
    // reset all the externally-setting, non-conf control parameters
    initialized_ = false;
}


/**
 * @brief reset internal states for mrac controller
 */
void MracAdrcController::ResetStates()
{
    _input_desired.setZero(1,1);
    // reset the internal states, anti-windup compensations and status
    _state_action.setZero(model_order_+1, 1);
    _state_reference.setZero(model_order_+1, 1);
}


/**
 * @brief reset adaptive gains for mrac controller
 */
void MracAdrcController::ResetGains()
{
    _gain_state_adaption.setZero(model_order_+1, 1);
    _gain_input_adaption.setZero(1, 1);
    _gain_state_adaption = _gain_state_adaption_init;
    _gain_input_adaption = _gain_input_adaption_init;

}


/**
 * @brief set initial values for state components in reference model dynamics
 * @param state_reference_init initial reference states
 */
void MracAdrcController::SetInitialReferenceState(const Eigen::MatrixXd &state_reference_init)
{
     if (state_reference_init.rows() != model_order_+1 ||
        state_reference_init.cols() != 1) {
            std::cout<<"MracAdrcController::SetInitialReferenceState failed!"<<std::endl;
               return;
      } else {
         _state_reference = state_reference_init;
      }
}

/**
 * @brief set initial values for state components in actual actuator dynamics
 * @param state_reference_init initial action states
 */
void MracAdrcController::SetInitialActionState(const Eigen::MatrixXd &state_action_init)
{
    if (state_action_init.rows() != model_order_+1 ||
        state_action_init.cols() != 1) {
           std::cout<<"MracAdrcController::SetInitialActionState failed!"<<std::endl;
            return;
   } else {
      _state_action = state_action_init;
   }
}

/**
 * @brief set initial command (desired input)
 * @param command_init initial desired input
 */
void MracAdrcController::SetInitialCommand(const double command_init)
{
    _input_desired(0, 0) = command_init;
}

/**
 * @brief set initial values of state adaption gains for mrac control
 * @param gain_state_adaption_init initial state adaption gains
 */
void MracAdrcController::SetInitialStateAdaptionGain(const Eigen::MatrixXd &gain_state_adaption_init)
{   
    if (gain_state_adaption_init.rows() != model_order_+1 ||
        gain_state_adaption_init.cols() != 1) {
            return;
   } else {
      _gain_state_adaption.col(0) = gain_state_adaption_init;
   }

}

/**
 * @brief set initial value of input adaption gain for mrac control
 * @param gain_input_adaption_init initial input adaption gain
 */
void MracAdrcController::SetInitialInputAdaptionGain(const double gain_input_adaption_init)
{
     _gain_input_adaption(0, 0) = gain_input_adaption_init;
}


/**
 * @brief get current state for reference system
 * @return current state
 */
Eigen::MatrixXd MracAdrcController::CurrentReferenceState() const
{
    return _state_reference;
}

/**
 * @brief get current state adaptive gain for mrac control
 * @return current state adaptive gain
 */
Eigen::MatrixXd MracAdrcController::CurrentStateAdaptionGain() const
{
    return _gain_state_adaption;
}

/**
 * @brief get current input adaptive gain for mrac control
 * @return current input adaptive gain
 */
Eigen::MatrixXd MracAdrcController::CurrentInputAdaptionGain() const
{
    return _gain_input_adaption;
}



/**
 * @brief execute the reference state interation with respect to the designed 
 * inputs in discrete-time form, with the bilinear transform (trapezoidal integration)
 *  method
*/
void MracAdrcController::UpdateReference(double ts)
{
    Matrix matrix_i = Matrix::Identity(model_order_+1, model_order_+1);
      _state_reference =
        (matrix_i - ts * 0.5 * _matrix_a_reference).inverse() *
        ((matrix_i + ts * 0.5 *_matrix_a_reference) * _state_reference +
            ts * _matrix_b_reference * _input_desired(0,0));
}


/**
 * @brief execute the adaption interation with respect to the designed law in
 *  discrete-time form, with the bilinear transform (trapezoidal integration)
 * method
 */
void MracAdrcController::UpdateAdaption(double ts)
{
      Matrix state_error = _state_reference - _state_action ;
      double sq_state = math::Sqr(_state_action.col(0).norm());
      double sq_input = math::Sqr(_input_desired.col(0).norm());

      double rou = gama_ * wa_ / (sq_state + sq_input + 1.0);
      double mu = std::max(0.0,std::min(1.0,(state_error.col(0).norm() - delta_*e0_)/(e0_*(1-delta_))));
    
      Matrix dgain_state_adaption = rou*mu*_matrix_b_adaption.col(0).transpose()* _matrix_p_adaption*
        state_error.col(0) *_state_action.col(0) -
        rou*mu*lv_*_state_action.col(0)*_state_action.col(0).transpose()*_gain_state_adaption+
        wa_ *_gain_state_adaption_init - wa_ *_gain_state_adaption.col(0);

      Matrix dgain_input_adaption = rou*mu*_matrix_b_adaption.col(0).transpose()* _matrix_p_adaption*
        state_error.col(0)*_input_desired.col(0) -
        rou*mu*lv_*_input_desired.col(0)*_input_desired.col(0).transpose()*_gain_input_adaption+
        wa_ *_gain_input_adaption_init - wa_*_gain_input_adaption.col(0);

      // Projective operations
      Matrix gain_ext = Matrix::Zero(_gain_input_adaption_init.size() + _gain_state_adaption_init.size(),1);
        gain_ext << _gain_state_adaption.col(0),_gain_input_adaption.col(0);

      Matrix dgain_ext = Matrix::Zero(_gain_input_adaption_init.size() + _gain_state_adaption_init.size(),1);
        dgain_ext << dgain_state_adaption.col(0),dgain_input_adaption.col(0);

      dgain_ext = _proj_oper.Update(gain_ext,dgain_ext);
      dgain_state_adaption= dgain_ext.topRows(_gain_state_adaption_init.size());
      dgain_input_adaption= dgain_ext.bottomRows(_gain_input_adaption_init.size());

      // update gain_state_adaption and gain_input_adaption
      _gain_state_adaption = _gain_state_adaption + ts * dgain_state_adaption;
      _gain_input_adaption = _gain_input_adaption + ts * dgain_input_adaption;
}

/**
 * @brief intialize state adaption control gain
 */
void MracAdrcController::InitGainStateAdaption(void)
{   
     if(model_order_ == 1){
         _gain_state_adaption_init(0,0) = -wc_;
         _gain_state_adaption_init(1,0) = -1;
      }else if(model_order_ == 2){
         _gain_state_adaption_init(0,0) = -wc_*wc_;
         _gain_state_adaption_init(1,0) = -2*wc_;
         _gain_state_adaption_init(2,0) = -1;
      }

}

/**
 *@brief intilaize state input contrl gain 
 */
void MracAdrcController::InitGainInputAdaption(void)
{
      if(model_order_ == 1){
         _gain_input_adaption_init(0,0) = wc_;
      }else if(model_order_ == 2){
         _gain_input_adaption_init(0,0) = wc_*wc_;
      }
}

/**
 *@brief Set reference model 
 */
void MracAdrcController::SetReferenceModel()
{
     if(model_order_ == 1){
         _matrix_a_reference(0,0) = -wc_;
         _matrix_b_reference(0,0) = wc_;
      }else if(model_order_ ==2){
         _matrix_a_reference(0,1) = 1;
         _matrix_a_reference(1,0) = -wc_ *wc_;
         _matrix_a_reference(1,1) = -2*wc_; 
         _matrix_b_reference(1,0) = wc_*wc_;
      }
}

/**
 *@brief Set adaption model
 */
void MracAdrcController::SetAdaptionModel()
{

      if(model_order_ == 1){
        _matrix_b_adaption(0,0) = 1;
        // compute matrix P from given matrix Q
        double q1 = matrix_Q_(0,0);
        _matrix_p_adaption(0,0) = q1/(2*wc_);
      }else if(model_order_ ==2){
         _matrix_b_adaption(1,0) = 1;
         // solve Ap + PA^T +Q = 0;
         double q1 = matrix_Q_(0,0);
         double q2 = matrix_Q_(1,1);
         double p1 = q1/(2*math::Sqr(wc_));
         double p2 = (0.5f*q2 + p1) /(2*wc_);
         double p3 = math::Sqr(wc_)*p2 + 2*wc_*p1;

         _matrix_p_adaption(0,0) = p3;
         _matrix_p_adaption(0,1) = p1;
         _matrix_p_adaption(1,0) = p1;
         _matrix_p_adaption(1,1) = p2;
      }
}