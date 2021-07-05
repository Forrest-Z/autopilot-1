#include <Eigen/Core>
#include <stdint.h>
#include "mrac_adrc_conf.h"
#include "proj.h"

using Matrix  = Eigen::MatrixXd;
using Vectorx = Eigen::VectorXd;

/**
 * @class MracController
 * @brief A mrac controller for actuation system (throttle/brake and steering)
 */
class MracAdrcController{
public:
      
    MracAdrcController()=default;
    virtual ~MracAdrcController()=default;

    /**
     * @brief initialize mrac controller
     * @param conf configuration for mrac controller
     * @param dt sampling time interval
     */
    void Init(const MracAdrcConf &conf);

    /**
     * @brief compute control value based on the original command
     * @param command original command as the input of the actuation system
     * @param state actual output state of the actuation system
     * @param ts sampling time interval
     * @param b0 physical or designed accelearation
     * @return control value based on mrac controller architecture
     */
    virtual double Control(const double command, const Eigen::MatrixXd state,
                           const double b0,const double ts);

    /**
     * @brief reset all the variables (including all the states, gains and
     * externally-setting control parameters) for mrac controller
     */
    void Reset();

    /**
     * @brief reset internal states for mrac controller
     */
    void ResetStates();

    /**
     * @brief reset adaptive gains for mrac controller
     */
    void ResetGains();


    /**
     * @brief set initial values for state components in reference model dynamics
     * @param state_reference_init initial reference states
     */
    void SetInitialReferenceState(const Eigen::MatrixXd &state_reference_init);

    /**
     * @brief set initial values for state components in actual actuator dynamics
     * @param state_reference_init initial action states
     */
    void SetInitialActionState(const Eigen::MatrixXd &state_action_init);

    /**
     * @brief set initial command (desired input)
     * @param command_init initial desired input
     */
    void SetInitialCommand(const double command_init);

    /**
     * @brief set initial values of state adaption gains for mrac control
     * @param gain_state_adaption_init initial state adaption gains
     */
    void SetInitialStateAdaptionGain(const Eigen::MatrixXd &gain_state_adaption_init);

    /**
     * @brief set initial value of input adaption gain for mrac control
     * @param gain_input_adaption_init initial input adaption gain
     */
    void SetInitialInputAdaptionGain(const double gain_input_adaption_init);


    /**
     * @brief get current state for reference system
     * @return current state
     */
    Eigen::MatrixXd CurrentReferenceState() const;

    /**
     * @brief get current state adaptive gain for mrac control
     * @return current state adaptive gain
     */
    Eigen::MatrixXd CurrentStateAdaptionGain() const;

    /**
     * @brief get current input adaptive gain for mrac control
     * @return current input adaptive gain
     */
    Eigen::MatrixXd CurrentInputAdaptionGain() const;

private:
    
    /**
     * @brief execute the reference state interation with respect to the designed 
     * inputs in discrete-time form, with the bilinear transform (trapezoidal integration)
     *  method
    */
    void UpdateReference(double ts);


    /**
     * @brief execute the adaption interation with respect to the designed law in
       discrete-time form, with the bilinear transform (trapezoidal integration)
       method
     */
    void UpdateAdaption(double ts);

    /**
     * @brief intialize state adaption control gain
     */
    void InitGainStateAdaption(void);

    /**
     *@brief intilaize state input contrl gain 
     */
     void InitGainInputAdaption(void);

    /**
     *@brief Set reference model 
     */
     void SetReferenceModel();

    /**
     *@brief Set adaption model
     */
     void SetAdaptionModel();

private:

    // Paramters
    uint8_t model_order_;
    double wc_;
    double wa_;
    double gama_;
    Matrix matrix_Q_;
    double param_tolerance_;
    double param_limit_;
    double e0_{0.0523};
    double lv_{1.0};

    const double delta_{0.5};

private:

    bool initialized_{false};

    Proj _proj_oper;

    // Reference system matrix in continuous-time domain
    Matrix _matrix_a_reference;       ///< Matrix A in reference models
    Matrix _matrix_b_reference;       ///< Matrix B in reference models

    // Adaption system matrix in discrete-time domain
    Matrix _matrix_p_adaption;        ///< Matrix P in adaption models
    Matrix _matrix_b_adaption;        ///< Matrix B in adaption models

    Matrix _gain_state_adaption;      ///< State adaption vector
    Matrix _gain_input_adaption;      ///< Desired command adaption vector

    Matrix _gain_state_adaption_init; ///< Initial state adaption
    Matrix _gain_input_adaption_init; ///< Initial input adaption

    // Adaption system input variables in discrete-time domain
    Matrix _input_desired;            ///< Updated desired command vector
    Matrix _state_action;             ///< Updated actuation states vector
    Matrix _state_reference;          ///< Reference states vector

};