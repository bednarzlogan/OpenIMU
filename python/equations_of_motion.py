#import time
from typing import List

import sympy as sp
import utils.rotation_matrices as rt
import utils.stochastic as st
import utils.utils as ut
import utils.write_to_hpp as hpp
#import web_resources.web_resources as wr


def r_dot():
    """
    Define the velocity state transition -- mostly included for completeness
    """
    # define the position contribution
    position_contribution = sp.zeros(3, 3)

    # define the velocity contribution
    velocity_contribution = sp.eye(3, 3)

    # define the attitude contribution
    attitude_contribution = sp.zeros(3, 3)

    # define the moving accelerometer bias contribution
    accel_bias_contribution = sp.zeros(3, 3)

    # define the moving gyro bias contribution
    gyro_bias_contribution = sp.zeros(3, 3)

    # horizontally concatenate state contributions to form final matrix
    state_transition = sp.Matrix(
        sp.Matrix.hstack(position_contribution, velocity_contribution, attitude_contribution,
                         accel_bias_contribution, gyro_bias_contribution)
    )

    # the control input to the acceleration will be the accelerometer perturbations
    control_input = sp.Matrix(
        sp.Matrix.hstack(sp.zeros(3, 3), sp.zeros(3, 3))
    )

    return state_transition, control_input


def v_dot():
    """
    Compute the linearized acceleration dynamics and extract state transition
    and control input matrices.

    Each of these functions implicitly returns a 3x3 matrix for the state transition
    and control input model. The stochastic elements are defined separately.

    Args:
        prev_state_vector (dict): Dictionary of state variables and their previous values.

    Returns:
        tuple: (state_transition_matrix, control_input_matrix) as symbolic matrices.
    """
    # Transform accelerometer readings to navigation frame
    rotation_matrix = rt.R_ENU_BODY

    # Nominal accelerometer readings in navigation frame
    fbx, fby, fbz = sp.symbols('fbx fby fbz')

    # Nominal accelerometer readings
    fb = sp.Matrix([fbx, fby, fbz])

    transformed_accel = rotation_matrix * fb 

    # account for the position contribution to the linear acceleration
    position_contribution = sp.zeros(3, 3)

    # account for the velocity effect on linear acceleration
    velocity_contribution = sp.zeros(3, 3)

    # formulate cross product effect of the body rotations into a skew-symmetric
    # matrix
    attitude_contribution = ut.skew_symmetric_matrix(transformed_accel)  

    # define the moving accelerometer bias contribution
    accel_bias_contribution = -1*rotation_matrix

    # define the moving gyro bias contribution
    gyro_bias_contribution = sp.zeros(3, 3)

    # horizontally concatenate state contributions to form final matrix
    state_transition = sp.Matrix(
        sp.Matrix.hstack(position_contribution, velocity_contribution, attitude_contribution,
                         accel_bias_contribution, gyro_bias_contribution)
    )

    # the control input to the acceleration will be the accelerometer perturbations
    control_input = sp.Matrix(
        sp.Matrix.hstack(rotation_matrix, sp.zeros(3, 3))
    )

    return state_transition, control_input


def E_dot():
    """
    Define the attitude equatiuons without the effects of earth's rotation.
    The primary contributor is simply the angular rate mapping function on the
    gyroscope readings.

    Returns:
        transformed_gyro: the readings from the gyroscope transformed into the navigation
                          frame
    """
    # the attitude will be defined by the perturbations in
    # E_dot = Q_be_inv(omega_IB)
    # which results in:
    # E_dot \approx (&Q_be_inv + del_Qbe_inv) * (&omega_NB + del_omega_NB)
    #       \approx &Qbe_inv*del_omega_NB + &omega_NB*del_Qbe_inv
    # where the proof for the variation in Qbe_inv (del_Qbe_inv) is contained
    # in our NSF document in our sources folder

    # define the position contribution
    position_contribution = sp.zeros(3, 3)

    # define the velocity contribution
    velocity_contribution = sp.zeros(3, 3)

    # define the perturbation in Q_be portion
    attitude_contribution = rt.eval_dQbe_kronecer()

    # define the moving accelerometer bias contribution
    accel_bias_contribution = sp.zeros(3, 3)

    # define the moving gyro bias contribution
    gyro_bias_contribution = -1*rt.Qbe_inv

    # horizontally concatenate state contributions to form final matrix
    state_transition = sp.Matrix(
        sp.Matrix.hstack(position_contribution, velocity_contribution, attitude_contribution,
                         accel_bias_contribution, gyro_bias_contribution)
    )

    # define control input (gyro reading portion)
    control_input = sp.Matrix(
        sp.Matrix.hstack(sp.zeros(3, 3), rt.Qbe_inv)
    )

    return state_transition, control_input


def discretize_system(state_transition: sp.Matrix,
                      control_input: sp.Matrix,
                      noise_input: sp.Matrix):
    # take the continuous time dynamics and use the discretization
    # tools in utils to form discrete dynamics
    # these will all default to second order

    # get discrete state transition
    Phi = ut.produce_discrete_state_transition(state_transition)

    # get discrete control input
    Gamma = ut.produce_discrete_control_input(state_transition,
                                              control_input)

    # define the process noise covariance in general
    # this was not tokenizing well into cpp, so I had to do all the terms explicitly
    # _, n_states = noise_input.shape
    # qs = sp.MatrixSymbol('qs', n_states, 1)

    # accelerometer bias variance
    sig_ax, sig_ay, sig_az = sp.symbols("sig_ax sig_ay sig_az")

    # gyro bias variance
    sig_gx, sig_gy, sig_gz = sp.symbols("sig_gx sig_gy sig_gz")

    # white noise on accelerometer bias
    sig_tax, sig_tay, sig_taz = sp.symbols("sig_tax sig_tay sig_taz")

    # white noise on gyro bias
    sig_tgx, sig_tgy, sig_tgz = sp.symbols("sig_tgx sig_tgy sig_tgz")

    # define the list of all white noise terms
    qs = [sig_ax, sig_ay, sig_az, 
          sig_gx, sig_gy, sig_gz, 
          sig_tax, sig_tay, sig_taz, 
          sig_tgx, sig_tgy, sig_tgz]

    noise_covariance = sp.diag(*qs)

    # get discrete noise input
    Gwk = ut.perform_van_loans(state_transition,
                               noise_input,
                               noise_covariance)

    return [Phi, Gamma, Gwk]


def main() -> List[sp.Matrix]:
    # collect EOM matrix rows
    state_trasition_row_vel, control_input_row_vel = r_dot()
    state_trasition_row_acc, control_input_row_acc = v_dot()
    state_trasition_row_att, control_input_row_att = E_dot()

    # add in the stochastic elements (white noise in acceleration and attitude rate eqs)
    # and the FOGMP for the accel and gyro biases
    state_transition_fogmp_acc, control_input_acc = st.FOGMP_accelerometer()
    state_transition_fogmp_gyro, control_input_gyro = st.FOGMP_gyro()

    # define full state transition matrix
    state_transition = sp.Matrix.vstack(state_trasition_row_vel,
                                        state_trasition_row_acc,
                                        state_trasition_row_att,
                                        state_transition_fogmp_acc,
                                        state_transition_fogmp_gyro)

    # define full control input matrix
    control_input = sp.Matrix.vstack(control_input_row_vel,
                                     control_input_row_acc,
                                     control_input_row_att,
                                     control_input_acc,
                                     control_input_gyro)

    # define the process noise input matrix
    r_dot_noise = st.r_dot_noise()
    v_dot_noise = st.v_dot_noise()
    E_dot_noise = st.E_dot_noise()
    accel_bias_noise = st.transient_accel_bias()
    gyro_bias_noise = st.transient_gyro_bias()

    # define the process noise input matrix
    process_noise_input = sp.Matrix.vstack(r_dot_noise,
                                           v_dot_noise,
                                           E_dot_noise,
                                           accel_bias_noise,
                                           gyro_bias_noise)

    # discretize the continuous time dynamics
    discrete_system = discretize_system(state_transition,
                                        control_input,
                                        process_noise_input)

    # print out some example matrices
    # wr.publish_matrix(state_transition, "Continuous Phi")
    # time.sleep(5)
    # wr.publish_matrix(control_input, "Continuous Gamma")
    # time.sleep(5)
    # wr.publish_matrix(discrete_system[2], "Discrete Q")

    # TODO - review matrix outputs and output to Cpp with jacobians where needed

    # set the IDs and comments for the CPP export
    state_transition_ID = "phi_k"
    state_transition_comment = "discrete time IMU state transition matrix"
    state_transition_discrete = discrete_system[0]
    state_tranistion_dict = {state_transition_ID: (state_transition_discrete, state_transition_comment)}

    control_input_ID = "gamma_uk"
    control_input_comment ="matrix which projects IMU accelerometer/gyro readouts onto state"
    control_input_discrete = discrete_system[1]
    control_input_dict = {control_input_ID: (control_input_discrete, control_input_comment)}

    discrete_noise_covariance_ID = "gamma_wk"
    discrete_noise_covariance_comment ="matrix which projects IMU accelerometer/gyro reading noise onto state"
    discrete_noise_covariance = discrete_system[2]
    noisel_input_dict = {discrete_noise_covariance_ID: (discrete_noise_covariance, discrete_noise_covariance_comment)}
    
    # test write out
    hpp.export_matrices_to_hpp([state_tranistion_dict, control_input_dict, noisel_input_dict], filename="IMU_Matrices.hpp")

    return discrete_system


if __name__ == "__main__":
    system_dynamics = main()  # TMP: main shouldn't return anything, except maybe a code like 1
