import sympy as sp
from typing import List
import python.utils.rotation_matrices as rt
import python.utils.utils as ut
import python.utils.stochastic as st


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
    fb = sp.MatrixSymbol('fb', 3, 1)  # Nominal accelerometer readings
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
    _, n_states = noise_input.shape()
    qs = sp.MatrixSymbol('qs', n_states, 1)
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

    return discrete_system


if __name__ == "__main__":
    system_dynamics = main()  # TMP: main shouldn't return anything, except maybe a code like 1
