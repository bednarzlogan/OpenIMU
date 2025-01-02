import sympy as sp
import rotation_matrices as rt


def skew_symmetric_matrix(vector):
    """
    Construct the skew-symmetric matrix of a 3D vector.

    Args:
        vector (list or sympy.Matrix): A 3-element list or
        sympy.Matrix representing the vector.

    Returns:
        sympy.Matrix: The skew-symmetric matrix representation of the vector.
    """
    if len(vector) != 3:
        raise ValueError("Input vector must have exactly 3 elements.")

    # Extract components of the vector
    v_x, v_y, v_z = vector

    # Construct the skew-symmetric matrix
    skew_matrix = sp.Matrix([
        [0, -v_z, v_y],
        [v_z, 0, -v_x],
        [-v_y, v_x, 0]
    ])

    return skew_matrix


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

    # horizontally concatenate state contributions to form final matrix
    state_transition = sp.Matrix(
        sp.Matrix.hstack(position_contribution, velocity_contribution, attitude_contribution)
    )

    # the control input to the acceleration will be the accelerometer perturbations
    control_input = sp.Matrix(
        sp.Matrix.hstack(sp.zeros(3, 3), sp.zeros(3, 3))
    )

    return state_transition, control_input


def dv_dot():
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
    attitude_contribution = skew_symmetric_matrix(transformed_accel)

    # horizontally concatenate state contributions to form final matrix
    state_transition = sp.Matrix(
        sp.Matrix.hstack(position_contribution, velocity_contribution, attitude_contribution)
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

    # horizontally concatenate state contributions to form final matrix
    state_transition = sp.Matrix(
        sp.Matrix.hstack(position_contribution, velocity_contribution, attitude_contribution)
    )

    # define control input (gyro reading portion)
    control_input = sp.Matrix(
        sp.Matrix.hstack(sp.zeros(3, 3), rt.Qbe_inv)
    )

    return state_transition, control_input


if __name__ == "__main__":
    pass
