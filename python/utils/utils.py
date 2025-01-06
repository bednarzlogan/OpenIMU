import sympy as sp
import math


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


def approx_matrix_exponential(Matrix, order):
    """
    Approximate the matrix exponential using a truncated Taylor series.

    Args:
        Matrix (sympy.Matrix): Input square matrix.
        order (int): Order of the Taylor series expansion.

    Returns:
        sympy.Matrix: Approximation of the matrix exponential.
    """
    rows, cols = Matrix.shape
    if rows != cols:
        raise ValueError(f'The matrix is not square. Shape is: ({rows}, {cols})')

    # Initialize with the identity matrix
    approx_mexp = sp.eye(rows)

    # Compute the Taylor series up to the specified order
    for i in range(1, order):  # Start at 1 to skip the first identity term
        approx_mexp += Matrix**i / math.factorial(i)

    return approx_mexp


def produce_discrete_state_transition(State_Transition: sp.Matrix, phi_order=2):
    """
    Produces the discrete-time state transition matrix from the continuous
    time version.
    """
    Ts = sp.symbols('Ts')
    Phi = approx_matrix_exponential(State_Transition*Ts, phi_order)
    return Phi


def produce_discrete_control_input(State_Transition: sp.Matrix,
                                   Control_Input: sp.Matrix,
                                   order: int = 2):
    """
    Produces the discrete control input matrix from the continuous time
    version
    """
    Ts = sp.symbols('Ts')

    # get size of the continuous time control input and allocate an
    # identity matrix to start our series with
    rows, _ = State_Transition.shape()
    Gamma = sp.eye(rows)*Ts

    def approx_input_order(Gamma, order):
        if order == 1:
            return Gamma

        for i in range(2, order + 1):
            state_transition_order = State_Transition**(i-1)
            Gamma += (1/math.factorial(i)) * state_transition_order * (Ts**i)

        return Gamma

    Gamma = approx_input_order(Gamma, order)*Control_Input

    return Gamma


def perform_van_loans(A, G, Q, order=2):
    """
    Perform Van Loan's method to compute the discrete-time process noise covariance.

    Args:
        A (sympy.Matrix): Continuous-time state transition matrix.
        G (sympy.Matrix): Noise input matrix.
        Q (sympy.Matrix): Continuous-time noise covariance matrix.
        order (int): Order of the matrix exponential approximation (default: 2).

    Returns:
        sympy.Matrix: Discrete-time process noise covariance matrix Qd.
    """
    # Validate dimensions
    if A.shape[0] != A.shape[1]:
        raise ValueError("Matrix A must be square.")
    if G.shape[0] != A.shape[0]:
        raise ValueError("Matrix G must have the same number of rows as A.")
    if Q.shape[0] != G.shape[1] or Q.shape[1] != G.shape[1]:
        raise ValueError("Matrix Q must match the dimensions of G.")

    rows, _ = A.shape

    # Construct the Van Loan matrix
    M = sp.Matrix.vstack(
        sp.Matrix.hstack(-A.T, G * Q * G.T),
        sp.Matrix.hstack(sp.zeros(rows, rows), A)
    )

    # Compute the matrix exponential
    exp_M = approx_matrix_exponential(M, order)

    # Extract the relevant blocks for Qd
    E11 = exp_M[:rows, :rows]
    E12 = exp_M[:rows, rows:]
    Qd = E11.T * E12

    return Qd
