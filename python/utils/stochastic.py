import sympy as sp
import utils.rotation_matrices as rt


def r_dot_noise():
    """
    Included mostly for completeness, we define the contributions of the
    accelerometer noise and gyro noise on the velocity estimates
    """
    # define the accelerometer noise contribution
    accelerometer_noise_contribution = sp.zeros(3, 3)

    # define the gyroscope noise contribution
    gyroscope_noise_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # accelerometer bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # gyro bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the rows of the noise input matrix
    noise_input = sp.Matrix.hstack(accelerometer_noise_contribution, gyroscope_noise_contribution,
                                   transient_accel_bias_contribution, transient_accel_bias_contribution)

    return noise_input


def v_dot_noise():
    """
    Defines the input of the accelerometer noise into the IMU model.
    This is a simple addition of body-frame innacuracy into the navigation frame
    """
    # Transform accelerometer noise to navigation frame
    rotation_matrix = rt.R_ENU_BODY

    # accelerometer noise contribution
    accelerometer_noise_contribution = -1*rotation_matrix

    # gyroscope noise contribution
    gyroscope_noise_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # accelerometer bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # gyro bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the rows of the noise input matrix
    noise_input = sp.Matrix.hstack(accelerometer_noise_contribution, gyroscope_noise_contribution,
                                   transient_accel_bias_contribution, transient_accel_bias_contribution)

    return noise_input


def E_dot_noise():
    """
    Defines the input of the gyro noise into the IMU model.
    This is a simple addition of body-frame innacuracy into the navigation frame
    """
    # Transform accelerometer noise to navigation frame
    angular_rate_mapping = rt.Qbe_inv

    # accelerometer noise contribution
    accelerometer_noise_contribution = sp.zeros(3, 3)

    # gyroscope noise contribution
    gyroscope_noise_contribution = -1*angular_rate_mapping

    # define the contribution of the noise in the FOGMP in the
    # accelerometer bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # gyro bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the rows of the noise input matrix
    noise_input = sp.Matrix.hstack(accelerometer_noise_contribution, gyroscope_noise_contribution,
                                   transient_accel_bias_contribution, transient_accel_bias_contribution)

    return noise_input


def transient_accel_bias():
    """
    Defines the white noise input to the FOGMP in the accelerometer bias.
    """
    # accelerometer noise contribution
    accelerometer_noise_contribution = sp.zeros(3, 3)

    # gyroscope noise contribution
    gyroscope_noise_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # accelerometer bias
    transient_accel_bias_contribution = sp.eye(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # gyro bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the rows of the noise input matrix
    noise_input = sp.Matrix.hstack(accelerometer_noise_contribution, gyroscope_noise_contribution,
                                   transient_accel_bias_contribution, transient_accel_bias_contribution)

    return noise_input


def transient_gyro_bias():
    """
    Defines the white noise input to the FOGMP in the accelerometer bias.
    """
    # accelerometer noise contribution
    accelerometer_noise_contribution = sp.zeros(3, 3)

    # gyroscope noise contribution
    gyroscope_noise_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # accelerometer bias
    transient_accel_bias_contribution = sp.zeros(3, 3)

    # define the contribution of the noise in the FOGMP in the
    # gyro bias
    transient_accel_bias_contribution = sp.eye(3, 3)

    # define the rows of the noise input matrix
    noise_input = sp.Matrix.hstack(accelerometer_noise_contribution, gyroscope_noise_contribution,
                                   transient_accel_bias_contribution, transient_accel_bias_contribution)

    return noise_input


def FOGMP_accelerometer():
    """
    Defines the FOGMP dynamic model for the accelerometer noise.
    """
    # define the time constant symbol for the accel bias
    tau_ax, tau_ay, tau_az = sp.symbols('tau_ax tau_ay tau_az')  # Nominal accelerometer readings

    # define the position contribution
    position_contribution = sp.zeros(3, 3)

    # define the velocity contribution
    velocity_contribution = sp.zeros(3, 3)

    # define the attitude contribution
    attitude_contribution = sp.zeros(3, 3)

    # define the moving accelerometer bias contribution
    accel_bias_contribution = sp.diag(
        -1 / tau_ax,
        -1 / tau_ay,
        -1 / tau_az
    )

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


def FOGMP_gyro():
    """
    Defines the FOGMP dynamic model for the accelerometer noise.
    """
    # define the time constant symbol for the accel bias
    tau_gx, tau_gy, tau_gz = sp.symbols('tau_gx tau_gy tau_gz')  # Nominal accelerometer readings

    # define the position contribution
    position_contribution = sp.zeros(3, 3)

    # define the velocity contribution
    velocity_contribution = sp.zeros(3, 3)

    # define the attitude contribution
    attitude_contribution = sp.zeros(3, 3)

    # define the moving accelerometer bias contribution
    accel_bias_contribution = sp.zeros(3, 3)

    # define the moving gyro bias contribution
    gyro_bias_contribution = sp.diag(
        -1 / tau_gx,
        -1 / tau_gy,
        -1 / tau_gz
    )

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
