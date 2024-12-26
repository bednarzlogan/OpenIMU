import sympy as sp
from sp import Matrix as matrix
from sp import sin, cos, tan, sec

###### MASSIVE NOTE:: we're defining all these models as though the rotataion of earth is irrelevant
######         NOTE:: to add these, we just undergo an additive process for these terms, and the kronecker_product changes      

# define core states 
lat, lon = sp.symbols('lat lon')

# define roll, pitch and yaw
phi, theta, psi = sp.symbols('phi theta psi')

# define some shorthands for the trig functions 
s_ph = sin(phi)
c_ph = cos(phi)
t_ph = tan(phi)
sec_ph = sec(phi)

s_th = sin(theta)
c_th = cos(theta)
t_th = tan(theta)
sec_th = sec(theta)

s_ps = sin(psi)
c_ps = cos(psi)
t_ps = tan(psi)
sec_ps = sec(psi)

s_lat = sin(lat)
c_lat = cos(lat)

s_lon = sin(lon)
c_lon = cos(lon)


##  Note: these matrices are well-defined in our literature, so I'm trying to save
##        space, more than making the individual matrices as readable as possible 

# define the ECEF to ENU rotation matrix
R_ECEF_ENU = matrix([
    [-s_lon, c_lon, 0],
    [-s_lat*c_lon, -s_lat*s_lon, c_lat],
    [c_lat*c_lon, c_lat*s_lon, s_lat]
])

# define the euler angle matrix 
R_ENU_BODY = matrix([
    [c_th*c_ps, s_ph*s_th*c_psi - c_ph*s_ps, c_ph*s_th*c_ps + s_ph*s_ps],
    [c_th*s_ps, s_ph*s_th*s_ps + c_ph*c_ps, c_ph*s_th*s_ps - s_ph*c_ps],
    [-s_th, s_ph*c_th, c_ph*c_theta]
])

# define "Qbe_inv": this matrix transfers the body-frame angular rates into a 
#                    navigation frame representation; this comes from a 3-1-2 
#                    transformation, rather than a sines/cosines rotation 
#                    matrix 
Qbe_inv = matrix([
    [1, s_ph*t_th, c_ph*t_th],
    [0, c_ph, -s_ph],
    [0, s_ph*sec_th, c_ph*sec_th]
])

# during the derivations of the linearized attitude representation, we arrive 
# at this form 
# E_dot = Qbe_inv * (omega_NB - R_ENU_BODY * omega_IE)

# to linearize, we take a perturbation form as 
# E_dot \approx (&Qbe_inv + del_Qbe_inv) * ((&omega_NB + del_omega_NB) - R_ENU_BODY * omega_IE) 
# here, &X notation means "X evaluated about our last solution" and "del_X" means "a perturbation in X"

# the term del_Qbe_inv is not simple to evaluate. We have to resolve this as: &omega_NB * (d/dE) * Qbe_inv * del_E
# in our derivations, we show that this can also be expressed as: kronecker(I, &omega_NB) * (d/dE) * Qbe_inv * del_E
# where kronecker is the Kronecker product

# finally, we define dQbe_inv_dE
dQbe_inv_dE = matrix([
    [0, 0, 0],
    [c_ph*t_th, s_ph*(sec_th**2), 0],
    [-s_ph*t_th, c_ph*(sec_th)**2, 0],
    [0, 0, 0],
    [-s_ph, 0, 0],
    [-c_phi, 0, 0],
    [0, 0, 0],
    [c_ph*sec_th, s_ph*sec_th*t_th, 0],
    [-s_ph*sec_th, c_ph*sec_th*t_th, 0]
])

# define the operation to produce this term with a kronecker product 
def eval_dQbe_kronecer(): 
    # Perform the Kronecker product between I and transpose(a)
    # Note: We manually implement the Kronecker product since SymPy doesn't provide a built-in function
    def kronecker_product(A, B):
        return sp.Matrix(sp.BlockMatrix([[A[i, j] * B for j in range(A.shape[1])] for i in range(A.shape[0])]))

    nominal_attitude = matrix([phi, theta, psi])
    kron_coefficient = (kronecker_product(sp.eye(3), nominal_attitude.T)

    return (kron_coefficient * dQbe_inv_dE)













# this file is not meant to be executed, so we omit if __name__ == "__main__""
