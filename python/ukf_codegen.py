from io import TextIOWrapper
from typing import Callable

import sympy as sp
from sympy import ccode

import ukf_model as ukf

def emit_f_cont_to_cpp(f_cont: Callable[..., sp.Matrix], params: dict, file: TextIOWrapper, state_size=15, input_size=6):
    # define symbolic state and input
    x = sp.Matrix(sp.symbols(f'x0:{state_size}'))
    u = sp.Matrix(sp.symbols(f'u0:{input_size}'))

    # "codegen" mode will get symbolic matrices
    dx = f_cont(x, u, params, codegen_mode=True)

    # sympy won't optimze sin(x) unless we make a variable "sin_x"
    phi, theta, psi = x[6], x[7], x[8]
    trig_subs = {
        sp.sin(phi): sp.Symbol("sin_phi"),
        sp.cos(phi): sp.Symbol("cos_phi"),
        sp.sin(theta): sp.Symbol("sin_theta"),
        sp.cos(theta): sp.Symbol("cos_theta"),
        sp.tan(theta): sp.Symbol("tan_theta"),
        1/sp.cos(theta): sp.Symbol("sec_theta"),
        sp.sin(psi): sp.Symbol("sin_psi"),
        sp.cos(psi): sp.Symbol("cos_psi"),
    }

    dx = dx.subs(trig_subs)

    # apply common subexpression elimination
    replacements, reduced = sp.cse(dx)

    # write C++ function header
    file.write("Eigen::Matrix<double, 15, 1> f_cont(\n")
    file.write("    const Eigen::Matrix<double, 15, 1>& x,\n")
    file.write("    const Eigen::Matrix<double, 6, 1>& u,\n")
    file.write("    double tau_a, double tau_g, double gx, double gy, double gz) {\n\n")
    file.write("    Eigen::Matrix<double, 15, 1> dx;\n\n")

    # write trig shorthands
    file.write("    // Trig shorthands\n")
    file.write("    double phi = x(6), theta = x(7), psi = x(8);\n")
    file.write("    double sin_phi = sin(phi);\n")
    file.write("    double cos_phi = cos(phi);\n")
    file.write("    double sin_theta = sin(theta);\n")
    file.write("    double cos_theta = cos(theta);\n")
    file.write("    double tan_theta = tan(theta);\n")
    file.write("    double sec_theta = 1.0 / cos_theta;\n")
    file.write("    double sin_psi = sin(psi);\n")
    file.write("    double cos_psi = cos(psi);\n\n")

    # CSE intermediate expressions
    file.write("    // common subexpressions\n")
    for sym, expr in replacements:
        file.write(f"    double {sym} = {ccode(expr)};\n")
    file.write("\n")

    # state derivative assignments
    file.write("    // state derivatives\n")
    for i, expr in enumerate(reduced[0]):
        file.write(f"    dx({i}) = {ccode(expr)};\n")

    file.write("\n    return dx;\n")
    file.write("}\n")


def main() -> None:
    # define symbolic parameters
    params = {
        'tau_a': sp.Symbol('tau_a'),
        'tau_g': sp.Symbol('tau_g'),
        'g_n': sp.Matrix([sp.Symbol('gx'), sp.Symbol('gy'), sp.Symbol('gz')])
    }

    # Write out dynamics
    full_file_path = "generated_f_cont.hpp"
    with open(full_file_path, 'w') as file:
        emit_f_cont_to_cpp(ukf.f_cont, params, file)

if __name__ == "__main__":
    main()