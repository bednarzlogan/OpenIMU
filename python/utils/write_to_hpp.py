import os
from typing import Dict, List, Set, Tuple

import sympy as sp
from sympy import ccode


def export_matrices_to_hpp(matrices_list: List[Dict[str, Tuple[sp.Matrix, str]]], filename: str="GeneratedMatrices.hpp", extra_entries=None) -> None:
    """
    Export symbolic matrices to a C++ header file using Eigen syntax, with common sub-expressions
    like sin, cos, and tan manually extracted and precomputed at the top of each function.

    Parameters:
        matrices (dict): Dictionary of matrices with labels as keys and SymPy matrices as values.
        filename (str): Name of the output header file.
    """
    # remove old file (if present)
    full_file_path = os.path.join("cpp_source", "include", filename)
    if os.path.exists(full_file_path):
        try:
            os.remove(full_file_path)
        except Exception as e:
            print(f"Got an exception trying to delete old generated hpp: {e}")

    # get all free symbols for member variables
    for matrices in matrices_list:
        # Collect all unique symbols in the matrices
        symbols: Set[sp.symbols] = set()
        for matrix, _ in matrices.values():
            symbols.update(matrix.free_symbols)
        if extra_entries: # TODO - I don't remember what this is for
            for _, (extra_matrix, _) in extra_entries.items():
                symbols.update(extra_matrix.free_symbols)
        
        # write in header info
        with open(full_file_path, "w") as file:
            file.write("// This file is automatically generated by 'write_to_hpp.py'.\n")
            file.write("#ifndef GENERATED_MATRICES_HPP\n#define GENERATED_MATRICES_HPP\n\n")
            file.write("#include <Eigen/Dense>\n\n")
            file.write("class GeneratedMatrices {\npublic:\n")

            # Declare member variables for each symbol
            file.write("    // Member variables for symbols\n")
            for symbol in sorted(symbols, key=lambda s: s.name):  # Sort by symbol name for readability
                file.write(f"    double {symbol};\n")
            file.write("\n")
  
        # Open the file for writing
        with open(full_file_path, "a") as file:
            # Generate a function for each matrix
            for matrices in matrices_list:  
                for label, (matrix, comment) in matrices.items():
                    if comment:
                        file.write(f"    // {comment}\n")

                    # Convert matrix to C++ code with Eigen syntax
                    rows, cols = matrix.shape
                    matrix_code = f"    Eigen::Matrix<double, {rows}, {cols}> eval_{label}()  {{\n"
                    matrix_code += f"        Eigen::Matrix<double, {rows}, {cols}> mat;\n\n"
                    
                    # Manually substitute common trigonometric expressions
                    trig_subs = {
                        sp.sin(sp.Symbol('theta')): sp.Symbol('sin_theta'),
                        sp.cos(sp.Symbol('theta')): sp.Symbol('cos_theta'),
                        sp.tan(sp.Symbol('theta')): sp.Symbol('tan_theta'),
                        sp.sin(sp.Symbol('phi')): sp.Symbol('sin_phi'),
                        sp.cos(sp.Symbol('phi')): sp.Symbol('cos_phi')
                    }
                    matrix_subbed = matrix.subs(trig_subs)
                    
                    # Extract common sub-expressions after substitution
                    replacements, reduced_matrix = sp.cse([matrix_subbed], optimizations='basic')
                    reduced_matrix = reduced_matrix[0]  # Extract the single matrix from the list

                    # Declare variables for manually substituted trigonometric expressions
                    for trig_expr, symbol in trig_subs.items():
                        matrix_code += f"        double {symbol} = {ccode(trig_expr)};\n"
                    matrix_code += "\n"
                    
                    # Declare variables for other common sub-expressions found by cse
                    for i, (common_expr, replacement) in enumerate(replacements):
                        matrix_code += f"        double x{i} = {ccode(replacement)};  // {common_expr}\n"
                    matrix_code += "\n"

                    # Populate each element in the matrix using the reduced matrix
                    for i in range(rows):
                        for j in range(cols):
                            cpp_expr = ccode(reduced_matrix[i, j]).replace("t", "t")
                            matrix_code += f"        mat({i}, {j}) = {cpp_expr};\n"
                    
                    matrix_code += "        return mat;\n    }\n\n"
                    file.write(matrix_code)

                # TODO - Add additional entries as raw matrix definitions
                #        could be used to declare other member variables 
                if extra_entries:
                    for name, (extra_matrix, comment) in extra_entries.items():
                        if comment:
                            file.write(f"    // {comment}\n")
                        rows, cols = extra_matrix.shape
                        file.write(f"    Eigen::Matrix<double, {rows}, {cols}> {name};\n\n")

            # Close the class and header guards
            file.write("};\n\n")
            file.write("#endif // GENERATED_MATRICES_HPP\n")