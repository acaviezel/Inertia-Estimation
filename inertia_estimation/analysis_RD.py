import sympy as sp

import sympy as sp

# Define symbolic variables for mass, x, y, z, and inertial parameters
ml, x, y, z, g = sp.symbols('ml x y z g')  # Mass, center of mass, and gravity
Ixx, Ixy, Ixz, Iyy, Iyz, Izz = sp.symbols('Ixx Ixy Ixz Iyy Iyz Izz')  # Inertial parameters

# Joint positions, velocities, accelerations
q = sp.Matrix(7, 1, sp.symbols('q1 q2 q3 q4 q5 q6 q7'))  # Joint positions
dq = sp.Matrix(7, 1, sp.symbols('dq1 dq2 dq3 dq4 dq5 dq6 dq7'))  # Joint velocities
ddq = sp.Matrix(7, 1, sp.symbols('ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7'))  # Joint accelerations

# End-effector properties
a = sp.Matrix(3, 1, sp.symbols('a_x a_y a_z'))  # End-effector accelerations
c = sp.Matrix(3, 1, sp.symbols('x y z'))  # End-effector center of mass
omega = sp.Matrix(3, 1, sp.symbols('omega_x omega_y omega_z'))  # End-effector angular velocities
omega_dot = sp.Matrix(3, 1, sp.symbols('omega_dot_x omega_dot_y omega_dot_z'))  # End-effector angular accelerations

# Correct inertia matrix definition
I = sp.Matrix([
    [Ixx, Ixy, Ixz],
    [Ixy, Iyy, Iyz],
    [Ixz, Iyz, Izz]
])

# Cross product calculation using sympy's Matrix method
def cross_product(v1, v2):
    return v1.cross(v2)

# Force vector
f = ml * a - sp.Matrix([0, 0, ml * g]) + cross_product(omega_dot, ml * c) + cross_product(omega, cross_product(omega, ml * c))

# Torque vector
tau = cross_product(ml * c, a) - cross_product(ml * c, sp.Matrix([0, 0, ml * g])) + I @ omega_dot + cross_product(omega, I @ omega)

# Combine force and torque vectors into a single 6x1 vector
expression = sp.Matrix([
    f[0], f[1], f[2],  # Force vector elements
    tau[0], tau[1], tau[2]  # Torque vector elements
])

# Display the resulting 6x1 matrix
sp.pprint(expression)


# Expand the full expression to make sure everything is multiplied out
expanded_expression = sp.expand(expression)

# Initialize the empty matrix A with 7 rows and 10 columns
A = sp.zeros(6, 10)

# The parameters we are interested in
parameters = [ml, x * ml, y * ml, z * ml, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]

# Loop through each element of the expression and collect the coefficients of the parameters
for i in range(6):
    for param_index, param in enumerate(parameters):
        # Collect the coefficients normally for all parameters
        A[i, param_index] = sp.collect(expanded_expression[i], param).coeff(param)

# Now remove the terms involving x, y, z from the first column (corresponding to ml)
for i in range(6):
    # Identify terms that involve x, y, or z in the first column
    unwanted_terms = [term for term in A[i, 0].as_ordered_terms() if term.has(x) or term.has(y) or term.has(z)]
    
    # Subtract the unwanted terms from A[i, 0]
    A[i, 0] = A[i, 0] - sum(unwanted_terms)

# Define b vector
b = sp.Matrix([ml, x * ml, y * ml, z * ml, Ixx, Ixy, Ixz, Iyy, Iyz, Izz])

# The equation A * b
equation = A * b

# Define the file path where you want to save the result
file_path = "/home/andri/observation_matrix/matrix_A_output.py"

# Open the file in write mode
with open(file_path, "w") as file:
    # Write import statement to make the file executable in Python
    file.write("import sympy as sp\n\n")
    
    # Write the matrix with K(i,j) notation
    file.write("K = sp.zeros(7, 10)\n")
    
    # Iterate over each entry in the matrix and write it to the file
    for i in range(A.rows):
        for j in range(A.cols):
            file.write(f"K[{i},{j}] = {A[i, j]}\n")

print(f"Matrix A has been saved to {file_path} as 'K(i,j)' format.")

