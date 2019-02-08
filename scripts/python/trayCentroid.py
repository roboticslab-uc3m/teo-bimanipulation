import yarp
import kinematics_dynamics
import numpy as np
import scipy.linalg as lin

yarp.Network.init()

options = yarp.Property()
options.put('device','CartesianControlClient')

options.put('cartesianRemote','/teo/leftArm/CartesianControl')
options.put('cartesianLocal','/cc/teo/leftArm')
ddLeft = yarp.PolyDriver(options)

options.put('cartesianRemote','/teo/rightArm/CartesianControl')
options.put('cartesianLocal','/cc/teo/rightArm')
ddRight = yarp.PolyDriver(options)

iccLeft = kinematics_dynamics.viewICartesianControl(ddLeft)
iccRight = kinematics_dynamics.viewICartesianControl(ddRight)

xLeft = yarp.DVector()
xRight = yarp.DVector()

# Se calcula la FK (cinematica directa)
# 6-element vector describing current position in cartesian space; first three elements denote translation (meters), last three denote rotation in scaled axis-angle representation (radians)
iccLeft.stat(xLeft) 
iccRight.stat(xRight)

# Esta matriz representa la matriz homogenea 4x4 que va de 0 a N (0~origen, N~gripper) 
H_left_0_N = np.eye(4)
H_right_0_N = np.eye(4)

# https://stackoverflow.com/a/25709323
# axis2dcm convierte: scaled axis-angle representation (radians) -> dcm (matriz de cosenos directores)
axis2dcm = lambda(axis): lin.expm3(np.cross(np.eye(3), axis)) 

# Matriz que queremos obtener: H(N-T) matriz homogenea desde el gripper(N) hasta el tool(T)
# H(0-T) = H(0-N) * H(N-T)
# H(N-T) = [H(0-N)]^-1 * H(0-T)
# 
#       r r r | px
#       r r r | py
#       r r r | pz
#       ----------
#       0 0 0 | 1
# 
# ---- Matriz H(0-N) a partir de los 6 valores del stat 
# H_left_0_N[:3,:3] -> disecciono la matriz unitaria, cogiendo las 3 primeras filas y las 3 primeras columnas
H_left_0_N[:3,:3] = axis2dcm(xLeft[3:]) # xLeft[3:] -> 3 ultimos elementos del vector obtenido por stat (orientacion)
# H_left_0_N[:3,3] -> cojo los valores de las 3 primeras filas de la cuarta columna 
H_left_0_N[:3,3] = xLeft[:3] # xLeft[:3] -> los 3 primeros elementos de stat que seria la traslacion

# Hago lo mismo con los valores del brazo derecho
H_right_0_N[:3,:3] = axis2dcm(xRight[3:])
H_right_0_N[:3,3] = xRight[:3]

# ---- Matriz H(0-T):
# partimos de una matriz unitaria: rotacion 0 = matriz unitaria
H_left_0_T = np.eye(4) 
H_right_0_T = np.eye(4)

# Puesto que queremos el centro de la bandeja, Y = 0. 
# Colocamos los valores de traslacion X y Z del stat (valores del vector [0] y [2]) en la matriz (cosenos directores)
H_left_0_T[:3,3] = [xLeft[0], 0, xLeft[2]]
H_right_0_T[:3,3] = [xRight[0], 0, xRight[2]]

# despejariamos la ecuacion, aplicando la inversa a la matriz H(0-N) y multiplicandola por H(0-T)
H_left_N_T = np.dot(lin.inv(H_left_0_N), H_left_0_T) # dot -> multiplicar matrices
H_right_N_T = np.dot(lin.inv(H_right_0_N), H_right_0_T)

# Obtenemos la matriz de transformacion que buscabamos H(N-T)
print("Transformation matrix between the gripper and the tray:")
print(H_right_N_T)
print(H_left_N_T)


# from yarp::math::dcm2axis
# Transformacion de cosenos directores a rotacion eje-angulo: scaled axis-angle representation (radians)
def dcm2axis(rot):
    axis = np.array([rot[2,1] - rot[1,2], rot[0,2] - rot[2,0], rot[1,0] - rot[0,1]])
    norm = lin.norm(axis)
    return (1.0 / norm) * axis * np.arctan2(0.5 * norm, 0.5 * (rot.trace() - 1))

twist_right_N_T = np.append(H_right_N_T[:3,3], dcm2axis(H_right_N_T[:3,:3]))
twist_left_N_T = np.append(H_left_N_T[:3,3], dcm2axis(H_left_N_T[:3,:3]))


print("Current position in cartesian space; translation (meters) and rotation (radians):")
print(twist_right_N_T)
print(twist_left_N_T)

# update tool coordinates
#iccLeft.tool(yarp.DVector(twist_left_N_T))
#iccRight.tool(yarp.DVector(twist_right_N_T))

# perform FK
#iccLeft.stat(xLeft)
#iccRight.stat(xRight)

# sample output (rounded and prettified):
# xLeft = xRight = [0.368 0.0 0.168 0.0 0.0 0.0]

ddLeft.close()
ddRight.close()

yarp.Network.fini()
