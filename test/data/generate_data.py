#!/usr/bin/env python3
import numpy as np
import transforms3d as t3d


def axis_direction_factor(a):
    for i in range(len(a)):
        if a[i] > 0:
            return 1.0
        elif a[i] < 0:
            return -1.0
    return 1.0


def random_transform():
    # random euler transform
    translation = np.random.rand(3) * 10.0 - 5.0
    euler_angles = (np.random.rand(3) * 2 * np.pi - np.pi).tolist()
    euler_axes = 'sxyz'

    # matrix
    rotation_matrix = t3d.euler.euler2mat(*euler_angles, euler_axes)
    matrix = t3d.affines.compose(translation, rotation_matrix, [1, 1, 1])

    return matrix


def dual_quaternion(translation, quaternion):
    qr = quaternion

    qt = np.concatenate(([0], translation))
    qd = 0.5 * t3d.quaternions.qmult(qt, qr)

    return qr, qd


def transform_vectors(matrix):
    # decompose matrix
    translation, rotation_matrix, _, _ = t3d.affines.decompose(matrix)

    # euler
    euler_axes = 'sxyz'
    euler_angles = t3d.euler.mat2euler(rotation_matrix, euler_axes)
    euler_angles = np.mod(np.array(euler_angles) + np.pi, 2 * np.pi) - np.pi

    # axangle and quaternion
    axangle_ax, axangle_ang = t3d.axangles.mat2axangle(rotation_matrix)
    axangle_ang = np.mod(axangle_ang + np.pi, 2 * np.pi) - np.pi
    if axangle_ang == 0:
        axangle_ax = np.array([1.0, 0.0, 0.0])
    else:
        axangle_factor = axis_direction_factor(axangle_ax)
        axangle_ang *= axangle_factor
        axangle_ax *= axangle_factor

    # quaternion
    quaternion = t3d.quaternions.mat2quat(rotation_matrix)
    quaternion *= axis_direction_factor(quaternion)

    # dual quaternion
    dq_real, dq_dual = dual_quaternion(translation, quaternion)

    # vectors
    axangle_vec = np.concatenate((translation, [axangle_ang], axangle_ax))
    dq_vec = np.concatenate((dq_real, dq_dual))
    euler_vec = np.concatenate((translation, euler_angles, [0]))
    matrix_vec = matrix[:3, :].reshape(12)
    quaternion_vec = np.concatenate((translation, quaternion))

    # norm
    rotation_norm = np.abs(axangle_ang)
    translation_norm = np.linalg.norm(translation)

    return {
        'axangle': axangle_vec,
        'dq': dq_vec,
        'euler': euler_vec,
        'matrix': matrix_vec,
        'quaternion': quaternion_vec,
        'rotation_norm': rotation_norm,
        'translation_norm': translation_norm,
    }


def print_matrix_cpp(suffix, matrix):
    data = transform_vectors(matrix)
    print_array_cpp(f"AxisAngle{suffix}", data['axangle'])
    print_array_cpp(f"DualQuaternion{suffix}", data['dq'])
    print_array_cpp(f"Euler{suffix}", data['euler'])
    print_array_cpp(f"Matrix{suffix}", data['matrix'])
    print_array_cpp(f"Quaternion{suffix}", data['quaternion'])
    print_value_cpp(f"RotationNorm{suffix}", data['rotation_norm'])
    print_value_cpp(f"TranslationNorm{suffix}", data['translation_norm'])
    print()


def print_point_cpp(suffix, point):
    print_array_cpp(f"Point{suffix}", point)


def print_matrix_py(suffix, matrix):
    data = transform_vectors(matrix)
    print(f"TRANSFORM_ARRAYS{suffix} = {{")
    print_array_py("AxisAngleTransform", data['axangle'])
    print_array_py("DualQuaternionTransform", data['dq'])
    print_array_py("EulerTransform", data['euler'])
    print_array_py("MatrixTransform", data['matrix'])
    print_array_py("QuaternionTransform", data['quaternion'])
    print("}")
    print_value_py(f"ROTATION_NORM{suffix}", data['rotation_norm'])
    print_value_py(f"TRANSLATION_NORM{suffix}", data['translation_norm'])
    print()


def print_point_py(suffix, point):
    a_str = np.array2string(point, separator=',', floatmode='fixed', max_line_width=np.inf, precision=16)
    print(f"POINT_{suffix} = {a_str}")


def print_array_cpp(name, a):
    a_str = np.array2string(a, separator=',', floatmode='fixed', max_line_width=np.inf, precision=16)
    a_str = a_str.replace('[', '')
    a_str = a_str.replace(']', '')
    print(f"static const auto k{name}Vec = (Eigen::Matrix<double, {len(a)}, 1>() << {a_str}).finished();")


def print_value_cpp(name, x):
    print(f"static const double k{name} = {x};")


def print_array_py(name, a):
    a_str = np.array2string(a, separator=',', floatmode='fixed', max_line_width=np.inf, precision=16)
    print(f"    m3d.{name}: {a_str},")


def print_value_py(name, x):
    print(f"{name} = {x}")


def main():
    # fix random seed
    np.random.seed(0)

    # matrices
    matrix_unit = np.eye(4)
    matrix1 = random_transform()
    matrix2 = random_transform()

    # points
    point0 = np.random.rand(3)
    point1 = (matrix1 @ [*point0, 1.0])[:3]

    # combinations
    matrix12 = matrix1.dot(matrix2)
    matrix1_inv = np.linalg.inv(matrix1)

    print("#####  Cpp  #####")
    print_matrix_cpp('Unit', matrix_unit)
    print_matrix_cpp('1', matrix1)
    print_matrix_cpp('2', matrix2)
    print_matrix_cpp('12', matrix12)
    print_matrix_cpp('1Inv', matrix1_inv)
    print_point_cpp('0', point0)
    print_point_cpp('1', point1)
    print()

    print("#####  Py  #####")
    print_matrix_py('_UNIT', matrix_unit)
    print_matrix_py('1', matrix1)
    print_matrix_py('2', matrix2)
    print_matrix_py('12', matrix12)
    print_matrix_py('1_INV', matrix1_inv)
    print_point_py('0', point0)
    print_point_py('1', point1)


if __name__ == '__main__':
    main()
