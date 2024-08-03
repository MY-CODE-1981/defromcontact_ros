from pxr import Gf
import omni.usd
import numpy as np

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/cube/cube/mesh_________")
matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
translate: Gf.Vec3d = matrix.ExtractTranslation()
# print(matrix)
# print(translate)

# Convert the matrix and translation to NumPy arrays
matrix_np = np.array(matrix).T
translate_np = np.array([translate[0], translate[1], translate[2]])

import quaternion
import tf
matrix = matrix_np[0:3, 0:3]
q = quaternion.from_rotation_matrix(matrix_np[0:3, 0:3])
rpy = np.array(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]))
q = np.array([q.x, q.y, q.z, q.w])
# Print the results
print("World Transform Matrix:")
print(matrix)
print("Translation (Position):")
print(translate)
print("Rotation (Quaternion):")
print(q)
print("Rotation (RPY):")
print(rpy)

# Save the results to an NPZ file
np.savez("/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/output/pose_cube/pose.npz", matrix=matrix, translate=translate_np, quat=q, rpy=rpy)