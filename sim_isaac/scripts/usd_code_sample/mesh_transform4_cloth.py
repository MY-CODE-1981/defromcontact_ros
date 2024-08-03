from pxr import Gf, Usd, UsdGeom, Sdf, Tf
import omni.usd
import numpy as np

# Get the current USD stage from Omniverse
stage = omni.usd.get_context().get_stage()

# Retrieve the prim at the specified path
prim_path = "/World/cloth2/cloth2/mesh______"
prim = stage.GetPrimAtPath(prim_path)

# Get the world transform matrix of the specified prim
matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)

# Extract the translation vector from the matrix
translate: Gf.Vec3d = matrix.ExtractTranslation()


# Get mesh vertex positions if the prim is a Mesh
vertex_positions = []
if prim.GetTypeName() == 'Mesh':
    mesh = UsdGeom.Mesh(prim)
    points_attr = mesh.GetPointsAttr()
    vertex_positions = points_attr.Get()

# Print the results
print("World Transform Matrix:")
print(matrix)
print("Translation (Position):")
print(translate)
print("Mesh Vertex Positions:")
for pos in vertex_positions:
    print(pos)


def save_mesh_to_obj(prim_path, file_path):
    # Get the current USD stage from Omniverse
    stage = omni.usd.get_context().get_stage()
    
    # Retrieve the prim at the specified path
    prim = stage.GetPrimAtPath(prim_path)
    
    # Get mesh vertex positions if the prim is a Mesh
    if prim.GetTypeName() != 'Mesh':
        print("The specified prim is not a Mesh.")
        return

    mesh = UsdGeom.Mesh(prim)
    points_attr = mesh.GetPointsAttr()
    vertex_positions = points_attr.Get()

    # Open the file for writing
    with open(file_path, 'w') as file:
        # Write the vertices
        for pos in vertex_positions:
            file.write(f"v {pos[0]} {pos[1]} {pos[2]}\n")
        
        # Get face vertex indices
        face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        
        # Write the faces
        index = 0
        for count in face_vertex_counts:
            file.write("f")
            for _ in range(count):
                # OBJ format indices start at 1, not 0
                file.write(f" {face_vertex_indices[index] + 1}")
                index += 1
            file.write("\n")

# Usage
# prim_path = "/World/cube_frame/cube/mesh_________"
file_path = "/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/mesh/deform_object/cloth/output_mesh.obj"
save_mesh_to_obj(prim_path, file_path)
