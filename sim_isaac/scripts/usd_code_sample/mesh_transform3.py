from pxr import Gf
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/cube_frame/cube/mesh_________")
matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
translate: Gf.Vec3d = matrix.ExtractTranslation()
print(matrix)
print(translate)