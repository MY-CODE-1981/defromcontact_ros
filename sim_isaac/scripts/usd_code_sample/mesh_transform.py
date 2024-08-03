from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf
from typing import List, Type

# def find_prims_by_type(stage: Usd.Stage, prim_type: Type[Usd.Typed]) -> List[Usd.Prim]:
#     found_prims = [x for x in stage.Traverse() if x.IsA(prim_type)]
#     return found_prims

def get_world_transform_xform(prim: Usd.Prim) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
    """
    Get the local transformation of a prim using Xformable.
    See https://openusd.org/release/api/class_usd_geom_xformable.html
    Args:
        prim: The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    xform = UsdGeom.Xformable(prim)
    time = Usd.TimeCode.Default() # The time at which we compute the bounding box
    world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    return translation, rotation, scale
    
def find_prims_by_name(stage: Usd.Stage, prim_name: str) -> List[Usd.Prim]:
    found_prims = [x for x in stage.Traverse() if x.GetName() == prim_name]
    return found_prims

# # Get the stage
# stage = omni.usd.get_context().get_stage()
# default_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
# stage.SetDefaultPrim(default_prim.GetPrim())

# find the prims with of type UsdGeom.Mesh
prims0: List[Usd.Prim] = find_prims_by_name(stage, "cube")

prim: Usd.Prim
for prim in prims0:
    # if prim.GetTypeName() == "Mesh":
    print(prim.GetName() )
    translation, rotation, scale = get_world_transform_xform(prim)
    print(translation)
    print(rotation)
    print(scale)
# for prim in prims1:
#     print(prim.GetName())



# import typing
# import omni.usd
# from pxr import Usd, Gf

# def get_local_transform_omni(prim: Usd.Prim) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
#     local_transform = omni.usd.get_local_transform_SRT(prim)
#     scale: Gf.Vec3d = local_transform[0]
#     rotation: Gf.Vec3d = local_transform[1]
#     rotation_order: float = local_transform[2]
#     translation: Gf.Vec3d = local_transform[3]
#     return translation, Gf.Rotation(rotation, rotation_order), scale

# #############
# # Full Usage
# #############

# import omni.kit.commands

# stage = omni.usd.get_context().get_stage()
# cube_prim = stage.GetPrimAtPath("/World/cube")
# print(cube_prim)
# transform = get_local_transform_omni(cube_prim)

# #assert transform[0] == cube_prim.GetAttribute('xformOp:translate').Get()
# #assert (100,0,0) == cube_prim.GetAttribute('xformOp:rotateXYZ').Get()
# #assert transform[2] == cube_prim.GetAttribute('xformOp:scale').Get()