from typing import List, Type
from pxr import Usd, UsdGeom

def find_prims_by_type(stage: Usd.Stage, prim_type: Type[Usd.Typed]) -> List[Usd.Prim]:
    found_prims = [x for x in stage.Traverse() if x.IsA(prim_type)]
    return found_prims


def find_prims_by_name(stage: Usd.Stage, prim_name: str) -> List[Usd.Prim]:
    found_prims = [x for x in stage.Traverse() if x.GetName() == prim_name]
    return found_prims

# Get the stage
stage = omni.usd.get_context().get_stage()
default_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
stage.SetDefaultPrim(default_prim.GetPrim())

# find the prims with of type UsdGeom.Mesh
prims0: List[Usd.Prim] = find_prims_by_type(stage, UsdGeom.Mesh)
prims1: List[Usd.Prim] = find_prims_by_name(stage, "Foo")

prim: Usd.Prim
for prim in prims0:
    if prim.GetTypeName() == "Mesh":
        print(prim.GetTypeName() )
        
for prim in prims1:
    print(prim.GetName())

