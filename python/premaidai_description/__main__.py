import json
import premaidai_description as pm
print(json.dumps({k: getattr(pm, k) for k in ('__version__', 'PACKAGE_PATH', 'DESCRIPTION_PATH', 'URDF_PATH', 'XACRO_PATH', 'MJCF_PATH', 'MESH_DIR')}, indent=2))
