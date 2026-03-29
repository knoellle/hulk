import yaml
from torch.nn import Sequential
from ultralytics.models.yolo import YOLO, pose
from ultralytics.nn.modules import Detect
from ultralytics.nn.tasks import DetectionModel, PoseModel


def get_foundation_model(model: DetectionModel | PoseModel) -> Sequential:
    num_backbone_modules = len(model.yaml.get("backbone")) - 1

    return model.model[: num_backbone_modules + 1]


def get_head_model(model: DetectionModel | PoseModel) -> Sequential:
    num_backbone_modules = len(model.yaml.get("backbone")) - 1

    return model.model[num_backbone_modules:]


def get_head_parent_module_index(yaml: dict) -> int:
    num_backbone_modules = len(yaml.get("backbone")) - 1
    return num_backbone_modules


def unified_yaml(
    object_detection_yaml: dict, pose_detection_yaml: dict
) -> dict:
    object_head = object_detection_yaml.get("head")
    pose_head = pose_detection_yaml.get("head")
    pose_head[0][0] = get_head_parent_module_index(pose_detection_yaml)

    {
        "nc": object_detection_yaml.get("nc"),
        "end2end": object_detection_yaml.get("end2end"),
        "reg_max": object_detection_yaml.get("reg_max"),
        "kpt_shape": pose_detection_yaml.get("kpt_shape"),
        "scales": object_detection_model.get("scales"),
        "backbone": object_detection_yaml.get("backbone"),
        "head": object_head + pose_head,
        "scale": object_detection_model.get("scale"),
        "yaml_file": "yolo26m-unified.yaml",
        "channels": object_detection_yaml.get("channels"),
    }


def append_head_to_model(
    patient_model: DetectionModel | PoseModel,
    donor_model: DetectionModel | PoseModel,
) -> Sequential:
    donor_head: Sequential = get_head_model(donor_model)
    patient


object_detection_model = YOLO("yolo26m")
pose_detection_model = YOLO("yolo26m-pose")

object_detection_yaml = object_detection_model.model.yaml
pose_detection_yaml = pose_detection_model.model.yaml
unified_yaml = unified_yaml(
    object_detection_model.model.yaml, pose_detection_model.model.yaml
)
with open("yolo26m-unified.yaml", "w") as f:
    yaml.dump(unified_yaml, f)

object_detection_pt = object_detection_model.model.model
pose_detection_pt = pose_detection_model.model
