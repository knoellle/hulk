import torch
import yaml
from torch.nn import Sequential
from ultralytics.models.yolo import YOLO
from ultralytics.nn.tasks import DetectionModel, PoseModel

from hydra import Hydra


def get_foundation_model(model: DetectionModel | PoseModel) -> Sequential:
    num_backbone_modules = len(model.yaml.get("backbone"))

    return model.model[:num_backbone_modules]


def get_head_model(model: DetectionModel | PoseModel) -> Sequential:
    num_backbone_modules = len(model.yaml.get("backbone"))

    return model.model[num_backbone_modules:]


def get_head_parent_module_index(yaml: dict) -> int:
    num_backbone_modules = len(yaml.get("backbone"))

    return num_backbone_modules - 1


def append_head_to_model(
    patient_model: DetectionModel | PoseModel,
    donor_model: DetectionModel | PoseModel,
) -> Sequential:
    patient_foundation = get_foundation_model(patient_model)
    patient_head: Sequential = get_head_model(patient_model)
    donor_head: Sequential = get_head_model(donor_model)

    model = Hydra(
        patient_foundation,
        [patient_head, donor_head],
        backbone_len=len(patient_model.yaml.get("backbone")),
    )
    torch.save(model.state_dict(), "unified.pt2")
    print(model.forward(torch.rand([1, 3, 640, 640])))

    yolo = YOLO(model)


def unified_yaml(
    object_detection_yaml: dict, pose_detection_yaml: dict
) -> dict:
    object_head = object_detection_yaml.get("head")
    pose_head = pose_detection_yaml.get("head")
    pose_head[0][0] = get_head_parent_module_index(pose_detection_yaml)

    return {
        "nc": object_detection_yaml.get("nc"),
        "end2end": object_detection_yaml.get("end2end"),
        "reg_max": object_detection_yaml.get("reg_max"),
        "kpt_shape": pose_detection_yaml.get("kpt_shape"),
        "scales": object_detection_yaml.get("scales"),
        "backbone": object_detection_yaml.get("backbone"),
        "head": object_head + pose_head,
        "scale": object_detection_yaml.get("scale"),
        "yaml_file": "yolo26m-unified.yaml",
        "channels": object_detection_yaml.get("channels"),
    }


object_detection_model = YOLO("yolo26m")
pose_detection_model = YOLO("yolo26m-pose")

object_detection_yaml = object_detection_model.model.yaml
pose_detection_yaml = pose_detection_model.model.yaml
unified_yaml = unified_yaml(
    object_detection_model.model.yaml, pose_detection_model.model.yaml
)
with open("yolo26m-unified.yaml", "w") as f:
    yaml.dump(unified_yaml, f)

object_detection_pt = object_detection_model.model
pose_detection_pt = pose_detection_model.model
append_head_to_model(object_detection_pt, pose_detection_pt)
