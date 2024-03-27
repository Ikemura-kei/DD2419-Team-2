import json

JSON_PATHS = [
    "c0.json",
    "c1.json",
    "c2.json",
    "c3.json",
    "c4.json",
    "c5.json",
    "c6.json",
    "c7.json",
]

CATEGORY_IDS = {
    "c0": 0,
    "c1": 1,
    "c2": 2,
    "c3": 3,
    "c4": 4,
    "c5": 5,
    "c6": 6,
    "c7": 7,
}

CATEGORIES = [
    {"id": 0, "name": "c0"},
    {"id": 1, "name": "c1"},
    {"id": 2, "name": "c2"},
    {"id": 3, "name": "c3"},
    {"id": 4, "name": "c4"},
    {"id": 5, "name": "c5"},
    {"id": 6, "name": "c6"},
    {"id": 7, "name": "c7"},
]
MERGED_JSON_PATH = "merged.json"
MERGED_JSON_PATH_TRAINING = "merged_training.json"
MERGED_JSON_PATH_VALIDATION = "merged_validation.json"


def update_file_paths(json_data: dict, image_path: str) -> None:
    """Update the file paths in the json file."""
    for image in json_data["images"]:
        image["file_name"] = image_path + "/" + image["file_name"]


def offset_image_ids(json_data: dict, offset: int) -> None:
    """Offset the image ids in the json file."""
    for image in json_data["images"]:
        image["id"] += offset


def offset_annotation_ids(
    json_data: dict, image_offset: int, annotations_offset: int, category_id: int
) -> None:
    """Offset the annotation ids in the json file and changes the category id to match the directory."""
    for annotation in json_data["annotations"]:
        annotation["id"] += annotations_offset
        annotation["image_id"] += image_offset
        annotation["category_id"] = category_id


def get_max_image_id(json_path: str) -> int:
    """Get the maximum image id in the json file."""
    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)
    return max([image["id"] for image in json_data["images"]])


def get_max_annotation_id(json_path: str) -> int:
    """Get the maximum annotation id in the json file."""
    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)
    return max([annotation["id"] for annotation in json_data["annotations"]])


def get_min_image_id(json_path: str) -> int:
    """Get the minimum image id in the json file."""
    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)
    return min([image["id"] for image in json_data["images"]])


def get_min_annotation_id(json_path: str) -> int:
    """Get the minimum annotation id in the json file."""
    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)
    return min([annotation["id"] for annotation in json_data["annotations"]])


def merge_json(json_paths: list, merged_json_path: str,merged_json_path_training: str,merged_json_path_validation: str) -> None:
    """Merge json files."""
    merged_images = []
    merged_annotations = []
    merged_categories = []
    merged_images_training = []
    merged_annotations_training = []
    merged_images_validation = []
    merged_annotations_validation = []
    image_offset = 0
    annotation_offset = 0

    for json_path in json_paths:
        with open(json_path, "r", encoding="utf-8") as f:
            json_data = json.load(f)
        update_file_paths(json_data, json_path.split(".")[0])
        offset_image_ids(json_data, image_offset)
        offset_annotation_ids(
            json_data,
            image_offset,
            annotation_offset,
            category_id=CATEGORY_IDS[json_path.split(".")[0]],
        )

        # Filter python objects with list comprehensions
        merged_images_training += list(filter(lambda x: x["id"]%10 != 1, json_data["images"])) 
        merged_images_validation += list(filter(lambda x: x["id"]%10 == 1, json_data["images"])) 
        merged_annotations_training += list(filter(lambda x: x["image_id"]%10 != 1, json_data["annotations"])) 
        merged_annotations_validation += list(filter(lambda x: x["image_id"]%10 == 1, json_data["annotations"])) 
        
        merged_images += json_data["images"]
        merged_annotations += json_data["annotations"]
        merged_categories += json_data["categories"]
        image_offset += get_max_image_id(json_path) + 1
        annotation_offset += get_max_annotation_id(json_path) + 1

    merged_json_data = {
        "images": merged_images,
        "annotations": merged_annotations,
        "categories": CATEGORIES,
    }

    merged_json_data_training = {
        "images": merged_images_training,
        "annotations": merged_annotations_training,
        "categories": CATEGORIES,
    }

    merged_json_data_validation = {
        "images": merged_images_validation,
        "annotations": merged_annotations_validation,
        "categories": CATEGORIES,
    }
    with open(merged_json_path, "w", encoding="utf-8") as f:
        json.dump(merged_json_data, f, ensure_ascii=False, indent="\t")

    with open(merged_json_path_training, "w", encoding="utf-8") as f:
        json.dump(merged_json_data_training, f, ensure_ascii=False, indent="\t")
    
    with open(merged_json_path_validation, "w", encoding="utf-8") as f:
        json.dump(merged_json_data_validation, f, ensure_ascii=False, indent="\t")


def find_annotation_by_image_id(json_data: dict, _id: int):
    """
    returns a dict with the annotation with the given id.
    """
    for annotation in json_data["annotations"]:
        if annotation["image_id"] == _id:
            return annotation


def get_annotation_by_file_name(json_data: dict, file_name: str):
    """
    returns a dict with the annotation with the given file_name.
    """
    for image in json_data["images"]:
        if image["file_name"] == file_name:
            return find_annotation_by_image_id(json_data, image["id"])


def verify_merged_json(json_paths: list, merged_json_path: str) -> None:
    """Verify the merged json file."""
    with open(merged_json_path, "r", encoding="utf-8") as f:
        merged_json_data = json.load(f)
    for json_path in json_paths:
        with open(json_path, "r", encoding="utf-8") as f:
            json_data = json.load(f)
        for image in json_data["images"]:
            orig_annotation = get_annotation_by_file_name(json_data, image["file_name"])
            if orig_annotation is not None:
                merged_annotation = get_annotation_by_file_name(
                    merged_json_data, json_path.split(".")[0] + "/" + image["file_name"]
                )

                assert orig_annotation["bbox"] == merged_annotation["bbox"]

                assert orig_annotation["area"] == merged_annotation["area"]
            else:
                print(f"Not found {image['file_name']} for {json_path}")


if __name__ == "__main__":
    merge_json(JSON_PATHS, MERGED_JSON_PATH, MERGED_JSON_PATH_TRAINING, MERGED_JSON_PATH_VALIDATION)
    #verify_merged_json(JSON_PATHS, MERGED_JSON_PATH)
