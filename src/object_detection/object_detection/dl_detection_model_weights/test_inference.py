"""Utility functions to handle object detection."""
from typing import Dict, List

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import torch
import os
from PIL import Image

import utils
from detector import Detector


device = "cuda"
detector = Detector().to(device)
model_path = "det_2023-02-18_15-46-29-649082.pt"

model= utils.load_model(detector, model_path, device)

test_images = []
directory = "./test_images2"
if not os.path.exists(directory):
    os.makedirs(directory)
for file_name in sorted(os.listdir(directory)):
    if file_name.endswith(".jpg"):
        file_path = os.path.join(directory, file_name)
        test_image = Image.open(file_path)
        torch_image, _  = detector.input_transform(test_image, [])
        test_images.append(torch_image)
    if file_name.endswith(".jpeg"):
        file_path = os.path.join(directory, file_name)
        test_image = Image.open(file_path)
        torch_image, _  = detector.input_transform(test_image, [])
        test_images.append(torch_image)


test_images = torch.stack(test_images)
test_images = test_images.to(device)



detector.eval()
with torch.no_grad():
    out = detector(test_images).cpu()
    bbs = detector.decode_output(out, 0.5)

    for i, test_image in enumerate(test_images):
        figure, ax = plt.subplots(1)
        print(test_image.shape)
        plt.imshow(test_image.cpu().permute(1, 2, 0))
        plt.imshow(
            out[i, 4, :, :],
            interpolation="nearest",
            extent=(0, 640, 480, 0),
            alpha=0.7,
        )
        title = ""
        for j in range (len(bbs[i])):
            title += "bb"+str(j)+" : "+ str(bbs[i][j]["category"])+ " | "
        plt.title(title)
        
        # add bounding boxes
        utils.add_bounding_boxes(ax, bbs[i])

        plt.show()
