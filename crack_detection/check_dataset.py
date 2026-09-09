import os

folders = [
    "datasets/crack-seg/images/train",
    "datasets/crack-seg/images/val",
    "datasets/crack-seg/images/test",
]

for folder in folders:
    print(folder, "=", len(os.listdir(folder)), "images")