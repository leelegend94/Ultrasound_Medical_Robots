import torch
from torch.utils.data import Dataset
from PIL import Image
import os

class UltraSoundDataSet(Dataset):
    def __init__(self, root_dir, transforms):
        self.root_dir = root_dir
        self.sample_list = os.listdir(root_dir)

        self.transform_image, self.transform_label = transforms
        
    def __len__(self):
        return len(self.sample_list)
    
    def __getitem__(self,idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        image_path = os.path.join(self.root_dir,self.sample_list[idx],"image.png")
        label_path = os.path.join(self.root_dir,self.sample_list[idx],"label.png")
        
        image = Image.open(image_path)
        label = Image.open(label_path)
        
        if self.transform_image is not None:
            image = self.transform_image(image)
            
        if self.transform_label is not None:
            label = self.transform_label(label)
        
        return image,label