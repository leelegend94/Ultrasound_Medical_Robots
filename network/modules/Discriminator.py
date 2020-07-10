import torch
import torch.nn as nn
import torch.nn.functional as F

class Discriminator(nn.Module):
    def __init__(self):
        super(Discriminator, self).__init__()
        
        self.img_conv1 = nn.Conv2d(1, 16, kernel_size = 3, padding=1)
        self.img_conv2 = nn.Conv2d(16, 64, kernel_size = 3, padding=1)
        self.img_pool = nn.MaxPool2d(2, 2)
        
        self.mask_conv = nn.Conv2d(1, 64, kernel_size = 3, padding=1)
        self.mask_pool = nn.MaxPool2d(2, 2)
        
        self.dis_conv1 = nn.Conv2d(128, 256, kernel_size = 3, padding=1)
        self.dis_pool1 = nn.MaxPool2d(2, 2)
        self.dis_conv2 = nn.Conv2d(256, 512, kernel_size = 3, padding=1)
        self.dis_pool2 = nn.MaxPool2d(2, 2)
        
        self.output = nn.Linear(512,1)
        
        
    def forward(self, input, mask):
        x = self.img_conv1(input)
        x = self.img_conv2(x)
        x = self.img_pool(x)
        
        y = self.mask_conv(mask)
        y = self.mask_pool(y)
        
        x = torch.cat([x,y],dim=1)
        x = self.dis_conv1(x)
        x = self.dis_pool1(x)
        x = self.dis_conv2(x)
        x = self.dis_pool2(x)
        x = F.max_pool2d(x,kernel_size=x.size()[2:]).squeeze()

        output = torch.sigmoid(self.output(x))
        
        return output