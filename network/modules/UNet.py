import torch
import torch.nn as nn

class UNet(nn.Module):
    def __init__(self,in_channels=1,out_channels=1,init_features=64):
        super(UNet, self).__init__()

        features = init_features
        self.encoder1 = UNet.DoubleConv2d(in_channels,features)
        self.pool1 = nn.MaxPool2d(2, 2)
        
        self.encoder2 = UNet.DoubleConv2d(features,2*features)
        self.pool2 = nn.MaxPool2d(2, 2)

        self.encoder3 = UNet.DoubleConv2d(2*features,4*features)
        self.pool3 = nn.MaxPool2d(2, 2)
        
        self.encoder4 = UNet.DoubleConv2d(4*features,8*features)
        self.pool4 = nn.MaxPool2d(2, 2)
        
        self.bottleneck = UNet.DoubleConv2d(8*features,16*features)
        
        self.upconv4 = nn.ConvTranspose2d(16*features,8*features,kernel_size=2,stride=2)        
        self.decoder4 = UNet.DoubleConv2d(16*features,8*features) #concate, 2*8=16
        
        self.upconv3 = nn.ConvTranspose2d(8*features,4*features,kernel_size=2,stride=2)
        self.decoder3 = UNet.DoubleConv2d(8*features,4*features) #concate, 2*4=8
        
        self.upconv2 = nn.ConvTranspose2d(4*features,2*features,kernel_size=2,stride=2)
        self.decoder2 = UNet.DoubleConv2d(4*features,2*features) #concate, 2*2=4
        
        
        self.upconv1 = nn.ConvTranspose2d(2*features,features,kernel_size=2,stride=2)
        self.decoder1 = UNet.DoubleConv2d(2*features,features) #concate, 2*1=2
        
        self.conv_out = nn.Conv2d(features, 1, 1)
        
    def forward(self, input):
        enc1 = self.encoder1(input)
        
        enc2 = self.encoder2(self.pool1(enc1))
        
        enc3 = self.encoder3(self.pool2(enc2))
        
        enc4 = self.encoder4(self.pool3(enc3))
        
        bottleneck = self.bottleneck(self.pool4(enc4))

        dec4 = torch.cat([enc4,self.upconv4(bottleneck)],dim=1)
        dec4 = self.decoder4(dec4)
        
        dec3 = torch.cat([enc3,self.upconv3(dec4)],dim=1)
        dec3 = self.decoder3(dec3)
        
        dec2 = torch.cat([enc2,self.upconv2(dec3)],dim=1)
        dec2 = self.decoder2(dec2)
        
        dec1 = torch.cat([enc1,self.upconv1(dec2)],dim=1)
        dec1 = self.decoder1(dec1)
        
        output = torch.sigmoid(self.conv_out(dec1))
        
        return output
    
    def DoubleConv2d(in_channels,features):
        return nn.Sequential(
                nn.Conv2d(in_channels, features, kernel_size=3, padding=1),
                nn.BatchNorm2d(features),
                nn.ReLU(inplace=True),
                nn.Conv2d(features, features, kernel_size=3, padding=1),
                nn.BatchNorm2d(features),
                nn.ReLU(inplace=True)
        )

class UNet2(nn.Module):
    def __init__(self,in_channels=1,out_channels=1,init_features=64):
        super(UNet2, self).__init__()

        features = init_features
        self.encoder1 = UNet.DoubleConv2d(in_channels,features)
        self.pool1 = nn.MaxPool2d(2, 2)
        
        self.encoder2 = UNet.DoubleConv2d(features,2*features)
        self.pool2 = nn.MaxPool2d(2, 2)

        self.encoder3 = UNet.DoubleConv2d(2*features,4*features)
        self.pool3 = nn.MaxPool2d(2, 2)
        
        self.encoder4 = UNet.DoubleConv2d(4*features,8*features)
        self.pool4 = nn.MaxPool2d(2, 2)
        
        self.bottleneck = UNet.DoubleConv2d(8*features,16*features)
        
        self.upconv4 = nn.ConvTranspose2d(16*features,8*features,kernel_size=2,stride=2)        
        self.decoder4 = UNet.DoubleConv2d(16*features,8*features) #concate, 2*8=16
        
        self.upconv3 = nn.ConvTranspose2d(8*features,4*features,kernel_size=2,stride=2)
        self.decoder3 = UNet.DoubleConv2d(8*features,4*features) #concate, 2*4=8
        
        self.upconv2 = nn.ConvTranspose2d(4*features,2*features,kernel_size=2,stride=2)
        self.decoder2 = UNet.DoubleConv2d(4*features,2*features) #concate, 2*2=4
        
        
        self.upconv1 = nn.ConvTranspose2d(2*features,features,kernel_size=2,stride=2)
        self.decoder1 = UNet.DoubleConv2d(2*features,features) #concate, 2*1=2
        
        self.conv_out = nn.Conv2d(features, 1, 1)
        
    def forward(self, input):
        enc1 = self.encoder1(input)
        
        enc2 = self.encoder2(self.pool1(enc1))
        
        enc3 = self.encoder3(self.pool2(enc2))
        
        enc4 = self.encoder4(self.pool3(enc3))
        
        bottleneck = self.bottleneck(self.pool4(enc4))

        dec4 = torch.cat([enc4,self.upconv4(bottleneck)],dim=1)
        dec4 = self.decoder4(dec4)
        
        dec3 = torch.cat([enc3,self.upconv3(dec4)],dim=1)
        dec3 = self.decoder3(dec3)
        
        dec2 = torch.cat([enc2,self.upconv2(dec3)],dim=1)
        dec2 = self.decoder2(dec2)
        
        dec1 = torch.cat([enc1,self.upconv1(dec2)],dim=1)
        dec1 = self.decoder1(dec1)
        
        output = torch.sigmoid(self.conv_out(dec1))
        
        return output
    
    def DoubleConv2d(in_channels,features):
        return nn.Sequential(
                nn.Conv2d(in_channels, features, kernel_size=[1,5], padding=[0,2]),
                nn.BatchNorm2d(features),
                nn.ReLU(inplace=True),
                nn.Conv2d(features, features, kernel_size=[5,1], padding=[2,0]),
                nn.BatchNorm2d(features),
                nn.ReLU(inplace=True)
        )

class UNet3(nn.Module):
    def __init__(self,in_channels=1,out_channels=1,init_features=64,input_size=256):
        super(UNet3, self).__init__()

        features = init_features
        self.encoder1 = UNet.DoubleConv2d(in_channels,features)
        self.pool1 = nn.MaxPool2d(2, 2)
        
        self.encoder2 = UNet.DoubleConv2d(features,2*features)
        self.pool2 = nn.MaxPool2d(2, 2)

        self.encoder3 = UNet.DoubleConv2d(2*features,4*features)
        self.pool3 = nn.MaxPool2d(2, 2)
        
        self.encoder4 = UNet.DoubleConv2d(4*features,8*features)
        self.pool4 = nn.MaxPool2d(2, 2)
        
        self.bottleneck = UNet.DoubleConv2d(8*features,16*features)
        self.fc = nn.Sequential(nn.Conv2d(16*features, 1, kernel_size=1),
        							nn.ReLU(inplace=True),
        							nn.Flatten(),
                                    nn.Linear(int((input_size/16)**2),64),
                                    nn.ReLU(inplace=True),
                                    nn.Linear(64,64),
                                    nn.ReLU(inplace=True),
                                    nn.Linear(64,int((input_size/16)**2)),
                                    nn.ReLU(inplace=True)
        							)
        
        self.upconv4 = nn.ConvTranspose2d(16*features+1,8*features,kernel_size=2,stride=2)        
        self.decoder4 = UNet.DoubleConv2d(16*features,8*features) #concate, 2*8=16
        
        self.upconv3 = nn.ConvTranspose2d(8*features,4*features,kernel_size=2,stride=2)
        self.decoder3 = UNet.DoubleConv2d(8*features,4*features) #concate, 2*4=8
        
        self.upconv2 = nn.ConvTranspose2d(4*features,2*features,kernel_size=2,stride=2)
        self.decoder2 = UNet.DoubleConv2d(4*features,2*features) #concate, 2*2=4
        
        
        self.upconv1 = nn.ConvTranspose2d(2*features,features,kernel_size=2,stride=2)
        self.decoder1 = UNet.DoubleConv2d(2*features,features) #concate, 2*1=2
        
        self.conv_out = nn.Conv2d(features, 1, 1)
        
    def forward(self, input):
        enc1 = self.encoder1(input)
        
        enc2 = self.encoder2(self.pool1(enc1))
        
        enc3 = self.encoder3(self.pool2(enc2))
        
        enc4 = self.encoder4(self.pool3(enc3))
        
        bottleneck = self.bottleneck(self.pool4(enc4))
        ext = self.fc(bottleneck).view(-1,1,bottleneck.shape[2],bottleneck.shape[3])

        bottleneck = torch.cat([ext,bottleneck],dim=1)

        dec4 = torch.cat([enc4,self.upconv4(bottleneck)],dim=1)
        dec4 = self.decoder4(dec4)
        
        dec3 = torch.cat([enc3,self.upconv3(dec4)],dim=1)
        dec3 = self.decoder3(dec3)
        
        dec2 = torch.cat([enc2,self.upconv2(dec3)],dim=1)
        dec2 = self.decoder2(dec2)
        
        dec1 = torch.cat([enc1,self.upconv1(dec2)],dim=1)
        dec1 = self.decoder1(dec1)
        
        output = torch.sigmoid(self.conv_out(dec1))
        
        return output
    
    def DoubleConv2d(in_channels,features):
        return nn.Sequential(
                nn.Conv2d(in_channels, features, kernel_size=3, padding=1),
                nn.BatchNorm2d(features),
                nn.ReLU(inplace=True),
                nn.Conv2d(features, features, kernel_size=3, padding=1),
                nn.BatchNorm2d(features),
                nn.ReLU(inplace=True)
        )

class DoubleConv2d(nn.Module):
    def __init__(self,in_channels,features):
        super(DoubleConv2d, self).__init__()
        self.block = nn.Sequential(
                nn.Conv2d(in_channels, features, kernel_size=3, padding=1),
                #nn.BatchNorm2d(features),
                nn.GroupNorm(num_groups=int(features/16), num_channels=features),
                nn.ReLU(inplace=True),
                nn.Conv2d(features, features, kernel_size=3, padding=1),
                #nn.BatchNorm2d(features),
                nn.GroupNorm(num_groups=int(features/16), num_channels=features),
                nn.ReLU(inplace=True)
                )

    def forward(self,x):
        return self.block(x)
# def DoubleConv2d(in_channels,features):
#         return nn.Sequential(
#                 nn.Conv2d(in_channels, features, kernel_size=3, padding=1),
#                 nn.BatchNorm2d(features),
#                 nn.ReLU(inplace=True),
#                 nn.Conv2d(features, features, kernel_size=3, padding=1),
#                 nn.BatchNorm2d(features),
#                 nn.ReLU(inplace=True)
#         )

class UNet_OF(nn.Module):
    def __init__(self,in_channels=1,out_channels=1,init_features=64,input_size=256):
        super(UNet_OF, self).__init__()

        features = init_features
        self.encoder_optfl = nn.Sequential(DoubleConv2d(in_channels,32),
                                            nn.Conv2d(32,1,kernel_size=1),
                                            nn.ReLU(inplace=True)
                                            )

        self.encoder1 = DoubleConv2d(in_channels,features)
        self.pool1 = nn.MaxPool2d(2, 2)
        
        self.encoder2 = DoubleConv2d(features,2*features)
        self.pool2 = nn.MaxPool2d(2, 2)

        self.encoder3 = DoubleConv2d(2*features,4*features)
        self.pool3 = nn.MaxPool2d(2, 2)
        
        self.encoder4 = DoubleConv2d(4*features,8*features)
        self.pool4 = nn.MaxPool2d(2, 2)
        
        self.bottleneck = DoubleConv2d(8*features,16*features)
        
        self.upconv4 = nn.ConvTranspose2d(16*features,8*features,kernel_size=2,stride=2)        
        self.decoder4 = DoubleConv2d(16*features,8*features) #concate, 2*8=16
        
        self.upconv3 = nn.ConvTranspose2d(8*features,4*features,kernel_size=2,stride=2)
        self.decoder3 = DoubleConv2d(8*features,4*features) #concate, 2*4=8
        
        self.upconv2 = nn.ConvTranspose2d(4*features,2*features,kernel_size=2,stride=2)
        self.decoder2 = DoubleConv2d(4*features,2*features) #concate, 2*2=4
        
        
        self.upconv1 = nn.ConvTranspose2d(2*features,features,kernel_size=2,stride=2)
        self.decoder1 = DoubleConv2d(2*features,features) #concate, 2*1=2
        
        self.conv_out = nn.Conv2d(features, 1, 1)
        
    def forward(self, input, mask, mode):
        if mode:
            # mode==1: with optical flow attention
            tmp = self.encoder1(input)
            enc1 = tmp*self.encoder_optfl(mask)+tmp
        else:
            # mode==0: normal UNet FP
            enc1 = self.encoder1(input)
     
        enc2 = self.encoder2(self.pool1(enc1))
        
        enc3 = self.encoder3(self.pool2(enc2))
        
        enc4 = self.encoder4(self.pool3(enc3))
        
        bottleneck = self.bottleneck(self.pool4(enc4))

        dec4 = torch.cat([enc4,self.upconv4(bottleneck)],dim=1)
        dec4 = self.decoder4(dec4)
        
        dec3 = torch.cat([enc3,self.upconv3(dec4)],dim=1)
        dec3 = self.decoder3(dec3)
        
        dec2 = torch.cat([enc2,self.upconv2(dec3)],dim=1)
        dec2 = self.decoder2(dec2)
        
        dec1 = torch.cat([enc1,self.upconv1(dec2)],dim=1)
        dec1 = self.decoder1(dec1)
        
        output = torch.sigmoid(self.conv_out(dec1))
        
        return output
    
    # def DoubleConv2d(in_channels,features):
    #     return nn.Sequential(
    #             nn.Conv2d(in_channels, features, kernel_size=3, padding=1),
    #             nn.BatchNorm2d(features),
    #             nn.ReLU(inplace=True),
    #             nn.Conv2d(features, features, kernel_size=3, padding=1),
    #             nn.BatchNorm2d(features),
    #             nn.ReLU(inplace=True)
    #     )