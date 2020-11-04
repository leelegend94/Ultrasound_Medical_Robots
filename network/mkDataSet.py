import os
from PIL import Image,ImageEnhance
import yaml

#root_path = os.path.expanduser("~/workspace/us_robot/DataSet/realDataSet/linear/vessel")
#sav_path = os.path.expanduser("~/workspace/us_robot/DataSet/realDataSet/linear/vessel_dataset")
root_path = "./"
sav_path = "./vessel_dataset"

if not os.path.isdir(sav_path):
    os.makedirs(os.path.join(sav_path,'img'))
    os.makedirs(os.path.join(sav_path,'label'))

entries = os.listdir(root_path)
entries = list(filter(lambda x: x.startswith('set'), entries))

stream_begin = []
idx = 1

for entry in entries:
    img_path = os.path.join(root_path,entry,'img')
    label_path = os.path.join(root_path,entry,'label')

    stream_begin += [idx]

    for (fimg,flabel) in zip(sorted(os.listdir(img_path)),sorted(os.listdir(label_path))):
        img = Image.open(os.path.join(img_path,fimg))
        label = Image.open(os.path.join(label_path,flabel))
        
        # img = img.resize((img.width,256),Image.LANCZOS)
        # img = img.resize((256,256),Image.CUBIC)
        img = img.resize((256,256),Image.LANCZOS)
        label = label.resize((256,256),Image.NEAREST)
        
        #img = ImageEnhance.Contrast(img).enhance(1.5)
        
        img.save(os.path.join(sav_path,'img',"img%04d.png" % idx))
        label.save(os.path.join(sav_path,'label',"label%04d.png" % idx))
        idx +=1

config = {"stream_begin": stream_begin}
with open(sav_path+"/config.yaml","w") as f:
    yaml.dump(config,f)
f.close()