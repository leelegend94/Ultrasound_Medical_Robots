
ndataset = 3000;
path = '~/workspace/us_robot/DataSet/SimRealDataset';
%path = '~/workspace/us_robot/DataSet/SimRealDatasetTest;
if exist(path,'dir')
    rmdir(path,'s')
    mkdir(path)
else
    mkdir(path)
end

R = eul2rotm([pi/3,0,-pi/2.2]);
T = [0.6; 0.1;-0.1];
Hvw = [R T; 0 0 0 1];

[Xc,Yc,Zc] = genVessel(0.01,Hvw);

Hdw_init = [-1 0  0  T(1);
             0 1  0  T(2);
             0 0 -1  0;
             0 0  0  1]; %detector init pose wrt. world frame

bg_path = '~/workspace/us_robot/DataSet/realDataSet/linear/background';
bg_list = dir(bg_path);
bg_list = bg_list(3:end);


cntr = 0;

while(cntr<ndataset)    
    %x: [-1.5 2] z:[-4 5]
    R1 = eul2rotm([random_in_range([1,2],[-0.1,0.1]),0]);
    T1 = [random_in_range(1,[-0.02,0.02]);0;random_in_range(1,[-0.02,0.02])];
    Hdw = [R1 T1; 0 0 0 1]*Hdw_init;

    bg_file = bg_list(randi(length(bg_list),1)).name;
    background = double(rgb2gray(imread([bg_path,'/',bg_file])))/255;
    
    [image,label] = vesselSim(Xc,Yc,Zc,Hdw,false,background);
    
    %if(~(sum(sum(label))<1000 && rand()<0.5))
    if(image~=-1)
        cntr = cntr+1;
        curr_sample_path = fullfile(path,num2str(cntr,'%04d'));
        mkdir(curr_sample_path)
        fimage = fullfile(curr_sample_path,'image.png');
        flabel = fullfile(curr_sample_path,'label.png');
        
        imwrite(image,fimage);
        imwrite(label,flabel); 
        disp(cntr)
        disp(sum(sum(label)))
    end
    %end

end
% figure()
% subplot('121')
% imshow(image)
% subplot('122')
% imshow(label)


function x = random_in_range(N,xrange)
xmin = xrange(1);
xmax = xrange(2);
x = (xmax-xmin).*rand(N)+xmin;
end