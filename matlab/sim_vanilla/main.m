
ndataset = 2000;
path = "/home/zhenyuli/workspace/us_robot/simulator/SimDataset2";
%path = "/home/zhenyuli/workspace/us_robot/simulator/SimDatasetTest2";
if exist(path,'dir')
    rmdir(path,'s')
    mkdir(path)
else
    mkdir(path)
end

R = eul2rotm([pi/3,0,-pi/2.2]);
T = [4;-2;-5];
Hvw = [R T; 0 0 0 1];

Hdw_init = [-1 0  0 0;
        0 1  0 0;
        0 0 -1 0;
        0 0  0 1]; %detector init pose wrt. world frame

cntr = 0;

while(cntr<ndataset)    
    %x: [-1.5 2] z:[-4 5]
    R1 = eul2rotm([random_in_range([1,2],[-0.5,0.5]),0]);
    T1 = [random_in_range(1,[-1,1]);0;random_in_range(1,[-2,3])];
    Hdw = [R1 T1; 0 0 0 1]*Hdw_init;

    [image,label] = vesselSim(random_in_range(1,[0.1,0.5]),Hvw,Hdw,false);
    
    if(~(sum(sum(label))<1000 && rand()<0.5))
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