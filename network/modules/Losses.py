import torch
def DiceLoss(pred,target,smooth=2):
    
    index = (2*torch.sum(pred*target)+smooth)/(torch.sum(pred)+torch.sum(target)+smooth)
    #if torch.sum(target).item() == 0:
    #print("instersection: ",torch.sum(pred*target).item())
    #print("pred: ",torch.sum(pred).item())
    #print("target: ",torch.sum(target).item())
    #print("Index: ", index.item())
    return 1-index