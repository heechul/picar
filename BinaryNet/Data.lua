--[[
This code create the training test and validation datasets and preform diffrent kinds of preprocessing
This code is based on elad hoffer Data.lua file from ConvNet-torch library (https://github.com/eladhoffer/ConvNet-torch.git) and uses:
  - Elad Hoffer DataProvidor.torch library: https://github.com/eladhoffer/DataProvider.torch.git
  - Nicholas Leonard dp library: https://github.com/nicholas-leonard/dp.git
  - Koray Kavukcuoglu dp library: https://github.com/koraykv/unsup.git
]]
require 'dp'
local DataProvider = require 'DataProvider'
local opt = opt or {}
local Dataset = opt.dataset or 'Cifar10'
local PreProcDir = opt.preProcDir or './PreProcData/'
local Whiten = opt.whiten or false
local NormelizeWhiten = opt.NormelizeWhiten or false
local DataPath = opt.datapath or '/home/itayh/Datasets/'
local normalization = opt.normalization or 'simple'
local format = opt.format or 'rgb'
local TestData
local TrainData
local ValidData
local Classes

if Dataset =='Cifar100' then
  local file_valid = paths.concat(PreProcDir, format .. 'whiten_valid.t7')
  local file_train = paths.concat(PreProcDir, format .. 'whiten_train.t7')
  local file_test = paths.concat(PreProcDir, format .. 'whiten_test.t7')
  if (paths.filep(file_valid) and paths.filep(file_train) and paths.filep(file_test)) then
    ValidData=torch.load(file_valid)
    TrainData=torch.load(file_train)
    TestData=torch.load(file_test)
  else
    if paths.dirp(PreProcDir)==false then
     sys.execute('mkdir PreProcData/Cifar100')
    end
    input_preprocess = {}
    table.insert(input_preprocess, dp.ZCA())
    ds = dp.Cifar100{scale={0,1}, valid_ratio=0.1,input_preprocess = input_preprocess}
    ValidData = {data=ds:validSet():inputs():input():clone():float(), label=ds:validSet():targets():input():clone():byte() }
    TrainData = {data=ds:trainSet():inputs():input():float(), label=ds:trainSet():targets():input():byte() }
    TestData  = {data=ds:testSet():inputs():input():float() , label=ds:testSet():targets():input():byte()  }
    collectgarbage()
    torch.save(file_valid,ValidData)
    torch.save(file_train,TrainData)
    torch.save(file_test,TestData)
  end
elseif Dataset == 'Cifar10' then
    local file_valid = paths.concat(PreProcDir, format .. 'whiten_valid.t7')
    local file_train = paths.concat(PreProcDir, format .. 'whiten_train.t7')
    local file_test = paths.concat(PreProcDir, format .. 'whiten_test.t7')
    if (paths.filep(file_valid) and paths.filep(file_train) and paths.filep(file_test)) then
      ValidData=torch.load(file_valid)
      TrainData=torch.load(file_train)
      TestData=torch.load(file_test)
    else
      if paths.dirp(PreProcDir)==false then
       sys.execute('mkdir PreProcData/Cifar10')
      end
      input_preprocess = {}
      table.insert(input_preprocess, dp.ZCA())
      ds = dp.Cifar10{scale={0,1},valid_ratio=0.1,input_preprocess = input_preprocess} --,input_preprocess = input_preprocess}  scale={0,1},
      ValidData = {data=ds:validSet():inputs():input():float(), label=ds:validSet():targets():input():clone():byte() }
      TrainData = {data=ds:trainSet():inputs():input():float(), label=ds:trainSet():targets():input():byte() }
      TestData  = {data=ds:testSet():inputs():input():float(), label=ds:testSet():targets():input():byte()  }
      collectgarbage()
      torch.save(file_valid,ValidData)
      torch.save(file_train,TrainData)
      torch.save(file_test,TestData)
    end
    Classes = {'airplane', 'automobile', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck'}
elseif Dataset == 'MNIST' then
  local file_valid = paths.concat(PreProcDir, format .. '_valid.t7')
  local file_train = paths.concat(PreProcDir, format .. '_train.t7')
  local file_test = paths.concat(PreProcDir, format .. '_test.t7')
  if (paths.filep(file_valid) and paths.filep(file_train) and paths.filep(file_test)) then
    ValidData=torch.load(file_valid)
    TrainData=torch.load(file_train)
    TestData=torch.load(file_test)
  else
    if paths.dirp(PreProcDir)==false then
     sys.execute('mkdir PreProcData/MNIST')
    end
    ds = dp.Mnist{scale={0,1}}
    ValidData = {data=ds:validSet():inputs():input():clone():float(), label=ds:validSet():targets():input():clone():byte() }
    TrainData = {data=ds:trainSet():inputs():input():float(), label=ds:trainSet():targets():input():byte() }
    TestData  = {data=ds:testSet():inputs():input():float() , label=ds:testSet():targets():input():byte()  }
    collectgarbage()
    torch.save(file_valid,ValidData)
    torch.save(file_train,TrainData)
    torch.save(file_test,TestData)
  end
  Classes = {1,2,3,4,5,6,7,8,9,0}
elseif Dataset == 'SVHN' then
    local LCNfile_valid = paths.concat(PreProcDir, format .. 'GCN_LCN_valid.t7')
    local LCNfile_train = paths.concat(PreProcDir, format .. 'GCN_LCN_train.t7')
    local LCNfile_test = paths.concat(PreProcDir, format .. 'GCN_LCN_test.t7')
    print(LCNfile_valid)
    if (paths.filep(LCNfile_valid) and paths.filep(LCNfile_train) and paths.filep(LCNfile_test)) then
      ValidData=torch.load(LCNfile_valid)
      TrainData=torch.load(LCNfile_train)
      TestData=torch.load(LCNfile_test)
    else
      if paths.dirp(PreProcDir)==false then
       sys.execute('mkdir PreProcData/SVHN')
      end
      local input_preprocess = {}
      table.insert(input_preprocess, dp.GCN{batch_size=5000,use_std=true,sqrt_bias=10})
      table.insert(input_preprocess, dp.LeCunLCN{kernel_size=9,divide_by_std=true,batch_size=5000,progress=true}) --,kernel_size=31,kernel_std=32})
      ds = dp.Svhn{scale={0,1}, input_preprocess = input_preprocess}
      ValidData = {data=ds:validSet():inputs():input():float(), label=ds:validSet():targets():input():byte() }; ValidData.data:div( ValidData.data:max())
      TrainData = {data=ds:trainSet():inputs():input():float(), label=ds:trainSet():targets():input():byte() }; TrainData.data:div( TrainData.data:max())
      TestData  = {data=ds:testSet():inputs():input():float(), label=ds:testSet():targets():input():byte() };  TestData.data:div( TestData.data:max())

      collectgarbage()
      torch.save(LCNfile_valid,ValidData)
      torch.save(LCNfile_train,TrainData)
      torch.save(LCNfile_test,TestData)
    end
    Classes = {1,2,3,4,5,6,7,8,9,0}
end

TrainData.data = TrainData.data:float()
TestData.data = TestData.data:float()

local TrainDataProvider = DataProvider.Container{
  Name = 'TrainingData',
  CachePrefix = nil,
  CacheFiles = false,
  Source = {TrainData.data,TrainData.label},
  MaxNumItems = 1e6,
  CopyData = false,
  TensorType = 'torch.FloatTensor',
}
local TestDataProvider = DataProvider.Container{
  Name = 'TestData',
  CachePrefix = nil,
  CacheFiles = false,
  Source = {TestData.data, TestData.label},
  MaxNumItems = 1e6,
  CopyData = false,
  TensorType = 'torch.FloatTensor',

}
local ValidDataProvider = DataProvider.Container{
  Name = 'ValidData',
  CachePrefix = nil,
  CacheFiles = false,
  Source = {ValidData.data, ValidData.label},
  MaxNumItems = 1e6,
  CopyData = false,
  TensorType = 'torch.FloatTensor',

}

--Preprocesss

  if format == 'yuv' then
    require 'image'
    TrainDataProvider:apply(image.rgb2yuv)
    TestDataProvider:apply(image.rgb2yuv)
  end
  if Whiten then
    require 'unsup'
    local meanfile = paths.concat(PreProcDir, format .. 'imageMean.t7')
    local mean, P, invP
    local Pfile = paths.concat(PreProcDir,format .. 'P.t7')
    local invPfile = paths.concat(PreProcDir,format .. 'invP.t7')

    if (paths.filep(Pfile) and paths.filep(invPfile) and paths.filep(meanfile)) then
      P = torch.load(Pfile)
      invP = torch.load(invPfile)
      mean = torch.load(meanfile)
      TrainDataProvider.Data = unsup.zca_whiten(TrainDataProvider.Data, mean, P, invP)
    else
      TrainDataProvider.Data, mean, P, invP = unsup.zca_whiten(TrainDataProvider.Data)
      torch.save(Pfile,P)
      torch.save(invPfile,invP)
      torch.save(meanfile,mean)
    end
      TestDataProvider.Data = unsup.zca_whiten(TestDataProvider.Data, mean, P, invP)
      ValidDataProvider.Data = unsup.zca_whiten(ValidDataProvider.Data, mean, P, invP)
  elseif dp_prepro then
        -- Do nothing since we use dp lib for GCN and LCN
  else
      local meanfile = paths.concat(PreProcDir, format .. normalization .. 'Mean.t7')
      local stdfile = paths.concat(PreProcDir,format .. normalization .. 'Std.t7')
      local mean, std
      local loaded = false

      if paths.filep(meanfile) and paths.filep(stdfile) then
        mean = torch.load(meanfile)
        std = torch.load(stdfile)
        loaded = true
      end

      mean, std = TrainDataProvider:normalize(normalization, mean, std)
      TestDataProvider:normalize(normalization, mean, std)
      ValidDataProvider:normalize(normalization, mean, std)
      if not loaded then
        torch.save(meanfile,mean)
        torch.save(stdfile,std)
      end
    end



return{
    TrainData = TrainDataProvider,
    TestData = TestDataProvider,
    ValidData = ValidDataProvider,
    Classes = Classes
}
