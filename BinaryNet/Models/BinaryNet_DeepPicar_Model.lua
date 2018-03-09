require 'nn'
require './BinaryLinear.lua'
require './BinarizedNeurons'

local SpatialConvolution
local SpatialMaxPooling
if opt.type =='cuda' then
  require 'cunn'
  require 'cudnn'
  require './cudnnBinarySpatialConvolution.lua'
  SpatialConvolution = cudnnBinarySpatialConvolution
else
  require './BinarySpatialConvolution.lua'
  SpatialConvolution = BinarySpatialConvolution
end

local model = nn.Sequential()

--Convolutional Layer 1
model:add(SpatialConvolution(3,24,5,5,1,1,1,1,opt.stcWeights))
model:add(nn.HardTanh())
model:add(BinarizedNeurons(opt.stcNeurons))

--Convolutional Layer 2
model:add(SpatialConvolution(24,36,5,5,1,1,1,1,opt.stcWeights))
model:add(nn.HardTanh())
model:add(BinarizedNeurons(opt.stcNeurons))

--Convolutional Layer 3
model:add(SpatialConvolution(36,48,5,5,1,1,1,1,opt.stcWeights))
model:add(nn.HardTanh())
model:add(BinarizedNeurons(opt.stcNeurons))

--Convolutional Layer 4
model:add(SpatialConvolution(48,64,3,3,1,1,1,1,opt.stcWeights))
model:add(nn.HardTanh())
model:add(BinarizedNeurons(opt.stcNeurons))

--Convolutional Layer 5
model:add(SpatialConvolution(64,64,3,3,1,1,1,1,opt.stcWeights))
model:add(nn.HardTanh())
model:add(BinarizedNeurons(opt.stcNeurons))

--Fully Connected Layer 1
model:add(nn.View(1152))
model:add(BinaryLinear(1152,100,opt.stcWeights))
model:add(nn.HardTanh())
model:add(nn.Dropout())
model:add(BinarizedNeurons(opt.stcNeurons))

--Fully Connected Layer 2
model:add(BinaryLinear(100,10,opt.stcWeights))
model:add(nn.HardTanh())
model:add(nn.Dropout())
model:add(BinarizedNeurons(opt.stcNeurons))

--Fully Connected Layer 3
model:add(BinaryLinear(10,1,opt.stcWeights))
 
local dE, param = model:getParameters()
local weight_size = dE:size(1)
local learningRates = torch.Tensor(weight_size):fill(0)
local clipvector = torch.Tensor(weight_size):fill(1)
local counter = 0
for i, layer in ipairs(model.modules) do
   if layer.__typename == 'BinaryLinear' then
      local weight_size = layer.weight:size(1)*layer.weight:size(2)
      local size_w=layer.weight:size();   GLR=1/torch.sqrt(1.5/(size_w[1]+size_w[2]))
      GLR=(math.pow(2,torch.round(math.log(GLR)/(math.log(2)))))
      learningRates[{{counter+1, counter+weight_size}}]:fill(GLR)
      clipvector[{{counter+1, counter+weight_size}}]:fill(1)
      counter = counter+weight_size
      local bias_size = layer.bias:size(1)
      learningRates[{{counter+1, counter+bias_size}}]:fill(GLR)
      clipvector[{{counter+1, counter+bias_size}}]:fill(0)
      counter = counter+bias_size
      counter = counter+bias_size
    elseif layer.__typename == 'cudnnBinarySpatialConvolution' then
      local size_w=layer.weight:size();
      local weight_size = size_w[1]*size_w[2]*size_w[3]*size_w[4]

      local filter_size=size_w[3]*size_w[4]
      GLR=1/torch.sqrt(1.5/(size_w[1]*filter_size+size_w[2]*filter_size))
      GLR=(math.pow(2,torch.round(math.log(GLR)/(math.log(2)))))
      learningRates[{{counter+1, counter+weight_size}}]:fill(GLR)
      clipvector[{{counter+1, counter+weight_size}}]:fill(1)
      counter = counter+weight_size
      local bias_size = layer.bias:size(1)
      learningRates[{{counter+1, counter+bias_size}}]:fill(GLR)
      clipvector[{{counter+1, counter+bias_size}}]:fill(0)
      counter = counter+bias_size
      elseif layer.__typename == 'BinarySpatialConvolution' then
        local size_w=layer.weight:size();
        local weight_size = size_w[1]*size_w[2]*size_w[3]*size_w[4]

        local filter_size=size_w[3]*size_w[4]
        GLR=1/torch.sqrt(1.5/(size_w[1]*filter_size+size_w[2]*filter_size))
        GLR=(math.pow(2,torch.round(math.log(GLR)/(math.log(2)))))
        learningRates[{{counter+1, counter+weight_size}}]:fill(GLR)
        clipvector[{{counter+1, counter+weight_size}}]:fill(1)
        counter = counter+weight_size
        local bias_size = layer.bias:size(1)
        learningRates[{{counter+1, counter+bias_size}}]:fill(GLR)
        clipvector[{{counter+1, counter+bias_size}}]:fill(0)
        counter = counter+bias_size

  end
end
-- clip all parameter
clipvector:fill(1)
--
print(learningRates:eq(0):sum())
print(learningRates:ne(0):sum())
print(clipvector:ne(0):sum())
print(counter)
return {
     model = model,
     lrs = learningRates,
     clipV =clipvector,
  }
