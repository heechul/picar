require 'torch'
require 'xlua'
require 'optim'
require 'gnuplot'
require 'pl'
require 'trepl'
require 'adaMax_binary_clip_shift'
require 'adam_binary_clip_b'
require 'nn'
require 'SqrHingeEmbeddingCriterion'

cmd = torch.CmdLine()
cmd:option('-type',               'cuda',                 'float or cuda')
cmd:option('-stcNeurons',         true,                   'use stochastic binarization for the neurons')
cmd:option('-stcWeights',         false,                  'use stochastic binarization for the weights')
opt = cmd:parse(arg or {})

local params = require('params')
local model = require("Models/BinaryNet_DeepPicar_Model").model
model:cuda()
require("LuaRandomSample")

local train_path = paths.concat('datasets', 'train.t7')
local val_path = paths.concat('datasets', 'val.t7')

local train_set = torch.load(train_path)
local val_set = torch.load(val_path)

--Turns Lua table into a tensor
--Source: https://groups.google.com/forum/#!topic/torch7/bdY_AveKn2k
function TableToTensor(table)
  local tensorSize = table[1]:size()
  local tensorSizeTable = {-1}
  for i=1,tensorSize:size(1) do
    tensorSizeTable[i+1] = tensorSize[i]
  end
  merge=nn.Sequential()
    :add(nn.JoinTable(1))
    :add(nn.View(unpack(tensorSizeTable)))

  return merge:forward(table)
end

local function load_batch(set)
    n = table.getn(set.frames)

    local numrange = range(1,n)
    numrange = permute(numrange,n,n)
    batchNums = subrange(numrange,params.batch_size)

    local batch_frames = {}
    local batch_angles = {}

    for i = 1,params.batch_size do
      batch_frames[#batch_frames+1] = set.frames[batchNums[i]]
      batch_angles[#batch_angles+1] = set.angles[batchNums[i]]
    end

    batch_frames = TableToTensor(batch_frames)
    batch_angles = torch.Tensor(batch_angles)

    return batchNums, batch_frames, batch_angles
end

criterion = nn.MSECriterion()

for i = 1,params.training_steps do
    frame_batch, angle_batch = load_batch(train_set)

    criterion:forward(model:forward(frame_batch), angle_batch)

    model:zeroGradParameters()
    model:backward(frame_batch, criterion:backward(angle_batch))
    model:updateParameters(2^-6)
end
