require 'torch'
require 'cutorch'
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
local model = require(params.model_file).model
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

    batch_frames = TableToTensor(batch_frames):cuda()
    batch_angles = torch.Tensor(batch_angles):cuda()

    return batch_frames, batch_angles
end

function getTimePassed(seconds)
  local minutes = math.floor(seconds / 60)
  local remaining = seconds % 60
  return minutes,remaining
end

criterion = nn.MSECriterion():cuda()
parameters,gradParameters = model:getParameters()
local optimState = {learningRate = 0.01}

starttime = os.time()
local save_path = paths.concat(params.save_dir, params.save_file)
local clone = model:clone('weight','bias','running_mean','running_std')

for i = 1,params.training_steps do
    frame_batch, angle_batch = load_batch(train_set)

    function feval(parameters)
      gradParameters:zero()

      local outputs = model:forward(frame_batch)
      local loss = criterion:forward(outputs, angle_batch)
      local dloss_doutputs = criterion:backward(outputs, angle_batch)
      model:backward(frame_batch, dloss_doutputs)

      return loss, gradParameters
    end
    optim.sgd(feval, parameters, optimState)

    if i % 10 == 0 then
      val_frames, val_angles = load_batch(train_set)
      local trainloss = criterion:forward(model:forward(frame_batch), angle_batch)
      local valloss = criterion:forward(model:forward(val_frames), val_angles)
      print("Finished "..i.." training steps, train loss "..trainloss..", val loss "..valloss)
    end

    if i % 100 == 0 then
      torch.save(save_path, model:clearState())
      timepassed = os.time() - starttime
      minutes,seconds = getTimePassed(timepassed)
      print('Model saved. Time passed: '..minutes..'m'..seconds..'s')
    end
end
