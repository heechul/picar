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
require("LuaRandomSample")

local train_path = paths.concat('datasets', 'train.t7')
local val_path = paths.concat('datasets', 'val.t7')

local train_set = torch.load(train_path)
local val_set = torch.load(val_path)

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

    return batchNums, batch_frames, batch_angles
end

--[[
for i = 1,params.training_steps do

end
]]
