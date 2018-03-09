--[[
   This file implements Shift based Batch Normalization based a variant of the vanilla BN as described in the paper:
   "Binarized Neural Networks: Training Deep Neural Networks with Weights and Activations Constrained to +1 or -1, Matthieu Courbariaux, Itay Hubara, Daniel Soudry, Ran El-Yaniv, Yoshua Bengio'

   The code is based on nn library
   --]]
local SpatialBatchNormalizationShiftPow2,parent = torch.class('SpatialBatchNormalizationShiftPow2', 'nn.Module')

function SpatialBatchNormalizationShiftPow2:__init(nFeature, runningVal, eps, momentum)
   parent.__init(self)
   assert(nFeature and type(nFeature) == 'number',
          'Missing argument #1: Number of feature planes. ' ..
          'Give 0 for no affine transform')
   self.eps = eps or 1e-5
   self.train = true
   self.momentum = momentum or 0.125
   self.runningVal = runningVal or true
   self.running_mean = torch.Tensor()
   self.running_std = torch.Tensor()
   self.running_std_ap2 = torch.Tensor()
   if nFeature > 0 then self.affine = true end

   if self.affine then
      self.weight = torch.Tensor(nFeature)
      self.weightSign = torch.Tensor(nFeature)
      self.weight_ap2 = torch.Tensor(nFeature)
      self.bias = torch.Tensor(nFeature)
      self.gradWeight = torch.Tensor(nFeature)
      self.gradBias = torch.Tensor(nFeature)
      self:reset()
   end
end

function SpatialBatchNormalizationShiftPow2:reset()
   self.weight:fill(1)
   self.bias:zero()
end

function SpatialBatchNormalizationShiftPow2:updateOutput(input)
   assert(input:dim() == 4, 'only mini-batch supported (4D tensor), got '
             .. input:dim() .. 'D tensor instead')
   local nBatch = input:size(1)
   local nFeature = input:size(2)
   local iH = input:size(3)
   local iW = input:size(4)

   -- buffers that are reused
   self.buffer = self.buffer or input.new()
   self.buffer2 = self.buffer2 or input.new()
   self.centered = self.centered or input.new()
   self.centered:resizeAs(input)
   self.centeredOrg = self.centeredOrg or input.new()
   self.centeredOrg:resizeAs(input)
   self.centeredSign = self.centeredSign or input.new()
   self.centeredSign:resizeAs(input)
   self.std = self.std or input.new()
   self.normalized = self.normalized or input.new()
   self.normalized:resizeAs(input)
   self.normalizedSign = self.normalizedSign or input.new()
   self.normalizedSign:resizeAs(input)
   self.output:resizeAs(input)
   self.gradInput:resizeAs(input)
   if self.train == false and self.runningVal == true then
      assert(self.running_mean:nDimension() ~= 0,
             'Module never run on training data. First run on some training data before evaluating.')
      self.output:copy(input)
      self.buffer:repeatTensor(self.running_mean:view(1, nFeature, 1, 1), nBatch, 1, iH, iW)
      self.output:add(-1, self.buffer)
      self.running_std_ap2:copy(torch.pow(2,torch.round(torch.log(self.running_std):div(math.log(2)))))
      self.buffer:repeatTensor(self.running_std_ap2:view(1, nFeature, 1, 1), nBatch, 1, iH, iW)
      self.output:cmul(self.buffer)
   else -- training mode
      if self.running_mean:nDimension() == 0 then
         self.running_mean:resize(nFeature):zero()
      end
      if self.running_std:nDimension() == 0 then
         self.running_std:resize(nFeature):zero()
         self.running_std_ap2:resize(nFeature):zero()
      end
      -- calculate mean over mini-batch, over feature-maps
      local in_folded = input:view(nBatch, nFeature, iH * iW)
      self.buffer:mean(in_folded, 1)
      self.buffer2:mean(self.buffer, 3)
      self.running_mean:mul(1 - self.momentum):add(self.momentum, self.buffer2) -- add to running mean
      self.buffer:repeatTensor(self.buffer2:view(1, nFeature, 1, 1),
                               nBatch, 1, iH, iW)

      -- subtract mean
      self.centered:add(input, -1, self.buffer)                  -- x - E(x)
      self.centeredOrg:copy(self.centered)
      self.centeredSign:copy(self.centered)

      self.centeredSign:sign()
      self.centered:copy(torch.pow(2,torch.round(torch.log(self.centered:abs()):div(math.log(2))))):cmul(self.centeredSign)
      -- calculate standard deviation over mini-batch

      self.buffer:copy(self.centered):cmul(self.centeredOrg) --:abs()
      -- calculate standard deviation over mini-batch

      local buf_folded = self.buffer:view(nBatch,nFeature,iH*iW)
      self.std:mean(self.buffer2:mean(buf_folded, 1), 3)
      self.std:add(self.eps):sqrt():pow(-1)      -- 1 / E([x - E(x)]^2)
      self.running_std:mul(1 - self.momentum):add(self.momentum, self.std) -- add to running stdv
      self.std:copy(torch.pow(2,torch.round(torch.log(self.std):div(math.log(2)))))


      self.buffer:repeatTensor(self.std:view(1, nFeature, 1, 1),
                               nBatch, 1, iH, iW)

      -- divide standard-deviation + eps
      self.output:cmul(self.centeredOrg, self.buffer)
      self.normalized:copy(self.output)
      self.normalizedSign:copy(self.normalized)
      self.normalizedSign:sign()
      self.normalized:copy(torch.pow(2,torch.round(torch.log(self.normalized:abs()):div(math.log(2)))):cmul(self.normalizedSign))
    --  self.normalized[self.normalized:lt(0)]=1; -- Can improve results
   end

   if self.affine then
      -- multiply with gamma and add beta
      self.weight_ap2:copy(self.weight)
      self.weightSign:copy(self.weight):sign()
      self.weight_ap2:copy(torch.pow(2,torch.round(torch.log(self.weight:clone():abs()):div(math.log(2))))):cmul(self.weightSign)
      --self.weight:fill(1) --Almost similar results
      self.buffer:repeatTensor(self.weight_ap2:view(1, nFeature, 1, 1),nBatch, 1, iH, iW)
      self.output:cmul(self.buffer)
      self.buffer:repeatTensor(self.bias:view(1, nFeature, 1, 1),
                               nBatch, 1, iH, iW)
      self.output:add(self.buffer)
   end

   return self.output
end

function SpatialBatchNormalizationShiftPow2:updateGradInput(input, gradOutput)
   assert(input:dim() == 4, 'only mini-batch supported')
   assert(gradOutput:dim() == 4, 'only mini-batch supported')
   assert(self.train == true, 'should be in training mode when self.train is true')
   local nBatch = input:size(1)
   local nFeature = input:size(2)
   local iH = input:size(3)
   local iW = input:size(4)

   self.gradInput:cmul(self.centered, gradOutput)
   local gi_folded = self.gradInput:view(nBatch, nFeature, iH * iW)
   self.buffer2:mean(self.buffer:mean(gi_folded, 1), 3)
   self.gradInput:repeatTensor(self.buffer2:view(1, nFeature, 1, 1),
                               nBatch, 1, iH, iW)
   self.gradInput:cmul(self.centered):mul(-1)
   self.buffer:repeatTensor(self.std:view(1, nFeature, 1, 1),
                            nBatch, 1, iH, iW)
   self.gradInput:cmul(self.buffer):cmul(self.buffer)

   self.buffer:mean(gradOutput:view(nBatch, nFeature, iH*iW), 1)
   self.buffer2:mean(self.buffer, 3)
   self.buffer:repeatTensor(self.buffer2:view(1, nFeature, 1, 1),
                            nBatch, 1, iH, iW)
   self.gradInput:add(gradOutput):add(-1, self.buffer)
   self.buffer:repeatTensor(self.std:view(1, nFeature, 1, 1),
                            nBatch, 1, iH, iW)
   self.gradInput:cmul(self.buffer)

   if self.affine then
      self.buffer:repeatTensor(self.weight_ap2:view(1, nFeature, 1, 1),
                               nBatch, 1, iH, iW)
      self.gradInput:cmul(self.buffer)
   end

   return self.gradInput
end

function SpatialBatchNormalizationShiftPow2:accGradParameters(input, gradOutput, scale)
   if self.affine then
      scale = scale or 1.0
      local nBatch = input:size(1)
      local nFeature = input:size(2)
      local iH = input:size(3)
      local iW = input:size(4)
      self.buffer2:resizeAs(self.normalized):copy(self.normalized)
      self.buffer2 = self.buffer2:cmul(gradOutput):view(nBatch, nFeature, iH*iW)
      self.buffer:sum(self.buffer2, 1) -- sum over mini-batch
      self.buffer2:sum(self.buffer, 3) -- sum over pixels
      self.gradWeight:add(scale, self.buffer2)

      self.buffer:sum(gradOutput:view(nBatch, nFeature, iH*iW), 1)
      self.buffer2:sum(self.buffer, 3)
      self.gradBias:add(scale, self.buffer2) -- sum over mini-batch
   end
end
