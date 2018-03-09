--[[
   This file implements Shift based Batch Normalization based a variant of the vanilla BN as described in the paper:
   "Binarized Neural Networks: Training Deep Neural Networks with Weights and Activations Constrained to +1 or -1, Matthieu Courbariaux, Itay Hubara, Daniel Soudry, Ran El-Yaniv, Yoshua Bengio'

   The code is based on nn library
   --]]


local BatchNormalizationShiftPow2,parent = torch.class('BatchNormalizationShiftPow2', 'nn.Module')

function BatchNormalizationShiftPow2:__init(nOutput, runningVal, eps, momentum, affine)
   parent.__init(self)
   assert(nOutput and type(nOutput) == 'number',
          'Missing argument #1: dimensionality of input. ')
   assert(nOutput ~= 0, 'To set affine=false call BatchNormalization'
     .. '(nOutput,  eps, momentum, false) ')
   if affine ~= nil then
      assert(type(affine) == 'boolean', 'affine has to be true/false')
      self.affine = affine
   else
      self.affine = true
   end
   self.eps = eps or 1e-5
   self.train = true
   self.momentum = momentum or 0.125
   self.runningVal = runningVal or true
   self.running_mean = torch.zeros(nOutput)
   self.running_std = torch.ones(nOutput)
   self.running_std_ap2 = torch.ones(nOutput)
   if self.affine then
      self.weight = torch.Tensor(nOutput)
      self.weightSign = torch.Tensor(nOutput)
      self.weight_ap2 = torch.Tensor(nOutput)
      self.bias = torch.Tensor(nOutput)
      self.gradWeight = torch.Tensor(nOutput)
      self.gradBias = torch.Tensor(nOutput)
      self:reset()
   end
end

function BatchNormalizationShiftPow2:reset()
   self.weight:fill(1)
   self.bias:zero()
   self.running_mean:zero()
   self.running_std:fill(1)
end

function BatchNormalizationShiftPow2:updateOutput(input)
   assert(input:dim() == 2, 'only mini-batch supported (2D tensor), got '
             .. input:dim() .. 'D tensor instead')
   local nBatch = input:size(1)
   -- buffers that are reused
   self.buffer = self.buffer or input.new()
   self.buffer2 = self.buffer2 or input.new()
   self.centered = self.centered or input.new()
   self.centered:resizeAs(input)
   self.centerSign = self.centerSign or input.new()
   self.centerSign:resizeAs(input)
   self.centeredOrg = self.centeredOrg or input.new()
   self.centeredOrg:resizeAs(input)
   self.std = self.std or input.new()
   self.normalized = self.normalized or input.new()
   self.normalized:resizeAs(input)
   self.normalizedSign = self.normalizedSign or input.new()
   self.normalizedSign:resizeAs(input)
   self.output:resizeAs(input)
   self.gradInput:resizeAs(input)
   if self.train == false and self.runningVal == true then
     self.output:copy(input)
     self.buffer:repeatTensor(self.running_mean, nBatch, 1)
     self.output:add(-1, self.buffer)
     self.running_std_ap2:copy(torch.pow(2,torch.round(torch.log(self.running_std):div(math.log(2)))))
     self.buffer:repeatTensor(self.running_std_ap2, nBatch, 1)
     self.output:cmul(self.buffer)
   else -- training mode
      -- calculate mean over mini-batch
      self.buffer:mean(input, 1)                        -- E(x) = expectation of x.
      self.running_mean:mul(1 - self.momentum):add(self.momentum, self.buffer) -- add to running mean
      self.buffer:repeatTensor(self.buffer, nBatch, 1)

      -- subtract mean
      self.centered:add(input, -1, self.buffer)         -- x - E(x)
      self.centeredOrg:copy(self.centered)
      self.centerSign:copy(self.centered)
      self.centerSign:sign()
      self.centered:copy(torch.pow(2,torch.round(torch.log(self.centered:abs()):div(math.log(2))))):cmul(self.centerSign)
      -- calculate standard deviation over mini-batch
      self.buffer:copy(self.centered):cmul(self.centeredOrg) -- [x - E(x)]^2
      -- 1 / E([x - E(x)]^2)
      self.std:mean(self.buffer, 1):add(self.eps):sqrt():pow(-1)
      self.running_std:mul(1 - self.momentum):add(self.momentum, self.std) -- add to running stdv
      self.std:copy(torch.pow(2,torch.round(torch.log(self.std):div(math.log(2)))))
      self.buffer:repeatTensor(self.std, nBatch, 1)

      -- divide standard-deviation + eps

      self.output:cmul(self.centeredOrg, self.buffer)
      self.normalized:copy(self.output)
      self.normalizedSign:copy(self.normalized)
      self.normalizedSign:sign()

      self.normalized:copy(torch.pow(2,torch.round(torch.log(self.normalized:abs()):div(math.log(2)))):cmul(self.normalizedSign))
      --self.normalized[self.normalized:lt(0)]=1; -- Can improve results
   end

   if self.affine then
      -- multiply with gamma and add beta
      self.weightSign:copy(self.weight)
      self.weightSign:sign()
      self.weight_ap2:copy(torch.pow(2,torch.round(torch.log(self.weight:clone():abs()):div(math.log(2))))):cmul(self.weightSign)
      --self.weight:fill(1) --Almost similar results
      self.buffer:repeatTensor(self.weight_ap2, nBatch, 1)
      self.output:cmul(self.buffer)
      self.buffer:repeatTensor(self.bias, nBatch, 1)
      self.output:add(self.buffer)
   end
   return self.output
end

function BatchNormalizationShiftPow2:updateGradInput(input, gradOutput)
   assert(input:dim() == 2, 'only mini-batch supported')
   assert(gradOutput:dim() == 2, 'only mini-batch supported')
   assert(self.train == true, 'should be in training mode when self.train is true')
   local nBatch = input:size(1)

   self.gradInput:cmul(self.centered, gradOutput)
   self.buffer:mean(self.gradInput, 1)
   self.gradInput:repeatTensor(self.buffer, nBatch, 1)
   self.gradInput:cmul(self.centered):mul(-1)
   self.buffer:repeatTensor(self.std, nBatch, 1)
   self.gradInput:cmul(self.buffer):cmul(self.buffer)

   self.buffer:mean(gradOutput, 1)
   self.buffer:repeatTensor(self.buffer, nBatch, 1)
   self.gradInput:add(gradOutput):add(-1, self.buffer)
   self.buffer:repeatTensor(self.std, nBatch, 1)
   self.gradInput:cmul(self.buffer)

   if self.affine then
      self.buffer:repeatTensor(self.weight_ap2, nBatch, 1)
      self.gradInput:cmul(self.buffer)
   end

   return self.gradInput
end

function BatchNormalizationShiftPow2:accGradParameters(input, gradOutput, scale)
   if self.affine then
      scale = scale or 1.0
      self.buffer2:resizeAs(self.normalized):copy(self.normalized)
      self.buffer2:cmul(gradOutput)
      self.buffer:sum(self.buffer2, 1) -- sum over mini-batch
      self.gradWeight:add(scale, self.buffer)
      self.buffer:sum(gradOutput, 1) -- sum over mini-batch
      self.gradBias:add(scale, self.buffer)
   end
end
