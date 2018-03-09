--[[
This Function implement the squared hinge loss criterion
]]
local SqrtHingeEmbeddingCriterion, parent = torch.class('SqrtHingeEmbeddingCriterion', 'nn.Criterion')

function SqrtHingeEmbeddingCriterion:__init(margin)
   parent.__init(self)
   self.margin = margin or 1
   self.sizeAverage = true
end

function SqrtHingeEmbeddingCriterion:updateOutput(input,y)
   self.buffer = self.buffer or input.new()
   if not torch.isTensor(y) then
      self.ty = self.ty or input.new():resize(1)
      self.ty[1]=y
      y=self.ty
   end

   self.buffer:resizeAs(input):copy(input)
   self.buffer:cmul(y):mul(-1):add(self.margin)
   self.buffer[torch.le(self.buffer ,0)]=0
   self.output=self.buffer:clone():pow(2):sum()

   if (self.sizeAverage == nil or self.sizeAverage == true) then
      self.output = self.output / input:nElement()
   end

   return self.output
end

function SqrtHingeEmbeddingCriterion:updateGradInput(input, y)
   if not torch.isTensor(y) then self.ty[1]=y; y=self.ty end
   self.gradInput:resizeAs(input):copy(y):mul(-2):cmul(self.buffer)
   self.gradInput[torch.cmul(y,input):gt(self.margin)] = 0
   if (self.sizeAverage == nil or self.sizeAverage == true) then
      self.gradInput:mul(1 / input:nElement())
   end
   return self.gradInput
end
