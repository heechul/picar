require 'torch'
local params = require('params')
local cv = require('cv')
require('cv.videoio')
require('cv.highgui')
require('cv.imgproc')
require('cv.imgcodecs')
require('dp')
require('LuaCSV.lua')

local train_set =
{
  frames = {},
  angles = {}
}

local val_set =
{
  frames = {},
  angles = {}
}

local test_set =
{
  frames = {}
}

local trainEpochs = table.getn(params.train_sets)

print("Training Data:")

for i = 1,trainEpochs do
  local epochNum = params.train_sets[i]

  print("    Getting data from epoch "..epochNum)

  local csvpath = paths.concat("epochs", "out-key-"..epochNum..".csv")
  file = io.open(csvpath, "r")

  assert(file~=nil, "Can't open "..csvpath)

  local firstLine = true
  for line in io.lines(csvpath) do
    if firstLine ~= true then
      local col = ParseCSVLine(line, ",")
      train_set.angles[#train_set.angles+1] = tonumber(col[3])
    else
      firstLine = false
    end
  end

  local avipath = paths.concat("epochs", "out-video-"..epochNum..".avi")
  local vid = cv.VideoCapture{avipath}

  local ret, frame = vid:read{}

  while ret do
    frame = cv.resize{frame, {params.img_width, params.img_height}}
    img = frame:permute(3,1,2):index(1,torch.LongTensor{3,2,1})
    train_set.frames[#train_set.frames+1] = img
    ret = vid:read{frame}
  end
end

local valEpochs = table.getn(params.val_sets)

print("Validation Data:")

for i = 1,valEpochs do
  local epochNum = params.val_sets[i]

  print("    Getting data from epoch "..epochNum)

  local csvpath = paths.concat("epochs", "out-key-"..epochNum..".csv")
  file = io.open(csvpath, "r")

  assert(file~=nil, "Can't open "..csvpath)

  local firstLine = true
  for line in io.lines(csvpath) do
    if firstLine ~= true then
      local col = ParseCSVLine(line, ",")
      val_set.angles[#val_set.angles+1] = tonumber(col[3])
    else
      firstLine = false
    end
  end

  local avipath = paths.concat("epochs", "out-video-"..epochNum..".avi")
  local vid = cv.VideoCapture{avipath}

  local ret, frame = vid:read{}

  while ret do
    frame = cv.resize{frame, {params.img_width, params.img_height}}
    img = frame:permute(3,1,2):index(1,torch.LongTensor{3,2,1})
    val_set.frames[#val_set.frames+1] = img
    ret = vid:read{frame}
  end
end

local testEpochs = table.getn(params.test_sets)

print("Testing Data:")

for i = 1,testEpochs do
  local epochNum = params.test_sets[i]

  print("    Getting data from epoch "..epochNum)

  local avipath = paths.concat("epochs", "out-video-"..epochNum..".avi")
  local vid = cv.VideoCapture{avipath}

  local ret, frame = vid:read{}

  while ret do
    test_set.frames[#test_set.frames+1] = img
    ret = vid:read{frame}
  end
end

assert(table.getn(train_set.frames) == table.getn(train_set.angles), "Unequal number of frames and angles")
assert(table.getn(val_set.frames) == table.getn(val_set.angles), "Unequal number of frames and angles")

print("# of Training frames/angles = "..table.getn(train_set.frames))
print("# of Validation frames/angles = "..table.getn(val_set.frames))
print("# of Testing frames/angles = "..table.getn(test_set.frames))

local train_file = paths.concat("datasets", "train.t7")
torch.save(train_file, train_set)
local val_file = paths.concat("datasets", "val.t7")
torch.save(val_file, val_set)
local test_file = paths.concat("datasets", "test.t7")
torch.save(test_file, test_set)
