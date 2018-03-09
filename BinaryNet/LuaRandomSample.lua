--Change the seed, otherwise numbers will always be the same
math.randomseed(os.time())

--Change order of a set
--Source: http://lua-users.org/wiki/RandomSample
function permute(tab, n, count)
  n = n or #tab
  for i = 1, count or n do
    local j = math.random(i, n)
    tab[i], tab[j] = tab[j], tab[i]
  end
  return tab
end

--Generate a table with all integer values between min and max
function range(min, max)
  local set = {}

  for i = min,max do
    set[#set+1] = i
  end
  return set
end

--Take the first n elements from the given set and return them as a subset
function subrange(set, n)
  local subset = {}

  for i = 1,n do
    subset[#subset + 1] = set[i]
  end
  return subset
end
