echo 0x00003000 > /sys/kernel/debug/palloc/palloc_mask
cgcreate -g palloc:part1
echo 0 > /sys/fs/cgroup/palloc/part1/palloc.bins
echo 1 > /sys/kernel/debug/palloc/use_palloc
echo 2 > /sys/kernel/debug/palloc/debug_level
