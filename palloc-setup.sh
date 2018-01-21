#Enable bits 12, 13, and 14
echo 0x00007000 > /sys/kernel/debug/palloc/palloc_mask

#Create 4 partitions
cgcreate -g palloc:part1
cgcreate -g palloc:part2
cgcreate -g palloc:part3
cgcreate -g palloc:part4
cgcreate -g palloc:part5
cgcreate -g palloc:part6
cgcreate -g palloc:part7
cgcreate -g palloc:part8

#Assign bins to partitions
echo 0-1 > /sys/fs/cgroup/palloc/part1/palloc.bins #Multimodel 4Nx1C and Bandwidth
echo 2-3 > /sys/fs/cgroup/palloc/part2/palloc.bins #"                            "
echo 4-5 > /sys/fs/cgroup/palloc/part3/palloc.bins #"                            "
echo 6-7 > /sys/fs/cgroup/palloc/part4/palloc.bins #"                            "
echo 0-3 > /sys/fs/cgroup/palloc/part5/palloc.bins #Multimodel 2Nx2C
echo 4-7 > /sys/fs/cgroup/palloc/part6/palloc.bins #"              "
echo 0-5 > /sys/fs/cgroup/palloc/part7/palloc.bins #Multicore 3c
echo 0-7 > /sys/fs/cgroup/palloc/part8/palloc.bins #Multicore 4c


#Enable PALLOC and set debug level
echo 1 > /sys/kernel/debug/palloc/use_palloc
echo 2 > /sys/kernel/debug/palloc/debug_level