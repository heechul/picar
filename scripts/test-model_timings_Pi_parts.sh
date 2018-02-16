timestamp=$(date +%y-%m-%d_%H-%M-%S)

echo "Which board is this test being run on?  Input a string and press [ENTER]"
read board

identification=$board
identification+="_"
identification+=$timestamp

mkdir -p ../logs/test-model/$identification
echo "This test will take approximately 45 minutes to complete"
echo "The output files can be found in: ../logs/test-model/$identification"

echo quad-core-4parts
echo $$ > /sys/fs/cgroup/palloc/part8/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/quad-core_4part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0,1,2,3 python test-model.py 4 > ../logs/test-model/$identification/quad-core_4part.txt &
wait
echo end quad-core
sleep 300

echo single-core-4parts
echo $$ > /sys/fs/cgroup/palloc/part8/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/single-core_4part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0 python test-model.py 1 > ../logs/test-model/$identification/single-core_4part.txt &
wait
echo end single-core
sleep 300

echo quad-core-3parts
echo $$ > /sys/fs/cgroup/palloc/part7/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/quad-core_3part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0,1,2,3 python test-model.py 4 > ../logs/test-model/$identification/quad-core_3part.txt &
wait
echo end quad-core
sleep 300

echo single-core-3parts
echo $$ > /sys/fs/cgroup/palloc/part7/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/single-core_3part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0 python test-model.py 1 > ../logs/test-model/$identification/single-core_3part.txt &
wait
echo end single-core
sleep 300

echo quad-core-2parts
echo $$ > /sys/fs/cgroup/palloc/part5/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/quad-core_2part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0,1,2,3 python test-model.py 4 > ../logs/test-model/$identification/quad-core_2part.txt &
wait
echo end quad-core
sleep 300

echo single-core-2parts
echo $$ > /sys/fs/cgroup/palloc/part5/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/single-core_2part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0 python test-model.py 1 > ../logs/test-model/$identification/single-core_2part.txt &
wait
echo end single-core
sleep 300

echo quad-core-1part
echo $$ > /sys/fs/cgroup/palloc/part1/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/quad-core_1part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0,1,2,3 python test-model.py 4 > ../logs/test-model/$identification/quad-core_1part.txt &
wait
echo end quad-core
sleep 300

echo single-core-1part
echo $$ > /sys/fs/cgroup/palloc/part1/tasks
chrt -f 99 perf stat -o ../logs/test-model/$identification/single-core_1part_perf.txt -e cache-misses,cache-references,LLC-loads,LLC-load-misses,r19,rc2 taskset -c 0 python test-model.py 1 > ../logs/test-model/$identification/single-core_1part.txt &
wait
echo end single-core
sleep 300