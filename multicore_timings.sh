timestamp=$(date +%y-%m-%d_%H:%M:%S)

mkdir -p logs/$timestamp

echo single-core
perf stat taskset -c 0 python test-model.py 1 > logs/$timestamp/single-core.txt &
wait
echo end single-core

echo 2-parallel-single-core
perf stat taskset -c 0 python test-model.py 1 > logs/$timestamp/2-single-core_cpu0.txt &
perf stat taskset -c 1 python test-model.py 1 > logs/$timestamp/2-single-core_cpu1.txt &
wait
echo end 2-parallel-single-core

echo 3-parallel-single-core
perf stat taskset -c 0 python test-model.py 1 > logs/$timestamp/3-single-core_cpu0.txt &
perf stat taskset -c 1 python test-model.py 1 > logs/$timestamp/3-single-core_cpu1.txt &
perf stat taskset -c 2 python test-model.py 1 > logs/$timestamp/3-single-core_cpu2.txt &
wait
echo end 3-parallel-single-core

echo 4-parallel-single-core
perf stat taskset -c 0 python test-model.py 1 > logs/$timestamp/4-single-core_cpu0.txt &
perf stat taskset -c 1 python test-model.py 1 > logs/$timestamp/4-single-core_cpu1.txt &
perf stat taskset -c 2 python test-model.py 1 > logs/$timestamp/4-single-core_cpu2.txt &
perf stat taskset -c 3 python test-model.py 1 > logs/$timestamp/4-single-core_cpu3.txt &
wait
echo end 4-parallel-single-core

echo dual-core
perf stat taskset -c 0,1 python test-model.py 2 > logs/$timestamp/dual-core.txt &
wait
echo end dual-core

echo 2-parallel-dual-core
perf stat taskset -c 0,1 python test-model.py 2 > logs/$timestamp/2-dual-core_cpu01.txt &
perf stat taskset -c 2,3 python test-model.py 2 > logs/$timestamp/2-dual-core_cpu23.txt &
wait
echo end 2-parallel-dual-core

echo tri-core
perf stat taskset -c 0,1,2 python test-model.py 3 > logs/$timestamp/tri-core.txt &
wait
echo end tri-core

echo quad-core
perf stat taskset -c 0,1,2,3 python test-model.py 4 > logs/$timestamp/quad-core.txt &
wait
echo end quad-core

