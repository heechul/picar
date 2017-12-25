if [ ! -f "epoch_id.txt" ]; then
    echo 0 > epoch_id.txt
fi

if [ ! -d "epochs" ]; then
    mkdir epochs
fi

epoch_id=`cat epoch_id.txt`

cp -v out-video.avi epochs/out-video-${epoch_id}.avi
cp -v out-key.csv epochs/out-key-${epoch_id}.csv
# cp -v out-key-btn.csv epochs/out-key-btn-${epoch_id}.csv

epoch_id=`expr ${epoch_id} + 1`
echo ${epoch_id} > epoch_id.txt
