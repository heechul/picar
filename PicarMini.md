This page shows how to train a neural network model to navigate the Picar-Mini, and how to evaluate the real-time performance of the Raspberry Pi 3.

## Data Collection
Before collecting data, it is necessary that OpenCV be installed on the Raspberry Pi 3. This can be done with pip using the command:
	
	$ sudo pip install python-opencv

It is also required that, when connecting to the Raspberry Pi through SSH, a xterm window be used. For both Mac and Linux users this can be achieved by adding the '-x' flag to the ssh terminal command. For Windows users, a X11 display server, like Xming, can be used but would require additional setup.

After the above conditions are met, data can be collected from the Picar-Mini by running the picar-mini-kbd-drv8835.py file. After a delay, this will open a xterm window which should be selected (if it isn't selected, all of the commands listed below will not work). This will give the user manual control over the Picar-Mini, and can operate the car as follows:
* 'a': drive the car forward
* 'z': drive the car backward
* 's': stop the car
* 'j': turn the car left
* 'k': center the car
* 'l': turn the car right
* 't': toggle video view (this will decrease the real-time performance of the car)
* 'r': start/stop recording a video for training. When stopped, the video will be written as 'out-video.avi' and the steering angle can be seen in 'out-key.csv'. If a dataset is to be used for training, it is necessary to rename both of these files, otherwise they will be lost upon the next recording of a video. Preferably, they should be renamed to 'out-video-#.avi' and 'out-key-#.csv', where # is an integer value, and is identical in both files.

##Model Training
The process for training a model for the Picar-Mini is similar to the procedure for developing a model for the original Picar (refer to the Model Training and Testing page). However, there are a few small differences:
* Training data for the Mini is taken from the epochs folder. All datasets are separated into two different files, a video recording of the car driving the track, and a csv file that holds the steering angle for each frame. Videos are named 'out-video-#.avi', and csv files are named 'out-csv-#.avi', where # is an integer value representing the dataset. This value is then used in params.py in either the epochs['train'] or epochs['val'] lists.
* While trivial, since the RC car utilized in the Picar-Mini only has three steering angles, there is no maxAngle variable that needs to be set.

## Model Testing
The Picar-Mini can be tested by running the picar-mini-dnn-drv8835.py file, which operates in a similar fashion to picar-mini-kbd-drv8835.py. In this case, once the car is started, it will turn on its own by feeding frames to model and using the predicted angles to determine the angle at which it should turn. The DAgger method can also be used with the Picar-Mini, as the car can still be controlled by the user and additional videos can be recorded for further model training. Note that the files will still have to be renamed if they are to be used for testing, otherwise the file(s) they are stored in will be overwritten when a new video is recorded.

## Embedded Computing Platform Real-Time Evaluation
In our testing and evaluation of the real-time performance of the Raspberry Pi 3, Intel UP Board, and NVIDIA Tegra TX2, we run the same file, test-model.py, on each one. By default, this will feed a single video, out-video-6.avi, for an approximate total of 700 frames, during which the times for each control loop are collected and used to calculate the statistics of the platforms performance (mean, 99pct, etc.). The total number of input frames can be changed by adding additional epoch numbers to the epochs_id list. Note that the ids in the list do not have to be disjoint (if epoch 6 was listed twice, then out-video-6 would be processed two times). In our experiments, we utilize different tools to determine the effect that certain factors have on platform performance:
* taskset: we use this to limit the number of cores used by a model.
* perf: we utilize this to measure the number of L1 and L2(LLC) cache references and misses.
* benchmark: a memory intensive synthetic benchmark that acts as a co-runner to the model. It can be found in the IsolBench suite.
* vcgencmd: specific to the Raspberry Pi 3, we use this to measure cpu frequency and temperature to ensure that it is always operating at 1.2 GHz.

### Changing the # of Cores used
The number of cores used by TensorFlow can be changed in test-model.py:

	NCPU = _ # Replace _ with the number of cores (between 0-3)

The specific cores to be used can be changed by using taskset:

	$ taskset -c [] python test-model.py 
	
Rather than a number, [] should be replaced with a list of each core to be used. For example, in a test using 4 cores, [] = 0,1,2,3.

###Script Usage
For convenience, we make use of a platform specific shell scripts that run all of the tests for their designated embedded computing platform. For example, the test-model_timings.sh script will run the tests on both the Raspberry Pi 3 and the Intel UP Board. When the scripts are first run, the user is asked to input the name of the platform, which will be used for storing the results as the tests are run. Note that it may take a while for all of the tests to finish. When completed, the results for each test can be found under the logs/test-model directory and in the folder that begins with the input given be the user at the beginning of the script (if the user input "Pi3", then the results folder will begin with Pi3). The date and timestamp of the test are also added to the folder name in order to track when the tests were performed, and prevent any folder naming conflicts from occurring.

