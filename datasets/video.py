from moviepy.editor import ImageSequenceClip
import argparse
import csv
import matplotlib.pyplot as pyplot


def main():
    parser = argparse.ArgumentParser(description='Create driving video.')
    parser.add_argument(
        'image_folder',
        type=str,
        default='',
        help='Path to image folder. The video will be created from these images.'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=60,
        help='FPS (Frames per second) setting for the video.')
    args = parser.parse_args()
	
    setNum = args.image_folder.split('dataset')[1]
    setNum = setNum.split('/')[0]
	
    print (setNum)
	
    imageArray = []
    with open('dataset%s/data.csv' % setNum,'rt') as file:
        reader = csv.reader(file) #Create a reader to read the csv file line by line
        for line in reader: #For each line in the csv fileine == 1): #If it's not the first line
            if(line[0] != 'center'):
                imagePath = "dataset{}/png/".format(setNum) + line[0].split('IMG\\')[1] #Get the local path to the image
                image = pyplot.imread(imagePath) #Read the image
                imageArray.append(image)
	
    video_file = args.image_folder + '.mp4'
    print("Creating video {}, FPS={}".format(video_file, args.fps))
    clip = ImageSequenceClip(imageArray, fps=args.fps)
    clip.write_videofile(video_file)


if __name__ == '__main__':
    main()
