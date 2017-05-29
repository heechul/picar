for duty in `seq 100`; do
	echo "duty=${duty}%"
	echo 0=${duty}% > /dev/servoblaster
	echo "press key"
	read xx
done

