src=src/*.265
dir_265=265
dir_mp4=mp4
dir_mkv=mkv



mkdir -p $dir_265 $dir_mp4 $dir_mkv


for fullfile in $src; do
	echo "proccessing" $fullfile
	fullname=$(basename -- "$fullfile")
	dirname=$(dirname -- "$fullfile")
	extension="${fullname##*.}"
	filename="${fullname%.*}"
	# split
	cp $fullfile $dir_265/$fullname
	gpac -i $fullfile hevcsplit:FID=1:nosei:delim=false:rcfg=false:rwdsi=false:rwnal=false -o $dir_265/$filename\_\$CropOrigin\$x\$Width\$x\$Height\$.265:SID=1#CropOrigin=\* -graph
done

rm $dir_mkv/*.mkv

for fullfile in $dir_265/*.265; do
	fullname=$(basename -- "$fullfile")
	dirname=$(dirname -- "$fullfile")
	extension="${fullname##*.}"
	filename="${fullname%.*}"
	mp4=$dir_mp4/$filename.mp4
	mkv=$dir_mkv/$filename.mkv
	echo $fullname "->" $mp4 "->" $mkv
	MP4Box -add $fullfile -new $mp4
	ffmpeg -i $mp4 -c:v copy $mkv
	mkclean --no-optimize $mkv $mkv.clean
	mv $mkv.clean $mkv
done
