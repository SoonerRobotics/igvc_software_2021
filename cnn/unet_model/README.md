# Usage

Install Python dependencies from `requirements.txt`

# Training

Adjust the directories and filename regex in `split_data()` in `train_model.py` to match input files.

Run `train_model.py`. Arguments can be used to modify the training, using `-h` to learn more.

# Data prepping

Convert video to images

```ffmpeg -i input.mp4 -r 8 example/video_frames/img_%04d.png```

Convert images to video

```ffmpeg -r 8 -f image2 -i example/video_prediction/f%04d.png output.mp4```

Put two videos next to each other

```
ffmpeg \
  -i input.mp4 \
  -i output.mp4 \
  -filter_complex '[0:v]pad=iw*2:ih[int];[int][1:v]overlay=W/2:0[vid]' \
  -map '[vid]' \
  -c:v libx264 \
  -crf 23 \
  -preset veryfast \
  combined_output.mp4
```

Resize args

```-vf scale=1024:576```