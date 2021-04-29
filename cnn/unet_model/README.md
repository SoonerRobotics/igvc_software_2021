# Usage

Install Python dependencies from `requirements.txt`

# Training

Adjust the directories and filename regex in `split_data()` in `train_model.py` to match input files.

Run `train_model.py`. Arguments can be used to modify the training, using `-h` to learn more.

# Data prepping

Convert images to a video

```ffmpeg -r 10 -f image2 -i img_%04d.png output.mp4```

Convert videos to an image

```ffmpeg -i output.mp4 -r 10 img_%04d.png```

Resize args

```-vf scale=1024:576```