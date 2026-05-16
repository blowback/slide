convert beasTTY.png -resize 1280x640 -strip -quality 85 beasTTY_social.jpg

# palette reduction
#convert input.png -strip -colors 256 PNG8:output.png

# JPEG fallback
# convert input.png -resize 1280x640 -strip -quality 85 output.jpg

# pipeline
# convert input.png -resize 1280x640 -strip png:- | pngquant --quality=80-95 --strip - -o output.png

# #################################
# PNG optimisation (pick one)
# after IM resize/palette reduction
# #################################
#
# pngquant — lossy palette quantization, usually 60-80% size reduction, near-invisible quality loss
#pngquant --quality=80-95 --strip output.png -o output-small.png

# oxipng — lossless, slower, gives an extra 5-20% on top
#oxipng -o max --strip safe output-small.png

# zopflipng — lossless, very slow, squeezes out the last few percent
#zopflipng -m output-small.png output-final.png
