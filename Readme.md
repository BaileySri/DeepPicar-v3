# DeepPicar


DeepPicar is a low-cost autonomous RC car platform using a deep
convolutional neural network (CNN). DeepPicar is a small scale replication
of NVIDIA's real self-driving car called Dave-2, which drove on public
roads using a CNN. DeepPicar uses the same CNN architecture of NVIDIA's
Dave-2 and can drive itself in real-time locally on a Raspberry Pi 3.

Preprocess:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Karenw1004/Deeppicar-v3/blob/main/notebooks/1_Preprocess.ipynb)

Train:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Karenw1004/Deeppicar-v3/blob/main/notebooks/2_Train.ipynb)

Inference:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Karenw1004/Deeppicar-v3/blob/main/notebooks/3_Inference.ipynb)


Video:

[![DeepPicar Driving](http://img.youtube.com/vi/SrS5iQV2Pfo/0.jpg)](http://www.youtube.com/watch?v=SrS5iQV2Pfo "DeepPicar_Video")

Some other examples of the DeepPicar driving can be found at: https://photos.app.goo.gl/q40QFieD5iI9yXU42



packages to install

apt install libatlas-base-dev
pip3 install opencv-python
pip3 install tflite_runtime

'''
    import json
    import platform
    from typing import List, NamedTuple

    import cv2
    import numpy as np
    from tflite_support import metadata

    # pylint: disable=g-import-not-at-top
    try:
    # Import TFLite interpreter from tflite_runtime package if it's available.
    from tflite_runtime.interpreter import Interpreter
    from tflite_runtime.interpreter import load_delegate
    except ImportError:
    # If not, fallback to use the TFLite interpreter from the full TF package.
    import tensorflow as tf

    Interpreter = tf.lite.Interpreter
    load_delegate = tf.lite.experimental.load_delegate
'''