import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import datasets, layers, models

def SCRUNet():
    inp = layers.Input(shape=(256, 256, 3))

    # Convolution layers to help learn some basic kernels
    pre_conv = layers.Conv2D(8, (3, 3), strides=(1, 1), padding='same', activation='relu')(inp)

    # Down sampling
    down_sample_0 = layers.Conv2D(16, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(pre_conv)
    down_sample_1 = layers.Conv2D(24, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(down_sample_0)
    down_sample_2 = layers.Conv2D(24, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(down_sample_1)

    # Most compressed layer in the network
    latent = layers.Conv2D(32, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(down_sample_2)

    # Upsampling with skip connections
    up_sample_0 = layers.Conv2DTranspose(24, (3, 3), strides=(2,2), padding='same', activation='relu')(latent)
    skip_0 = layers.Concatenate()([up_sample_0, down_sample_2])

    up_sample_1 = layers.Conv2DTranspose(24, (3, 3), strides=(2,2), padding='same', activation='relu')(skip_0)
    skip_1 = layers.Concatenate()([up_sample_1, down_sample_1])

    up_sample_2 = layers.Conv2DTranspose(16, (3, 3), strides=(2,2), padding='same', activation='relu')(skip_1)
    skip_2 = layers.Concatenate()([up_sample_2, down_sample_0])

    up_sample_3 = layers.Conv2DTranspose(16, (3, 3), strides=(2,2), padding='same', activation='relu')(skip_2)
    skip_3 = layers.Concatenate()([up_sample_3, pre_conv])

    # Post convolution layers
    post_conv = layers.Conv2DTranspose(8, (3, 3), strides=(1, 1), padding='same', activation='relu')(skip_3)

    output = layers.Conv2DTranspose(1, (1, 1), strides=(1,1), padding='same', activation='sigmoid')(post_conv)

    model = models.Model(inputs=inp, outputs=output)

    # Bind the optimizer and the loss function to the model
    model.compile(optimizer='adam',
              loss=tf.keras.losses.BinaryCrossentropy(),
              metrics=[tf.keras.metrics.BinaryAccuracy()])

    return model