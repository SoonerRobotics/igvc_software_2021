import numpy as np
import pickle
import argparse
import os
import tensorflow as tf
from tensorflow import keras
from sklearn.model_selection import train_test_split

from image_processing import *
from unet import *

def split_data(args, valid_percent = 0.15):
    # Fetch images
    outs, out_filenames = read_images_from_directory("input/igvc_outs", "out[\d]*.png", processing='out')
    ins, in_filenames = read_images_from_directory("input/igvc_ins", "out[\d]*.png", whitelist=out_filenames, processing='in')

    # Process outputs
    outs = np.expand_dims(outs, axis=3)

    # Split into training and validation
    ins_train, ins_valid, outs_train, outs_valid = train_test_split(ins, outs, test_size=valid_percent)

    return ins_train, outs_train, ins_valid, outs_valid

def train(args):
    # Extract the data sets.  This process uses rotation and Ntraining (among other exp args)
    ins, outs, ins_validation, outs_validation = split_data(args)

    print("ins: ", ins.shape)
    print("outs: ", outs.shape)

    model = SCRUNet()
        
    # Report if verbosity is turned on
    if args.verbose >= 1:
        print(model.summary())
    
    # Callbacks
    early_stopping_cb = keras.callbacks.EarlyStopping(monitor="val_loss",
                                                      patience=args.patience,
                                                      restore_best_weights=True,
                                                      min_delta=0)
    
    # Learn
    history = model.fit(x=ins, y=outs, epochs=args.epochs, verbose=args.verbose>=2,
                        validation_data=(ins_validation, outs_validation), 
                        callbacks=[early_stopping_cb])
        
    # Generate log data
    results = {}
    results['args'] = args
    results['predict_training'] = model.predict(ins)
    results['predict_training_eval'] = model.evaluate(ins, outs)
    results['predict_validation'] = model.predict(ins_validation)
    results['predict_validation_eval'] = model.evaluate(ins_validation, outs_validation)
    results['history'] = history.history
    
    # Save results
    filename = f"{args.results_path}/{args.model_name}"
    fp = open(f"{filename}_results.pkl", "wb")
    pickle.dump(results, fp)
    fp.close()
    
    # Save the model
    model.save(f"{filename}_model")
    print(f'Saved model as "{filename}_model"')
    return model

def get_args():
    parser = argparse.ArgumentParser(description='SCR UNet for Lane Detection')
    parser.add_argument('-epochs', type=int, default=2000, help='Training epochs')
    parser.add_argument('-results_path', type=str, default='./results/', help='Results directory')
    parser.add_argument('-model_name', type=str, default='SCRUNet', help='Model name for output files')
    parser.add_argument('-lrate', type=float, default=0.001, help="Learning rate")
    parser.add_argument('-patience', type=int, default=200, help="Patience for early termination")
    parser.add_argument('-verbose', '-v', action='count', default=2, help="Verbosity level")
    
    return parser.parse_args()

if __name__ == "__main__":
    # Fetch args from command line
    args = get_args()

    # Train!
    train(args)