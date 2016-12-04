#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import chainer
from chainer import cuda, optimizers

# import dataset script
import cifar10

# import from Masalachai
from masalachai import DataFeeder
from masalachai import Logger
from masalachai import trainers
from masalachai import models

# import model network
import convnet

# argparse
parser = argparse.ArgumentParser(description='Convolutional Network Trainer on CIFAR-10')
parser.add_argument('--epoch', '-e', type=int, default=300, help='training epoch (default: 100)')
parser.add_argument('--batch', '-b', type=int, default=500, help='training batchsize (default: 100)')
parser.add_argument('--valbatch', '-v', type=int, default=1000, help='validation batchsize (default: 100)')
parser.add_argument('--model', '-m', type=str, default='shallow', choices=convnet.models.keys(), help='Model you train (default: shallow)')
parser.add_argument('--output', '-o', type=str, default='cifar10.h5', help='Name of trained model file (default: cifar10.h5)')
parser.add_argument('--gpu', '-g', type=int, default=-1, help='GPU device #, if you want to use cpu, use -1 (default: -1)')
args = parser.parse_args()


def cifar_preprocess(data):
    data['data'] /= 255.
    return data

# Logger setup
logger = Logger('CIFAR-10 TRAINING')

# Configure GPU Device
if args.gpu >= 0:
    cuda.check_cuda_available()
xp = cuda.cupy if args.gpu >= 0 else np

# loading dataset
dataset = cifar10.load()

dim = dataset['train']['data'][0].size
N_train = len(dataset['train']['target'])
N_test = len(dataset['test']['target'])
train_data_dict = {'data':dataset['train']['data'].astype(np.float32),
                   'target':dataset['train']['target'].astype(np.int32)}
test_data_dict = {'data':dataset['test']['data'].astype(np.float32),
                  'target':dataset['test']['target'].astype(np.int32)}
train_data = DataFeeder(train_data_dict, batchsize=args.batch)
test_data = DataFeeder(test_data_dict, batchsize=args.valbatch)

train_data.hook_preprocess(cifar_preprocess)
test_data.hook_preprocess(cifar_preprocess)


# Model Setup
model = models.ClassifierModel(AllConvNet())
if args.gpu >= 0:
    cuda.get_device(args.gpu).use()
    model.to_gpu()


# Opimizer Setup
optimizer = optimizers.Adam()
optimizer.setup(model)
optimizer.add_hook(chainer.optimizer.WeightDecay(0.00002))


trainer = trainers.SupervisedTrainer(optimizer, logger, train_data, test_data, args.gpu)
trainer.train(int(args.epoch*N_train/args.batch), 
              log_interval=int(N_train/args.batch), 
              test_interval=int(N_train/args.batch),
              test_nitr=int(N_test/args.valbatch))

serializers.save_hdf5(args.output, model)
