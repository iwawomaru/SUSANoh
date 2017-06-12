#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import sys
import six
import numpy as np
import chainer
from chainer import cuda, optimizers, serializers
import chainer.functions as F

# import dataset script
sys.path.append('../data/')
import cifar10loader

# import from Masalachai
from masalachai import DataFeeder
from masalachai import Logger
from masalachai import trainers
from masalachai import models

# import model network
import convnet


# argparse
parser = argparse.ArgumentParser(description='Convolutional Network Trainer on CIFAR-10')
parser.add_argument('--valbatch', '-v', type=int, default=1000, help='validation batchsize (default: 1000)')
parser.add_argument('--trainbatch', '-t', type=int, default=60, help='training batchsize (default: 60)')
parser.add_argument('--epoch', '-e', type=int, default=5, help='training epoch (default: 5)')
parser.add_argument('--alpha', '-a', type=float, default=1.8, help='alpha parameter of models (default: 1.8)')
parser.add_argument('--gpu', '-g', type=int, default=-1, help='GPU device #, if you want to use cpu, use -1 (default: -1)')
args = parser.parse_args()


def cifar_preprocess(data):
    data['data'] /= 255.
    return data

# -----------parameter setting-------------- #
batchsize = args.trainbatch
training_epoch = args.epoch
alpha = args.alpha # threshold to return
# ------------------------------------------ #

# Configure GPU Device
if args.gpu >= 0:
    cuda.check_cuda_available()
xp = cuda.cupy if args.gpu >= 0 else np

# loading dataset
dataset = cifar10loader.load('../data/cifar10.pkl')

dim = dataset['train']['data'][0].size
N_test = len(dataset['test']['target'])
train_data_dict = {'data':dataset['train']['data'].astype(np.float32),
                   'target':dataset['train']['target'].astype(np.int32)}
test_data_dict = {'data':dataset['test']['data'].astype(np.float32),
                  'target':dataset['test']['target'].astype(np.int32)}
test_data = DataFeeder(test_data_dict, batchsize=args.valbatch)
test_data.hook_preprocess(cifar_preprocess)


# Model Setup
model_files = ['shallow_b.h5', 'shallow_a.h5', 'thin.h5']
model_names = ['shallow', 'shallow', 'thin']
model_list = [models.ClassifierModel(convnet.models[m]()) for m in model_names]
for model, f in zip(model_list, model_files):
    serializers.load_hdf5(f, model)
    if args.gpu >= 0:
        cuda.get_device(args.gpu).use()
        model.to_gpu()


# initialize models
opts = []
for i, model in enumerate(model_list):
    # Logger setup
    logger = Logger('INIT: CIFAR-10 TRAINING Layer '+str(i))
    # Opimizer Setup
    opts.append(optimizers.Adam())
    opts[-1].setup(model)

# the first under_alpha_data is all of training data.
under_alpha_data = [idx for idx in range(len(train_data_dict['data']))]

# training loop
while len(under_alpha_data) != 0:
    for i, model in enumerate(model_list):
        # Train
        if len(under_alpha_data) != 0:
            # Logger setup
            logger = Logger('TRAIN: CIFAR-10 TRAINING Layer '+str(i))
            N_train = len(under_alpha_data)
            t_data_dict = {
                    'data': train_data_dict['data'][under_alpha_data], 
                    'target': train_data_dict['target'][under_alpha_data]
                    }
            train_data = DataFeeder(t_data_dict, batchsize=batchsize)
            train_data.hook_preprocess(cifar_preprocess)
            trainer = trainers.SupervisedTrainer(opts[i], logger, train_data, test_data, args.gpu)
            trainer.train(int(training_epoch*(N_train/batchsize+1)), 
                          log_interval=int(N_train/batchsize+1), 
                          test_interval=int(N_train/batchsize+1),
                          test_nitr=int(N_test/args.valbatch+1))
            serializers.save_hdf5('ensemble_'+model_files[i], model)

        # Pick Up Under Alpha Value Data for Next Classifier
        under_alpha_data = []
        for idx in six.moves.range(0, len(train_data_dict['data']), batchsize):
            data_dict = {
                    'data': train_data_dict['data'][idx:idx+batchsize], 
                    'target': train_data_dict['target'][idx:idx+batchsize]
                    }
            data_list = [{k : v[i] if getattr(v,'__iter__',False) else v
                for k,v in data_dict.items()} for i in six.moves.range(len(data_dict['data']))]
            dict_list = [cifar_preprocess(d) for d in data_list]
            vx = tuple( [chainer.Variable( xp.asarray([d['data'] for d in dict_list]) ) ] )
            outputs = xp.asnumpy(F.softmax(model.predict(vx)).data.max(axis=-1))
            batch_idx_array = np.arange(len(outputs))[outputs<=alpha] + idx # idx is offset
            under_alpha_data.extend(list(batch_idx_array))


