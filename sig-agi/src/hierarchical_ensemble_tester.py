#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import six
import numpy as np
import chainer
from chainer import cuda, optimizers, serializers

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
parser.add_argument('--valbatch', '-v', type=int, default=1000, help='validation batchsize (default: 1000)')
parser.add_argument('--gpu', '-g', type=int, default=-1, help='GPU device #, if you want to use cpu, use -1 (default: -1)')
args = parser.parse_args()


def cifar_preprocess(data):
    data['data'] /= 255.
    return data

# Configure GPU Device
if args.gpu >= 0:
    cuda.check_cuda_available()
xp = cuda.cupy if args.gpu >= 0 else np

# loading dataset
dataset = cifar10.load()

dim = dataset['train']['data'][0].size
N_test = len(dataset['test']['target'])
train_data_dict = {'data':dataset['train']['data'].astype(np.float32),
                   'target':dataset['train']['target'].astype(np.int32)}
test_data_dict = {'data':dataset['test']['data'].astype(np.float32),
                  'target':dataset['test']['target'].astype(np.int32)}
test_data = DataFeeder(test_data_dict, batchsize=args.valbatch)
test_data.hook_preprocess(cifar_preprocess)

# -----------parameter setting-------------- #
alpha = 0.9 # threshold to return
# ------------------------------------------ #

# Model Setup
model_files = ['shallow_a.h5', 'shallow_b.h5', 'thin.h5']
model_names = ['shallow', 'shallow', 'thin']
model_list = [models.ClassifierModel(convnet.models[m]()) for m in model_names]
for model, f in zip(model_list, model_files):
    serializers.load_hd5(f, model)
    if args.gpu >= 0:
        cuda.get_device(args.gpu).use()
        model.to_gpu()

for i, model in enumerate(model_list):
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
        outputs = xp.asnumpy(model.predict(vx).data.max(axis=-1))
        under_alpha_data.extend(list(np.arange(len(outputs))[outputs<=alpha]))

res = []
for idx in six.moves.range(0, len(test_data_dict['data']), args.batch):
    # prepare batch
    data_dict = {
            'data': train_data_dict['data'][idx:idx+args.batch], 
            'target': train_data_dict['target'][idx:idx+args.batch]
            }
    data_list = [{k : v[i] if getattr(v,'__iter__',False) else v
        for k,v in data_dict.items()} for i in six.moves.range(len(data_dict['data']))]
    dict_list = [cifar_preprocess(d) for d in data_list]
    vx = tuple( [chainer.Variable( xp.asarray([d['data'] for d in dict_list]) ) ] )

    outputs = np.zeros((args.batch, 10), dtype=float32)
    for model in model_list:
        outputs = outputs + xp.asnumpy(model.predict(vx).data) * (outputs.max(axis=-1, keepdims=True) <= alpha)
    res.extend(list(outputs / outputs.sum(axis=-1, keepdims=True))

print('Accuracy: ', accuracy_score(test_dict['target'], res))
print('Precision: ', precision_score(test_dict['target'], res))
print('Recall: ', recall_score(test_dict['target'], res))
