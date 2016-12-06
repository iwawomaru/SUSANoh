#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import sys
import six
import numpy as np
import chainer
from chainer import cuda, optimizers, serializers
import chainer.functions as F
from sklearn.metrics import accuracy_score
from sklearn.metrics import precision_score
from sklearn.metrics import recall_score

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
parser.add_argument('--batch', '-b', type=int, default=1000, help='batchsize (default: 1000)')
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
dataset = cifar10loader.load('../data/cifar10.pkl')

dim = dataset['test']['data'][0].size
N_test = len(dataset['test']['target'])
test_data_dict = {'data':dataset['test']['data'].astype(np.float32),
                  'target':dataset['test']['target'].astype(np.int32)}
test_data = DataFeeder(test_data_dict, batchsize=args.batch)
test_data.hook_preprocess(cifar_preprocess)

# Model Setup
#beta = [1, 0.99, 0.98]
beta = [1, 1, 1]
model_files = ['shallow_b.h5', 'shallow_a.h5', 'thin.h5']
model_names = ['shallow', 'shallow', 'thin']
model_list = [models.ClassifierModel(convnet.models[m]()) for m in model_names]
for model, f in zip(model_list, model_files):
    serializers.load_hdf5(f, model)
    if args.gpu >= 0:
        cuda.get_device(args.gpu).use()
        model.to_gpu()

res = []
for idx in six.moves.range(0, len(test_data_dict['data']), args.batch):
    # prepare batch
    data_dict = {
            'data': test_data_dict['data'][idx:idx+args.batch], 
            'target': test_data_dict['target'][idx:idx+args.batch]
            }
    data_list = [{k : v[i] if getattr(v,'__iter__',False) else v
        for k,v in data_dict.items()} for i in six.moves.range(len(data_dict['data']))]
    dict_list = [cifar_preprocess(d) for d in data_list]
    vx = tuple( [chainer.Variable( xp.asarray([d['data'] for d in dict_list]) ) ] )

    outputs = np.zeros((args.batch, 10), dtype=np.float32)
    for model,b in zip(model_list, beta):
        #outputs = outputs + xp.asnumpy(model.predict(vx).data)
        outputs = outputs + b * xp.asnumpy(F.softmax(model.predict(vx)).data)
    res.extend(list(outputs.argmax(-1)))

print('Accuracy: ', accuracy_score(test_data_dict['target'], res))
print('Precision: ', precision_score(test_data_dict['target'], res, average='micro'))
print('Recall: ', recall_score(test_data_dict['target'], res, average='micro'))

