# -*- coding: utf-8 -*-

# Convolutional Network for CIFAR-10

import chainer
import chainer.functions as F
import chainer.links as L


class ShallowConvNet(chainer.Chain):

    def __init__(self):
        super(ShallowConvNet, self).__init__(
                conv1 = L.Convolution2D(3, 96, 3, pad=1),
                conv2 = L.Convolution2D(96, 96, 3, pad=1),
                conv3 = L.Convolution2D(96, 128, 3, pad=1),
                conv4 = L.Convolution2D(128, 128, 3, pad=1),
                fc1 = L.Linear(None, 10),
        )

    def __call__(self, x, train=True):
        h = F.max_pooling_2d(F.relu(self.conv1(x)), 2, 2)
        h = F.max_pooling_2d(F.relu(self.conv2(h)), 2, 2)
        h = F.relu(self.conv3(h))
        h = F.relu(self.conv4(h))
        h = self.fc1(h)
        return h

class ThinConvNet(chainer.Chain):

    def __init__(self):
        super(ShallowConvNet, self).__init__(
                conv1 = L.Convolution2D(3, 32, 3, pad=1),
                conv2 = L.Convolution2D(32, 32, 3, pad=1),
                conv3 = L.Convolution2D(32, 32, 3, pad=1),
                conv4 = L.Convolution2D(32, 64, 3, pad=1),
                conv5 = L.Convolution2D(64, 64, 3, pad=1),
                fc1 = L.Linear(None, 10),
        )

    def __call__(self, x, train=True):
        h = F.max_pooling_2d(F.elu(self.conv1(x)), 2, 2)
        h = F.elu(self.conv2(h))
        h = F.elu(self.conv3(h))
        h = F.max_pooling_2d(F.elu(self.conv4(h)), 2, 2)
        h = F.elu(self.conv5(h))
        h = self.fc1(h)
        return h

class AllConvNet(chainer.Chain):

    def __init__(self):
        super(AllConvNet, self).__init__(
                conv1 = L.Convolution2D(3, 96, 3),
                conv2 = L.Convolution2D(96, 96, 3, pad=1),
                # stridced conv
                conv3 = L.Convolution2D(96, 96, 3, stride=2),
                conv4 = L.Convolution2D(96, 192, 3, pad=1),
                conv5 = L.Convolution2D(192, 192, 3, pad=1),
                # stridced conv
                conv6 = L.Convolution2D(192, 192, 3, stride=2),
                conv7 = L.Convolution2D(192, 192, 3, pad=1),
                conv8 = L.Convolution2D(192, 192, 1),
                conv9 = L.Convolution2D(192, 10, 1),
        )

    def __call__(self, x, t=None, train=True):
        h = F.relu(self.conv1(F.dropout(x, ratio=0.2, train=train)))
        h = F.relu(self.conv2(h))
        h = F.dropout(F.relu(self.conv3(h)), ratio=0.5, train=train)
        h = F.relu(self.conv4(h))
        h = F.relu(self.conv5(h))
        h = F.dropout(F.relu(self.conv6(h)), ratio=0.5, train=train)
        h = F.relu(self.conv7(h))
        h = F.relu(self.conv8(h))
        h = F.relu(self.conv9(h))
        # global average pooling
        h = F.reshape(F.average_pooling_2d(h, 6), (x.data.shape[0], 10))
        return h

models = {
        'shallow': ShallowConvNet,
        'thin': ThinConvNet,
        'allconv': AllConvNet,
        }
