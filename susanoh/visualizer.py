# -*- coding: utf-8 -*-

import six
import json


class Visualizer(object):
    def __init__(self, server='http://localhost:5000/api'):
        self.server = server

    def send_to_viewer(fired_module):
        req = six.moves.urllib.request.Request(self.server)
        req.add_header('Content-Type', 'application/json')
        send_data = {
                "cells":[fired_module]
        }
        res = six.moves.urllib.request.urlopen(req, json.dumps(send_data))
