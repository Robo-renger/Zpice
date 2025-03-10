#!/usr/bin/env python3
import pickle

class RegionLoader:
    def __init__(self,filename):
        self.contours_list = []
        self.__filename = filename
        self.load_regions()

    def load_regions(self):
        with open(self.__filename, 'rb') as f:
            self.contours_list = pickle.load(f)

    def get_region(self,region_num):
        return self.contours_list[region_num]