#!/usr/bin/env python

from robotBookShelf.pub import MyPublisher
from robotBookShelf.sub import MyListener

if __name__ == '__main__':
    angleOutput = MyPublisher()
    angleOutput.linear2Angle(MyListener().gripper)