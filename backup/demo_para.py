import numpy as np
import config as cf

para = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'lane right': True, 'lane center': False, 'two lane': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}

stage_1 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'lane right': True, 'lane center': False, 'two lane': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': True}

stage_2 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'lane right': True, 'lane center': False, 'two lane': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}

stage_3 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'lane right': True, 'lane center': False, 'two lane': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}


print(para)
para = stage_1
print(para)


