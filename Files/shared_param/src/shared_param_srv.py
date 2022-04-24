#!/bin/python
from numpy import empty
import rospy  
from shared_param.srv import SharedParam,SharedParamResponse
import pandas
import os

file_path=''
file_n='parameters.csv'
rate=0

"""
find and read parameters.csv file 
that contains important parameters for control algorithm
and send data to control algorithm
"""
for root, dirs, files in os.walk('/home'):
    for name in files:
        if name == file_n:
            file_path=os.path.abspath(os.path.join(root, name))
            break  
    rate+=1   
    if(file_path):break
    elif(rate % 5000==0):
        print('search Shared Parameters File...')
print(file_path)
if(not file_path):
    print('Shared Parameters File Not Found...exit')
    exit()


shParamDf=pandas.read_csv(file_path)
def shared_response(request):
    
    shParamDf=pandas.read_csv(file_path)
    return SharedParamResponse(
            h_p=shParamDf['h_p'][0],
            h_i=shParamDf['h_i'][0],
            h_d=shParamDf['h_d'][0],
            v_p=shParamDf['v_p'][0],
            v_i=shParamDf['v_i'][0],
            v_d=shParamDf['v_d'][0],
            allowed_error=shParamDf['allowed_error'][0]
        )

rospy.init_node('shared_param_service')                     # initialize a ROS node
shared_service = rospy.Service(                        # create a service, specifying its name,
    '/shared_param', SharedParam, shared_response         # type, and callback
)
rospy.spin()  