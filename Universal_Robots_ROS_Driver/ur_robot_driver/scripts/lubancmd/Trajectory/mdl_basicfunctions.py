#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import copy as cp

def QuaterDot(quater1, quater2):
    q1 = np.array([quater1.x, quater1.y, quater1.z, quater1.w])
    q2 = np.array([quater2.x, quater2.y, quater2.z, quater2.w])
    Dot = np.dot(q1, q2)
    if Dot > 1.0:
        return 1.0
    else:
        return Dot

def QuaterNegate(quater):
    return geometry_msgs.msg.Quaternion(x = -quater.x, y = -quater.y, z = -quater.z, w = -quater.w)

def QuaterLog(quaterarray):
    if quaterarray[3] > 1.0:
        quaterarray[3] = 1.0
    theta = math.acos(quaterarray[3])
    if theta == 0:
        v = np.array([0., 0., 0.])
    else:
        v = np.array([quaterarray[0], quaterarray[1], quaterarray[2]])/math.sin(theta)
    return np.array([v[0]*theta, v[1]*theta, v[2]*theta, 0.0])

def QuaterExp(quaterarray):
    theta = math.sqrt(quaterarray[0]**2 + quaterarray[1]**2 + quaterarray[2]**2)
    if theta == 0:
        v = np.array([0., 0., 0.])
    else:
        v = quaterarray[0:3]/theta
    stheta = math.sin(theta)
    return np.array([v[0]*stheta, v[1]*stheta, v[2]*stheta, math.cos(theta)])    

def LineInterpolationFindSpan(TrajLengthVector, TrajLength, InitialIndex):
    if TrajLengthVector[InitialIndex] <= TrajLength and TrajLength < TrajLengthVector[InitialIndex+1]:
        return InitialIndex

    N = len(TrajLengthVector)

    if InitialIndex < 0:
        initidx = 0
    elif InitialIndex >= N-2:
        initidx = N-2
    else:
        initidx = InitialIndex
    b_initial_correct = False
    while not(b_initial_correct):
        if TrajLengthVector[initidx] <= TrajLength:
            b_initial_correct = True
        else:
            initidx -= 1
            if initidx < 0:
                print('error input of TrajLengthVector and TrajLength')
                return -1
    
    b_find_next_index = False
    next_index = initidx+ 1
    while not(b_find_next_index):
        if TrajLength < TrajLengthVector[next_index]:
            b_find_next_index = True
        else:
            next_index += 1
            initidx += 1
            if next_index > N-1:
                # print('error input of TrajLengthVector and TrajLength')
                return N-2
    index = initidx
    return index

def AxisQuinticInterpolation(JointBegin, JointSpeedBegin, JointAccelerationBegin, JointEnd, JointSpeedEnd, JointAccelerationEnd, SetDuration, SetPeriod):
    n = len(JointBegin)
    T = SetPeriod * math.ceil(SetDuration/SetPeriod)
    A = []
    for i in range(n):
        Ai = QuinticPara(JointBegin[i], JointSpeedBegin[i], JointAccelerationBegin[i], JointEnd[i], JointSpeedEnd[i], JointAccelerationEnd[i], T)
        A.append(Ai)
    tick = SetPeriod
    joints = []
    while tick < T:
        jnts = np.zeros(n)
        for i in range(n):
            jnts[i] = Quintic(A[i], tick)
        joints.append(jnts)
        tick +=SetPeriod
    return joints


def Quintic(QuinticPar, t):
    return QuinticPar[0] + QuinticPar[1]*t + QuinticPar[2]*t**2 + QuinticPar[3]*t**3 + QuinticPar[4]*t**4+ QuinticPar[5]*t**5

def QuinticPara(p0, s0, a0, p1, s1, a1, t):
    QuinticPar = np.zeros(6)
    QuinticPar[0] = p0
    QuinticPar[1] = s0
    QuinticPar[2] = a0/2.0
    QuinticPar[3] = (20.0*(p1 - p0) - (8.0*s1 + 12.0*s0)*t - (3.0*a0 - a1)*t**2)/(2*t**3)
    QuinticPar[4] = (30.0*(p0 - p1) + (14.0*s1 + 16.0*s0)*t + (3.0*a0 - 2.0*a1)*t**2)/(2*t**4)
    QuinticPar[5] = (12.0*(p1 - p0) - (6.0*s1 + 6.0*s0)*t - (a0 - a1)*t**2)/(2.0*t**5)
    return QuinticPar

def MeanWindowFilter(SignalArray, WindowSize):
    """ return filtered data of input SignalArray with WindowSize*2+1 """
    assert (WindowSize >= 1 and len(SignalArray) > WindowSize), 'error input value of MeanWindowFilter'

    N = len(SignalArray)
    signal_filtered = np.zeros(N, dtype = float)

    signal_filtered[0] = SignalArray[0]
    for i in range(1, WindowSize):
        signal_filtered[i] = sum(SignalArray[0:i*2+1])/(2*i+1)
    
    for i in range(WindowSize, N-WindowSize):
        signal_filtered[i] = sum(SignalArray[i-WindowSize:i+WindowSize+1])/(2*WindowSize+1)

    for i in range(N-WindowSize, N-1):
        _i = N - i -1
        signal_filtered[i] = sum(SignalArray[i - _i: i + _i + 1])/(2*_i+1)
    
    signal_filtered[-1] = SignalArray[-1]

    return signal_filtered

def SignalDiff(SignalArray, dt):
    """return signal difference divided by time dt"""
    N = len(SignalArray)

    assert (N > 3 and dt > 0.0), 'error input variable of SignalDiff'

    SignalArray_Out = np.zeros(N)
    SignalArray_Out[1:-1] = (SignalArray[2:] - SignalArray[0:-2])/(2.0*dt)
    SignalArray_Out[-1] = SignalArray_Out[-2]
    SignalArray_Out[0] = SignalArray_Out[1]
    return SignalArray_Out
    
def FreqMultiplier(SignalArray, MultiTimes):
    """return frequency multiplied signal of input SignalArray"""
    assert (MultiTimes > 1 and len(SignalArray) > 3), 'error input value of FreqMultiplier'

    N = len(SignalArray)
    dt_origin = 1.0
    dt_divided = dt_origin/MultiTimes
    SignalArray_Out = np.zeros((N-1)*MultiTimes + 1, dtype=float)
    SignalArray_Out[0] = SignalArray[0]

    SignalArray_Speed = SignalDiff(SignalArray, dt_origin)
    SignalArray_Acc   = SignalDiff(SignalArray_Speed, dt_origin)

    parray = np.zeros(MultiTimes)

    for i in range(1, N):
        p0 = SignalArray[i-1];          p1 = SignalArray[i]
        s0 = SignalArray_Speed[i-1];    s1 =SignalArray_Speed[i]
        a0 = SignalArray_Acc[i-1];      a1 = SignalArray_Acc[i]
        quinticpar = QuinticPara(p0, s0, a0, p1, s1, a1, dt_origin)
        for t in range(MultiTimes):
            parray[t] = Quintic(quinticpar, t*dt_divided)
        SignalArray_Out[(i-1)*MultiTimes+1:i*MultiTimes+1] = parray
    return SignalArray_Out

def SignalResample(SignalArray, SampleTimes, Shift = 0):
    """return signal re sampled with every SampleTimes signal"""
    N = len(SignalArray)
    assert (N > SampleTimes and Shift >=0 and Shift <  SampleTimes), 'error input value of FreqMultiplier'
    
    SignalArray_Out = np.array([])

    for i in range(N):
        if i%SampleTimes == Shift:
            SignalArray_Out = np.append(SignalArray_Out, SignalArray[i])
    return SignalArray_Out

def lcm(x, y):
    if x > y:
        greater = x
    else:
        greater = y
    while(True):
        if ((greater%x==0)and (greater%y==0)):
            lcm = greater
            break
        greater += 1
    return lcm

def JointReSample(JointList, dt_origin, dt_set, filter_set = 2):
    if filter_set ==0:
        filter_window_size = 0
    elif filter_set == 1:
        filter_window_size = 3
    elif filter_set == 2:
        filter_window_size = 5
    elif filter_set == 3:
        filter_window_size = 20
    else:# default filter_set = 2
        filter_window_size = 10

    _dt_orgin = int(np.floor(dt_origin*10000))
    _dt_set = int(np.floor(dt_set*10000))

    lest_common_multiple = lcm(_dt_orgin, _dt_set)

    m = int(lest_common_multiple/_dt_orgin)
    n = int(lest_common_multiple/_dt_set)

    if m == n:
        return JointList

    j_tmp = JointList[0]
    N_col = len(j_tmp)
    N_row = len(JointList)

    Joint_array = np.array([np.array(j) for j in  JointList])
    Joint_Resample = np.zeros([int(np.ceil(float(N_row)*n/m)), N_col])

    for i in range(N_col):
        jnts = Joint_array[:, i]
        if filter_window_size == 0:
            jnts_filtered = jnts
        else:
            jnts_filtered = MeanWindowFilter(jnts, filter_window_size)
        if n == 1:
            jnts_freqmultiple = jnts_filtered
        else:
            jnts_freqmultiple = FreqMultiplier(jnts_filtered, n)
        if m == 1:
            jnts_resample = jnts_freqmultiple
        else:
            jnts_resample = SignalResample(jnts_freqmultiple, m)

        if len(Joint_Resample) != len(jnts_resample):
            Joint_Resample = np.zeros([len(jnts_resample), N_col])

        Joint_Resample[:,i] = jnts_resample

    return Joint_Resample.tolist()
