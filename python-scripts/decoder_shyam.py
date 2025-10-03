import struct
import pandas as pd
import numpy as np

with open("LOG70.TXT", 'rb') as f:

    characters=f.read()
    maxByte=len(characters)

    currentByte=0

    #These are the initial unpacking lists that store the timestamps and the attributes
    savedBMPList=[]
    savedIMUList=[]
    savedMagnetList=[]


    bmpCounter=0
    imuCounter=0
    magnetCounter=0

    constBMPHex=66
    constIMUHex=73
    constMagnet=77
    constZero=0

    while True:
        if currentByte<=15:
            currentByte+=1
        else:
            if characters[currentByte]==constBMPHex:
                savedBMPList.append([struct.unpack('>I',b'\x00'+characters[currentByte+1:currentByte+4]),struct.unpack('<ff',characters[currentByte+4:currentByte+12])])
                bmpCounter+=1
                currentByte+=12
            elif characters[currentByte]==constIMUHex:
                savedIMUList.append([struct.unpack('>I',b'\x00'+characters[currentByte+1:currentByte+4]),struct.unpack('<ffffff',characters[currentByte+4:currentByte+28])])
                imuCounter+=1
                currentByte+=28
            elif characters[currentByte]==constMagnet:
                savedMagnetList.append([struct.unpack('>I',b'\x00'+characters[currentByte+1:currentByte+4]),struct.unpack('<fff',characters[currentByte+4:currentByte+16])])
                magnetCounter+=1
                currentByte+=16
            elif characters[currentByte]==constZero:
                currentByte+=1
            else:
                break


    #I'm taking out each timestamp for ech packet typ and putting it in order(To be changed)
    timestampsBMP=[savedBMPList[x][0][0] for x in range(len(savedBMPList))]
    timeStampsImu=[savedIMUList[x][0][0] for x in range(len(savedIMUList))]
    timeStampsMagnet=[savedMagnetList[x][0][0] for x in range(len(savedMagnetList))]

    timestampsCombined=timestampsBMP+timeStampsImu+timeStampsMagnet
    
    #Note the sublists that are made based on the number of attributes per packet type
    #For "n" attributes, the values of "n" attributes are stored in the lists below. For example, BMP will have 3 attributes but the list will show however many BMP packets it picked up for each attribute
    attributesBMP=[[savedBMPList[y][1][z] for y in range(len(savedBMPList))] for z in range(2) ]
    attributesImu=[[savedIMUList[y][1][z] for y in range(len(savedIMUList))] for z in range(6)]
    attributesMag=[[savedMagnetList[y][1][z] for y in range(len(savedMagnetList))] for z in range(3)]
    
    #White space creation
    whiteSpacesBMP=["" for x in range(len(savedBMPList))]
    whiteSpacesImu=["" for x in range(len(savedIMUList))]
    whiteSpacesMag=["" for x in range(len(savedMagnetList))]


    #Padding
    for x in range(len(attributesBMP)):
        attributesBMP[x]=attributesBMP[x]+whiteSpacesImu+whiteSpacesMag
    
    for x in range(len(attributesImu)):
        attributesImu[x]=whiteSpacesBMP+attributesImu[x]+whiteSpacesMag

    for x in range(len(attributesMag)):
        attributesMag[x]=whiteSpacesBMP+whiteSpacesImu+attributesMag[x]
    

    data={
        "Time Stamp(S)":timestampsCombined,
        'Temperature(C)':attributesBMP[0],
        'Pressure(Pa)':attributesBMP[1],

    
        'Acceleration_x(G)':attributesImu[0],
        'Acceleration_y(G)':attributesImu[1],
        'Acceleration_z(G)':attributesImu[2],

        'Gyro_x(rad/s)':attributesImu[3],
        'Gyro_y(rad/s)':attributesImu[4],
        'Gyro_z(rad/s)':attributesImu[5],

        'Mag_x(T)':attributesMag[0],
        'Mag_y(T)':attributesMag[1],
        'Mag_z(T)':attributesMag[2],
    }

    
    Combined_Data_Frame=pd.DataFrame(data)
    Combined_Data_Frame=Combined_Data_Frame.sort_values(by="Time Stamp(S)",ascending=True)
    Combined_Data_Frame.to_csv("newCombined.csv")