import pandas as pd
import math

nanCheck=float('nan')

dataFile="goodFile.csv"
applicableField=['Acceleration_x(G)','Acceleration_y(G)','Acceleration_z(G)','Gyro_x(rad/s)','Gyro_y(rad/s)','Gyro_z(rad/s)']



readCSV=pd.read_csv(dataFile,skipinitialspace=True,usecols=applicableField)

fieldElementDict={}


for key in readCSV:
    elementCounter=0
    currentColumn=readCSV[key]
    elementList=[]
    while elementCounter<len(currentColumn):
        if math.isnan(currentColumn[elementCounter]):
            elementCounter+=1
        elif key=='Acceleration_z(G)':
            elementList.append(float(currentColumn[elementCounter])-float(1))
            elementCounter+=1
        else:
            elementList.append(float(currentColumn[elementCounter]))
            elementCounter+=1

    fieldElementDict.update({f'{key}':elementList})
    

averagesDict={}
applicableFieldCounter=0

for field in fieldElementDict:

    currentSum=sum(fieldElementDict[field])
    average=currentSum/len(fieldElementDict[field])
    currentLabel=f'{field}'

    averagesDict.update({f'{currentLabel}':average})

print(f'These are the average: {averagesDict}')


for field in fieldElementDict:

    elementCounter=0
    currentItems=fieldElementDict[field]
    emptyList=[]
    newList=[-.1,.1,.2]

    while elementCounter<len(currentItems):
        emptyList.append(currentItems[elementCounter]-averagesDict[field])
        elementCounter+=1
    
    fieldElementDict.update({f'{field}':emptyList})



data=fieldElementDict
newData=pd.DataFrame(data)
newData.to_csv("NewFileDataPython.csv")


for field in fieldElementDict:

    currentSum=sum(fieldElementDict[field])
    average=currentSum/len(fieldElementDict[field])
    currentLabel=f'{field}'

    averagesDict.update({f'{currentLabel}':average})

print(averagesDict)
    


