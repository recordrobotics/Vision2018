from networktables import NetworkTables as nt

nt.initialize(server='roborio-6731-frc.local')

sd = nt.getTable('SmartDashboard')

fin = open("out.txt", "r")

while 1:
    # get values from file
    # put files in dashbaord
    fin.seek(2)
    lineList = fin.readlines
    # print(lineList)
    # print(len(lineList))
    temp = lineList[:-1]
    # temp = temp[:-1]

    x, y = temp.split()
    x = int(x)
    y = int(y)

    print(x, y)
    #sd.putNumber(x, 
    #sd.putNumber(, y


fin.close()

