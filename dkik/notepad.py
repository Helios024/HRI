z=35
def calculatePixelpercm(z):
    pixel_13cm = 0.161*z**2 - 19.64*z + 756.9087
    return pixel_13cm/13 

def centimeterToPixel(z,x,y):
    pixel_1cm =  calculatePixelpercm(z)
    return [x*pixel_1cm, y*pixel_1cm]

def pixelToCentimeter(z,x,y):
    pixel_1cm = calculatePixelpercm(z)
    return [x/pixel_1cm, y/pixel_1cm]

print(pixelToCentimeter(z,441,234))
#Pickup Area = [-0.5778189623008027, 0.3508951946120477, 0.195769494940588, 2.8159421561357774, 1.2461713871922777, 0.048093213242353654]
#WorkSpace = [-0.16748922132429073, 0.38461191053013977, 0.189807704689328, 2.817720711566736, 1.2470568710796477, 0.04611026390787605]
#default = [-0.21689619389127873, 0.35089918035575435, 0.30227183893677323, 2.815952658802643, 1.2462488100121598, 0.047969059287191744]
