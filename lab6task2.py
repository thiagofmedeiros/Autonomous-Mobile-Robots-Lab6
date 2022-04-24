# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

MAX_PHI = 2.5
MAX_SIMULATION_TIME = 30 * 60 * 1000
ACCEPTED_ERROR = 0.001
K = 10

NORTH = 0
SOUTH = math.pi
WEST = math.pi / 2
EAST = -math.pi / 2

MAX_DISTANCE_WALL = 7

DIRECTIONS = [WEST, NORTH, EAST, SOUTH]

labyrinth = [(True, True, False, True),  # 01
             (False, True, False, True),  # 02
             (False, True, False, False),  # 03
             (False, True, True, False),  # 04
             (True, True, False, False),  # 05
             (False, True, True, False),  # 06
             (True, False, True, False),  # 07
             (True, False, True, False),  # 08
             (True, False, True, False),  # 09
             (True, False, False, True),  # 10
             (False, False, True, True),  # 11
             (True, False, True, False),  # 12
             (True, False, False, True),  # 13
             (False, True, False, True),  # 14
             (False, True, False, True),  # 15
             (False, False, True, True)  # 16
             ]

X = [-15, -5, 5, 15,
     -15, -5, 5, 15,
     -15, -5, 5, 15,
     -15, -5, 5, 15]

Y = [15, 15, 15, 15,
     5, 5, 5, 5, 5,
     -5, -5, -5, -5,
     -15, -15, -15, -15]

LABYRINTH_SIZE_X = 4
LABYRINTH_SIZE_Y = 4

NOT_CALCULATED = 0

# init cell matrix
cells = [NOT_CALCULATED] * LABYRINTH_SIZE_X * LABYRINTH_SIZE_Y

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# enable distance sensors
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

time = 0


def metersToInches(x):
    return x * 39.37


def getYawRadians():
    return imu.getRollPitchYaw()[2]


def getSensors():
    fdsVal = metersToInches(frontDistanceSensor.getValue())
    ldsVal = metersToInches(leftDistanceSensor.getValue())
    rdsVal = metersToInches(rightDistanceSensor.getValue())

    return fdsVal, ldsVal, rdsVal


def setSpeedsRPS(rpsLeft, rpsRight):
    if rpsLeft > MAX_PHI:
        leftMotor.setVelocity(MAX_PHI)
    elif rpsLeft < -MAX_PHI:
        leftMotor.setVelocity(-MAX_PHI)
    else:
        leftMotor.setVelocity(rpsLeft)

    if rpsRight > MAX_PHI:
        rightMotor.setVelocity(MAX_PHI)
    elif rpsRight < -MAX_PHI:
        rightMotor.setVelocity(-MAX_PHI)
    else:
        rightMotor.setVelocity(rpsRight)


# uses proportional control to move robot to a desired direction
def correctDirection(desiredDirection):
    global time
    error = getYawRadians() - desiredDirection

    while abs(error) > ACCEPTED_ERROR:
        speed = K * error

        setSpeedsRPS(speed, -speed)
        robot.step(timestep)
        time += timestep

        error = getYawRadians() - desiredDirection


# returns the next position given a current location and the movement direction
def getNewPosition(direction, position):
    if direction == WEST:
        position -= 1
    elif direction == NORTH:
        position -= LABYRINTH_SIZE_X
    elif direction == EAST:
        position += 1
    elif direction == SOUTH:
        position += LABYRINTH_SIZE_X

    return position


# verify if there is no wall blocking
# movement in desired direction
def canMove(direction, walls):
    if direction is None:
        return True
    elif direction == WEST:
        return not walls[0]
    elif direction == NORTH:
        return not walls[1]
    elif direction == EAST:
        return not walls[2]
    else:
        return not walls[3]


# Print data from pose
def printWaveFrontPlan():
    # print("Cell {0}".format(cell))
    # print("Position: ({0:.2f}, {1:.2f}) Yaw: {2:.2f})".format(x, y, yaw))
    for i in range(LABYRINTH_SIZE_X * LABYRINTH_SIZE_Y):

        print(cells[i], end="\t")

        if (i + 1) % LABYRINTH_SIZE_X == 0:
            print("")
    print("")
    # print("({0:.2f}, {1:.2f}, {2}, {3:.2f})".format(X[cell], Y[cell], cell + 1, getYawRadians()))


# randomly choose direction
# from possible movements
def chooseDirection(position):
    minimum = LABYRINTH_SIZE_X * LABYRINTH_SIZE_Y
    chosenDirection = EAST

    for direction in DIRECTIONS:
        if canMove(direction, labyrinth[position]):
            next_position = getNewPosition(direction, position)
            if cells[next_position] < minimum:
                minimum = cells[next_position]
                chosenDirection = direction

    return chosenDirection


# move robot 1 cell or
# up to the middle of the sema cell
def move1Cell(position):
    global time

    walls = labyrinth[position]

    direction = chooseDirection(position)
    # direction = chooseDirection(walls, direction)

    correctDirection(direction)

    distanceTraveled = 0
    front = getSensors()[0]

    setSpeedsRPS(MAX_PHI, MAX_PHI)

    while front > 3.5 and distanceTraveled < 10:
        frontPrevious = getSensors()[0]

        robot.step(timestep)
        time += timestep

        front = getSensors()[0]

        distanceTraveled += abs(frontPrevious - front)

    return getNewPosition(direction, position)


def isAllCellsCalculated():
    for cell in cells:
        if cells[cell] == NOT_CALCULATED:
            return False
    return True


def calculateWaveFront(count):

    while not isAllCellsCalculated():
        for i in range(len(cells)):
            if cells[i] == count:
                for direction in DIRECTIONS:
                    if canMove(direction, labyrinth[i]):
                        next_cell = getNewPosition(direction, i)
                        if cells[next_cell] == NOT_CALCULATED:
                            cells[next_cell] = count + 1
        count += 1

    printWaveFrontPlan()


# First step to get sensor readings
robot.step(timestep)
time += timestep

initCell = 13

goalCell = 4

cells[goalCell - 1] = 2

current_count = 2

currentCell = initCell - 1

calculateWaveFront(current_count)

# print labyrinth

# execute until map is covered or 3 minutes
while not currentCell == goalCell - 1:  # and time < MAX_SIMULATION_TIME:
    currentCell = move1Cell(currentCell)
    print("({0:.2f}, {1:.2f}, {2}, {3:.2f})".format(X[currentCell], Y[currentCell], currentCell + 1, getYawRadians()))
