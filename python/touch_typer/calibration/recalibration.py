from ..util.utils import *


class RecalibrationDetector():

    # needs to get data from all fingers at once to decide if they are coordinating and all motionless
    # or maybe they all (within some window) tapped a surface


    def __init__(self) -> None:
        self.threshold = 10
        pass

    # Append the self.prevTaps array to the self.allTaps array and remove oldest if space is needed
    def updateAllTaps(self, prevTaps: np.array, allTaps: np.array) -> np.array:
        if len(allTaps) >= self.threshold:
            allTaps = allTaps[1:]
            
        allTaps = np.append(allTaps, [prevTaps])

        return allTaps


    def is_recalibrating(self, prevTaps: np.array, allTaps: np.array) -> (bool, np.array):
        allTaps = self.updateAllTaps(prevTaps, allTaps)
        # If there are not at least 10 cycles logged in allTaps, not in recalibration mode
        if len(allTaps) < self.threshold:
            return False, allTaps

        # Check if all 10 keys have been pressed at least once in the last 10 cycles
        return all(any(tapSet[tap] for tapSet in allTaps) for tap in range(len(allTaps[0]))), allTaps


    # Should return true if y velocity changes from multiple negative values to positive (naive)
    def is_tap_motion(self, prevData: np.array) -> bool:
        # Stores the previous 4 y velocities in an np.array
        prevYVelocities = np.array([prevData[0][4], prevData[1][4], prevData[2][4], prevData[3][4]])

        # stores most recent y velocity
        currYVel = prevData[4][4]

        # check if previous 4 y velocities are negative
        allNegVel = all(vel <= 0 for vel in prevYVelocities)

        # check if previous 4 y velocities are increasing
        allIncreasing = all(prevYVelocities[i] < prevYVelocities[i + 1] for i in range(len(prevYVelocities) - 1))

        return (allNegVel and allIncreasing and currYVel > 0)

