
import csv
import random
import time

index = 0
posX = 0
posY = 0
posZ = 0

fieldnames = ["time", " final position x", " final position y", " final position z"]


with open('data.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()

while True:

    with open('data.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        info = {
            "time": index,
            " final position x": posX,
            " final position y": posY,
            " final position z": posZ
        }

        csv_writer.writerow(info)
        print(index, posX, posY, posZ)

        index += 1
        posX = posX + random.randint(-6, 8)
        posY = posY - random.randint(-5, 6)
        posZ = posZ + random.randint(-1, 1)

    time.sleep(0.005)
