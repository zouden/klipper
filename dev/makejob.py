a1 = [-10, 99]
# columns are Y
# A1-A12 > along Y axis (not X)

def wellToMM(row, col):
    if col <= 11 and row <= 7:
        xMM = row * 9 + a1[0]
        yMM = col * 9 + a1[1]
        return xMM, yMM
    else:
        raise ValueError("Error at row %s and col %s" % (row, col))

print("G28")
print("G1 Z20 F6000")
for row in range(8):
    for col in range(12):
        print("G1 X%.2f Y%.2f" % wellToMM(row, col)) # go to well
        print("G4 P100") # pause 
        print("G1 Z14") # go down
        print("G91 ;relative mode")
        # diagonal cross
        print("G1 X2 Y2 F2000") 
        print("G1 X-4 Y-4")
        print("G1 X2 Y2")
        print("G1 X2 Y-2")
        print("G1 X-4 Y4")
        print("G1 X2 Y-2")
        print("G90 ;absolute mode")
        print("G1 Z20 F6000") # go up



print("G1 X%.2f Y%.2f" % wellToMM(0, 0))
print("")