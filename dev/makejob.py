a1 = [-8, 99]
# columns are Y
# A1-A12 > along Y axis (not X)

def wellToMM(row, col):
    if col <= 11 and row <= 7:
        xMM = row * 9 + a1[0]
        yMM = col * 9 + a1[1]
        return xMM, yMM
    else:
        raise ValueError("Error at row %s and col %s" % (row, col))

print("G1 F7600")
for row in range(8):
    for col in range(12):
        print("G1 X%.2f Y%.2f" % wellToMM(row, col))
        print("G4 P50")
print("G1 X%.2f Y%.2f" % wellToMM(0, 0))