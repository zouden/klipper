import moonrakerpy
printer = moonrakerpy.MoonrakerPrinter('http://localhost:7125')

#define position a1
a1 = [-8, 99]

def gohome():
    printer.send_gcode('G28')

def wellindex_to_mm(row, col):
    # returns gcode coordinates (mm) from indices
    if col <= 11 and row <= 7:
        xMM = row * 9 + a1[0]
        yMM = col * 9 + a1[1]
        return xMM, yMM
    else:
        raise ValueError("Error at row %s and col %s" % (row, col))

def wellname_to_index(wellname):
    try:
        row = 'ABCDEFGH'.index(wellname[0].upper())
        col = int(wellname[1:])-1
    except ValueError:
        print('Well not recognised')
    return row, col

while True:
    wellname = input("Go to well: ")
    coords = wellindex_to_mm(*wellname_to_index(wellname))
    print('Moving to', coords)
    gcode = "G1 X%.2f Y%.2f F8000" % coords
    response = printer.send_gcode(gcode)
    if not response:
        print("Printer must be homed first")
        gohome()