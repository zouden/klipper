def wash():
    print("goto well=H6")
    print("p0 v-10 f5")
    print("p0 v10 f5")
    print("g1 z24")

# tak
# print('''
# ; Pump gradient
# goto well=H6 ;quick dip
# goto well=A1 ;yellow well

# ''')
print("G28")

def takeAndDispense(from_well, to_well, amount):
    print("goto well="+from_well)
    print("p0 v-%s" % amount)
    print("G4 P500")
    print("goto well="+to_well)
    print("p0 v%s f5" % amount)

for i in range(2,12):
    takeAndDispense("A2", "C%s" % i, i*2)
    print("p2 v30 f10")
    print("G4 P200")

print("G1 Z24")
print("M84")
print("")