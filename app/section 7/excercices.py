colors = [11, 34, 98, 43, 45, 54, 54]

for item in colors:
    print(item)

colors = [11, 34, 98, 43, 45, 54, 54]

for item in colors:
    if 50 < item:
        print (item)

colors = [11, 34.1, 98.2, 43, 45.1, 54, 54]

for item in colors:
    if isinstance(item, int):
        print(item)

colors = [11, 34.1, 98.2, 43, 45.1, 54, 54]

for item in colors:
    if isinstance(item, int) and 50 < item:
        print(item)

        
agendadict = {"John Smith"    : 37682929928,
              "Marry Simpons" : 423998200919
            }
            
for key, value in agendadict.items():
    print(f"{key}: +{value}")