with open('bear.txt') as file:
    content = file.read()

print(content[:90])

def file_process(char, path):
    with open(path) as file:
        strings = file.read()
    return strings.count(char)

with open('file.txt',"w") as file:
    file.write('snail')

with open('bear.txt', 'r') as file:
    bear_chars = file.read()

with open('first.txt', "w") as file:
    file.write(bear_chars[:90])

with open('bear1.txt', 'r') as file:
    bear_chars = file.read()

with open('bear2.txt', "a") as file:
    #file.write("\n")
    file.write(bear_chars)